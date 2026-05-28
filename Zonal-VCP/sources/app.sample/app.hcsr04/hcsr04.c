/*
 * hcsr04.c
 * HC-SR04 Ultrasonic Sensor Driver for TOPST VCP-G (FreeRTOS / Cortex-R5F)
 *
 * [흐름 요약]
 *
 *  [CollisionAvoidTask]  매 60ms
 *       |
 *       |-- 1. Trig_SendPulse()       GPIO HIGH 12us -> LOW
 *       |-- 2. echo_done 대기 (spin, max HCSR04_ECHO_TIMEOUT_US)
 *       |-- 3. 거리 계산             duration_us / 58
 *       |-- 4. xQ_Emer 전송          ON(충돌 위험) / OFF(정상)
 *
 *  [Echo_ISR]  GIC 외부 인터럽트 (both-edge, GIC_EXT0)
 *       |-- rising  edge : echo_start = PMU cycle counter
 *       |-- falling edge : echo_end   = PMU cycle counter, echo_done = 1
 */

#include "hcsr04.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <sal_com.h>
//include <sal_os.h>    /* SAL_TaskSleep */
#include <task.h> /* SAL_TaskSleep */
#include <gpio.h>
#include <gic.h>
#include "vcp_types.h" /* VCP_IO_ACTION_ON, VCP_IO_ACTION_OFF */

/* xQ_Emer 는 main.c(VCP_CreateApp)에서 생성된 외부 전역 큐 */
extern QueueHandle_t xQ_Emer;

/* -----------------------------------------------------------------------
 * PMU Cycle Counter 인라인 헬퍼 (Cortex-R5F, MMAP 불필요)
 * ----------------------------------------------------------------------- */

static inline void Pmu_EnableCycleCounter(void)
{
    /* PMCR  : E(enable) | P(reset PMCCNTR) | C(reset counters) = 0x7 */
    __asm volatile ("MCR p15, 0, %0, c9, c12, 0" :: "r"(0x7UL));
    /* PMCNTENSET : CCNT enable bit[31] = 1 */
    __asm volatile ("MCR p15, 0, %0, c9, c12, 1" :: "r"(0x80000000UL));
}

static inline uint32 Pmu_ReadCycleCounter(void)
{
    uint32 val;
    __asm volatile ("MRC p15, 0, %0, c9, c13, 0" : "=r"(val));
    return val;
}

/* -----------------------------------------------------------------------
 * ISR 공유 변수 (volatile: 컴파일러 최적화로 레지스터 캐싱 방지)
 * ----------------------------------------------------------------------- */
static volatile uint32  sEchoStart  = 0UL;
static volatile uint32  sEchoEnd    = 0UL;
static volatile uint8   sEchoDone   = 0U;

/* -----------------------------------------------------------------------
 * Trig 펄스 발사
 *   12us busy-wait: Cortex-R5F @300MHz 기준 ~3600 cycle.
 *   타이머 ISR 세팅보다 오버헤드가 작으므로 단순 루프 사용.
 * ----------------------------------------------------------------------- */
static void Trig_SendPulse(void)
{
    uint32 i;
    uint32 loopCount = HCSR04_TRIG_PULSE_US * HCSR04_CPU_MHZ;

    GPIO_Set(HCSR04_TRIG_PIN, 1UL);

    /* busy-wait: 루프 1회 ≒ 1 cycle (최적화 방지용 volatile 캐스트) */
    for (i = 0UL; i < loopCount; i++)
    {
        __asm volatile ("nop");
    }

    GPIO_Set(HCSR04_TRIG_PIN, 0UL);
}

/* -----------------------------------------------------------------------
 * Echo ISR — GIC 외부 인터럽트 (both-edge)
 *
 *   [rising edge]  : ECHO 핀 = HIGH -> echo_start 저장
 *   [falling edge] : ECHO 핀 = LOW  -> echo_end 저장, sEchoDone = 1
 *
 *   ISR 내부에서 판단·큐 전송 금지.
 *   측정값은 volatile 변수에만 기록하고 Task에서 소비한다.
 * ----------------------------------------------------------------------- */
static void Echo_ISR(void *pArg)
{
    (void)pArg;

    if (GPIO_Get(HCSR04_ECHO_PIN) != 0UL)
    {
        /* Rising edge: Echo 신호 시작 */
        sEchoStart = Pmu_ReadCycleCounter();
        mcu_printf("!\r\n");
    }
    else
    {
        /* Falling edge: Echo 신호 종료 */
        sEchoEnd  = Pmu_ReadCycleCounter();
        sEchoDone = 1U;
        mcu_printf(".\r\n");
    }
}

/* -----------------------------------------------------------------------
 * Hcsr04_Init
 *   1. Trig 핀: OUTPUT 설정
 *   2. Echo 핀: INPUT + 내부 풀다운 (신호 없을 때 LOW 유지)
 *   3. PMU cycle counter 활성화
 *   4. Echo ISR GIC 등록 (both-edge)
 * ----------------------------------------------------------------------- */
void Hcsr04_Init(void)
{
    SALRetCode_t ret;
    uint32       echoVal;

    /* 1. Trig: OUTPUT */
    GPIO_Config(HCSR04_TRIG_PIN,
                GPIO_FUNC(0UL) | GPIO_OUTPUT | GPIO_NOPULL | GPIO_DS(3UL));
    GPIO_Set(HCSR04_TRIG_PIN, 0UL);

    /* 2. Echo: INPUT, 풀다운 */
    GPIO_Config(HCSR04_ECHO_PIN,
                GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN);

    /* 3. Echo 핀 초기 레벨 확인 — 0이어야 정상 (HC-SR04 idle = LOW) */
    echoVal = GPIO_Get(HCSR04_ECHO_PIN);
    mcu_printf("[HCSR04] Echo pin idle level = %d (expect 0)\r\n", (int)echoVal);

    /* 4. PMU cycle counter 활성화 */
    Pmu_EnableCycleCounter();

    /* 5. GPIO_IntExtSet — Echo 핀을 GIC_EXT0 소스로 연결 */
    ret = GPIO_IntExtSet(HCSR04_ECHO_GIC_INT, HCSR04_ECHO_PIN);
    mcu_printf("[HCSR04] GPIO_IntExtSet ret=%d (expect 0=OK)\r\n", (int)ret);

    /* 6. GIC_IntVectSet — ISR 등록 (EDGE_BOTH → 내부적으로 IRQ 2개 등록) */
    ret = GIC_IntVectSet(HCSR04_ECHO_GIC_INT,
                         GIC_PRIORITY_NO_MEAN,
                         GIC_INT_TYPE_EDGE_BOTH,
                         (GICIsrFunc)&Echo_ISR,
                         (void *)0);
    mcu_printf("[HCSR04] GIC_IntVectSet ret=%d (expect 0=OK)\r\n", (int)ret);

    /* 7. GIC_IntSrcEn — rising용 IRQ 활성화 */
    ret = GIC_IntSrcEn(HCSR04_ECHO_GIC_INT);
    mcu_printf("[HCSR04] GIC_IntSrcEn(EXT0) ret=%d (expect 0=OK)\r\n", (int)ret);

    mcu_printf("[HCSR04] Init done. TRIG=GPA(21), ECHO=GPA(22)\r\n");
}

/* -----------------------------------------------------------------------
 * Hcsr04_CollisionAvoidTask
 *
 *   매 60ms 실행 사이클:
 *     1. Trig 발사
 *     2. sEchoDone 폴링 대기 (timeout: HCSR04_ECHO_TIMEOUT_US)
 *     3. 거리 계산
 *     4. 임계값 비교 -> xQ_Emer 전송
 *     5. SAL_TaskSleep(60) -> 다음 주기
 *
 *   [timeout 설계]
 *     ECHO_TIMEOUT_US 동안 sEchoDone == 0 이면:
 *       - 센서 미응답 or 측정 범위 초과(>400cm)
 *       - 안전을 위해 ON(경고) 전송하지 않고 그냥 다음 주기로 넘김
 *       - 필요 시 별도 에러 카운터로 연속 실패 감지 가능
 * ----------------------------------------------------------------------- */
void Hcsr04_CollisionAvoidTask(void *pArg)
{
    uint32   timeoutCount;
    uint32   durationUs;
    uint32   distanceCm;
    uint8    msg[2];
    boolean  isCollisionActive = FALSE;

    (void)pArg;

    Hcsr04_Init();

    for (;;)
    {
        /* 1. 이전 측정 플래그 클리어 */
        sEchoDone = 0U;

        /* 2. Trig 발사 */
        Trig_SendPulse();

        /* 3. Echo ISR 완료 대기 (spin-wait, max HCSR04_ECHO_TIMEOUT_US)
         *
         *    왜 SAL_TaskSleep 대신 spin?
         *      - osDelay/TaskSleep 최소 단위 = 1ms = 17cm 오차
         *      - Echo 신호 자체가 ISR로 처리되므로 CPU는 플래그만 확인
         *      - 최대 대기 = 25ms (400cm 기준), 60ms 주기 안에서 여유 있음
         */
        timeoutCount = 0UL;
        while ((sEchoDone == 0U) && (timeoutCount < HCSR04_ECHO_TIMEOUT_US))
        {
            __asm volatile ("nop");
            timeoutCount++;
        }

        /* 4. 거리 계산 및 판단 */
        if (sEchoDone == 1U)
        {
            /* cycle 차이 -> microsecond 변환 */
            durationUs  = (sEchoEnd - sEchoStart) / HCSR04_CPU_MHZ;

            /* 왕복 시간 -> 거리 (음속 343m/s, 왕복 58us/cm) */
            distanceCm  = durationUs / 58UL;

            mcu_printf("[HCSR04] dist=%d cm\r\n", (int)distanceCm);

            if (distanceCm < HCSR04_COLLISION_THRESHOLD_CM)
            {
                /* 충돌 위험 — 아직 ON을 안 보낸 상태이면 한 번만 전송 */
                if (isCollisionActive == FALSE)
                {
                    msg[0] = (uint8)VCP_IO_ACTION_ON;
                    msg[1] = 0U;
                    (void)xQueueSend(xQ_Emer, msg, 0);
                    isCollisionActive = TRUE;
                    mcu_printf("[HCSR04] COLLISION WARNING! dist=%d cm\r\n",
                               (int)distanceCm);
                }
            }
            else
            {
                /* 안전 거리 — ON 상태였으면 OFF 전송 */
                if (isCollisionActive == TRUE)
                {
                    msg[0] = (uint8)VCP_IO_ACTION_OFF;
                    msg[1] = 0U;
                    (void)xQueueSend(xQ_Emer, msg, 0);
                    isCollisionActive = FALSE;
                }
            }
        }
        else
        {
            /* timeout: 센서 무응답 — 로그만 남기고 계속 */
            mcu_printf("[HCSR04] Echo timeout (no object or >400cm)\r\n");
        }

        /* 5. 다음 주기까지 대기 (60ms - 이미 소비한 Echo 대기 시간 포함) */
        (void)SAL_TaskSleep(HCSR04_POLL_PERIOD_MS);
    }
}