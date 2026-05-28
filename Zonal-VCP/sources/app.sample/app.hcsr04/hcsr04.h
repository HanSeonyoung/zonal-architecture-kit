/*
 * hcsr04.h
 * HC-SR04 Ultrasonic Sensor Driver for TOPST VCP-G (FreeRTOS / Cortex-R5F)
 *
 * [구조 요약]
 *   Trig : CollisionAvoidTask 내부 폴링, 60ms 주기, GPIO High 12us -> Low
 *   Echo : GIC 외부 인터럽트 ISR (both-edge), PMU cycle counter 타임스탬프
 *   판단 : CollisionAvoidTask -> xQ_Emer Queue -> EmergencySignalTask (기존 재사용)
 */

#ifndef HCSR04_H
#define HCSR04_H

#include <sal_com.h>   /* uint8, uint32, boolean, SALRetCode_t */
#include <gpio.h>      /* GPIO_GPx(), GPIO_Config, GPIO_Set, GPIO_Get */
#include <gic.h>       /* GIC_IntVectSet, GIC_IntSrcEn, GIC_EXT0 */

/* -----------------------------------------------------------------------
 * [핀 설정] — 실제 보드 배선 후 아래 두 매크로만 교체
 *   TRIG : OUTPUT 전용. GPIO_GPA/B/C/K 어느 핀이든 가능.
 *   ECHO : INPUT + 외부 인터럽트 필요. GPIO_IntExtSet 지원 핀이어야 함.
 *          지원 핀 목록: GPIO_GPA(0-30), GPB(0-28), GPC(0-27), GPK(0-17)
 * ----------------------------------------------------------------------- */
#define HCSR04_TRIG_PIN         GPIO_GPB(20UL)  /* TODO: 배선 후 수정 */
#define HCSR04_ECHO_PIN         GPIO_GPB(21UL)  /* TODO: 배선 후 수정 */
#define HCSR04_ECHO_GIC_INT     GIC_EXT0        /* Echo ISR 사용할 GIC 외부 인터럽트 번호 */

/* -----------------------------------------------------------------------
 * [타이밍 상수]
 * ----------------------------------------------------------------------- */
#define HCSR04_TRIG_PULSE_US    (12UL)   /* Trig HIGH 유지 시간 (spec: >=10us) */
#define HCSR04_POLL_PERIOD_MS   (60UL)   /* CollisionAvoidTask 주기 */
#define HCSR04_ECHO_TIMEOUT_US  (25000UL)/* 400cm * 58us/cm = 23200us, 여유 포함 */

/* -----------------------------------------------------------------------
 * [거리 임계값]
 *   충돌 경고를 발생시킬 거리 (cm). 필요에 따라 조정.
 * ----------------------------------------------------------------------- */
#define HCSR04_COLLISION_THRESHOLD_CM   (20UL)

/* -----------------------------------------------------------------------
 * [PMU 클럭]
 *   Cortex-R5F @ 300MHz 기준. BSP 클럭 설정 확인 후 수정.
 *   duration_us = cycle_diff / HCSR04_CPU_MHZ
 * ----------------------------------------------------------------------- */
#define HCSR04_CPU_MHZ          (300UL)

/* -----------------------------------------------------------------------
 * [공개 API]
 * ----------------------------------------------------------------------- */

/*
 * Hcsr04_Init
 *   GPIO 설정 + PMU cycle counter 활성화 + Echo ISR 등록
 *   VCP_CreateApp() 또는 Main_StartTask() 초기화 시점에 1회 호출
 */
void Hcsr04_Init(void);

/*
 * Hcsr04_CollisionAvoidTask
 *   SAL_TaskCreate로 등록할 태스크 함수
 *   - 60ms마다 Trig 발사
 *   - echo_done 플래그 대기 (timeout 포함)
 *   - 거리 계산 후 xQ_Emer 큐로 ON/OFF 전송
 */
void Hcsr04_CollisionAvoidTask(void *pArg);

#endif /* HCSR04_H */