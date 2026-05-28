#include "pdm.h"
#include "gpio.h"
#include "stdio.h"
#include "pdm_ctrl.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <vcp_types.h>

extern QueueHandle_t xQ_MotorSpeed;
extern QueueHandle_t xQ_MotorWheel;


#define IN1     GPIO_GPA(22)
#define IN2     GPIO_GPA(21)
#define IN3     GPIO_GPA(20)
#define IN4     GPIO_GPA(19)

#define ENA_SEL 0
#define ENA_PORT GPIO_PERICH_CH0
#define ENB_SEL 4
#define ENB_PORT GPIO_PERICH_CH1

#define PDM_PERIOD_NS       (250000)
#define SPEED_MAX           (80)
#define DUTY_MAX            (80)
#define MIN_ON_NS           (15000)

#define MIN_DUTY            (35)


static PDMModeConfig_t cfgA, cfgB;


// TG
#define ESC_ARM_VALUE_FIX               ((uint32)40U << 8)
#define ESC_ARM_DELAY_MS                1000U


/*
#define ESC_PERIOD_NS                   20000000UL
#define ESC_STOP_DUTY_NS                1000000UL
#define ESC_MAX_DUTY_NS                 2000000UL
#define ESC_MAX_ANGLE_FIX               ((uint32)100U << 8)
static PDMModeConfig_t esc_cfg;
*/

static boolean esc_inited = FALSE;
static ESCState_t esc_state = ESC_STATE_OFF;

static SALRetCode_t ESC_SetAngle256(uint32 angle_fix);

uint32 duty_pct_to_ns(uint32 pct, uint32 period_ns);
sint8 duty_from_speed(sint8 speed);
void MotorA_Set(uint32 duty_pct, uint32 forward);
void MotorB_Set(uint32 duty_pct, uint32 forward);

static inline void pin_out(uint32 p) { GPIO_Config(p, GPIO_OUTPUT | GPIO_FUNC(0) | GPIO_DS(3));}
static inline void pin_hi(uint32 p) { GPIO_Set(p, 1);}
static inline void pin_lo(uint32 p) { GPIO_Set(p, 0);}
static int pdm_apply(uint32 ch, PDMModeConfig_t* cfg)
{
    (void)PDM_Disable(ch, PMM_OFF);
    uint32 wait = 0;

    while(PDM_GetChannelStatus(ch) && wait < 100){
        SAL_TaskSleep(1);
        wait++;
    }

    if(PDM_SetConfig(ch, cfg) != SAL_RET_SUCCESS) return -1;
    if(PDM_Enable(ch, PMM_OFF) != SAL_RET_SUCCESS) return -2;
    return 0;
}

uint32 duty_pct_to_ns(uint32 pct, uint32 period_ns)
{
    if (pct == 0) return 0;
    if (pct > 100) pct = 100;
    if (pct < MIN_DUTY && pct > 0) pct = MIN_DUTY;
    uint64 num = (uint64)period_ns * (uint64)pct + 50ULL;
    return (uint32)(num / 100ULL);
}

sint8 duty_from_speed(sint8 speed)
{
    if(speed == 0)
    {
        return 0;
    }

    if(speed > 80)
    {
        speed = 80;
    }

    if(speed < 0)
    {
        return 0;
    }

    return 40 + ((speed - 1) * 80) / 79;
}

void MotorA_Set(uint32 duty_pct, uint32 forward)  // 모터 A 제어
{
    if (duty_pct == 0) {
        pin_lo(IN1); pin_lo(IN2);  // 정지
    } else if (forward) {
        pin_lo(IN1); pin_hi(IN2);  // 정방향
    } else {
        pin_hi(IN1); pin_lo(IN2);  // 역방향
    }

    cfgA.mcDutyNanoSec1 = duty_pct_to_ns(duty_pct, cfgA.mcPeriodNanoSec1);
    (void)pdm_apply(ENA_SEL, &cfgA);
}

void MotorB_Set(uint32 duty_pct, uint32 forward)  // 모터 B 제어
{
    if (duty_pct == 0) {
        pin_lo(IN3); pin_lo(IN4);  // 정지
    } else if (forward) {
        pin_hi(IN3); pin_lo(IN4);  // 정방향
    } else {
        pin_lo(IN3); pin_hi(IN4);  // 역방향
    }

    cfgB.mcDutyNanoSec1 = duty_pct_to_ns(duty_pct, cfgB.mcPeriodNanoSec1);
    (void)pdm_apply(ENB_SEL, &cfgB);
}

SALRetCode_t ESC_PWM_Init(void)
{
    if (esc_inited != FALSE)
    {
        return SAL_RET_SUCCESS;
    }

    // GPIO 초기화
    pin_out(IN1); pin_out(IN2);
    pin_out(IN3); pin_out(IN4);
    pin_lo(IN1); pin_lo(IN2);
    pin_lo(IN3); pin_lo(IN4);

    // PDM 초기화
    PDM_Init();
    PDM_CfgSetWrPw();
    PDM_CfgSetWrLock(0);
    
    // 모터 A 설정
    SAL_MemSet(&cfgA, 0, sizeof(cfgA));
    cfgA.mcPortNumber      = ENA_PORT;
    cfgA.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    cfgA.mcPeriodNanoSec1  = PDM_PERIOD_NS;
    cfgA.mcDutyNanoSec1    = 0;
    (void)pdm_apply(ENA_SEL, &cfgA);
    
    // 모터 B 설정
    SAL_MemSet(&cfgB, 0, sizeof(cfgB));
    cfgB.mcPortNumber      = ENB_PORT;
    cfgB.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    cfgB.mcPeriodNanoSec1  = PDM_PERIOD_NS;
    cfgB.mcDutyNanoSec1    = 0;
    (void)pdm_apply(ENB_SEL, &cfgB);


    esc_inited = TRUE;
    mcu_printf("ESC PWM Init Done\n");
    return SAL_RET_SUCCESS;
}



boolean ESC_IsReady(void)
{
    return (esc_state == ESC_STATE_ARMED) ? TRUE : FALSE;
}

static SALRetCode_t ESC_SetAngle256(uint32 angle_fix)
{
    if (esc_inited == FALSE)
    {
        return SAL_RET_FAILED;
    }

    sint8 speed = (angle_fix >> 8) * 80 / 100; // 0~80
    static sint8 duty = 0;

    if(speed > 80) speed = 80;

    if(speed==0)
    {
        MotorA_Set(0, 1);
        MotorB_Set(0, 1);
        return SAL_RET_SUCCESS;
    }
    else if(speed > 0)
    {
        duty = duty_from_speed(speed);
        MotorA_Set(duty, 1);
        MotorB_Set(duty, 1);
        return SAL_RET_SUCCESS;
    }
    else 
    {
        duty = duty_from_speed(speed);
        MotorA_Set(duty, 0);
        MotorB_Set(duty, 0);
        return SAL_RET_SUCCESS;
    }

    return SAL_RET_FAILED;
}

void ConfigureServoPWM(uint32 channel, uint32 port, uint32 angle_deg)
{
    PDMModeConfig_t pwm_cfg;
    uint32 duty_ns = 500000 + (angle_deg * (2000000 / 180)); // 0~180도 → 0.5~2.5ms
    uint32 wait_cnt = 0;

    pwm_cfg.mcPortNumber      = port;
    pwm_cfg.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    pwm_cfg.mcInversedSignal  = 0;
    pwm_cfg.mcOutSignalInIdle = 0;
    pwm_cfg.mcLoopCount       = 0;
    pwm_cfg.mcOutputCtrl      = 0;

    pwm_cfg.mcPeriodNanoSec1  = 20000000; // 20ms (50Hz)
    pwm_cfg.mcDutyNanoSec1    = duty_ns;
    pwm_cfg.mcPeriodNanoSec2  = 0;
    pwm_cfg.mcDutyNanoSec2    = 0;

    PDM_Disable(channel, PMM_ON);
    while (PDM_GetChannelStatus(channel))
    {
        SAL_TaskSleep(1);
        if (++wait_cnt > 100)
        {
            mcu_printf("Timeout on channel %d\n", channel);
            return;
        }
    }

    if (PDM_SetConfig(channel, &pwm_cfg) != SAL_RET_SUCCESS)
    {
        mcu_printf("SetConfig fail (CH:%d)\n", channel);
        return;
    }

    if (PDM_Enable(channel, PMM_ON) != SAL_RET_SUCCESS)
    {
        mcu_printf("Enable fail (CH:%d)\n", channel);
        return;
    }

    mcu_printf("CH%d angle: %3d° → duty: %d ns\n", channel, angle_deg, duty_ns);
}

/* -------------------------- Task -------------------------- */

void MotorSpeedTask(void *pvParameters) 
{
    uint8 recvBuf[2];
    uint32 speed_fix = 0;

    (void)pvParameters;

    esc_state = ESC_STATE_INIT;
    mcu_printf("ESC Arming...\n");

    if (ESC_PWM_Init() != SAL_RET_SUCCESS)
    {
        esc_state = ESC_STATE_ERROR;
        mcu_printf("ESC Init Failed!\n");
    }
    else
    {
        esc_state = ESC_STATE_ARMING;
        if (ESC_SetAngle256(ESC_ARM_VALUE_FIX) != SAL_RET_SUCCESS)
        {
            esc_state = ESC_STATE_ERROR;
            mcu_printf("ESC Arming Failed!\n");
        }
        else
        {
            SAL_TaskSleep(ESC_ARM_DELAY_MS);
            esc_state = ESC_STATE_ARMED;
            mcu_printf("ESC Ready!\n");
        }
    }

    for (;;) 
    {
        if (xQueueReceive(xQ_MotorSpeed, recvBuf, portMAX_DELAY) == pdPASS) 
        {
            if (esc_state != ESC_STATE_ARMED)
            {
                continue;
            }

            speed_fix = ((uint32)recvBuf[0] << 8) | (uint32)recvBuf[1];

            if (ESC_SetAngle256(speed_fix) != SAL_RET_SUCCESS)
            {
                esc_state = ESC_STATE_ERROR;
                mcu_printf("ESC Update Failed!\n");
            }
        }
    }
}

void MotorWheelTask(void *pvParameters)
{
    uint8 recvBuf[2];
    int8_t input = 0;
    int angle = 0;

    (void)pvParameters;

    for (;;)
    {
        if (xQueueReceive(xQ_MotorWheel, recvBuf, portMAX_DELAY) == pdPASS)
        {
            input = (int8_t)recvBuf[0];

            if (input < 0) input = 0;
            if (input > 127) input = 127;

            angle = 90 + ((input - 65) * 45) / 65;

            ConfigureServoPWM(5, GPIO_PERICH_CH3, angle);
        }
    }
}
