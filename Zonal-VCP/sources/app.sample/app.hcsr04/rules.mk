# Zonal-VCP/sources/app.sample/app.hcsr04/rules.mk
# HC-SR04 Ultrasonic Sensor Driver Build Rules

# SPDX-License-Identifier: Apache-2.0
MCU_BSP_APP_SAMPLE_HCSR04_PATH := $(MCU_BSP_BUILD_CURDIR)

VPATH    += $(MCU_BSP_APP_SAMPLE_HCSR04_PATH)
INCLUDES += -I$(MCU_BSP_APP_SAMPLE_HCSR04_PATH)

SRCS += hcsr04.c