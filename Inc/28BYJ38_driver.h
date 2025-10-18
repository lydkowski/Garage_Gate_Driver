#pragma once
#include "stm32l4xx_hal.h"
#include <stdint.h>

/*
 * Wymagania:
 * - TIM1 musi pracować z częstotliwością 1 MHz (Prescaler=79 przy SYSCLK=80 MHz)
 *   i być wystartowany (HAL_TIM_Base_Start(&htim1)).
 * - Wyprowadzenia IN1..IN4 oraz ich porty są zdefiniowane w main.h.
 * - Stepper_Handler() wywołuj często w pętli głównej (nieblokujące krokowanie).
 */

typedef struct {
    volatile int32_t steps_left;   // ile półkroków pozostało do wykonania
    int8_t  dir;                   // 1 = CW, -1 = CCW
    uint8_t seq_idx;               // bieżący indeks sekwencji (0..7)
    uint16_t interval_us;          // odstęp między półkrokami [µs]
    uint16_t t_last;               // znacznik czasu z timera 1 [µs], 16 bit
    uint8_t active;                // 1 = trwa ruch
} stepper_t;


void StepMotor_Run(int32_t steps, int8_t direction, uint16_t interval_us);


void StepMotor_Handler(void);

void StepMotor_Stop(void);


uint8_t  Step_IsActive(void);
int32_t  Step_StepsLeft(void);
