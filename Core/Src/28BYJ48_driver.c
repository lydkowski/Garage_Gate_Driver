#include <28BYJ38_driver.h>
#include "main.h"                  // IN1..IN4_* oraz deklaracja htim1

extern TIM_HandleTypeDef htim1;    // zapewnia dostęp do licznika TIM1

/* --- Stan sterownika (prywatny dla modułu) --- */
static stepper_t st = {0};

/* Wyłączenie wszystkich cewek */
static inline void allCoilsOff(void) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

/* Sekwencja półkrokowa 8x4 (const → do flash) */
static const uint8_t step_sequence[8][4] = {
    {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},
    {0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}
};

/* Ustaw piny wg sekwencji */
static inline void setPins(const uint8_t step[4]) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, step[0]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, step[1]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, step[2]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, step[3]?GPIO_PIN_SET:GPIO_PIN_RESET);
}

/* Start ruchu (nieblokujący) */
void StepMotor_Run(int32_t steps, int8_t direction, uint16_t interval_us)
{
    if (steps <= 0) return;
    st.steps_left  = steps;
    st.dir         = (direction >= 0) ? 1 : -1;
    st.interval_us = (interval_us == 0) ? 1 : interval_us;  // min. 1 µs
    st.t_last      = __HAL_TIM_GET_COUNTER(&htim1);
    st.active      = 1;
    // Jeśli chcesz zawsze zaczynać od „0”: odkomentuj poniższą linię:
    // st.seq_idx = 0;
}

/* Zadanie wykonywane „w tle”: 1 półkrok, gdy minie czas */
void StepMotor_Handler(void)
{
    if (!st.active || st.steps_left <= 0) return;

    /* TIM1 @1 MHz, licznik 16-bit. Różnica modulo 2^16: */
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim1);
    uint16_t dt  = (uint16_t)(now - st.t_last);
    if (dt < st.interval_us) return;   // jeszcze nie czas
    st.t_last = now;

    /* Półkrok */
    setPins(step_sequence[st.seq_idx]);
    st.seq_idx = (st.dir > 0) ? (uint8_t)((st.seq_idx + 1) & 0x07)
                              : (uint8_t)((st.seq_idx + 7) & 0x07);

    if (--st.steps_left <= 0) {
        st.active = 0;
        allCoilsOff();
    }
}

/* Pomocnicze: status */
uint8_t Step_IsActive(void) { return st.active; }
int32_t Step_StepsLeft(void){ return st.steps_left; }
