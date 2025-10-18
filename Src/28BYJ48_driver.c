#include <28BYJ38_driver.h>
#include "main.h"

extern TIM_HandleTypeDef htim1;


static stepper_t st = {0};


static inline void allCoilsOff(void) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}


static const uint8_t step_sequence[8][4] = {
    {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},
    {0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}
};


static inline void setPins(const uint8_t step[4]) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, step[0]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, step[1]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, step[2]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, step[3]?GPIO_PIN_SET:GPIO_PIN_RESET);
}


void StepMotor_Run(int32_t steps, int8_t direction, uint16_t interval_us)
{
    if (steps <= 0) return;
    st.steps_left  = steps;
    st.dir         = (direction >= 0) ? 1 : -1;
    st.interval_us = (interval_us == 0) ? 1 : interval_us;  // min. 1 µs
    st.t_last      = __HAL_TIM_GET_COUNTER(&htim1);
    st.active      = 1;

}


void StepMotor_Handler(void)
{
    if (!st.active || st.steps_left <= 0) return;


    uint16_t now = __HAL_TIM_GET_COUNTER(&htim1);
    uint16_t dt  = (uint16_t)(now - st.t_last);
    if (dt < st.interval_us) return;
    st.t_last = now;

    setPins(step_sequence[st.seq_idx]);
    st.seq_idx = (st.dir > 0) ? (uint8_t)((st.seq_idx + 1) & 0x07)
                              : (uint8_t)((st.seq_idx + 7) & 0x07);
    if (--st.steps_left <= 0) {
        st.active = 0;
        allCoilsOff();
    }
}

void StepMotor_Stop(void)
{
    st.active     = 0;
    st.steps_left = 0;
    allCoilsOff();
}


uint8_t Step_IsActive(void) { return st.active; }
int32_t Step_StepsLeft(void){ return st.steps_left; }
