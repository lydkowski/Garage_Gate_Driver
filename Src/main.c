/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <ir.h>
#include <string.h>
#include <stdlib.h>
#include <28BYJ38_driver.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MODE_MANUAL = 0,
  MODE_AUTO
} gate_mode_t;

typedef enum {
  IDLE = 0,
  OPENING_GATE,
  CLOSING_GATE,
  OBSTACLE_PAUSE
} motor_state_t;

typedef enum {
  EV_NONE = 0,
  EV_MENU_TOGGLE,
  EV_IR_START_OPEN,     // ON/OFF + CLOSED
  EV_IR_START_CLOSE,    // ON/OFF + OPEN
  EV_REACHED_OPEN,      // krańcówka OPEN=1 w trakcie OPENING
  EV_REACHED_CLOSED,    // krańcówka CLOSED=1 w trakcie CLOSING
  EV_OBSTACLE,          // czujnik < próg w trakcie CLOSING
  EV_PAUSE_TIMEOUT,     // minął czas pauzy po przeszkodzie
  EV_AUTO_TIMEOUT       // minął czas auto-domknięcia po pełnym OPEN
} event_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum {
  B_ONOFF = 1u << 0,
  B_CLS   = 1u << 1,
  B_OPN   = 1u << 2
};

#define DIR_RIGHT       		(1)     	// otwieranie musi byc na odwrot bo silnik jest w nasza strone
#define DIR_LEFT        		(-1)      	// zamykanie tak samo
#define STEPSIZE       			(1000) 		// prędkość kroków
#define STEPNUM   				(1000000)   // duża liczba kroków (jedź aż do krańcówki)
#define OBSTACLE_THRESH      	(400U)    	// próg z czujnika
#define OBSTACLE_PAUSE_MS    	(1000U)  	// pauza 1 s przed rewersem
#define MENU_TOGGLE_DEBOUNCE_MS (400U)
#define AUTO_DELAY_MIN_MS      	(1000U)   	// >= 1 s
#define AUTO_DELAY_MAX_MS      	(600000U)   // <= 10 min
#define UART_RX_BUF_SZ 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
motor_state_t motor_state =IDLE; // defaultowo jest w idle
gate_mode_t   gate_mode   = MODE_MANUAL;
uint32_t obstacle_t0 = 0;
uint32_t auto_t0     = 0;   // czas t0 od tego mierzymy w trybie auto
uint8_t  auto_ready  = 0;   // flaga informujaca o tym że brama jest gotowa do autozamykania
uint32_t last_menu_toggle_ms = 0; // do debouncu
volatile uint32_t auto_close_delay_ms = 10000U;

uint8_t LS_Closed_ON(void) {
  return HAL_GPIO_ReadPin(isClosed_GPIO_Port, isClosed_Pin) == GPIO_PIN_RESET;
}
uint8_t LS_Open_ON(void) {
  return HAL_GPIO_ReadPin(isOpen_GPIO_Port, isOpen_Pin) == GPIO_PIN_RESET;
}


// Do uarta
uint8_t  uart_rx_ch;
char     uart_line[UART_RX_BUF_SZ];
uint16_t uart_line_len = 0;
volatile uint8_t uart_line_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void BlinkAliveLED(void);
static void process_uart_line(void);
static inline uint32_t read_distance(void);
static inline uint32_t build_key(int pilot_code, uint8_t GateClosed, uint8_t GateOpened);
static event_t poll_event(uint8_t GateClosed, uint8_t GateOpened, int pilot_code, uint32_t key,uint32_t dist_sensor, uint32_t now);
static void handle_event(event_t ev, uint8_t GateOpened);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return 1;
}

void BlinkAliveLED(void) { // Blink Alive Led, dioda miga sprawdzajac czy pętla się wykonuje
  static uint32_t t0 = 0;
  uint32_t t = HAL_GetTick();
  if ((t - t0) >= 1000U) { HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); t0 = t; }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    switch (HAL_TIM_GetActiveChannel(&htim2))
    {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        ir_tim_interrupt();
        break;
      default:
        break;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    if (!uart_line_ready) {
      if (uart_rx_ch == '\r' || uart_rx_ch == '\n') {
        if (uart_line_len > 0) {
          uart_line[uart_line_len] = '\0';
          uart_line_ready = 1;
          uart_line_len = 0;
        }
      } else if (uart_line_len < (UART_RX_BUF_SZ - 1)) {
        uart_line[uart_line_len++] = (char)uart_rx_ch;
      } else {
        uart_line_len = 0; // overflow -> czyść
      }
    }
    HAL_UART_Receive_IT(&huart2, &uart_rx_ch, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    (void)HAL_UART_Receive_IT(&huart2, &uart_rx_ch, 1);
  }
}

// Komendy UART:
//- MODE=MANUAL
//   - MODE=AUTO
//   - AUTO_DELAY=<ms>
//   - GET

static void process_uart_line(void)
{
  if (!uart_line_ready) return;
  uart_line_ready = 0;

  if (strncmp(uart_line, "MODE=", 5) == 0) {
    const char *p = uart_line + 5;
    if (strcmp(p, "MANUAL") == 0) {
      gate_mode = MODE_MANUAL;
      auto_ready = 0;
      printf("OK MODE=MANUAL\r\n");
    } else if (strcmp(p, "AUTO") == 0) {
      gate_mode = MODE_AUTO;
      if (LS_Open_ON()) { auto_ready = 1; auto_t0 = HAL_GetTick(); }
      else              { auto_ready = 0; }
      printf("OK MODE=AUTO\r\n");
    } else {
      printf("ERR MODE (use MANUAL|AUTO)\r\n");
    }
  }
  else if (strncmp(uart_line, "DELAY=", 11) == 0) {
    const char *p = uart_line + 11;
    unsigned long ms = strtoul(p, NULL, 10);
    if (ms >= AUTO_DELAY_MIN_MS && ms <= AUTO_DELAY_MAX_MS) {
      auto_close_delay_ms = (uint32_t)ms;
      printf("OK AUTO MODE DELAY=%lu ms\r\n", ms);
    } else {
      printf("ERR range %u..%u ms\r\n",
             (unsigned)AUTO_DELAY_MIN_MS, (unsigned)AUTO_DELAY_MAX_MS);
    }
  }
  else if (strcmp(uart_line, "GET") == 0) {
    printf("MODE=%s, AUTO_DELAY=%lu ms\r\n",
           (gate_mode == MODE_AUTO ? "AUTO" : "MANUAL"),
           (unsigned long)auto_close_delay_ms);
  }
  else {
    printf("ERR unknown cmd\r\n");
  }
}



uint32_t read_distance(void)
{
  uint32_t a = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
  uint32_t b = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
  return b - a;
}

uint32_t build_key(int pilot_code, uint8_t GateClosed, uint8_t GateOpened)
{
uint32_t key = 0;
  if (pilot_code == IR_CODE_ONOFF) key |= B_ONOFF;
  if (GateClosed)                  key |= B_CLS;
  if (GateOpened)                  key |= B_OPN;
  return key;
}

event_t poll_event(uint8_t GateClosed, uint8_t GateOpened, int pilot_code, uint32_t key, uint32_t dist_sensor, uint32_t now)
{
   // Typy Eventów i priorytetyzacja

  // 1) Przeszkoda ma najwyższy priorytet w trakcie zamykania
  if (motor_state == CLOSING_GATE && dist_sensor < OBSTACLE_THRESH)
    return EV_OBSTACLE;

  // 2) MENU z debounce
  if (pilot_code == IR_CODE_MENU) {
    if ((now - last_menu_toggle_ms) >= MENU_TOGGLE_DEBOUNCE_MS) {
      last_menu_toggle_ms = now;
      return EV_MENU_TOGGLE;
    }
  }

  // 3) Krańcówki podczas ruchu
  switch (motor_state) {
    case OPENING_GATE:
      if (GateOpened) return EV_REACHED_OPEN;
      break;
    case CLOSING_GATE:
      if (GateClosed) return EV_REACHED_CLOSED;
      break;
    default: break;
  }

  // 4) Start z pilota w IDLE
  if (motor_state == IDLE) {
    switch (key) {
      case (B_ONOFF | B_CLS): return EV_IR_START_OPEN;
      case (B_ONOFF | B_OPN): return EV_IR_START_CLOSE;
      default: break; // B_ONOFF|B_CLS|B_OPN -> ignoruj
    }
  }

  // 5) Timeouty
  if (motor_state == OBSTACLE_PAUSE) {
    if ((now - obstacle_t0) >= OBSTACLE_PAUSE_MS) return EV_PAUSE_TIMEOUT;
  }

  if (gate_mode == MODE_AUTO &&
      motor_state == IDLE &&
      GateOpened && auto_ready)
  {
    if ((now - auto_t0) >= auto_close_delay_ms) return EV_AUTO_TIMEOUT;
  }

  return EV_NONE;
}


void handle_event(event_t ev, uint8_t GateOpened)
{
   // Reakcje na Eventy
  switch (ev) {
    case EV_MENU_TOGGLE:
      gate_mode = (gate_mode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
      if (gate_mode == MODE_AUTO) {
        if (GateOpened) { auto_ready = 1; auto_t0 = HAL_GetTick(); }
        else            { auto_ready = 0; }
        printf("Mode=AUTO\r\n");
      } else {
        auto_ready = 0;
        printf("Mode=MANUAL\r\n");
      }
      break;

    case EV_IR_START_OPEN:
      StepMotor_Run(STEPNUM, DIR_RIGHT, STEPSIZE);
      motor_state = OPENING_GATE;
      printf("Opening...\r\n");
      break;

    case EV_IR_START_CLOSE:
      StepMotor_Run(STEPNUM, DIR_LEFT, STEPSIZE);
      motor_state = CLOSING_GATE;
      printf("Closing...\r\n");
      break;

    case EV_REACHED_OPEN:
      StepMotor_Stop();
      motor_state = IDLE;
      printf("Gate fully open.\r\n");
      if (gate_mode == MODE_AUTO) { auto_ready = 1; auto_t0 = HAL_GetTick(); }
      else { auto_ready = 0; }
      break;

    case EV_REACHED_CLOSED:
      StepMotor_Stop();
      motor_state = IDLE;
      printf("Gate fully closed.\r\n");
      auto_ready = 0;
      break;

    case EV_OBSTACLE:
      StepMotor_Stop();
      obstacle_t0 = HAL_GetTick();
      motor_state = OBSTACLE_PAUSE;
      printf("Warning: obstacle, pause %u ms\r\n", (unsigned)OBSTACLE_PAUSE_MS);
      break;

    case EV_PAUSE_TIMEOUT:
      StepMotor_Run(STEPNUM, DIR_RIGHT, STEPSIZE);
      motor_state = OPENING_GATE;
      printf("Reversing: opening after obstacle.\r\n");
      break;

    case EV_AUTO_TIMEOUT:
      StepMotor_Run(STEPNUM, DIR_LEFT, STEPSIZE);
      motor_state = CLOSING_GATE;
      auto_ready = 0;
      printf("AUTO: closing after %lu ms\r\n", (unsigned long)auto_close_delay_ms);
      break;

    case EV_NONE:
    default:
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);  // Start Timer 1
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart2, &uart_rx_ch, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  printf("System ready. Mode=MANUAL\r\n");

  while (1)
    {
	  	  process_uart_line();
	      BlinkAliveLED();
	      StepMotor_Handler();

	      uint32_t now         = HAL_GetTick();
	      uint32_t dist_sensor = read_distance();
	      uint8_t  GateClosed  = LS_Closed_ON();
	      uint8_t  GateOpened  = LS_Open_ON();
	      int      pilot_code  = ir_read();

	      uint32_t key = build_key(pilot_code, GateClosed, GateOpened);
	      event_t  ev  = poll_event(GateClosed, GateOpened, pilot_code, key, dist_sensor, now);
	      handle_event(ev, GateOpened);
    }
  }
	/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
