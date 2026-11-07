/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Servo angle control over UART (STM32F103C8T6)
  *
  *   Servo 1 signal -> PB6 (TIM4 CH1)
  *   Servo 2 signal -> PB7 (TIM4 CH2)
  *   UART           -> PA9 (TX), PA10 (RX), 115200 8N1
  *
  * Type a command in the terminal and press Enter:
  *
  *   1 90        servo 1 to 90 degrees
  *   2 45        servo 2 to 45 degrees
  *   b 90 45     both servos (servo 1 = 90, servo 2 = 45)
  *   c           center both servos (90 degrees)
  *   ?           show help
  *
  * Angle range: 0 to 180.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Pulse limits in microseconds (1 timer tick = 1 us) */
#define SERVO_MIN_US     1000     /* 0 degrees   */
#define SERVO_MAX_US     2000     /* 180 degrees */

#define CMD_BUFFER_SIZE  32

/* Onboard LED: PC13, active LOW */
#define LED_PORT         GPIOC
#define LED_PIN          GPIO_PIN_13

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim4;

/* USER CODE BEGIN PV */
static int servo1_angle = 90;
static int servo2_angle = 90;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void UART_Print(const char *text);
static void UART_Printf(const char *format, ...);
static void Print_Help(void);
static uint16_t Angle_To_Pulse(int angle);
static void Servo1_SetAngle(int angle);
static void Servo2_SetAngle(int angle);
static void Process_Command(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART_Print(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text,
                      (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void UART_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    UART_Print(buffer);
}

static void Print_Help(void)
{
    UART_Print("\r\nCommands (press Enter after each):\r\n");
    UART_Print("  1 <angle>         servo 1, e.g.  1 90\r\n");
    UART_Print("  2 <angle>         servo 2, e.g.  2 45\r\n");
    UART_Print("  b <a1> <a2>       both servos, e.g.  b 90 45\r\n");
    UART_Print("  c                 center both (90 degrees)\r\n");
    UART_Print("  ?                 show this help\r\n");
    UART_Print("Angle range: 0 to 180\r\n\r\n");
}

/* Convert angle 0..180 to pulse width in microseconds */
static uint16_t Angle_To_Pulse(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    return (uint16_t)(SERVO_MIN_US +
           ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180);
}

static void Servo1_SetAngle(int angle)
{
    servo1_angle = angle;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Angle_To_Pulse(angle));
}

static void Servo2_SetAngle(int angle)
{
    servo2_angle = angle;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Angle_To_Pulse(angle));
}

static void Report_Positions(void)
{
    UART_Printf("OK -> Servo1 = %d deg (%u us) | Servo2 = %d deg (%u us)\r\n",
                servo1_angle, Angle_To_Pulse(servo1_angle),
                servo2_angle, Angle_To_Pulse(servo2_angle));
}

/* Parse and execute one command line */
static void Process_Command(char *line)
{
    int servo, a1, a2;
    char *p;

    /* Allow ':' and ',' as separators too (e.g. "1:90", "b 90,45") */
    for (p = line; *p; p++)
    {
        if (*p == ':' || *p == ',')
            *p = ' ';
    }

    /* Skip leading spaces */
    while (*line == ' ')
        line++;

    if (*line == '\0')
        return;

    /* ---- help ---- */
    if (*line == '?' || tolower((unsigned char)*line) == 'h')
    {
        Print_Help();
        return;
    }

    /* ---- center ---- */
    if (tolower((unsigned char)*line) == 'c')
    {
        Servo1_SetAngle(90);
        Servo2_SetAngle(90);
        Report_Positions();
        return;
    }

    /* ---- both servos: b <a1> <a2> ---- */
    if (tolower((unsigned char)*line) == 'b')
    {
        if (sscanf(line + 1, "%d %d", &a1, &a2) == 2)
        {
            if (a1 < 0 || a1 > 180 || a2 < 0 || a2 > 180)
            {
                UART_Print("ERROR: angle must be 0 to 180\r\n");
                return;
            }

            Servo1_SetAngle(a1);
            Servo2_SetAngle(a2);
            Report_Positions();
        }
        else
        {
            UART_Print("ERROR: use  b <angle1> <angle2>\r\n");
        }
        return;
    }

    /* ---- single servo: <1|2> <angle> ---- */
    if (sscanf(line, "%d %d", &servo, &a1) == 2)
    {
        if (servo != 1 && servo != 2)
        {
            UART_Print("ERROR: servo must be 1 or 2\r\n");
            return;
        }

        if (a1 < 0 || a1 > 180)
        {
            UART_Print("ERROR: angle must be 0 to 180\r\n");
            return;
        }

        if (servo == 1)
            Servo1_SetAngle(a1);
        else
            Servo2_SetAngle(a1);

        Report_Positions();
        return;
    }

    UART_Print("ERROR: unknown command. Type ? for help\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char    cmd[CMD_BUFFER_SIZE];
  uint8_t cmdIndex = 0;
  uint8_t ch;
  /* USER CODE END 1 */

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   /* LED off */

  UART_Print("\r\n========================================\r\n");
  UART_Print("   STM32 SERVO CONTROL OVER UART\r\n");
  UART_Print("========================================\r\n");
  UART_Print("Servo 1: PB6   Servo 2: PB7\r\n");

  /* Start at center */
  Servo1_SetAngle(90);
  Servo2_SetAngle(90);
  Report_Positions();

  Print_Help();
  UART_Print("> ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Wait up to 10 ms for one character from the terminal */
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK)
    {
        if (ch == '\r' || ch == '\n')
        {
            UART_Print("\r\n");

            if (cmdIndex > 0)
            {
                cmd[cmdIndex] = '\0';
                cmdIndex = 0;

                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                Process_Command(cmd);
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

                UART_Print("> ");
            }
        }
        else if (ch == 8 || ch == 127)          /* backspace */
        {
            if (cmdIndex > 0)
            {
                cmdIndex--;
                UART_Print("\b \b");
            }
        }
        else if (cmdIndex < (CMD_BUFFER_SIZE - 1))
        {
            cmd[cmdIndex++] = (char)ch;

            /* Echo the typed character back */
            HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
        }
    }

    /* Clear a UART overrun error if one happened, so reception keeps working */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (HSI/2 x16 = 64 MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 32 MHz -> TIM4 clock 64 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization (PA9 TX, PA10 RX, 115200 8N1)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization: 50 Hz PWM on CH1 (PB6) and CH2 (PB7)
  *
  * 64 MHz / (63 + 1) = 1 MHz  -> 1 tick = 1 us
  * Period 19999 -> 20 ms -> 50 Hz
  * Compare value = pulse width in microseconds
  */
static void MX_TIM4_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;                       /* start at center */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization
  *
  * PB6 = Servo 1 (TIM4 CH1), PB7 = Servo 2 (TIM4 CH2), PC13 = LED
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* LED off */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Servo PWM pins */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for (volatile uint32_t i = 0; i < 500000; i++)
    {
    }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
