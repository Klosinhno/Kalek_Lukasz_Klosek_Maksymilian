/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PI + BH1750 + ADC (Potencjometr) - BEZ LCD
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* ===== INCLUDES ===== */
#include "main.h"
#include "adc.h"       // Biblioteka ADC (Potencjometr)
#include "i2c.h"       // I2C1 (BH1750)
#include "tim.h"       // PWM
#include "usart.h"     // UART
#include "gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ===== DEFINICJE I STRUKTURY ===== */

#define BH1750_ADDRESS_L (0x23 << 1)

// Makro do przeliczania % na wartość rejestru timera
#define __DUTY_TO_COMPARE(htim,duty) \
 ((duty * (__HAL_TIM_GET_AUTORELOAD(htim)+1))/100)

typedef struct {
    float Kp;
    float Ki;
    float Setpoint;
    float Integral;
    float OutMin;
    float OutMax;
} PI_HandleTypeDef;

typedef struct {
    I2C_HandleTypeDef* I2C;
    uint8_t Address;
    float Readout;
} BH1750_HandleTypeDef;

typedef struct {
    TIM_HandleTypeDef* Timer;
    uint32_t Channel;
    float Duty;
} PWM_HandleTypeDef;

typedef struct {
    PWM_HandleTypeDef Output;
    int ActiveState;
} LED_PWM_HandleTypeDef;

/* ===== ZMIENNE GLOBALNE ===== */

// PID NIENASTROJONY (0.1 / 0.1)
PI_HandleTypeDef hpi = {
  .Kp = 0.1f,
  .Ki = 0.1f,
  .Setpoint = 100.0f, // Startowa wartosc zadana
  .Integral = 0.0f,
  .OutMin = 0.0f,
  .OutMax = 100.0f
};

BH1750_HandleTypeDef hbh1750 = { &hi2c1, BH1750_ADDRESS_L, 0.0f };
LED_PWM_HandleTypeDef hled = { { &htim4, TIM_CHANNEL_1, 0.0f }, 1 };

float Lux = 0.0f;
char UART_Cmd[] = "X000";
uint8_t UART_Len;
int Mode = 1; // 1 = Tryb PI, 0 = Manual

/* ===== PROTOTYPY FUNKCJI ===== */
void SystemClock_Config(void);

/* ===== IMPLEMENTACJA FUNKCJI POMOCNICZYCH ===== */

// Przekierowanie printf na UART
int _write(int file, char* ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Obsługa BH1750
void BH1750_Init(BH1750_HandleTypeDef* h)
{
    uint8_t cmd = 0x01; // Power On
    HAL_I2C_Master_Transmit(h->I2C, h->Address, &cmd, 1, 10);
    cmd = 0x10; // High Res Mode
    HAL_I2C_Master_Transmit(h->I2C, h->Address, &cmd, 1, 10);
}

float BH1750_ReadLux(BH1750_HandleTypeDef* h)
{
    uint8_t buf[2];
    if (HAL_I2C_Master_Receive(h->I2C, h->Address, buf, 2, 10) == HAL_OK)
        h->Readout = ((buf[0] << 8) | buf[1]) / 1.2f;
    return h->Readout;
}

// Obsługa PWM LED
void LED_PWM_Write(LED_PWM_HandleTypeDef* h, float duty)
{
    h->Output.Duty = duty;
    uint32_t cmp = __DUTY_TO_COMPARE(h->Output.Timer, duty);
    __HAL_TIM_SET_COMPARE(h->Output.Timer, h->Output.Channel, cmp);
}

// Algorytm PI
float PI_Compute(PI_HandleTypeDef* pi, float meas, float dt)
{
    float error = pi->Setpoint - meas;
    pi->Integral += error * dt;

    // Anti-windup dla małego Ki
    if (pi->Ki > 0.0001f) {
        float max_integral = pi->OutMax / pi->Ki;
        if (pi->Integral > max_integral) pi->Integral = max_integral;
        else if (pi->Integral < -max_integral) pi->Integral = -max_integral;
    }

    // Zabezpieczenie ujemne całki
    if (pi->Integral < 0.0f) pi->Integral = 0.0f;

    float u = (pi->Kp * error) + (pi->Ki * pi->Integral);

    if (u > pi->OutMax) u = pi->OutMax;
    if (u < pi->OutMin) u = pi->OutMin;

    return u;
}

// Obsługa przerwania UART (komendy z terminala)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart3)
    {
        int val = atoi(&UART_Cmd[1]);
        if (UART_Cmd[0] == 'D') { // Direct PWM
            Mode = 0;
            LED_PWM_Write(&hled, (float)val);
        }
        else if (UART_Cmd[0] == 'S') { // Setpoint
            Mode = 1;
            hpi.Setpoint = (float)val;
        }
        HAL_UART_Receive_IT(&huart3, (uint8_t*)UART_Cmd, UART_Len);
    }
}

/* ===== MAIN ===== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    // MX_I2C2_Init(); <-- USUNIĘTE (LCD)
    MX_USART3_UART_Init();
    MX_TIM4_Init();
    MX_ADC1_Init(); // <-- POTENCJOMETR JEST

    setvbuf(stdout, NULL, _IONBF, 0);

    BH1750_Init(&hbh1750);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    UART_Len = strlen(UART_Cmd);
    HAL_UART_Receive_IT(&huart3, (uint8_t*)UART_Cmd, UART_Len);

    /* Zmienne do kontroli czasu */
    uint32_t LastControlTick = 0;
    uint32_t LastDisplayTick = 0;

    /* Stałe czasowe */
    const uint32_t ControlPeriod = 100; // Regulacja co 100ms
    const uint32_t DisplayPeriod = 250; // UART co 250ms

    while (1)
    {
        uint32_t CurrentTick = HAL_GetTick();

        /* === ZADANIE 1: REGULACJA (Priorytet czasu) === */
        if (CurrentTick - LastControlTick >= ControlPeriod)
        {
            LastControlTick = CurrentTick;

            // 1. Odczyt ADC (Zadajnik - potencjometr)
            // Twoja oryginalna logika: Potencjometr zmienia Setpoint +/-
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
            {
                uint32_t raw = HAL_ADC_GetValue(&hadc1);
                float voltage = (raw * 3.3f) / 4095.0f;

                if (voltage > 3.0f) hpi.Setpoint += 0.5f;       // Zwiększaj cel
                else if (voltage < 1.0f) hpi.Setpoint -= 0.5f;  // Zmniejszaj cel

                // Limity wartości zadanej
                if (hpi.Setpoint > 600.0f) hpi.Setpoint = 600.0f;
                if (hpi.Setpoint < 20.0f)  hpi.Setpoint = 20.0f;
            }
            HAL_ADC_Stop(&hadc1);

            // 2. Odczyt Czujnika
            Lux = BH1750_ReadLux(&hbh1750);

            // 3. Obliczenia PI
            if (Mode == 1)
            {
                float u = PI_Compute(&hpi, Lux, (float)ControlPeriod / 1000.0f);
                LED_PWM_Write(&hled, u);
            }
        }

        /* === ZADANIE 2: INTERFEJS (Tylko UART) === */
        if (CurrentTick - LastDisplayTick >= DisplayPeriod)
        {
            LastDisplayTick = CurrentTick;

            // LCD usunięte - zostaje tylko logowanie UART
            printf("Set: %.1f | Lux: %.1f | PWM: %.1f%%\r\n", hpi.Setpoint, Lux, hled.Output.Duty);
        }
    }
}

/* ===== KONFIGURACJA ZEGARÓW (Domyślna z CubeMX) ===== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
}
#endif