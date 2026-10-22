/*
 * main.c - demo testbed firmware for the Hiwonder ROS Robot Controller v1.2
 * (STM32F407VET6). Written CubeMX-style with the ST HAL, so it reads like a
 * CubeIDE project. It is the firmware the simulator is tested against, and a
 * working reference for the real board.
 *
 *   Clock    16 MHz HSE -> PLL (M=8 N=168 P=2) -> 168 MHz, APB1 42 MHz, APB2 84 MHz
 *   ESC1..4  PA11 TIM1_CH4, PB4 TIM3_CH1, PC8 TIM8_CH3, PC9 TIM8_CH4, 400 Hz, CCR = us
 *   Joints   TIM5 encoder (PA0/PA1) = joint 0, TIM2 encoder (PA15/PB3) = joint 1
 *   IMU      QMI8658 on I2C2 (PB10/PB11), 400 kHz
 *   Link     USART3 (PD8/PD9) 1 Mbaud, DMA1 Stream3 TX / Stream1 RX circular,
 *            telemetry protocol from telemetry/ (same as tbbridge.py)
 *   Debug    USART1 (PA9/PA10) 115200, plain text
 *   Tick     TIM6 1 kHz -> tlm_tick()
 *   Misc     LED PE10 (active low), KEY1 PE1 (kill switch, pressed = low),
 *            battery ADC1_IN8 on PB0 (x11 divider), IWDG ~250 ms
 */
#include "stm32f4xx_hal.h"
#include "telemetry.h"
#include "qmi8658.h"
#include <string.h>

UART_HandleTypeDef huart1, huart3;
DMA_HandleTypeDef hdma_usart3_rx, hdma_usart3_tx;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1, htim2, htim3, htim5, htim6, htim8;
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;

static volatile uint8_t g_tick_1khz;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_Init(void);
static void MX_USART3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM_PWM_Init(void);
static void MX_TIM_Encoder_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
void Error_Handler(void);

/* ---- tiny debug print on USART1 (no printf, keeps the image small) ---- */
void dbg(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 50);
}

static void dbg_num(const char *label, int32_t v, const char *unit)
{
    char b[16];
    int i = 15, neg = v < 0;
    uint32_t u = neg ? (uint32_t)(-v) : (uint32_t)v;
    b[i] = 0;
    do { b[--i] = (char)('0' + u % 10); u /= 10; } while (u);
    if (neg) b[--i] = '-';
    dbg(label); dbg(&b[i]); dbg(unit);
}

/* Battery voltage in millivolts from PB0 (ADC1_IN8, 100k/10k divider). */
static uint32_t battery_mv(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) return 0;
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    return raw * 3300u * 11u / 4095u;
}

/* defined in tlm_port_f407.c */
void tlm_port_init(void);
void tlm_port_service(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_Init();
    MX_USART3_Init();
    MX_I2C2_Init();
    MX_TIM_PWM_Init();
    MX_TIM_Encoder_Init();
    MX_TIM6_Init();
    MX_ADC1_Init();

    dbg("\r\nRRC v1.2 testbed demo, SYSCLK ");
    dbg_num("", (int32_t)(HAL_RCC_GetSysClockFreq() / 1000000u), " MHz\r\n");

    tlm_port_init();             /* ESCs at 1000 us, IMU init + gyro calibration */
    MX_IWDG_Init();              /* after the slow IMU calibration */
    tlm_init(&huart3);
    HAL_TIM_Base_Start_IT(&htim6);

    uint32_t last_report = 0;
    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);
        tlm_port_service();
        tlm_poll();
        if (g_tick_1khz) {
            g_tick_1khz = 0;
            tlm_tick();
        }
        if (HAL_GetTick() - last_report >= 1000u) {
            last_report = HAL_GetTick();
            dbg_num("t=", (int32_t)(last_report / 1000u), " s");
            dbg_num(" bat=", (int32_t)battery_mv(), " mV");
            dbg_num(" enc0=", (int32_t)__HAL_TIM_GET_COUNTER(&htim5), "");
            dbg_num(" enc1=", (int32_t)__HAL_TIM_GET_COUNTER(&htim2), "");
            dbg(tlm_is_armed() ? " ARMED\r\n" : " disarmed\r\n");
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    o.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    o.HSEState = RCC_HSE_ON;
    o.LSIState = RCC_LSI_ON;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 8;
    o.PLL.PLLN = 168;
    o.PLL.PLLP = RCC_PLLP_DIV2;
    o.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);   /* LED off (active low) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  /* buzzer off */

    g.Pin = GPIO_PIN_10;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;                        /* KEY2, KEY1: external pull-ups */
    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOE, &g);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

static void MX_USART1_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART3_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init = huart1.Init;
    huart3.Init.BaudRate = 1000000;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (h->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (h->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        g.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &g);

        hdma_usart3_rx.Instance = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(h, hdmarx, hdma_usart3_rx);

        hdma_usart3_tx.Instance = DMA1_Stream3;
        hdma_usart3_tx.Init = hdma_usart3_rx.Init;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(h, hdmatx, hdma_usart3_tx);

        HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    if (h->Instance != I2C2) return;
    g.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C2_CLK_ENABLE();
}

/* 400 Hz PWM with 1 us ticks: timer clock / (PSC+1) = 1 MHz, ARR+1 = 2500. */
static void pwm_init(TIM_HandleTypeDef *h, TIM_TypeDef *inst, uint32_t psc)
{
    h->Instance = inst;
    h->Init.Prescaler = psc;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = 2499;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.RepetitionCounter = 0;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(h) != HAL_OK) Error_Handler();
}

static void pwm_channel(TIM_HandleTypeDef *h, uint32_t ch)
{
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1000;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, ch) != HAL_OK) Error_Handler();
}

static void MX_TIM_PWM_Init(void)
{
    pwm_init(&htim1, TIM1, 167);   /* APB2 timers 168 MHz */
    pwm_channel(&htim1, TIM_CHANNEL_4);
    pwm_init(&htim3, TIM3, 83);    /* APB1 timers 84 MHz */
    pwm_channel(&htim3, TIM_CHANNEL_1);
    pwm_init(&htim8, TIM8, 167);
    pwm_channel(&htim8, TIM_CHANNEL_3);
    pwm_channel(&htim8, TIM_CHANNEL_4);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    if (h->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        g.Pin = GPIO_PIN_11;
        g.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (h->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        g.Pin = GPIO_PIN_4;
        g.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &g);
    } else if (h->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        g.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &g);
    }
}

static void enc_init(TIM_HandleTypeDef *h, TIM_TypeDef *inst)
{
    TIM_Encoder_InitTypeDef e = {0};
    h->Instance = inst;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = 0xFFFFFFFF;          /* 32-bit timers: no wrap handling needed */
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    e.EncoderMode = TIM_ENCODERMODE_TI12; /* x4: 720 CPR -> 2880 counts/rev */
    e.IC1Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = 5;
    e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter = 5;
    if (HAL_TIM_Encoder_Init(h, &e) != HAL_OK) Error_Handler();
    HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL);
}

static void MX_TIM_Encoder_Init(void)
{
    enc_init(&htim5, TIM5);
    enc_init(&htim2, TIM2);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    if (h->Instance == TIM5) {
        __HAL_RCC_TIM5_CLK_ENABLE();
        g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        g.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &g);
    } else if (h->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        g.Pin = GPIO_PIN_15;
        g.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

/* 84 MHz / 84 = 1 MHz, / 1000 = 1 kHz. */
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 999;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) g_tick_1khz = 1;
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    ch.Channel = ADC_CHANNEL_8;
    ch.Rank = 1;
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    if (h->Instance != ADC1) return;
    __HAL_RCC_ADC1_CLK_ENABLE();
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);
}

/* LSI 32 kHz / 32 = 1 kHz, reload 250 -> ~250 ms. */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = 250;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {}   /* the watchdog (if started) resets us; ESC outputs keep their last pulse */
}

/* ---- interrupt handlers (normally in stm32f4xx_it.c) ---- */
void SysTick_Handler(void)          { HAL_IncTick(); }
void USART3_IRQHandler(void)        { HAL_UART_IRQHandler(&huart3); }
void DMA1_Stream1_IRQHandler(void)  { HAL_DMA_IRQHandler(&hdma_usart3_rx); }
void DMA1_Stream3_IRQHandler(void)  { HAL_DMA_IRQHandler(&hdma_usart3_tx); }
void TIM6_DAC_IRQHandler(void)      { HAL_TIM_IRQHandler(&htim6); }
void HardFault_Handler(void)        { for (;;) {} }
