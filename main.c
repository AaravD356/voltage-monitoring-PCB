/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Voltage monitor + CAN + fault detection (STM32L476 + MCP2562)
  * @note           : 6 isolated cell voltage readings, ADC1..ADC6:
  *                   - ADC1: PC4  (ADC1_IN14)  82k/10k
  *                   - ADC2: PC5  (ADC1_IN15)  68k/10k
  *                   - ADC3: PB0  (ADC1_IN8)   47k/10k
  *                   - ADC4: PB13 (ADC1_IN5)   33k/10k
  *                   - ADC5: PB14 (ADC1_IN6)   20k/10k
  *                   - ADC6: PB15 (ADC1_IN7)    6.8k/10k
  *
  *                   Relay JST connectors: PC10, PC11 (initialized, unused)
  *                   MCP2562 STBY: PB2 (LOW = normal)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdbool.h>

/* =================== CONFIG =================== */
// Firmware version
#define FW_MAJOR        1
#define FW_MINOR        0
#define FW_PATCH        0
#define FW_VERSION_U32  ((FW_MAJOR<<24)|(FW_MINOR<<16)|(FW_PATCH))

// ADC config
#define NUM_ADC_CH      6
#define ADC_SAMPLES     16
#define VREF_MV         3300
#define ADC_FS_COUNTS   4095.0f

// ADC scan order indices
enum {
  CH_ADC1 = 0, // PC4
  CH_ADC2,     // PC5
  CH_ADC3,     // PB0
  CH_ADC4,     // PB13
  CH_ADC5,     // PB14
  CH_ADC6      // PB15
};

// Divider gains (Rtop+Rbot)/Rbot
static const float k_divider_gain[NUM_ADC_CH] = {
  9.2f,   // ADC1: 82k/10k
  7.8f,   // ADC2: 68k/10k
  5.7f,   // ADC3: 47k/10k
  4.3f,   // ADC4: 33k/10k
  3.0f,   // ADC5: 20k/10k
  1.68f   // ADC6: 6.8k/10k
};

// Fault thresholds
#define UV_TRIP_MV        3000
#define UV_WARN_MV        3200
#define OV_WARN_MV        4200
#define OV_TRIP_MV        4250
#define IMBALANCE_WARN_MV 100
#define FAULT_DEBOUNCE_MS 500

// MCP2562 STBY pin
#define CAN_STBY_GPIO_Port     GPIOB
#define CAN_STBY_Pin           GPIO_PIN_2

// Relay JSTs (initialized, unused)
#define RELAY1_GPIO_Port       GPIOC
#define RELAY1_Pin             GPIO_PIN_10
#define RELAY2_GPIO_Port       GPIOC
#define RELAY2_Pin             GPIO_PIN_11

// CAN IDs
#define CAN_ID_HEARTBEAT   0x300
#define CAN_ID_VOLTAGES_1  0x301
#define CAN_ID_VOLTAGES_2  0x302
#define CAN_ID_FAULTS      0x303

/* =================== PRIVATE VARS =================== */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim2;

static volatile uint16_t adc_dma_buf[NUM_ADC_CH * ADC_SAMPLES];
static volatile uint16_t meas_mV[NUM_ADC_CH];

static volatile uint32_t uptime_ms = 0;
static uint32_t last_hb_ms = 0;
static uint32_t last_tx_ms = 0;
static uint32_t status_bits = 0;

/* =================== PROTOTYPES =================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);

static void start_adc_dma(void);
static void process_adc_window(void);
static void evaluate_faults(void);
static void can_send_std(uint16_t id, const uint8_t *data, uint8_t len);
static void tx_heartbeat(void);
static void tx_voltages(void);
static void tx_faults(void);

/* =================== MAIN =================== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();

  // Enable CAN transceiver
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_TIM_Base_Start_IT(&htim2);

  start_adc_dma();

  while (1)
  {
    uint32_t now = uptime_ms;

    if (now - last_hb_ms >= 500) {
      tx_heartbeat();
      last_hb_ms = now;
    }
    if (now - last_tx_ms >= 100) {
      tx_voltages();
      tx_faults();
      last_tx_ms = now;
    }
    HAL_Delay(5);
  }
}

/* =================== CALLBACKS =================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2) uptime_ms++;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1) {
    process_adc_window();
    evaluate_faults();
  }
}

/* =================== FUNCTIONS =================== */
static void start_adc_dma(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, NUM_ADC_CH * ADC_SAMPLES);
}

static void process_adc_window(void)
{
  for (int ch = 0; ch < NUM_ADC_CH; ch++) {
    uint32_t acc = 0;
    for (int s = 0; s < ADC_SAMPLES; s++)
      acc += adc_dma_buf[ch + s*NUM_ADC_CH];
    float avg_counts = (float)acc / ADC_SAMPLES;
    float mv_adc = (avg_counts * VREF_MV) / ADC_FS_COUNTS;
    float mv_node = mv_adc * k_divider_gain[ch];
    if (mv_node < 0) mv_node = 0;
    if (mv_node > 65535) mv_node = 65535;
    meas_mV[ch] = (uint16_t)mv_node;
  }
}

static void evaluate_faults(void)
{
  uint32_t faults = 0;
  uint16_t vmin = 0xFFFF, vmax = 0;

  for (int ch = 0; ch < NUM_ADC_CH; ch++) {
    uint16_t v = meas_mV[ch];
    if (v <= UV_TRIP_MV) faults |= (1u << ch);
    if (v >= OV_TRIP_MV) faults |= (1u << (8 + ch));
    vmin = (v < vmin) ? v : vmin;
    vmax = (v > vmax) ? v : vmax;
  }
  if ((uint16_t)(vmax - vmin) >= IMBALANCE_WARN_MV)
    faults |= (1u << 16);

  static uint32_t first_fault_ms = 0;
  if (faults) {
    if (!first_fault_ms) first_fault_ms = uptime_ms;
    if (uptime_ms - first_fault_ms >= FAULT_DEBOUNCE_MS)
      status_bits = faults;
  } else {
    first_fault_ms = 0;
    status_bits = 0;
  }
}

static void can_send_std(uint16_t id, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mbox;
  tx.StdId = id;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = len & 0x0F;
  HAL_CAN_AddTxMessage(&hcan, &tx, (uint8_t*)data, &mbox);
}

static void tx_heartbeat(void)
{
  uint8_t p[8] = {0};
  uint32_t up100ms = uptime_ms / 100;
  p[0]= (uint8_t)(up100ms);
  p[1]= (uint8_t)(up100ms>>8);
  p[2]= (uint8_t)(up100ms>>16);
  p[3]= (uint8_t)(up100ms>>24);
  p[4]= (uint8_t)(FW_VERSION_U32>>24);
  p[5]= (uint8_t)(FW_VERSION_U32>>16);
  p[6]= (uint8_t)(FW_VERSION_U32>>8);
  p[7]= (uint8_t)(FW_VERSION_U32);
  can_send_std(CAN_ID_HEARTBEAT, p, 8);
}

static void tx_voltages(void)
{
  uint8_t p1[8] = {
    (uint8_t)(meas_mV[CH_ADC1]>>8), (uint8_t)(meas_mV[CH_ADC1]),
    (uint8_t)(meas_mV[CH_ADC2]>>8), (uint8_t)(meas_mV[CH_ADC2]),
    (uint8_t)(meas_mV[CH_ADC3]>>8), (uint8_t)(meas_mV[CH_ADC3]),
    (uint8_t)(meas_mV[CH_ADC4]>>8), (uint8_t)(meas_mV[CH_ADC4])
  };
  can_send_std(CAN_ID_VOLTAGES_1, p1, 8);

  uint8_t p2[8] = {
    (uint8_t)(meas_mV[CH_ADC5]>>8), (uint8_t)(meas_mV[CH_ADC5]),
    (uint8_t)(meas_mV[CH_ADC6]>>8), (uint8_t)(meas_mV[CH_ADC6]),
    0,0,0,0
  };
  can_send_std(CAN_ID_VOLTAGES_2, p2, 8);
}

static void tx_faults(void)
{
  uint8_t p[8] = {0};
  uint32_t sb = status_bits;
  p[0]=(uint8_t)(sb);
  p[1]=(uint8_t)(sb>>8);
  p[2]=(uint8_t)(sb>>16);
  p[3]=(uint8_t)(sb>>24);
  can_send_std(CAN_ID_FAULTS, p, 8);
}

/* =================== CubeMX Init Stubs =================== */
/* Configure ADC ranks in CubeMX:
   Rank1: PC4  (ADC1_IN14)
   Rank2: PC5  (ADC1_IN15)
   Rank3: PB0  (ADC1_IN8)
   Rank4: PB13 (ADC1_IN5)
   Rank5: PB14 (ADC1_IN6)
   Rank6: PB15 (ADC1_IN7)
   DMA: Circular, half-word
*/
/* =================== CubeMX Init Stubs / Minimal implementations =================== */

/**
  * @brief System Clock Configuration
  * @note  Replace this function with the CubeMX-generated SystemClock_Config()
  *        so your clocks / flash latency / PLL settings match your project.
  */
void SystemClock_Config(void)
{
  /* IMPORTANT:
     Use CubeMX to generate this function for your target clock (MSI/HSI/HSE/PLL).
     Leaving a wrong clock configuration can prevent peripherals (TIM/CAN/ADC) from working.
  */
}

/**
  * @brief ADC1 Initialization
  * @note Configure ADC in scan mode, continuous, DMA circular.
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = NUM_ADC_CH;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure regular channels in the same order as the DMA buffer expects */
  /* Rank 1: PC4  -> ADC_CHANNEL_14 */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5; // long sample for high impedance dividers
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* Rank 2: PC5 -> ADC_CHANNEL_15 */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* Rank 3: PB0 -> ADC_CHANNEL_8 */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* Rank 4: PB13 -> ADC_CHANNEL_5 */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* Rank 5: PB14 -> ADC_CHANNEL_6 */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* Rank 6: PB15 -> ADC_CHANNEL_7 */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief CAN init function
  * @note Basic config + open filter (accept-all) for bringup.
  *       Adjust timing with CubeMX for your APB clock to hit your desired bitrate.
  */
static void MX_CAN_Init(void)
{
  CAN_FilterTypeDef sFilterConfig = {0};

  hcan.Instance = CAN; /* on some HAL versions this might be CAN1 */
  hcan.Init.Prescaler = 3; /* tune this in CubeMX for correct bitrate */
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure a catch-all filter so we receive nothing-specific (development) */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 init function
  * @note  Configure a 1 kHz tick based on SystemCoreClock.
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;

  /* Build prescaler so that timer clock = 1000 Hz,
     then set Period = 0 so update events occur every 1 ms. */
  if (SystemCoreClock < 1000) {
    /* Fallback if caller didn't configure clock yet */
    htim2.Init.Prescaler = 7999;
  } else {
    htim2.Init.Prescaler = (uint32_t)(SystemCoreClock / 1000U) - 1U;
  }
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  /* Enable TIM IRQ so HAL_TIM_PeriodElapsedCallback runs */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief DMA controller init
  * @note  Minimal DMA init for ADC circular transfers. Verify DMA channel in CubeMX.
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_adc1 on DMA1_Channel1 (verify in CubeMX) */
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Link DMA handle to ADC handle */
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  /* DMA IRQ init (optional) */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @note Initializes pins used in firmware: CAN STBY (PB2), Relay JST pins (PC10, PC11).
  *       CubeMX will also configure PA11/PA12 for CAN AF and pull settings.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure CAN STBY pin (PB2) as output and drive LOW (normal mode) */
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = CAN_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STBY_GPIO_Port, &GPIO_InitStruct);

  /* Relay JST outputs (PC10, PC11) as general outputs (unused by firmware now) */
  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = RELAY1_Pin | RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY1_GPIO_Port, &GPIO_InitStruct);

  /* Note: CAN TX/RX pins (PA12/PA11) alternate function config normally generated by CubeMX */
}

/* Error handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Blink LED or stay here */
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add reporting here */
}
#endif
