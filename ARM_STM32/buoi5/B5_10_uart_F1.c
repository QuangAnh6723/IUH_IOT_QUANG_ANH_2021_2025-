/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  FALSE = 0,
  TRUE = 1
} bool_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// khai bao bien dung cho uart
uint8_t u_receive = 0;
uint8_t data_get[25] = {0};
uint8_t uart_flag = 0;
uint8_t u_cnt = 0;
uint8_t temp = 0; // dung cho debug
void write_to_lcd_4bit(uint8_t rs, uint8_t data)
{
  uint8_t i;
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);
  HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);

  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data >> 3) & 0x01));
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data >> 2) & 0x01));
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data >> 1) & 0x01));
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data >> 0) & 0x01));
  for (i = 0; i < 72; i++)
  {
    asm("NOP"); // delay khoảng 1-2us
  }

  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
  HAL_Delay(2);
}

void lcd_send_command(uint8_t data)
{ // vì dùng 4bit nên phải gửi 2 lần, nếu dùng 8bit thì chỉ cần gửi 1 lần
  write_To_LCD_4bits(0, (data >> 4) & 0x0f);
  write_To_LCD_4bits(0, (data >> 0) & 0x0f);
  HAL_Delay(2);
}

void lcd_display_char(uint8_t data)
{ // hàm viết được 1 kí tự
  write_To_LCD_4bits(1, (data >> 4) & 0x0f);
  write_To_LCD_4bits(1, (data >> 0) & 0x0f);
  HAL_Delay(2);
}
void lcd_display_string(uint8_t *data, uint8_t len)
{
  for (uint8_t j = 0; j < len; j++)
  {
    lcd_Send_Data(data[j]);
  }
}

void lcd_goto_xy(uint8_t row, uint8_t col)
{ 
  uint8_t temp;
  switch (row)
  {
  case 0:
    temp = col | 0x80;
    break;
  case 1:
    temp = col | 0xc0;
    break;
  default:
    temp = col | 0x80;
  }
  lcd_send_command(temp);
}

void lcd_clear()
{
  send_Command(0x01);
  HAL_Delay(20);
}

void lcd_init(void)
{

  send_Command(0x28); // gui che do 4 bit 2line
  HAL_Delay(50);
  send_Command(0x28); // gui che do 4 bit 2line
  HAL_Delay(50);
  send_Command(0x0C); // hien thi lcd tat con tro
  HAL_Delay(1);
  send_Command(0x06); // che do tu tang con tro
  HAL_Delay(1);
  send_Command(0x01); // xoa man hinh
  HAL_Delay(20);
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  //
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  uart_flag = 1;
  data_get[u_cnt] = u_receive;
  u_cnt++;

  HAL_UART_Receive_IT(&huart1, &u_receive, 1);
  /* USER CODE END USART1_IRQn 1 */
}

void uart_send_ok()
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"OK\r\n", 4, 10);
}

void uart_send_error()
{
  HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR\r\n", 7, 10);
}

// void uart_send_test() // dung cho debug
//{
//	HAL_UART_Transmit(&huart1, (uint8_t *)"test\r\n", 6, 10);
// }

uint8_t demKiTu(uint8_t cnt)
{
  uint8_t cnt_data = 0;
  uint8_t i = 2;

  while (cnt--)
  {
    if (data_get[i++] == 0x23)
      break;
    cnt_data++;
  }
  return cnt_data;
}

void display_data_ok()
{
  lcd_clear();
  lcd_goto_xy(0, 0);
  for (int i = 0; i < data_get[1]; i++)
  {
    lcd_display_char(data_get[i + 2]);
    //		HAL_Delay(1);
  }
}

void reset_var()
{
  u_cnt = 0;
  uart_flag = 0;
  for (int i = 0; i < 25; i++)
  {
    data_get[i] = 0;
  }
}

bool_t isNotSharp(uint8_t data)
{
  if (data != 0x23 && data != 0)
  {
    return TRUE;
  }
  return FALSE;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  // lcd_gotoxy(1, 1);
  // lcd_display("ABCDDD", 6);
  HAL_UART_Receive_IT(&huart1, &u_receive, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (uart_flag == 1)
    {
      if (u_receive == 0x23) // kiem tra ki tu #
      {
        if (data_get[0] == 0x2a) // kiem tra ki tu *
        {
          if (demKiTu(data_get[1]) == data_get[1]) // kiem tra so luong ki tu
          {
            uart_send_ok();
            display_data_ok();
            reset_var();
          }
          else // khac so luong
          {
            uart_send_error();
            reset_var();
          }
        }
        else
        {
          uart_send_error(); // sai ki tu dau *
          reset_var();
        }
      }
      // data_get[1] là số lượng byte của data
      // vi dụ *4aABC# 
      // thì # là data_get[6]
      if (isNotSharp(data_get[data_get[1] + 2]))
      {
        uart_send_error();
        reset_var();
      }
    }
    //  		asm("nop");
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
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
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D7_Pin | D6_Pin | D5_Pin | D4_Pin | led0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RW_Pin | RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           led0_Pin */
  GPIO_InitStruct.Pin = D7_Pin | D6_Pin | D5_Pin | D4_Pin | led0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_Pin */
  GPIO_InitStruct.Pin = EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RW_Pin RS_Pin */
  GPIO_InitStruct.Pin = RW_Pin | RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
