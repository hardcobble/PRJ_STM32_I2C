/**
  ******************************************************************************
  * @file    I2C/I2C_TwoBoards_ComPolling/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32H7xx I2C HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          Polling transfer.
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/********************************************************************************/
// V0.1: based on I2C_TwoBoards_ComPolling; Read Register 0x20 of ACS37800 every 1s;
// V0.2: Add HAL_UART_MspInit() and HAL_UART_MspDeInit() in stm32h7xx.hal.msp.c;
//       Enable #define HAL_USART_MODULE_ENABLED in stm32h7xx_hal_conf.h
//       Be able to send Vrms and Irms to PC through UART.
//       testserial_ACS37800.exe can show Vrms,Irms and save data to csv file.
// V0.3: Disable UART code; change I2C timeout from 10000ms to 500ms.
// V0.4: Support both read and write to ACS37800 EEPROM.
// V0.5: Support ACS37800 demo DC and AC current test and voltage test; support power board AC test.
// V0.6: support MAX31856(stm32h7xx_hal_msp.c,main.h,stm32h7xx_hal_conf.h;Add stm32h7xx_hal_spi.c);
//       CLKPhase = SPI_PHASE_2EDGE; Configure the CS pin;
//       Display hot junction temperature;
//       LED2 blinks if the thermocouple is open as power on; LED1 is on if if the thermocouple is open after power on.
// V0.7: #define ACS37800_CURRENT_16BIT_GAIN;
//      Set crs_sns to 0(gain=4); 16bit fractional (irms_float = ConvertUnsignedFixedPoint(irms, 16, 16))
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
//#define ACS37800_I2C_AVAILABLE //STM32H7 is master and ACS37800 is slave.
//#define MAX31856_SPI_AVAILABLE //STM32H7 is master and MAX31856 is slave.
#define UART_AVAILABLE //
//#define ACS37800_DEMO_BOARD
//#define ACS37800_DCTEST
#define ACS37800_CURRENT_16BIT_GAIN

#ifdef MAX31856_SPI_AVAILABLE
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
void MAX31856_CS_Init();
uint8_t MAX31856_ReadReg(uint8_t RegAddr);
void MAX31856_WriteReg(uint8_t RegAddr,uint8_t RegData);
void MAX31856_Get_AllReg(void);
void MAX31856_Init(void);
float MAX31856_GetTemp_DegreeC(void);
uint8_t MAX31856_Reg0x00_CR0_Val;
uint8_t MAX31856_Reg0x01_CR1_Val;
uint8_t MAX31856_Reg0x02_MASK_Val;
uint8_t MAX31856_Reg0x03_CJHF_Val;
uint8_t MAX31856_Reg0x04_CJLF_Val;
uint8_t MAX31856_Reg0x05_LTHFTH_Val;
uint8_t MAX31856_Reg0x06_LTHFTL_Val;
uint8_t MAX31856_Reg0x07_LTLFTH_Val;
uint8_t MAX31856_Reg0x08_LTLFTL_Val;
uint8_t MAX31856_Reg0x09_CJTO_Val;
uint8_t MAX31856_Reg0x0A_CJTH_Val;
uint8_t MAX31856_Reg0x0B_CJTL_Val;
uint8_t MAX31856_Reg0x0C_LTCBH_Val;
uint8_t MAX31856_Reg0x0D_LTCBM_Val;
uint8_t MAX31856_Reg0x0E_LTCBL_Val;
uint8_t MAX31856_Reg0x0F_SR_Val;

#define MAX31856_CR0_REG    (0x00)
#define MAX31856_CR1_REG    (0x01)
#define MAX31856_MASK_REG   (0x02)
#define MAX31856_CJHF_REG   (0x03)
#define MAX31856_CJLF_REG   (0x04)
#define MAX31856_LTHFTH_REG (0x05)
#define MAX31856_LTHFTL_REG (0x06)
#define MAX31856_LTLFTH_REG (0x07)
#define MAX31856_LTLFTL_REG (0x08)
#define MAX31856_CJTO_REG   (0x09)
#define MAX31856_CJTH_REG   (0x0A)
#define MAX31856_CJTL_REG   (0x0B)
#define MAX31856_LTCBH_REG  (0x0C)
#define MAX31856_LTCBM_REG  (0x0D)
#define MAX31856_LTCBL_REG  (0x0E)
#define MAX31856_SR_REG     (0x0F)

float ThermoCoupleTemperatureDegreeC;

static void EXTI15_10_IRQHandler_Config(void);
#endif

#ifdef ACS37800_I2C_AVAILABLE
//********************ACS37800****************************//
//For both demo board and power brd,  both DIO pins are pulled to VCC, then the internal slave address stored in EEPROM is used.
//By default,the value of i2c_slv_addr is programmed at the Allegro factory to 127
//Target device address: The device 7 bits address value in datasheet must be shifted to the left before calling HAL_I2C_Master_Transmit
#define I2C_ADDRESS        127 // ACS37800 default slave address=127
/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 100 MHz */
/* This example use TIMING to 0x00901954 to reach 400 kHz speed (Rise time = 100 ns, Fall time = 10 ns) */
//#define I2C_TIMING      0x00901954//500k
//#define I2C_TIMING      0x10901954//300k
//#define I2C_TIMING      0x30901954//193k
#define I2C_TIMING      0x70901954//100k
//#define I2C_TIMING      0x4030081F
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

uint32_t Reg0x0B_Val_set,Reg0x0C_Val_set,Reg0x0D_Val_set,Reg0x0E_Val_set,Reg0x0F_Val_set;
uint32_t Reg0x0B_qvo_fine_set,Reg0x0B_sns_fine_set,Reg0x0B_crs_sns_set,Reg0x0B_iavgselen_set,Reg0x0B_pavgselen_set;
uint32_t Reg0x0C_rms_avg_1_set,Reg0x0C_rms_avg_2_set,Reg0x0C_vchan_offset_code_set;
uint32_t Reg0x0D_ichan_del_en_set,Reg0x0D_chan_del_sel_set,Reg0x0D_fault_set,Reg0x0D_fltdly_set;
uint32_t Reg0x0E_vevent_cycs_set,Reg0x0E_overvreg_set,Reg0x0E_undervreg_set,Reg0x0E_delaycnt_sel_set,Reg0x0E_halfcycle_en_set;
uint32_t Reg0x0E_squarewave_en_set,Reg0x0E_zerocrosschansel_set,Reg0x0E_zerocrossedgesel_set;
uint32_t Reg0x0F_i2c_slv_addr_set,Reg0x0F_i2c_dis_slv_addr_set,Reg0x0F_dio_0_sel_set,Reg0x0F_dio_1_sel_set,Reg0x0F_n_set,Reg0x0F_bypass_n_en_set;


uint32_t Reg0x0B_Val,Reg0x0C_Val,Reg0x0D_Val,Reg0x0E_Val,Reg0x0F_Val;
uint32_t Reg0x0B_qvo_fine,Reg0x0B_sns_fine,Reg0x0B_crs_sns,Reg0x0B_iavgselen,Reg0x0B_pavgselen;
uint32_t Reg0x0C_rms_avg_1,Reg0x0C_rms_avg_2,Reg0x0C_vchan_offset_code;
uint32_t Reg0x0D_ichan_del_en,Reg0x0D_chan_del_sel,Reg0x0D_fault,Reg0x0D_fltdly;
uint32_t Reg0x0E_vevent_cycs,Reg0x0E_overvreg,Reg0x0E_undervreg,Reg0x0E_delaycnt_sel,Reg0x0E_halfcycle_en;
uint32_t Reg0x0E_squarewave_en,Reg0x0E_zerocrosschansel,Reg0x0E_zerocrossedgesel;
uint32_t Reg0x0F_i2c_slv_addr,Reg0x0F_i2c_dis_slv_addr,Reg0x0F_dio_0_sel,Reg0x0F_dio_1_sel,Reg0x0F_n,Reg0x0F_bypass_n_en;
uint32_t Reg0x20_Val;//irms and vrms

uint32_t irms;
float_t irms_float;
float_t current_rms_amp;
uint32_t vrms;
float_t vrms_float;
float_t  voltage_rms_mv;
float_t voltage_Line;

uint32_t Reg0x26_Val;//irmsavgonesec and vrmsavgonesec
uint32_t vrmsavgonesec;
uint32_t irmsavgonesec;
uint32_t Reg0x27_Val;//irmsavgonemin and vrmsavgonemin
uint32_t vrmsavgonemin;
uint32_t irmsavgonemin;
uint32_t Reg0x2A_Val;//icodes and vcodes
int16_t icodes;
uint16_t vcodes;

float_t irms_float_codes;
float_t current_rms_amp_codes;


/* Private function prototypes -----------------------------------------------*/
float_t ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
float_t ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width);
int32_t SignExtendBitfield(uint32_t data, uint16_t width);
uint32_t ACS37800_Read(uint8_t RegAddr);
uint32_t ACS37800_Write(uint8_t RegAddr,uint32_t RegData);
void ACS37800_GetEEPROM();
void ACS37800_SetEEPROM();
void ACS37800_SetEEPROM_Reg(uint8_t Reg,uint32_t Data);
/* Private functions ---------------------------------------------------------*/
#endif
//********************UART****************************//
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
uint8_t aTxBuffer[] = "test start!";
//uint8_t UART_TxBuffer[] = "ACS37800";
//uint8_t UART_TxBuffer[] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x0D,0x0A};
//uint8_t UART_TxBuffer[] = {0x55,0x55,0x31,0x32,0x33,0x34,0x0D,0x0A};
//uint8_t UART_TXBUFFER_SIZE = 8;//
uint8_t UART_TxBuffer[21];
uint8_t UART_TXBUFFER_SIZE = 21;//
//************************************************//
static void SystemClock_Config(void);
//static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void Timeout_Error_Handler(void);
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  static uint32_t m_sysclk,m_hclk,m_pclk1;
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32H7xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  SystemClock_Config();

  //SysClock Freq
  m_sysclk = HAL_RCC_GetSysClockFreq();
  m_hclk = HAL_RCC_GetHCLKFreq();
  m_pclk1 = HAL_RCC_GetPCLK1Freq();
  printf("sysclk = %lu\r\n", (unsigned long)m_sysclk);
  printf("hclk = %lu\r\n", (unsigned long)m_hclk);
  printf("pclk = %lu\r\n", (unsigned long)m_pclk1);

  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

#ifdef ACS37800_I2C_AVAILABLE

  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance             = I2Cx;
  I2cHandle.Init.Timing          = I2C_TIMING;
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
  //I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2     = 0xFF;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  
  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);

  /* Configure User push-button */
  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);

  /* Delay to avoid that possible signal rebound is taken as button release */
  HAL_Delay(200);

  ACS37800_SetEEPROM();
  HAL_Delay(500);//see error without this delay
  //if(TestData != ACS37800_Read(TestReg))
  //{
	//  Error_Handler();
  //}
  ACS37800_GetEEPROM();
#endif /* ACS37800_I2C_AVAILABLE */

#ifdef UART_AVAILABLE
  
  /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = None
        - BaudRate = 115200 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance        = USARTx;

    //UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.BaudRate     = 9600;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
    {
      Error_Handler();
    }
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
      Error_Handler();
    }
    BSP_LED_On(LED1);
    /*##-2- Start the transmission process #####################################*/
    /* While the UART in reception process, user can transmit data through
       "aTxBuffer" buffer */
    //if(HAL_UART_Transmit(&UartHandle, (uint8_t*)UART_TxBuffer, UART_TXBUFFER_SIZE, 5000)!= HAL_OK)
    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 5000)!= HAL_OK)
    {
      Error_Handler();
    }
    BSP_LED_Off(LED1);
#endif

#ifdef MAX31856_SPI_AVAILABLE
    /*##-1- Configure the SPI peripheral #######################################*/
    /* Set the SPI parameters */
    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;//
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    SpiHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommanded setting to avoid glitches */

    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
    //Enable CS pin clock.
    MAX31856_CS_Init();
    //MAX31856_Reg0x01_Val = MAX31856_Read(0x01);//
    MAX31856_Get_AllReg();
    //Initialize Max31856
    MAX31856_Init();
	//while(GPIO_PIN_RESET == HAL_GPIO_ReadPin(MAX31856_FAULT_GPIO_PORT,MAX31856_FAULT_PIN))
    while(MAX31856_ReadReg(MAX31856_SR_REG))
	{
		/* Toggle LED2 on */
		BSP_LED_On(LED2);
		HAL_Delay(500);
		BSP_LED_Off(LED2);
		HAL_Delay(500);
	}
    // Configure EXTI15_10 (connected to PC.13 pin) to interrupt CPU1
    EXTI15_10_IRQHandler_Config();
#endif

  /* Infinite loop */
  while (1)
  {
#ifdef ACS37800_I2C_AVAILABLE
	#ifdef ACS37800_DEMO_BOARD
	  Reg0x20_Val = ACS37800_Read(0x20);//
	  vrms = Reg0x20_Val&0x0000FFFF;
	  vrms_float = ConvertUnsignedFixedPoint(vrms, 16, 16);//0<=valid vrms_float<=0.84
	  voltage_rms_mv = vrms_float*297.5;//delt VINRmax=250mV,  voltage_rms_mv=vrms_float*delt VINRmax*1.19=vrms_float*297.5;
	  //voltage_Line = (voltage_rms_mv/1000)*(RISO1+RISO1+RISO1+RISO1+Rsense)/Rsense=(voltage_rms_mv/1000)*(1000+1000+1000+1000+1.21)/1.21
	  //              = voltage_rms_mv * 3.306785
	  voltage_Line =  voltage_rms_mv * 3.306785;// 31.42195 * 3.306785 = 103V.
	  irms = (Reg0x20_Val&0xFFFF0000)>>16;
      #ifdef ACS37800_CURRENT_16BIT_GAIN
	  irms_float = ConvertUnsignedFixedPoint(irms, 16, 16);//0<=valid irms_float<=0.84
      #else
	  irms_float = ConvertSignedFixedPoint(irms, 15, 16);//0<=valid irms_float<=0.84
      #endif
	  current_rms_amp = irms_float*107.1;//Iprmax=90A,  current_rms_amp=irms_float*Iprmax*1.19=irms_float*107.1;
	  Reg0x26_Val = ACS37800_Read(0x26);//
	  vrmsavgonesec = Reg0x26_Val&0x0000FFFF;
	  irmsavgonesec = (Reg0x26_Val&0xFFFF0000)>>16;
	  Reg0x27_Val = ACS37800_Read(0x27);//
	  vrmsavgonemin = Reg0x27_Val&0x0000FFFF;
	  irmsavgonemin = (Reg0x27_Val&0xFFFF0000)>>16;
	  Reg0x2A_Val = ACS37800_Read(0x2A);//
	  vcodes = Reg0x2A_Val&0x0000FFFF;
	  icodes = (Reg0x2A_Val&0xFFFF0000)>>16;
	  irms_float_codes = ConvertSignedFixedPoint(icodes, 15, 16);//0<=valid irms_float<=0.84
	  current_rms_amp_codes = irms_float_codes*107.1;//Iprmax=90A,  current_rms_amp=irms_float*Iprmax*1.19=irms_float*107.1;
	#else
	  Reg0x20_Val = ACS37800_Read(0x20);//
	  vrms = Reg0x20_Val&0x0000FFFF;
	  vrms_float = ConvertUnsignedFixedPoint(vrms, 16, 16);//0<=valid vrms_float<=0.84
	  //delt VINRmax=250mV,  voltage_rms_mv=vrms_float*delt VINRmax*1.19=vrms_float*297.5;
	  voltage_rms_mv = vrms_float*297.5;//0.10562*297.5=31.42195
	  //voltage_Line = (voltage_rms_mv/1000)*(RISO1+RISO1+RISO1+RISO1+Rsense)/Rsense=(voltage_rms_mv/1000)*(1000+1000+1000+1000+1.21)/1.21
	  //              = voltage_rms_mv * 3.306785
	  //voltage_Line =  voltage_rms_mv * 3.306785;// 31.42195 * 3.306785 = 103V.
	  voltage_Line =  voltage_rms_mv * 2.590;// 82.4*2.590 = 213 V
	  irms = (Reg0x20_Val&0xFFFF0000)>>16;
      #ifdef ACS37800_CURRENT_16BIT_GAIN
	  irms_float = ConvertUnsignedFixedPoint(irms, 16, 16);//0<=valid irms_float<=0.84
      #else
	  irms_float = ConvertSignedFixedPoint(irms, 15, 16);//0<=valid irms_float<=0.84
      #endif
	  current_rms_amp = irms_float*107.1;//Iprmax=90A,  current_rms_amp=irms_float*Iprmax*1.19=irms_float*107.1;
	#endif
#endif

#ifdef UART_AVAILABLE
	    //printf("vrms = %1.8f\r\n", vrms_float);
	    //printf("vrms = %1.8f\r\n", irms_float);
	    //sprintf(UART_TxBuffer, "%1.8f%1.8f\n", vrms_float,irms_float);
	    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)UART_TxBuffer, UART_TXBUFFER_SIZE, 5000)!= HAL_OK)
	    {
	      Error_Handler();
	    }
#endif
#ifdef MAX31856_SPI_AVAILABLE
	    ThermoCoupleTemperatureDegreeC = MAX31856_GetTemp_DegreeC();
#endif
	    HAL_Delay(1000);//1000ms delay
  }
}

#ifdef MAX31856_SPI_AVAILABLE

void MAX31856_CS_Init()
{
	GPIO_InitTypeDef  gpio_init_structure;
	SPIx_CS_GPIO_CLK_ENABLE();

    /* Configure the CS pin */
    gpio_init_structure.Pin   = SPIx_CS_PIN;
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SPIx_CS_GPIO_PORT, &gpio_init_structure);

    HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);//CS=1
}

uint8_t MAX31856_ReadReg(uint8_t RegAddr)
{
	uint8_t MAX31856_Reg_Get[4] = {0x00,0x00,0x00,0x00};
	uint8_t MAX31856_Reg_Addr[] = {0x00};
	//The MSB (A7) of this byte determines whether the following byte will be written or read.
	//If A7 is 0, one or more byte reads will follow the address byte.
	//If A7 is 1, one or more byte writes will follow the address byte.
	MAX31856_Reg_Addr[0] = RegAddr&0x7F;//
	//Write register address
	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET);
	//HAL_Delay(1);
	switch(HAL_SPI_Transmit(&SpiHandle, (uint8_t*)MAX31856_Reg_Addr,1,500))
	{
	    case HAL_OK:
	      break;
	    case HAL_TIMEOUT:
	      Timeout_Error_Handler();
	      break;
	    case HAL_ERROR:
	      Error_Handler();
	      break;
	    default:
	      break;
	}
	switch(HAL_SPI_Receive(&SpiHandle, (uint8_t*)MAX31856_Reg_Get, 1, 500))
	{
	    case HAL_OK:
	      break;
	    case HAL_TIMEOUT:
	      Timeout_Error_Handler();
	      break;
	    case HAL_ERROR:
	      Error_Handler();
	      break;
	    default:
	      break;
	}

	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
	//HAL_Delay(1);
	return MAX31856_Reg_Get[0];

}

void MAX31856_WriteReg(uint8_t RegAddr,uint8_t RegData)
{
	uint8_t MAX31856_Reg_WR_Buf[] = {0x00,0x00};
	//The MSB (A7) of this byte determines whether the following byte will be written or read.
	//If A7 is 0, one or more byte reads will follow the address byte.
	//If A7 is 1, one or more byte writes will follow the address byte.
	MAX31856_Reg_WR_Buf[0] = RegAddr|0x80;//
	MAX31856_Reg_WR_Buf[1] = RegData;//
	//Write register address
	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET);
	switch(HAL_SPI_Transmit(&SpiHandle, (uint8_t*)MAX31856_Reg_WR_Buf,2,500))
	{
	    case HAL_OK:
	      break;
	    case HAL_TIMEOUT:
	      Timeout_Error_Handler();
	      break;
	    case HAL_ERROR:
	      Error_Handler();
	      break;
	    default:
	      break;
	}
	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
}

void MAX31856_Get_AllReg(void)
{
	MAX31856_Reg0x00_CR0_Val = MAX31856_ReadReg(MAX31856_CR0_REG);//
	MAX31856_Reg0x01_CR1_Val = MAX31856_ReadReg(MAX31856_CR1_REG);//
	MAX31856_Reg0x02_MASK_Val= MAX31856_ReadReg(MAX31856_MASK_REG);//
	MAX31856_Reg0x03_CJHF_Val= MAX31856_ReadReg(MAX31856_CJHF_REG);//
	MAX31856_Reg0x04_CJLF_Val = MAX31856_ReadReg(MAX31856_CJLF_REG);
	MAX31856_Reg0x05_LTHFTH_Val = MAX31856_ReadReg(MAX31856_LTHFTH_REG);
	MAX31856_Reg0x06_LTHFTL_Val = MAX31856_ReadReg(MAX31856_LTHFTL_REG);
	MAX31856_Reg0x07_LTLFTH_Val= MAX31856_ReadReg(MAX31856_LTLFTH_REG);
	MAX31856_Reg0x08_LTLFTL_Val= MAX31856_ReadReg(MAX31856_LTLFTL_REG);
	MAX31856_Reg0x09_CJTO_Val= MAX31856_ReadReg(MAX31856_CJTO_REG);
	MAX31856_Reg0x0A_CJTH_Val= MAX31856_ReadReg(MAX31856_CJTH_REG);
	MAX31856_Reg0x0B_CJTL_Val= MAX31856_ReadReg(MAX31856_CJTL_REG);
	MAX31856_Reg0x0C_LTCBH_Val= MAX31856_ReadReg(MAX31856_LTCBH_REG);
	MAX31856_Reg0x0D_LTCBM_Val= MAX31856_ReadReg(MAX31856_LTCBM_REG);
	MAX31856_Reg0x0E_LTCBL_Val= MAX31856_ReadReg(MAX31856_LTCBL_REG);
	MAX31856_Reg0x0F_SR_Val= MAX31856_ReadReg(MAX31856_SR_REG);
}

void MAX31856_Init(void)
{
	uint8_t ThermocoupleType = 0B0011;// K type
	uint8_t RegVal;
	RegVal = MAX31856_ReadReg(MAX31856_CR0_REG);//
	//Bit7,CMODE=1---Automatic Conversion mode:Conversions occur continuously every 100ms (nominal).
	//Bit5~4=20, Open Circuit Detection, 40kΩ > RS > 5kΩ;
	MAX31856_WriteReg(MAX31856_CR0_REG,RegVal|0xA0);

	RegVal = MAX31856_ReadReg(MAX31856_CR1_REG);//
	RegVal &= 0xF0;
	RegVal |= ThermocoupleType&0x0F;//K type
	MAX31856_WriteReg(MAX31856_CR1_REG,RegVal);

	//assert on any fault
	MAX31856_WriteReg(MAX31856_MASK_REG,0x00);

	//set thermocouple temperature high threshold to 2048 C degrees(0x7fff).
	MAX31856_WriteReg(MAX31856_LTHFTH_REG,0x7f);
	MAX31856_WriteReg(MAX31856_LTHFTL_REG,0xff);
}

float MAX31856_GetTemp_DegreeC(void)
{
	uint32_t TemperatureDegreeC_int;
	float TemperatureDegreeC;
	uint8_t MAX31856_Reg_Get[3] = {0x00,0x00,0x00};
	uint8_t MAX31856_Reg_Addr[] = {0x00};
	//The MSB (A7) of this byte determines whether the following byte will be written or read.
	//If A7 is 0, one or more byte reads will follow the address byte.
	//If A7 is 1, one or more byte writes will follow the address byte.
	MAX31856_Reg_Addr[0] = MAX31856_LTCBH_REG&0x7F;//
	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET);
	//Write register address
	switch(HAL_SPI_Transmit(&SpiHandle, (uint8_t*)MAX31856_Reg_Addr,1,500))
	{
	    case HAL_OK:
	      break;
	    case HAL_TIMEOUT:
	      Timeout_Error_Handler();
	      break;
	    case HAL_ERROR:
	      Error_Handler();
	      break;
	    default:
	      break;
	}

	//Read register values.
	switch(HAL_SPI_Receive(&SpiHandle, (uint8_t*)MAX31856_Reg_Get, 3, 500))
	{
	    case HAL_OK:
	      break;
	    case HAL_TIMEOUT:
	      Timeout_Error_Handler();
	      break;
	    case HAL_ERROR:
	      Error_Handler();
	      break;
	    default:
	      break;
	}
	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
    //MAX31856_Reg_Get[0~2]---- Linearized TC Temperature, Byte2~0
	TemperatureDegreeC_int = ((uint32_t)MAX31856_Reg_Get[0]<<16)+((uint32_t)MAX31856_Reg_Get[1]<<8)+(uint32_t)MAX31856_Reg_Get[2];
    //effectively shift raw_read >> 12 to convert pseudo-float
	TemperatureDegreeC = (float)TemperatureDegreeC_int/4096;
	return TemperatureDegreeC;
}
#endif

#ifdef ACS37800_I2C_AVAILABLE
void ACS37800_SetEEPROM_Reg(uint8_t  Reg,uint32_t Data)
{
	/*The ACS37800 supports factory and customer EEPROM space as
		well as volatile registers. The customer access code must be sent
		prior to writing these customer EEPROM spaces. In addition, the
		device includes a set of free space EEPROM registers that are
		accessible with or without writing the access code.*/
	if(!ACS37800_Write(0x2F,0x4F70656E))//Customer code: 0x4F70656E
		Error_Handler();
	if(!ACS37800_Write(Reg,Data))
		Error_Handler();
}
void ACS37800_SetEEPROM( )
{
#ifdef ACS37800_DEMO_BOARD
    #ifdef ACS37800_DCTEST
    #ifdef ACS37800_CURRENT_16BIT_GAIN
	Reg0x0B_qvo_fine_set  = 22;//range:-256~255 default = 22
	Reg0x0B_sns_fine_set  = 35;//range:-256~255,default = 247
	Reg0x0B_crs_sns_set   =  0;//range:0~7, 0:1x,1:2x,2:3x,3:3.5x,4:4x,5:4.5x,6:5.5x,7:8x,   default = 3 //V0.7
	Reg0x0B_iavgselen_set =  0;//range:0,1 default = 0
	Reg0x0B_pavgselen_set =  0;//range:0,1 default = 0
	Reg0x0B_Val_set = (Reg0x0B_pavgselen_set<<23)|(Reg0x0B_iavgselen_set<<22)|(Reg0x0B_crs_sns_set<<19)|(Reg0x0B_sns_fine_set<<9)|Reg0x0B_qvo_fine_set;
	ACS37800_SetEEPROM_Reg(0x0B,Reg0x0B_Val_set);
    #endif
   /*
    Reg0x0B_qvo_fine_set  = 27;//range:-256~255 default = 22
	Reg0x0B_sns_fine_set  =  0;//range:-256~255,default = 247
	Reg0x0B_crs_sns_set   =  1;//range:0~7, 0:1x,1:2x,2:3x,3:3.5x,4:4x,5:4.5x,6:5.5x,7:8x,   default = 3
	Reg0x0B_iavgselen_set =  1;//range:0,1 default = 0
	Reg0x0B_pavgselen_set =  1;//range:0,1 default = 0
	Reg0x0B_Val_set = (Reg0x0B_pavgselen_set<<23)|(Reg0x0B_iavgselen_set<<22)|(Reg0x0B_crs_sns_set<<19)|(Reg0x0B_sns_fine_set<<9)|Reg0x0B_qvo_fine_set;
	ACS37800_SetEEPROM_Reg(0x0B,Reg0x0B_Val_set);

	Reg0x0C_rms_avg_1_set =  64;//range:0~127
	Reg0x0C_rms_avg_2_set =  256;//range:0~1023
	Reg0x0C_vchan_offset_code_set = 255;//range: -128~127
	Reg0x0C_Val_set = (Reg0x0C_vchan_offset_code_set<<17)|(Reg0x0C_rms_avg_2_set<<7)|Reg0x0C_rms_avg_1_set;
	ACS37800_SetEEPROM_Reg(0x0C,Reg0x0C_Val_set);

	Reg0x0D_ichan_del_en_set = 0;//range:0,1
	Reg0x0D_chan_del_sel_set = 0;//range:0~7
	Reg0x0D_fault_set = 71;//range:0~255
	Reg0x0D_fltdly_set = 0;//range:0~7
	Reg0x0D_Val_set = (Reg0x0D_fltdly_set<<21)|(Reg0x0D_fault_set<<13)|(Reg0x0D_chan_del_sel_set<<9)|(Reg0x0D_ichan_del_en_set<<7);
	ACS37800_SetEEPROM_Reg(0x0D,Reg0x0D_Val_set);

	Reg0x0E_vevent_cycs_set = 0;////range:0~63
	Reg0x0E_overvreg_set = 32;//range:0~63
	Reg0x0E_undervreg_set = 32;//range:0~63
	Reg0x0E_delaycnt_sel_set = 0;//range:0,1
	Reg0x0E_halfcycle_en_set = 0;//range:0,1
	Reg0x0E_squarewave_en_set = 0;//range:0,1
	Reg0x0E_zerocrosschansel_set = 0;//range:0,1
	Reg0x0E_zerocrossedgesel_set = 0;//range:0,1
	Reg0x0E_Val_set = (Reg0x0E_zerocrossedgesel_set<<24)|(Reg0x0E_zerocrosschansel_set<<23)|(Reg0x0E_squarewave_en_set<<22)|(Reg0x0E_halfcycle_en_set<<21)|(Reg0x0E_delaycnt_sel_set<<20)|(Reg0x0E_undervreg_set<<14)|(Reg0x0E_overvreg_set<<8)|(Reg0x0E_vevent_cycs_set);
	ACS37800_SetEEPROM_Reg(0x0E,Reg0x0E_Val_set);
    */
	Reg0x0F_i2c_slv_addr_set = I2C_ADDRESS;//I2C_ADDRESS = 127
	Reg0x0F_i2c_dis_slv_addr_set = 0;//range:0,1
	Reg0x0F_dio_0_sel_set = 0;//range:0~3
	Reg0x0F_dio_1_sel_set = 0;//range:0~3
	//For DC power monitoring applications using the ACS37800 or applications only using
	//the current measurement capability of the ACS37800, the following device settings are recommended.
	//Set bypass_n_en = 1. This setting disables the dynamic calculation of n based off voltage zero crossings
	//and sets n to a fixed value, which is set using EERPOM field n. See the Register	Details – EEPROM section for additional details.
	Reg0x0F_n_set = 512;//range: 0~1024
	Reg0x0F_bypass_n_en_set = 1;//range:0,1
	Reg0x0F_Val_set = (Reg0x0F_bypass_n_en_set<<24)|(Reg0x0F_n_set<<14)|(Reg0x0F_dio_1_sel_set<<12)|(Reg0x0F_dio_0_sel_set<<10)|(Reg0x0F_i2c_dis_slv_addr_set<<9)|(Reg0x0F_i2c_slv_addr_set<<2); //0x0F:0x1ff
	ACS37800_SetEEPROM_Reg(0x0F,Reg0x0F_Val_set);
    #else //AC test
	/*
	Reg0x0B_qvo_fine_set  = 22;//range:-256~255 default = 22
	Reg0x0B_sns_fine_set  =  0;//range:-256~255,default = 247
	Reg0x0B_crs_sns_set   =  1;//range:0~7, 0:1x,1:2x,2:3x,3:3.5x,4:4x,5:4.5x,6:5.5x,7:8x,   default = 3
	Reg0x0B_iavgselen_set =  1;//range:0,1 default = 0
	Reg0x0B_pavgselen_set =  1;//range:0,1 default = 0
	Reg0x0B_Val_set = (Reg0x0B_pavgselen_set<<23)|(Reg0x0B_iavgselen_set<<22)|(Reg0x0B_crs_sns_set<<19)|(Reg0x0B_sns_fine_set<<9)|Reg0x0B_qvo_fine_set;
	ACS37800_SetEEPROM_Reg(0x0B,Reg0x0B_Val_set);

	Reg0x0C_rms_avg_1_set =  64;//range:0~127
	Reg0x0C_rms_avg_2_set =  256;//range:0~1023
	Reg0x0C_vchan_offset_code_set = 255;//range: -128~127
	Reg0x0C_Val_set = (Reg0x0C_vchan_offset_code_set<<17)|(Reg0x0C_rms_avg_2_set<<7)|Reg0x0C_rms_avg_1_set;
	ACS37800_SetEEPROM_Reg(0x0C,Reg0x0C_Val_set);

	Reg0x0D_ichan_del_en_set = 0;//range:0,1
	Reg0x0D_chan_del_sel_set = 0;//range:0~7
	Reg0x0D_fault_set = 71;//range:0~255
	Reg0x0D_fltdly_set = 0;//range:0~7
	Reg0x0D_Val_set = (Reg0x0D_fltdly_set<<21)|(Reg0x0D_fault_set<<13)|(Reg0x0D_chan_del_sel_set<<9)|(Reg0x0D_ichan_del_en_set<<7);
	ACS37800_SetEEPROM_Reg(0x0D,Reg0x0D_Val_set);

	Reg0x0E_vevent_cycs_set = 0;////range:0~63
	Reg0x0E_overvreg_set = 32;//range:0~63
	Reg0x0E_undervreg_set = 32;//range:0~63
	Reg0x0E_delaycnt_sel_set = 0;//range:0,1
	Reg0x0E_halfcycle_en_set = 0;//range:0,1
	Reg0x0E_squarewave_en_set = 0;//range:0,1
	Reg0x0E_zerocrosschansel_set = 0;//range:0,1
	Reg0x0E_zerocrossedgesel_set = 0;//range:0,1
	Reg0x0E_Val_set = (Reg0x0E_zerocrossedgesel_set<<24)|(Reg0x0E_zerocrosschansel_set<<23)|(Reg0x0E_squarewave_en_set<<22)|(Reg0x0E_halfcycle_en_set<<21)|(Reg0x0E_delaycnt_sel_set<<20)|(Reg0x0E_undervreg_set<<14)|(Reg0x0E_overvreg_set<<8)|(Reg0x0E_vevent_cycs_set);
	ACS37800_SetEEPROM_Reg(0x0E,Reg0x0E_Val_set);
    */
	Reg0x0F_i2c_slv_addr_set = I2C_ADDRESS;//I2C_ADDRESS = 127
	Reg0x0F_i2c_dis_slv_addr_set = 0;//range:0,1
	Reg0x0F_dio_0_sel_set = 0;//range:0~3
	Reg0x0F_dio_1_sel_set = 0;//range:0~3
	//For DC power monitoring applications using the ACS37800 or applications only using
	//the current measurement capability of the ACS37800, the following device settings are recommended.
	//Set bypass_n_en = 1. This setting disables the dynamic calculation of n based off voltage zero crossings
	//and sets n to a fixed value, which is set using EERPOM field n. See the Register	Details – EEPROM section for additional details.
	Reg0x0F_n_set = 0;//range: 0~1024
	Reg0x0F_bypass_n_en_set = 0;//range:0,1
	Reg0x0F_Val_set = (Reg0x0F_bypass_n_en_set<<24)|(Reg0x0F_n_set<<14)|(Reg0x0F_dio_1_sel_set<<12)|(Reg0x0F_dio_0_sel_set<<10)|(Reg0x0F_i2c_dis_slv_addr_set<<9)|(Reg0x0F_i2c_slv_addr_set<<2); //0x0F:0x1ff
	ACS37800_SetEEPROM_Reg(0x0F,Reg0x0F_Val_set);
	#endif

#else
	Reg0x0B_qvo_fine_set  = 22;//range:-256~255 default = 22
	Reg0x0B_sns_fine_set  =  35;//range:-256~255,default = 258
    #ifdef ACS37800_CURRENT_16BIT_GAIN
	Reg0x0B_crs_sns_set   =  4;//range:0~7, 0:1x,1:2x,2:3x,3:3.5x,4:4x,5:4.5x,6:5.5x,7:8x, default = 3
    #else
	Reg0x0B_crs_sns_set   =  1;//range:0~7, 0:1x,1:2x,2:3x,3:3.5x,4:4x,5:4.5x,6:5.5x,7:8x, default = 3
    #endif
	Reg0x0B_iavgselen_set =  0;//range:0,1
	Reg0x0B_pavgselen_set =  0;//range:0,1
	Reg0x0B_Val_set = (Reg0x0B_pavgselen_set<<23)|(Reg0x0B_iavgselen_set<<22)|(Reg0x0B_crs_sns_set<<19)|(Reg0x0B_sns_fine_set<<9)|Reg0x0B_qvo_fine_set;
	ACS37800_SetEEPROM_Reg(0x0B,Reg0x0B_Val_set);


#endif
}



void ACS37800_GetEEPROM()
{
	Reg0x0B_Val = ACS37800_Read(0x0B);//default=0x19ee16
	Reg0x0C_Val = ACS37800_Read(0x0C);//default=0x1fe0000
	Reg0x0D_Val = ACS37800_Read(0x0D);//default=0x8c000
	Reg0x0E_Val = ACS37800_Read(0x0E);//default=0x82000
	Reg0x0F_Val = ACS37800_Read(0x0F);//default=0x1ff=511
	Reg0x0B_qvo_fine  =  Reg0x0B_Val&0x000001FF;
	Reg0x0B_sns_fine  =  (Reg0x0B_Val>>9)&0x000003FF;
	Reg0x0B_crs_sns   =  (Reg0x0B_Val>>19)&0x00000007;
	Reg0x0B_iavgselen =  (Reg0x0B_Val>>22)&0x00000001;
	Reg0x0B_pavgselen =  (Reg0x0B_Val>>23)&0x00000001;
	Reg0x0C_rms_avg_1 =  Reg0x0C_Val&0x0000007F;
	Reg0x0C_rms_avg_2 =  (Reg0x0C_Val>>7)&0x000003FF;
	Reg0x0C_vchan_offset_code = (Reg0x0C_Val>>17)&0x000000FF;
	Reg0x0D_ichan_del_en = (Reg0x0D_Val>>7)&0x00000001;
	Reg0x0D_chan_del_sel = (Reg0x0D_Val>>9)&0x00000007;
	Reg0x0D_fault = (Reg0x0D_Val>>13)&0x000000FF;
	Reg0x0D_fltdly = (Reg0x0D_Val>>21)&0x00000007;
	Reg0x0E_vevent_cycs = Reg0x0E_Val&0x0000003F;
	Reg0x0E_overvreg = (Reg0x0E_Val>>8)&0x0000003F;
	Reg0x0E_undervreg = (Reg0x0E_Val>>14)&0x0000003F;
	Reg0x0E_delaycnt_sel = (Reg0x0E_Val>>20)&0x00000001;
	Reg0x0E_halfcycle_en = (Reg0x0E_Val>>21)&0x00000001;
	Reg0x0E_squarewave_en = (Reg0x0E_Val>>22)&0x00000001;
	Reg0x0E_zerocrosschansel = (Reg0x0E_Val>>23)&0x00000001;
	Reg0x0E_zerocrossedgesel = (Reg0x0E_Val>>24)&0x00000001;
	Reg0x0F_i2c_slv_addr = (Reg0x0F_Val>>2)&0x0000007F;
	Reg0x0F_i2c_dis_slv_addr = (Reg0x0F_Val>>9)&0x00000001;
	Reg0x0F_dio_0_sel = (Reg0x0F_Val>>10)&0x00000003;
	Reg0x0F_dio_1_sel = (Reg0x0F_Val>>12)&0x00000003;
	Reg0x0F_n = (Reg0x0F_Val>>14)&0x000003FF;
	Reg0x0F_bypass_n_en = (Reg0x0F_Val>>24)&0x00000001;

}



uint32_t ACS37800_Write(uint8_t RegAddr,uint32_t RegData)
{
	uint8_t ACS37800_Reg_Set[5] = {0x00,0x00,0x00,0x00,0x00};
	ACS37800_Reg_Set[0] = RegAddr;//
	ACS37800_Reg_Set[1] = (uint8_t)(RegData&0x000000FF);
	ACS37800_Reg_Set[2] = (uint8_t)((RegData>>8)&0x000000FF);
	ACS37800_Reg_Set[3] = (uint8_t)((RegData>>16)&0x000000FF);
	ACS37800_Reg_Set[4] = (uint8_t)((RegData>>24)&0x000000FF);
	/* Timeout is set to 500ms */
	while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)(I2C_ADDRESS<<1), (uint8_t*)ACS37800_Reg_Set, 5, 500)!= HAL_OK)
	{
	  /* Error_Handler() function is called when Timeout error occurs.
	     When Acknowledge failure occurs (Slave don't acknowledge its address)
	     Master restarts communication */
	  if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
	  {
		  BSP_LED_On(LED3);
		  return 0;
	  }
	}
    return 1;
}

uint32_t ACS37800_Read(uint8_t RegAddr)
{
	uint32_t RegData;
	uint8_t ACS37800_Reg_Get[4] = {0x00,0x00,0x00,0x00};
	uint8_t ACS37800_Reg_Addr[] = {0x00};
	ACS37800_Reg_Addr[0] = RegAddr;//
	/* Timeout is set to 500ms */
	while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)(I2C_ADDRESS<<1), (uint8_t*)ACS37800_Reg_Addr, 1, 500)!= HAL_OK)
	{
	  /* Error_Handler() function is called when Timeout error occurs.
	     When Acknowledge failure occurs (Slave don't acknowledge its address)
	     Master restarts communication */
	  if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
	  {
		  return 0;
	  }
	}

	/*##-3- Put I2C peripheral in reception process ############################*/
	/* Timeout is set to 10S */
	//while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 10000) != HAL_OK)
	while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)(I2C_ADDRESS<<1), (uint8_t *)ACS37800_Reg_Get, 4, 500) != HAL_OK)
	{
	  /* Error_Handler() function is called when Timeout error occurs.
	     When Acknowledge failure occurs (Slave don't acknowledge it's address)
	     Master restarts communication */
	  if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
	  {
		  return 0;
	  }
	}
	RegData = (uint32_t)ACS37800_Reg_Get[0]+((uint32_t)ACS37800_Reg_Get[1]<<8)+((uint32_t)ACS37800_Reg_Get[2]<<16)+((uint32_t)ACS37800_Reg_Get[3]<<24);
    return RegData;
}

/*
 * Convert an unsigned bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float_t ConvertUnsignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width)
{
    uint32_t mask;

    if (width == 32)
    {
        mask = 0xFFFFFFFF;
    }
    else
    {
        mask = (1UL << width) - 1UL;
    }

    return (float_t)(inputValue & mask) / (float_t)(1L << binaryPoint);
}


/*
 * Convert a signed bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be sign extended then converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float_t ConvertSignedFixedPoint(uint32_t inputValue, uint16_t binaryPoint, uint16_t width)
{
    int32_t signedValue = SignExtendBitfield(inputValue, width);
    return (float_t)signedValue / (float_t)(1L << binaryPoint);
}

/*
 * Sign extend a bitfield which if right justified
 *
 *    data        - the bitfield to be sign extended
 *    width       - the width of the bitfield
 *    returns     - the sign extended bitfield
 */
int32_t SignExtendBitfield(uint32_t data, uint16_t width)
{
	// If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
    if (width == 32)
    {
        return (int32_t)data;
    }

    int32_t x = (int32_t)data;
    int32_t mask = 1L << (width - 1);

    x = x & ((1 << width) - 1); // make sure the upper bits are zero

    return (int32_t)((x ^ mask) - mask);
}
#endif

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE BYPASS)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

}
/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave don't acknowledge it's address, Master restarts communication.
    * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
    */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
  {
    /* Turn Off LED1 */
    BSP_LED_Off(LED1);

    /* Turn On LED3 */
    BSP_LED_On(LED3);
  }
}
/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  Error_Handler();
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED1 off */
  BSP_LED_Off(LED1);
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Timeout_Error_Handler(void)
{
  /* Toggle LED3 on */
  while(1)
  {
    BSP_LED_On(LED3);
    HAL_Delay(500);
    BSP_LED_Off(LED3);
    HAL_Delay(500);
  }
}
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
/*static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MAX31856_FAULT_PIN)
  {
    /* Toggle LED1 */
    BSP_LED_Toggle(LED1);
  }
}

/**
  * @brief  Configures EXTI lines 15 to 10 (connected to PD.15 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI15_10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;    /* current CPU (CM7) config in IT rising/falling */
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = MAX31856_FAULT_PIN;
  HAL_GPIO_Init(MAX31856_FAULT_GPIO_PORT, &GPIO_InitStructure);

  /* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* Configure the second CPU (CM4) EXTI line for IT*/
  //HAL_EXTI_D2_EventInputConfig(EXTI_LINE13 , EXTI_MODE_IT,  ENABLE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
