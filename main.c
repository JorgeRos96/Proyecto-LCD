/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   STM32F4xx HAL API Template project 
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "Driver_SPI.h"
#include "Arial12x12.h"
#include <string.h>

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif

#ifdef RTE_CMSIS_RTOS2_RTX5
/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}

#endif

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t posicionL1=0;
uint16_t posicionL2=256;
uint16_t comienzo = 0;
int j;
unsigned char buffer[512];
const int valores[11]={0xAE, 0xA2, 0xA0, 0xC8, 0x22, 0x2F, 0x40, 0xAF, 0x81, 0x17, 0xA6};
char L1[128];	//El máximo de caracteres pueden ser 128 por linea, en concreto 128 exclamaciones (!)
char L2[128];
uint8_t recoger1;
uint16_t recoger2;
char lcd_text[2][20+1];
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
static void MX_GPIO_Init(void);
void retardo(uint32_t n_microsegundos);
int EscribeLetra_L1 (uint8_t letra);
int EscribeLetra_L2 (uint16_t letra);
void posicion_l2_reset(void);
void init_lcd(void);
void wr_data(unsigned char data);
void wr_cmd(unsigned char cmd);
void LCD_reset(void);
void copy_to_lcd(void);
void escribir_frecuencia(float norte, float sur, float este, float oeste);
void actualizar(char lcd_text[2][20+1]);
void lcd_clean(void);

extern ARM_DRIVER_SPI Driver_SPI1;
ARM_DRIVER_SPI* SPIdrv = &Driver_SPI1;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();

  /* Add your application code here
     */
	MX_GPIO_Init();
  
	init_lcd();
	LCD_reset();
	lcd_clean();
  sprintf (lcd_text[0], "FUNCIONA");
  sprintf (lcd_text[1], "POR FIN");
	actualizar(lcd_text);
#ifdef RTE_CMSIS_RTOS2
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();

  /* Create thread functions that start executing, 
  Example: osThreadNew(app_main, NULL, NULL); */

  /* Start thread execution */
  osKernelStart();
#endif

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 165;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_PIN_7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_PIN_13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_PIN_14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}
void retardo(uint32_t n_milisegundos){
	uint32_t i=0;
	uint32_t cuenta = 0;
	cuenta = n_milisegundos*1000*80;
	for(i=0;i < cuenta; i++);
}
int EscribeLetra_L1 (uint8_t letra){
		uint8_t i, valor1 , valor2;

	if(posicionL1<=115){	//Si la cadena se va a salir de la linea dejas de escribir, hemos tomado como referencia el caracter mas grande (@) que son 12 lineas 128 -12 -1=115
	comienzo = 25 *(letra - ' ');	//Coge el primer byte de la tabla (que es el ancho del caracter)
	for (i=0; i<12; i++){

		valor1 = Arial12x12[comienzo+i*2+1];	//guardas el byte de la parte superior del caracter
		valor2 = Arial12x12[comienzo+i*2+2];	//guardas el byte de la parte inferior del caracter
		
		buffer[i+posicionL1] = valor1;				//copias en el buffer el byte guardado y lo pone
		buffer[i+128+posicionL1] = valor2;
	}

	
	posicionL1 = posicionL1 + Arial12x12[comienzo];	// actualiza el valor de la dirección para que escriba el siguiente caracter (le suma el ancho del caracter)
}
	return 0;	
}

int EscribeLetra_L2 (uint16_t letra){
	uint8_t i, valor1 , valor2;
	
	if(posicionL2<=371){	//Si la cadena se va a salir de la linea dejas de escribir, hemos tomado como referencia el caracter mas grande (@) que son 12 lineas 384 -12 -1=371
	
	comienzo = 25 *(letra - ' ');	//Coge el primer byte de la tabla (que es el ancho del caracter)
	for (i=0; i<12; i++){

		valor1 = Arial12x12[comienzo+i*2+1];	//guardas el byte de la parte superior del caracter
		valor2 = Arial12x12[comienzo+i*2+2];	//guardas el byte de la parte inferior del caracter
		
		buffer[i+posicionL2] = valor1;				//copias en el buffer el byte guardado y lo pone
		buffer[i+128+posicionL2] = valor2;
	}

	
	posicionL2 = posicionL2 + Arial12x12[comienzo];	// actualiza el valor de la dirección para que escriba el siguiente caracter (le suma el ancho del caracter)
}
	return 0;	
}
void posicion_l2_reset(void){
posicionL2=256;

}
void init_lcd(void){
	int32_t	status = 0;

	  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	retardo(2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	retardo(20);
	
	status = SPIdrv->Initialize(NULL);
	status = SPIdrv->PowerControl(ARM_POWER_FULL);
	status = SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_DATA_BITS(8), 20000000);
	status = SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	
	
}

void wr_data(unsigned char data){
  
	ARM_SPI_STATUS stat;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);	// Seleccionar CS = 0;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
	SPIdrv -> Send(&data,sizeof(data));		// Escribir un dato (data) 
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);	// Seleccionar CS = 1;
  stat = SPIdrv->GetStatus ();
	while(stat.busy){
	stat = SPIdrv->GetStatus ();
	}
}

void wr_cmd(unsigned char cmd){
	
	ARM_SPI_STATUS stat;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);	// Seleccionar CS = 0;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);	// Seleccionar A0 = 1;
  SPIdrv -> Send(&cmd,sizeof(cmd));		// Escribir un dato (data) 
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);	// Seleccionar CS = 1;
	stat = SPIdrv->GetStatus ();
	while(stat.busy){
	stat = SPIdrv->GetStatus ();
	}

}
void LCD_reset(){
	int i;
		for(i=0;i<11;i++){
			wr_cmd(valores[i]);
	}

}

void copy_to_lcd(void){
    int i;
    wr_cmd(0x00);      // 4 bits de la parte baja de la dirección a 0
    wr_cmd(0x10);      // 4 bits de la parte alta de la dirección a 0
    wr_cmd(0xB0);      // Página 0
    
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
    for(i=0;i<128;i++){
        wr_data(buffer[i]);
        }
  
     
    wr_cmd(0x00);      // 4 bits de la parte baja de la dirección a 0
    wr_cmd(0x10);      // 4 bits de la parte alta de la dirección a 0
    wr_cmd(0xB1);      // Página 1
    
					
    for(i=128;i<256;i++){
        wr_data(buffer[i]);
        }
    
    wr_cmd(0x00);       
    wr_cmd(0x10);      
    wr_cmd(0xB2);      //Página 2
				
		
    for(i=256;i<384;i++){
        wr_data(buffer[i]);
        }
    
				
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
    wr_cmd(0x00);       
    wr_cmd(0x10);       
    wr_cmd(0xB3);      // Pagina 3
     
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET);
				
    for(i=384;i<512;i++){
        wr_data(buffer[i]);
        }
}


/*void escribir_frecuencia(float norte, float sur, float este, float oeste){
	posicionL1=0;
	posicionL2=256;
  int cadena1 = 0;
	float cantidad=norte+sur+este+oeste;
	cadena1=sprintf(L2,"%0.2f eur", cantidad);
		for (j=0; j<cadena1;j++){
		recoger2= (uint8_t)L2[j];
		EscribeLetra_L2(recoger2);
		}
	sprintf(L1,"Cantidad");
	for (j=0; j<strlen(L1);j++){
		recoger1= (uint8_t)L1[j];
		EscribeLetra_L1(recoger1);
		}
	copy_to_lcd();
}*/

void actualizar(char lcd_text[2][20+1]){
		posicionL1=0;
	posicionL2=256;
		for (j=0; j<strlen(lcd_text[0]);j++){
		recoger1= lcd_text[0][j];
		EscribeLetra_L1(recoger1);
		}
	for (j=0; j<strlen(lcd_text[1]);j++){
		recoger2= lcd_text[1][j];
		EscribeLetra_L2(recoger2);
		}
	copy_to_lcd();
}
void lcd_clean(){
	for(int i=0; i<512;i++){
		buffer[i]=0;
	}
	copy_to_lcd();
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

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
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
