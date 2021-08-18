/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   Proyecto para el manejo de la pantalla LCD de la tarjeta de 
	*				 	 aplicaciones a traves del uso del SPI para comunicarse con el
	*					 microcontrolador. Para representar el texto por pantalla se utiliza 
	*			 		 la fuente de texto Arial a traves de la librería Arial12x12.
	*					 Se emplea el SPI1 con la configuración:
	*					
	*					 - SPI Mode Master
	*					 - CPOL1 y CPHA1
	*					 - MSB a LSB
	*					 - Envío de 8 bits de datos
	*					 - Velocidad del SPI 1 MHz
	*
	*					 Los pines MOSI y SCK se configuran a traves del fichero RTE_Device.h.
	*					 El resto de los pines se configuran en la función GPIO_INIT de la  
	*					 librería LCD. Los pines que se utilizan son los correspondientes al 
	*					 conexionado de la mbed application shield y que vienen definidos en  
	*					 el fichero mbedApplicationBoard_PINOUT.
	*					 Debido al conflicto entre el SPI y el ETHERNET se ha realizado el 
	*					 el cambio del Solder Bridge SB121 para colocarlo en el SB122. Con esto
	*					 al conectar el MOSI en el pin PA7 en realidad se conecta al pin PB5,
	*					 por lo que la configuración de los pines es la siguiente:
	*					
	*					 PIN MOSI-> PB5	
	*					 PIN SCK->	PA5	
	*					 PIN Reset->PA6	
	*					 PIN A0->	 	PF13
	*					 PIN nCs-> 	PD14	
  *
	*					 Se configura el reloj del sistema para que trabaje a una frecuencia 
	*					 de 180 MHz utilizando como fuente de reloj el PLL con el HSI.
	*					  
	*	
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LCD.h"
#include "Delay.h"
#include "Watchdog.h"

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif

/* Private variables ---------------------------------------------------------*/
char text[2][20+1];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(int fallo);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*Inicialización del IWDG*/
	if (init_Watchdog() != 0)
			Error_Handler(5);
	
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  if (HAL_Init() != HAL_OK)
		Error_Handler(0);

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();

	/*Inicializaciñon del Delay*/
	Init_Delay(180,4);
  /*Inicialización de los pines A0, nCs y Reset de la pantalla LCD*/
	GPIO_INIT();
	/*Inicialización de la pantalla LCD*/
	if (LCD_reset() != HAL_OK)
		Error_Handler(4);
	
	/*Texto a escribir por pantalla*/
//	sprintf (text[0], "Prueba de texto");
//  sprintf (text[1], "Satisfactoria");
	/*Envío del texto a la pantalla*/
	actualizar(text);
	/*Pueba del funcionamiento de todos los pixeles*/
	pant_neg();


  /* Infinite loop */
  while (1)
  {
		reset_Watchdog();
  }
}

/**
  * @brief  Función de configuración del reloj del sistema en el que se utiliza
	*					como fuente de reloj del sistema el PLL con el HSI como fuente de 
	*					reloj. Se configura una frecuencia del sistema de 180 MHz.
  *         Se configuran los paramteros de la siguiente manera: 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 8
  *            PLL_N                          = 180
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{ 
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
  /** Se configura el HSI como fuente de reloj del PLL y se configuran
	* 	los parametros del PLL para ajusta la frecuencia a 180 MHz con una
	* 	frecuencia del HSI de 16 MHZ (por defecto).
	* 	SYSCLK =[(16MHz(frecuencia HSI)/8(PLLM))*180 (PLLN)]/2 (PLLP) = 180 MHz
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(1);
  }
  /** Se activa el modo de Over Drive para poder alcanzar los 180 MHz
	* 	como frecuencia del sistema
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler(1);
  }
  /** Se selecciona el PLL como fuente de reloj del sistema y se configuran los parametros
	*		para configurar el HCLK, PCLK1 y PCLK2. La frecuencia máxima del HCLK es 180 MHZ, la 
	*		frecuencia máxima del PCLK1 es de 45 MHZ y la frecuencia máxima del PCLK2 es de 90 MHz
	*		HCLK = SYSCK/AHB = 180 MHz / 1 = 180 MHz
	*		PCLK1 = HCLK/APB1 = 180 MHz / 4 = 45 MHZ
	*		PCLK2 = HCLK/APB2 = 180 MHz / 2 = 90 MHZ
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler(1);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(int fallo)
{
	char buf[100];
  
	if(fallo == 0)
		/* Mensaje si se ha producido un error en la inicializacón de la librería HAL*/
		printf(buf,"\r Se ha producido un error al inicializar la librería HAL\n");
	else if (fallo == 1)
		/* Mensaje si se ha producido un error en la inicializacón del reloj del sistema*/
		printf(buf,"\r Se ha producido un error al inicializar el reloj del sistema\n");
	else if (fallo == 4)
		/* Mensaje si se ha producido un error en la inicializacón de la pantalla*/
		printf(buf,"\r Se ha producido un error al inicializar la pantalla LCD\n");
	else if (fallo == 5)
		/* Mensaje si se ha producido un error en la inicialización del Watchdog*/
		printf(buf,"\r Se ha producido un error al inicializar el Watchdog\n");
 
	while(1)
  {
  }
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
