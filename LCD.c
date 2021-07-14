/**
  ******************************************************************************
  * @file    Templates/Src/LCD.c
  * @author  MCD Application Team
  * @brief   Fichero de inicialización del la pantalla LCD de la tarjeta de 
	*					 aplicaciones y funciones para representar texto.					 
	*					 Pines del LCD:
	*					 PIN MOSI-> PB5	
	*					 PIN SCK->	PA5	
	*					 PIN Reset->PA6	
	*					 PIN A0->	 	PF13
	*					 PIN nCs-> 	PD14	
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * 
  ******************************************************************************
  */


#include "LCD.h"
#include "Arial12x12.h"
#include "Driver_SPI.h"
#include "mbedAppBoard_PINOUT.h"

unsigned char buffer[512];
uint8_t posicionL1=0;
uint16_t posicionL2=256;
uint16_t comienzo = 0;
int j;
char L1[128];	//El máximo de caracteres pueden ser 128 por linea, en concreto 128 exclamaciones (!)
char L2[128];
uint8_t recoger1;
uint16_t recoger2;
char lcd_text[2][20+1];

extern ARM_DRIVER_SPI Driver_SPI1;
ARM_DRIVER_SPI* SPIdrv = &Driver_SPI1;

void wr_data(unsigned char data){	
  
	ARM_SPI_STATUS stat;
	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_A0.Port, LCD_A0.IO, GPIO_PIN_SET);	// Seleccionar A0 = 1;
	SPIdrv -> Send(&data,sizeof(data));		// Escribir un dato (data) 
		stat = SPIdrv->GetStatus ();
	while(stat.busy){
	stat = SPIdrv->GetStatus ();
	}
	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_SET);

	

}

void wr_cmd(unsigned char cmd){
	
	ARM_SPI_STATUS stat;
	
	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_A0.Port, LCD_A0.IO, GPIO_PIN_RESET);	// Seleccionar A0 = 0;
	SPIdrv -> Send(&cmd,sizeof(cmd));		// Escribir un comando (cmd) 
	stat = SPIdrv->GetStatus ();
	while(stat.busy){
	stat = SPIdrv->GetStatus ();
	}
	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_SET);
}

void LCD_reset(void){
	int i;
	
	int32_t	status = 0;
	
	status = SPIdrv->Initialize(NULL);
	status = SPIdrv->PowerControl(ARM_POWER_FULL);
	status = SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_DATA_BITS(8), 1000000);


  /*Configure GPIO pin Output Level */
	
	HAL_GPIO_WritePin(LCD_RESET.Port, LCD_RESET.IO, GPIO_PIN_RESET); //RESET = 0
	HAL_Delay(2);
	HAL_GPIO_WritePin(LCD_RESET.Port, LCD_RESET.IO, GPIO_PIN_SET);		//RESET = 1
	HAL_Delay(20);
	
	//	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_RESET);
			wr_cmd(0xAE);
			wr_cmd(0xA2);
			wr_cmd(0xA0);
			wr_cmd(0xC8);
			wr_cmd(0x22);
			wr_cmd(0x2F);
			wr_cmd(0x40);
			wr_cmd(0xAF);
			wr_cmd(0x81);
			wr_cmd(0x17);
			wr_cmd(0xA6);
	//	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_SET);
		lcd_clean();

}

void copy_to_lcd(void){
    int i;
	//HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_RESET);
    wr_cmd(0x00);      // 4 bits de la parte baja de la dirección a 0
    wr_cmd(0x10);      // 4 bits de la parte alta de la dirección a 0
    wr_cmd(0xB0);      // Página 0
    
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
    
				
    wr_cmd(0x00);       
    wr_cmd(0x10);       
    wr_cmd(0xB3);      // Pagina 3
     				
				
    for(i=384;i<512;i++){
        wr_data(buffer[i]);
        }
	//	HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_SET);
}

void lcd_clean(){
	for(int i=0; i<512;i++){
		buffer[i]=0;
	}
	copy_to_lcd();
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

void pixel (int x, int y, int color){
	if(x > 128 || y > 32 || x < 0 || y < 0) return;
        if(color == 0)
            buffer[x + ((y/8) * 128)] &= ~(1 << (y%8));  // erase pixel
        else
            buffer[x + ((y/8) * 128)] |= (1 << (y%8));   // set pixel
    copy_to_lcd();
}

void pant_neg (void){
		for(int i=0; i<512;i++){
		buffer[i]=0xff;
	}
	copy_to_lcd();
}

void GPIO_INIT(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET.Port, LCD_RESET.IO, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_A0.Port, LCD_A0.IO, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_N.Port, LCD_CS_N.IO, GPIO_PIN_SET);

  /*Configure GPIO pin : Reset */
  GPIO_InitStruct.Pin = LCD_RESET.IO;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET.Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A0 */
  GPIO_InitStruct.Pin = LCD_A0.IO;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_A0.Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS */
  GPIO_InitStruct.Pin = LCD_CS_N.IO;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_N.Port, &GPIO_InitStruct);

}