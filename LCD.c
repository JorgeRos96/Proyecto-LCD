#include "LCD.h"
#include "Arial12x12.h"
#include "Driver_SPI.h"

const int valores[11]={0xAE, 0xA2, 0xA0, 0xC8, 0x22, 0x2F, 0x40, 0xAF, 0x81, 0x17, 0xA6};
unsigned char buffer[512];
uint8_t posicionL1=0;
uint16_t posicionL2=256;
uint16_t comienzo = 0;
int j;
char L1[128];	//El m�ximo de caracteres pueden ser 128 por linea, en concreto 128 exclamaciones (!)
char L2[128];
uint8_t recoger1;
uint16_t recoger2;
char lcd_text[2][20+1];

extern ARM_DRIVER_SPI Driver_SPI1;
ARM_DRIVER_SPI* SPIdrv = &Driver_SPI1;

void wr_data(unsigned char data){
  
	ARM_SPI_STATUS stat;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
	SPIdrv -> Send(&data,sizeof(data));		// Escribir un dato (data) 
  stat = SPIdrv->GetStatus ();
	while(stat.busy){
	stat = SPIdrv->GetStatus ();
	}
}

void wr_cmd(unsigned char cmd){
	
	ARM_SPI_STATUS stat;
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);	// Seleccionar A0 = 0;
  SPIdrv -> Send(&cmd,sizeof(cmd));		// Escribir un comando (cmd) 
	stat = SPIdrv->GetStatus ();
	while(stat.busy){
	stat = SPIdrv->GetStatus ();
	}

}

void LCD_reset(void){
	int i;
	
	int32_t	status = 0;
	
	status = SPIdrv->Initialize(NULL);
	status = SPIdrv->PowerControl(ARM_POWER_FULL);
	status = SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_DATA_BITS(8), 1000000);
	
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);    //A0 = 0

  /*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);	// Seleccionar CS = 0 ya que solo se controla un periferico con SPI


  /*Configure GPIO pin Output Level */
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //RESET = 0
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//RESET = 1
	HAL_Delay(20);
		
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
		for(int i=0; i<512;i++){
		buffer[i]=0;
	}
	copy_to_lcd();

}

void copy_to_lcd(void){
    int i;
    wr_cmd(0x00);      // 4 bits de la parte baja de la direcci�n a 0
    wr_cmd(0x10);      // 4 bits de la parte alta de la direcci�n a 0
    wr_cmd(0xB0);      // P�gina 0
    
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
    for(i=0;i<128;i++){
        wr_data(buffer[i]);
        }
  
     
    wr_cmd(0x00);      // 4 bits de la parte baja de la direcci�n a 0
    wr_cmd(0x10);      // 4 bits de la parte alta de la direcci�n a 0
    wr_cmd(0xB1);      // P�gina 1
    
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
    for(i=128;i<256;i++){
        wr_data(buffer[i]);
        }
    
    wr_cmd(0x00);       
    wr_cmd(0x10);      
    wr_cmd(0xB2);      //P�gina 2
				
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	// Seleccionar A0 = 1;
    for(i=256;i<384;i++){
        wr_data(buffer[i]);
        }
    
				
    wr_cmd(0x00);       
    wr_cmd(0x10);       
    wr_cmd(0xB3);      // Pagina 3
     
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13,GPIO_PIN_SET);
				
				
    for(i=384;i<512;i++){
        wr_data(buffer[i]);
        }
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

	
	posicionL1 = posicionL1 + Arial12x12[comienzo];	// actualiza el valor de la direcci�n para que escriba el siguiente caracter (le suma el ancho del caracter)
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

	
	posicionL2 = posicionL2 + Arial12x12[comienzo];	// actualiza el valor de la direcci�n para que escriba el siguiente caracter (le suma el ancho del caracter)
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

void GPIO_INIT(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_PIN_6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
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