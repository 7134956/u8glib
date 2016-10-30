/*
7134956@gmail.com 2016
*/

#include "stm32f10x.h"
#include "u8g.h"

/*---------------------Configure SPI for display ----------------------------*/
/* Тактируем выбраные блоки */
#define SPI_RCC				RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1
/* Выбор модуля SPI */
#define SPI_UNIT			SPI1
/* Настройка выводов SPI */
#define SPI_PORT			GPIOA
#define SPI_PINS			GPIO_Pin_5 | GPIO_Pin_7
/* Настройка вывода CS */
#define SPI_PORT_CS			GPIOA
#define SPI_PIN_CS			GPIO_Pin_4
/* Настройка вывода A0 */
#define SPI_PORT_A0			GPIOA
#define SPI_PIN_A0			GPIO_Pin_15
//#define SPI_SKIP_BUSY /* For speed SPI transfer. If SPI_CLK >= CPU_CLK/2 */
/* CPOL = 1, CPHA = 1 */
/*---------------------Configure I2C for display ----------------------------*/
#define I2C_RCC		RCC_APB1Periph_I2C2 | RCC_APB2Periph_GPIOB
#define I2C_UNIT	I2C2
#define	I2C_PORT	GPIOB
#define	I2C_PINS	GPIO_Pin_10 | GPIO_Pin_11
#define I2C_SPEED	100000
/*---------------------End configure for display ---------------------------*/

#define DELAY_TIM_FREQUENCY 1000000 /* = 1MHZ -> timer runs in microseconds */

#define SPI_8BIT 1
#define SPI_16BIT 2

#define CS_ON() GPIO_ResetBits (SPI_PORT_CS, SPI_PIN_CS)
#define CS_OFF() GPIO_SetBits (SPI_PORT_CS, SPI_PIN_CS)

#define A0_HIGH() GPIO_SetBits (SPI_PORT_A0, SPI_PIN_A0)
#define A0_LOW()  GPIO_ResetBits (SPI_PORT_A0, SPI_PIN_A0)

void SPIInit(uint8_t);
void DMA_SPI_init(void);
void SPI_DMA_Send(void *source, uint16_t count);
void I2C_start(uint8_t address, uint8_t direction);
void I2C_stop(void);

/******************************************************************************
 * Configure SPI for display
 *****************************************************************************/
void SPIInit(uint8_t param) {
	GPIO_InitTypeDef SPI_Pin_Init; //Настройка пинов SPI1.
        SPI_InitTypeDef SPI_Unit_Init; //Настройка SPI.

	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE); //Вкл. тактирование SPI и PIN-ов.
	SPI_Pin_Init.GPIO_Pin = SPI_PINS;
	SPI_Pin_Init.GPIO_Mode = GPIO_Mode_AF_PP; //Настраиваем SPI_SCK и SPI_MOSI
	SPI_Pin_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_PORT, &SPI_Pin_Init);

	SPI_Pin_Init.GPIO_Mode = GPIO_Mode_Out_PP;
	SPI_Pin_Init.GPIO_Pin = SPI_PIN_CS;
	SPI_Pin_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_PORT_CS, &SPI_Pin_Init);

	SPI_Pin_Init.GPIO_Mode = GPIO_Mode_Out_PP;
	SPI_Pin_Init.GPIO_Pin = SPI_PIN_A0;
	SPI_Pin_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_PORT_A0, &SPI_Pin_Init);

	CS_OFF(); //CS=1.

	SPI_Unit_Init.SPI_Direction = SPI_Direction_1Line_Tx; //Только TX = MOSI = выход.
	SPI_Unit_Init.SPI_Mode = SPI_Mode_Master; //Мастер.
	if(param == SPI_8BIT)
	SPI_Unit_Init.SPI_DataSize = SPI_DataSize_8b; //Можно и 16!
	if(param == SPI_16BIT)
	SPI_Unit_Init.SPI_DataSize = SPI_DataSize_16b; //Можно и 8!
	SPI_Unit_Init.SPI_CPHA = SPI_CPHA_2Edge; //Со 2-го фронта.
	SPI_Unit_Init.SPI_CPOL = SPI_CPOL_High; //В режиме ожидания SCK - 1.
	SPI_Unit_Init.SPI_NSS = SPI_NSS_Soft; //Програмный NSS (в железе отключено).
	//Need clock 10Mhz by datasheet. Work tested on full speed 36Mhz.
	SPI_Unit_Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //Скорость.
	SPI_Unit_Init.SPI_FirstBit = SPI_FirstBit_MSB; //Со старшего бита.
	SPI_Unit_Init.SPI_CRCPolynomial = 7; //Фигня какая-то.

	SPI_Init(SPI_UNIT, &SPI_Unit_Init);

	SPI_Cmd(SPI_UNIT, ENABLE); //Запуск SPI.
}

/*******************************************************************************
 *set TIM1 to run at DELAY_TIM_FREQUENCY
 ******************************************************************************/
void delay_init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB2RSTR = RCC_APB2RSTR_TIM1RST;
	RCC->APB2RSTR = 0;
	SystemCoreClockUpdate ();
	TIM1->PSC = (SystemCoreClock / DELAY_TIM_FREQUENCY) - 1;
	TIM1->CR1 |= TIM_CR1_CEN;
	RCC->APB2ENR &=~ RCC_APB2ENR_TIM1EN;
}

/*******************************************************************************
 *Delay by the provided number of micro seconds.
 *Limitation: "us" * System-Freq in MHz must now overflow in 32 bit.
 *Values between 0 and 1.000.000 (1 second) are ok.
 ******************************************************************************/
void delay_micro_seconds(uint32_t us) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CNT = (uint16_t)0;
	/* use 16 bit count wrap around */
	while ((uint16_t)(TIM1->CNT) <= us);
	RCC->APB2ENR &=~ RCC_APB2ENR_TIM1EN;
}

 /****************************************************************************/
/*
 The following delay procedures must be implemented for u8glib

 void u8g_Delay(uint16_t val)		Delay by "val" milliseconds
 void u8g_MicroDelay(void)		Delay be one microsecond
 void u8g_10MicroDelay(void)	Delay by 10 microseconds

 */

void u8g_Delay(uint16_t val) {
	delay_micro_seconds(1000UL * (uint32_t) val);
}

void u8g_MicroDelay(void) {
	delay_micro_seconds(1);
}

void u8g_10MicroDelay(void) {
	delay_micro_seconds(10);
}

/******************************************************************************
 * st7586s on SPI 8 bit

Com_fn unpack page buffer to display

2 pixels on byte

B-black
W-white
LG-Light gray
DG-Dark grey

BW mode

0b00011000, B W
0b00000000, W W
0b00011000, B W
0b11000000, W B
0b11011000, B B
0b10010000, B B
0b01001000, W W

Gray mode
0b00011000, B W
0b00000000, W W
0b00011000, B W
0b11000000, W B
0b11011000, B B
0b10010000, DG DG
0b01001000, LG LG

 *****************************************************************************/
uint8_t u8g_com_stm32_st7586s_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
	switch (msg) {
	case U8G_COM_MSG_STOP:
		//Остановить устройство
		break;
	case U8G_COM_MSG_INIT: 
		delay_init();
		SPIInit(SPI_8BIT); //Инициализация SPI (stm32f1)
		break;
	case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
		if (arg_val)
			A0_HIGH();
		else
			A0_LOW();
		break;
	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val == 0) {
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
			CS_OFF();
		} else {
			/* enable */
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			CS_ON();
		}
		break;
	case U8G_COM_MSG_RESET:
		u8g_10MicroDelay();
		break;
	case U8G_COM_MSG_WRITE_BYTE:
#ifndef SPI_SKIP_BUSY
		while (SPI_UNIT->SR & SPI_I2S_FLAG_BSY) {};
#endif
		SPI_UNIT->DR = arg_val;
		break;
	case U8G_COM_MSG_WRITE_SEQ: {
		register uint8_t *ptr = arg_ptr;
		uint8_t byte, i;
		while (arg_val > 0) {
			for (i = 0; i < 4; i++) {
				byte = ((*ptr & 128) / 8) | ((*ptr & 64) * 2); //Старший бит
				*ptr <<= 2;
#ifndef SPI_SKIP_BUSY
				while (SPI_UNIT->SR & SPI_I2S_FLAG_BSY) {};
#else
				__nop(); //Choose the number of "nop" from the code rate
//				__nop();
//				__nop();
//				__nop();
//				__nop();

#endif
				SPI_UNIT->DR = byte;
			}
			ptr++;
			arg_val--;
		}
	}
		break;
	}
	return 1;
}

/******************************************************************************
 * Display on SPI 8 bit
 *****************************************************************************/
uint8_t u8g_com_stm32_hw_spi_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
	switch (msg) {
	case U8G_COM_MSG_STOP:
		//Остановить устройство
		break;
	case U8G_COM_MSG_INIT:
		delay_init();
		SPIInit(SPI_8BIT); //Инициализация SPI (stm32f1)
		break;
	case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
		if (arg_val)
			A0_HIGH();
		else
			A0_LOW();
		break;
	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val == 0) {
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
			CS_OFF();
		} else {
			/* enable */
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			CS_ON();
		}
		break;
	case U8G_COM_MSG_RESET:
		u8g_10MicroDelay();
		break;
	case U8G_COM_MSG_WRITE_BYTE:
#ifndef SPI_SKIP_BUSY
		while (SPI_UNIT->SR & SPI_I2S_FLAG_BSY) {};
#endif
		SPI_UNIT->DR = arg_val;
		break;
	case U8G_COM_MSG_WRITE_SEQ: {
		register uint8_t *ptr = arg_ptr;
		while (arg_val > 0) {
#ifndef SPI_SKIP_BUSY
			while (SPI_UNIT->SR & SPI_I2S_FLAG_BSY) {};
#endif
			SPI_UNIT->DR = u8g_pgm_read(ptr);
			ptr++;
			arg_val--;
		}
	}
		break;
	}
	return 1;
}

/*******************************************************************************
 * Display on SPI 8 bit DMA
 ******************************************************************************/
uint8_t u8g_com_stm32_hw_spi_dma_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
	switch (msg) {
	case U8G_COM_MSG_STOP:
//Остановить устройство
		break;
	case U8G_COM_MSG_INIT:
		delay_init();
		SPIInit(SPI_8BIT);
		DMA_SPI_init();
		break;
	case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
		while (DMA1_Channel3->CCR & DMA_CCR3_EN) {};
		if (arg_val)
			A0_HIGH();
		else
			A0_LOW();
		break;
	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val == 0) {
			while (DMA1_Channel3->CNDTR) {}; // Wait transmit complete
			/* disable */
//			RCC->APB2ENR &= ~ RCC_APB2ENR_SPI1EN;
			CS_OFF();
		} else {
			/* enable */
//			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
			CS_ON();
		}
		break;
	case U8G_COM_MSG_RESET:
		u8g_10MicroDelay();
		break;
	case U8G_COM_MSG_WRITE_BYTE:
#ifndef SPI_SKIP_BUSY
		while (DMA1_Channel3->CNDTR) {}; // Wait transmit complete
		while (SPI_UNIT->SR & SPI_I2S_FLAG_BSY) {};
#endif
		SPI_UNIT->DR = arg_val;
		break;
	case U8G_COM_MSG_WRITE_SEQ: {
		register uint8_t *ptr = arg_ptr;
#ifndef SPI_SKIP_BUSY
		while (DMA1_Channel3->CNDTR) {}; // Wait transmit complete
		while (SPI_UNIT->SR & SPI_I2S_FLAG_BSY) {};
#endif
		SPI_DMA_Send(ptr, arg_val * 160);
	}
		break;
	}
	return 1;
}

/*******************************************************************************
 * Configure DMA
 ******************************************************************************/
void DMA_SPI_init() {
	DMA_InitTypeDef DMA_InitStructure;
	//Тактирование включается следующим образом:
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//Прежде всего сбросим предыдущие настройки DMA
	DMA_DeInit(DMA1_Channel3);
	//Указатель на регистр периферийного устройства
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI_UNIT->DR);
	//Адрес в памяти потом изменим
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;
	//Переферия это приемник
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	//Размер передаваемого буффера настроим при передаче
	DMA_InitStructure.DMA_BufferSize = 0;
	//Не инкрементировать адресс переферии
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//Инкрементировать адрес в памяти
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//Ширина слова переферии
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//Ширина слова в памяти
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//После выполнения работы сработает прерывание
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//Низкий приоритет.
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	//Отключаем режим передачи из памяти в память
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	//Применяем настройки
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	//Разрешаем SPI работать в DMA.
	SPI_I2S_DMACmd(SPI_UNIT, SPI_I2S_DMAReq_Tx, ENABLE);
	//Включаем прерывания от DMA по окончанию передачи
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	//Включаем общие прерывания в NVIC.
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/*******************************************************************************
 * DMA completion interrupt
 ******************************************************************************/
void DMA1_Channel3_IRQHandler(void) {
	if (DMA1->ISR & DMA1_IT_TC3) { //If interrupt from Tx complete channel 3
		/* DMA Clear IT Pending Bit */
		DMA1->IFCR = DMA1_IT_TC3;
		/* Disable DMA1_Channel3 */
		DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
		/* Disable clock DMA1 */
//		RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;
		/* Disable CS pin on SPI */
		CS_OFF();
	}
}

/*******************************************************************************
 * Sending data via SPI DMA
 ******************************************************************************/
void SPI_DMA_Send(void *source, uint16_t count) {
	while (DMA1_Channel3->CNDTR) {
	} // Wait transmit complete
	/* Enable clock DMA1 */
//	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	/* Set data counter for DMA1 Channel3 */
	DMA1_Channel3->CMAR = (uint32_t) source;
	DMA1_Channel3->CNDTR = count;
	/* Enable DMA1 Channel3 */
	DMA1_Channel3->CCR |= DMA_CCR3_EN;
}

void i2c_out(uint8_t data);
void i2c_init(void);

#define DELAY_TIM_FREQUENCY 1000000 /* = 1MHZ -> timer runs in microseconds */
#define SSD1306_I2C_ADDRESS   0x3C	//0x7A		// 011110+SA0+RW - 0x3C or 0x3D

uint8_t control = 0;

/*******************************************************************************
 *
 ******************************************************************************/
void i2c_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(I2C_RCC, ENABLE);
	
	I2C_DeInit(I2C_UNIT);
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_Init(I2C_UNIT, &I2C_InitStructure);

	GPIO_InitStructure.GPIO_Pin = I2C_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_PORT, &GPIO_InitStructure);

	I2C_Cmd(I2C_UNIT, ENABLE);
}

/*******************************************************************************
 * Display on I2C
 ******************************************************************************/
uint8_t u8g_com_stm32_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
	
	switch (msg) {
	case U8G_COM_MSG_STOP:
		break;

	case U8G_COM_MSG_INIT:
		i2c_init();
		delay_init();
		u8g_MicroDelay();
		break;

	case U8G_COM_MSG_ADDRESS: /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
		if (arg_val == 0) {
			control = 0;
		} else {
			control = 0x40;
		}
		u8g_10MicroDelay();
		break;

	case U8G_COM_MSG_RESET:

		u8g_10MicroDelay();
		break;

	case U8G_COM_MSG_WRITE_BYTE:

		i2c_out(arg_val);
		u8g_MicroDelay();
		break;

	case U8G_COM_MSG_WRITE_SEQ:
	case U8G_COM_MSG_WRITE_SEQ_P: {
		register uint8_t *ptr = arg_ptr;
		I2C_start(SSD1306_I2C_ADDRESS, I2C_Direction_Transmitter);
		I2C_SendData(I2C_UNIT, control);
		while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		}
		while (arg_val > 0) {
			I2C_SendData(I2C_UNIT, *ptr++);
			arg_val--;
			while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
			}
		}
		I2C_stop();
		u8g_MicroDelay();
	}
		break;
	}
	return 1;
}

/*******************************************************************************
 *
 ******************************************************************************/
void I2C_stop() {
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2C_UNIT, ENABLE);
}

/*******************************************************************************
 *
 ******************************************************************************/
void I2C_start(uint8_t address, uint8_t direction) {

	// wait until I2C1 is not busy anymore
	while (I2C_GetFlagStatus(I2C_UNIT, I2C_FLAG_BUSY)) {
	}
	// Send I2C1 START condition
	I2C_GenerateSTART(I2C_UNIT, ENABLE);
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_MODE_SELECT)) {
	}
	// Send slave Address for write
	I2C_Send7bitAddress(I2C_UNIT, address << 1, direction);
	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if (direction == I2C_Direction_Transmitter) {
		while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		}
	} else if (direction == I2C_Direction_Receiver) {
		while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		}
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
void i2c_out(uint8_t data) {
	I2C_start(SSD1306_I2C_ADDRESS, I2C_Direction_Transmitter);
	//Wire.write(control);
	I2C_SendData(I2C_UNIT, control);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
	}
	I2C_SendData(I2C_UNIT, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while (!I2C_CheckEvent(I2C_UNIT, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
	}
	I2C_stop();
}
