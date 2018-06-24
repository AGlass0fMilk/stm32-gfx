/**
 * @file main.cpp
 * @brief Brief Description
 * 
 * Detailed Description
 *
 * Link to [markdown like this](@ref PageTag)
 * Make sure you tag the markdown page like this:
 * # Page title {#PageTag}
 * 
 * <a href='MyPDF.pdf'> Link to PDF documents like this</a>
 * If you add document files, make sure to add them into a directory inside a "docs" folder
 * And then run hud-devices/tools/copy-dox-files.py 
 *
 * To use images, make sure they're in an "images" folder and follow the doxygen user manual to add images.
 * You must run copy-dox-files.py after adding images as well.
 *
 * @copyright Copyright &copy; 2018 Heads Up Display, Inc.
 *
 *  Created on: Jun 16, 2018
 *      Author: gdbeckstein
 */

#include "hx8357d_driver.h"
#include "i8080_8bit_api.h"
#include "stm32f4xx_hal_sram.h"
#include "mbed_debug.h"
#include "mbed_retarget.h"
#include "mbed_wait_api.h"
#include "drivers/DigitalOut.h"
#include "drivers/Ticker.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "drivers/Serial.h"
#include "drivers/AnalogIn.h"
#include "rtos/Thread.h"

extern "C" {

#include "lvgl.h"
#include "lv_tutorial_themes.h"
#include "demo.h"
#include "tpcal.h"
}
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
void _Error_Handler(char *file, int line);

#define CMD 0
#define DATA (1 << 16)
#define SRAM_BANK_ADDR_8BIT(d) *((__IO uint8_t*)(0x60000000 + d))
#define SRAM_BANK_ADDR_16BIT(d) *((__IO uint16_t*)(0x60000000 + d))
#define SRAM_BANK_ADDR_32BIT(d) *((__IO uint32_t*)(0x60000000 + d))

i8080_8bit_t bus;

//mbed::FileHandle* mbed_override_console(int fd)
//{

//}

mbed::DigitalOut led(LED1);
mbed::DigitalOut reset(PD_12);
mbed::DigitalOut chip_select(PB_3, 1);

rtos::Thread led_thread;

/*Sets horizontal filling range*/
void Set_Column(unsigned long SC, unsigned long EC)
{
		hx8357d_set_column_address(&bus, SC, EC);
}

/*Sets vertical filling range*/
void Set_Row(unsigned long SP, unsigned long EP)
{
	hx8357d_set_row_address(&bus, SP, EP);
}

/*Fill the entire screen with RGB color.*/
//rgb     :   RGB color (565 format)
void TFT_Fill_Screen(uint16_t rgb)
{
    unsigned long pixel=0;

    Set_Column(0,319);
    Set_Row(0,479);

    //Start Write Command
    //hx8357d_write_memory_start(&bus);

    *((__IO uint8_t*)(0x60000000)) = 0x2C;

    for(pixel=0;pixel<(320*480);pixel++)
    {
    	*((__IO uint16_t*)(0x60010000)) = rgb;
   	 //*((__IO uint8_t*)(0x60010000)) = (rgb & 0xFF00) >> 8;
   	 //*((__IO uint8_t*)(0x60010000)) = (rgb & 0x00FF);
    }
}

void TFT_Fillbox(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
    Set_Column(x1, x1+w-1);
    Set_Row(y1, y1+h-1);

    *((__IO uint8_t*)(0x60000000)) = 0x2C;

    for(int i = 0; i < (w*h); i++)
    {
    	*((__IO uint16_t*)(0x60010000)) = color;
    }
}

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_flush_ready()' has to be called when finished
 * This function is required only when LV_VDB_SIZE != 0 in lv_conf.h*/
static void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

	Set_Column(x1, x2);
	Set_Row(y1, y2);

    *((__IO uint8_t*)(0x60000000)) = 0x2C;

    int32_t x;
    int32_t y;
    uint16_t* ptr = (uint16_t*) color_p;
    uint16_t color_565;
    for(y = y1; y <= y2; y++) {
        for(x = x1; x <= x2; x++) {

        	uint8_t pix_color = 0;
        	pix_color = (color_p->red << 3) | ((color_p->green & 0x38) >> 3);
        	*((__IO uint8_t*)(0x60010000)) = pix_color;
        	pix_color = ((color_p->green & 0x7) << 5) | (color_p->blue);
        	*((__IO uint8_t*)(0x60010000)) = pix_color;

        	/*uint32_t pixelColor = 0xFF000000 |
        			color_p->red << 16 |
					color_p->green << 8 |
					color_p->blue;*/

        	/*color_565 |= ((color_p->red & 0x1F) << 11);
        	color_565 |= ((color_p->green & 0x3F) << 5);
        	color_565 |= ((color_p->blue & 0x1F));
        	*((__IO uint16_t*)(0x60010000)) = color_565; //(uint16_t)(color_p->full);*/
        	color_p = (const lv_color_t*) ++ptr;
        	//wait_ms(250);
        }
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_flush_ready();
}

/* Read the touchpad and store it in 'data'
 * Return false if no more data read; true for ready again */

#define X_PLUS_PIN 	PC_1
#define Y_PLUS_PIN 	PC_2
#define X_MINUS_PIN	PC_4
#define Y_MINUS_PIN	PC_5
#define PWM_BACKLITE PC_6

#define NUMSAMPLES 2

static void insert_sort(int array[], uint8_t size) {
  uint8_t j;
  int save;

  for (int i = 1; i < size; i++) {
    save = array[i];
    for (j = i; j >= 1 && save < array[j - 1]; j--)
      array[j] = array[j - 1];
    array[j] = save;
  }
}

static bool tp_read(lv_indev_data_t *data)
{
	lv_coord_t tp_x, tp_y;

	bool valid;
	int x, y;
	int samples[NUMSAMPLES];

	// Read X

	mbed::DigitalOut x_plus_out(X_PLUS_PIN, 1);
	mbed::DigitalOut x_minus_out(X_MINUS_PIN, 0);
	mbed::AnalogIn y_plus_in(Y_PLUS_PIN);
	mbed::AnalogIn y_minux_in(Y_MINUS_PIN);
	wait_us(20);

	for(int i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = y_plus_in.read_u16();
	}

   // Allow small amount of measurement noise, because capacitive
	// coupling to a TFT display's signals can induce some noise.
	if (samples[0] - samples[1] < -4 || samples[0] - samples[1] > 4) {
		valid = 0;
	} else {
		samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
	}

	//insert_sort(samples, NUMSAMPLES);

	x = (0x4096 - samples[NUMSAMPLES/2]);

	// Read Y
	mbed::AnalogIn x_plus_in(X_PLUS_PIN);
	mbed::AnalogIn x_minus_in(X_MINUS_PIN);
	mbed::DigitalOut y_plus_out(Y_PLUS_PIN, 1);
	mbed::DigitalOut y_minus_out(Y_MINUS_PIN, 0);
	wait_us(20);

	for(int i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = x_minus_in.read_u16();
	}

	//insert_sort(samples, NUMSAMPLES);
	// Allow small amount of measurement noise, because capacitive
	// coupling to a TFT display's signals can induce some noise.
	if (samples[0] - samples[1] < -4 || samples[0] - samples[1] > 4) {
		valid = 0;
	} else {
		samples[1] = (samples[0] + samples[1]) >> 1; // average 2 samples
	}

	y = (4096 - samples[NUMSAMPLES/2]);

	debug("Touch [%i, %i]\n", x, y);
	wait_ms(300);

    /* Read your touchpad */
    data->state = LV_INDEV_STATE_PR; //or LV_INDEV_STATE_PR;
    data->point.x = (lv_coord_t) x;
    data->point.y = (lv_coord_t) y;

    return false;   /*false: no more data to read because we are no buffering*/
}


void led_main(void)
{
	while(1)
	{
		led = !led;
		wait_ms(500);

	}
}

//mbed::Serial serial(USBTX, USBRX);

/*void uart_irq_handler(void)
{
	while(serial.readable())
	{
		serial.putc(serial.getc());
	}
}*/

/**
 * Read the display's 24-bit ID
 * @param[in] buf 3-byte buffer to hold the ID
 */
void read_display_id(uint8_t* buf)
{
	//chip_select = 0;
	SRAM_BANK_ADDR_8BIT(CMD) = (uint8_t) 0x04;	// Write the Read id command out
	uint8_t dummy = SRAM_BANK_ADDR_8BIT(DATA); 	// Dummy read
	buf[0] = SRAM_BANK_ADDR_8BIT(DATA);
	buf[1] = SRAM_BANK_ADDR_8BIT(DATA);
	buf[2] = SRAM_BANK_ADDR_8BIT(DATA);

	//*buf = SRAM_BANK_ADDR_32BIT(DATA);
	//chip_select = 1;
}

uint8_t read_manu_id(void)
{
	//chip_select = 0;
	SRAM_BANK_ADDR_8BIT(CMD) = 0xDA;
	uint8_t dummy;
	dummy = SRAM_BANK_ADDR_8BIT(DATA);
	dummy = SRAM_BANK_ADDR_8BIT(DATA);
	return dummy;
}

mbed::Ticker lv_tick;

void tick(void)
{
	lv_tick_inc(1);
}

int main(void) {
	//i8080_8bit_init(&bus);

	/*uint32_t* lcd_ptr;
	 lcd_ptr = (uint32_t*)((uint32_t)0x60000000);
	 uint32_t i = 0xF3;
	 *lcd_ptr = i;
	 //uint8_t i = 0xF3;*/
	//HAL_SRAM_Write_8b(&bus.sram_handle, (uint32_t*)(0x60000000), &i, 1);
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	//HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	//SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_FSMC_Init();
	/* USER CODE BEGIN 2 */

	chip_select = 0;

	reset = 0;
	wait_ms(200);
	reset = 1;
	wait_ms(300);

	//SRAM_BANK_ADDR_8BIT(CMD) = 0x01; // Soft reset

	uint8_t buf[3];
	read_display_id(buf);

	//uint8_t buf = read_manu_id();
	debug("Display ID: 0x%X, 0x%X, 0x%X\r\n", buf[0], buf[1], buf[2]);

	//debug("Display ID: 0x%X\r\n", buf);

	hx8357d_init(&bus);
	TFT_Fill_Screen(0xFFFF);

	lv_disp_drv_t disp_drv;
	lv_init();
	lv_disp_drv_init(&disp_drv);
	disp_drv.disp_flush = disp_flush;
	lv_disp_drv_register(&disp_drv);

    /*************************
     * Input device interface
     *************************/
    /*Add a touchpad in the example*/
    /*touchpad_init();*/                            /*Initialize your touchpad*/
//    lv_indev_drv_t indev_drv;                       /*Descriptor of an input device driver*/
//    lv_indev_drv_init(&indev_drv);                  /*Basic initialization*/
//    indev_drv.type = LV_INDEV_TYPE_POINTER;         /*The touchpad is pointer type device*/
//    indev_drv.read = tp_read;                 /*Library ready your touchpad via this function*/
//    lv_indev_drv_register(&indev_drv);              /*Finally register the driver*/

	lv_tick.attach_us(tick, 1000);

	led_thread.start(led_main);

	//tpcal_create();

	demo_create();

	//lv_tutorial_themes();

	while(true)
	{
		lv_task_handler();
		wait_ms(5);
	}

	uint16_t color = 0;
	while(true)
	{
		continue;
		TFT_Fill_Screen(color);
		color++;
		wait_ms(10);
	}


	uint16_t colors[] = {0xF800, 0x7E0, 0x1F };

	int i = 0;
	int j = 0;
	while (true) {

		if(i >= 3)
			i = 0;

		if(j < 9)
		{
			TFT_Fill_Screen(colors[i]);
		}
		else
		if(j > 9)
		{
			TFT_Fillbox(100, 100, 200, 200, colors[i]);
		}

		i++;
		j++;

		//debug("Hello World!");
		wait_ms(500);
		//i++;
		//*lcd_ptr = i;

		//if(i > 0x7F)
		//	i = 0;
		//SRAM_BANK_ADDR(0) = i;
		//HAL_SRAM_Write_8b(&bus.sram_handle, (uint32_t*) 0x64000000,
		//&i, 1);
		//*((__IO uint8_t*)(0x64010000)) = i++;
		//led = !led;
		//debug("Ping!");
		//wait_ms(500);
	}

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : PD12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {
	FSMC_NORSRAM_TimingTypeDef Timing;

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 3; //15;
	Timing.AddressHoldTime = 2;
	Timing.DataSetupTime = 3;
	Timing.BusTurnAroundDuration = 0; //15;
	Timing.CLKDivision = 0;//16;
	Timing.DataLatency = 0; //17;
	Timing.AccessMode = FSMC_ACCESS_MODE_B;//FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
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
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

