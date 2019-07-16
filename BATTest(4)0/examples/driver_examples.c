/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

void I2C_0_example(void)
{
	uint8_t lu8 = 0xD0;
	
	struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, 0x76, I2C_M_SEVEN);
	//io_write(I2C_0_io, (uint8_t *)&lu8, 1);
	//io_read(I2C_0_io,(uint8_t*)&RX,2);
}

/**
 * Example of using USART_0 to write "Hello World" using the IO abstraction.
 */
void USART_0_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&USART_0, &io);
	usart_sync_enable(&USART_0);

	//io_write(io, (uint8_t *)&RX, 12);
}

void delay_example(void)
{
	delay_ms(5000);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of ¡°5123¡± equals 51.23 DegC.
// t_fine carries temperature as global value
//BME280_S32_t t_fine;
//BME280_S32_t BME280_compensate_T_int32( BME280_S32_t adc_T)
//{
	//BME280_S32_t var1, var2, T;
	//var1 = ((((adc_T>>3) - (( BME280_S32_t )dig_T1<<1))) * (( BME280_S32_t )dig_T2)) >> 11;
	//var2 = (((((adc_T>>4) - (( BME280_S32_t )dig_T1)) * ((adc_T>>4) - (( BME280_S32_t )dig_T1))) >> 12) *
	//(( BME280_S32_t )dig_T3)) >> 14;
	//t_fine = var1 + var2;
	//T = (t_fine * 5 + 128) >> 8;
	//return T;
//}