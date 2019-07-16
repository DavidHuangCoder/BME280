#include <atmel_start.h>
#include <driver_examples.h>
#include "BME280/BME280.h"
#include <stdio.h>
#include "string.h"

int main(void)
{
	uint8_t bmp280_id = 0;
	uint8_t ctr_reg = 0;
	uint8_t status_reg = 0;
	char Temp[100];
	double tem = 0;
	double pressure = 0;
	double humidity = 0;
	char Tempi[10];
	char Tempf[10];
	char Pressurei[20];
	char Pressuref[10];
	char humidityi[10];
	char *a,*b,*c,*h;
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	//gpio_set_pin_level(LED,
	//// <y> Initial level
	//// <id> pad_initial_level
	//// <false"> Low
	//// <true"> High
	//true);
	//
    struct bmp280* b1 = bmp280_init(&I2C_0);
	if(b1->valid == true) {				
		while(1) {
			//ctr_reg = bmp280_read_register(&I2C_0, 0xF4);
			//status_reg = bmp280_read_register(&I2C_0, 0xF3);
			tem = bmp280_get_temperature(b1);
			pressure = bmp280_get_pressure(b1);
			humidity = bmp280_get_humidity(b1);
			
			sprintf(Tempi,"TEMP: %d",(int)tem);
			sprintf(Tempf,"%d",(int)(tem*10)%10);
			a = strcat(Tempi,".");
			a = strcat(a,Tempf);
			a = strcat(a,", ");
			
			sprintf(humidityi,"RH: %d",(int)humidity);
			//sprintf(Pressuref,"%d",(int)(pressure*10)%10);
			//b = strcat(Pressurei,".");
			//b = strcat(b,Pressuref);
			h = strcat(humidityi,"%, ");
			
			sprintf(Pressurei,"PRESS: %d",(int)pressure);
			//sprintf(Pressuref,"%d",(int)(pressure*10)%10);
			//b = strcat(Pressurei,".");
			//b = strcat(b,Pressuref);
			b = strcat(Pressurei,"\r\n");
			
			c = strcat(a,h);
			c = strcat(c,b);
			
			struct io_descriptor *io;
			usart_sync_get_io_descriptor(&USART_0, &io);
			usart_sync_enable(&USART_0);
			
 			io_write(io, (uint8_t *)c, strlen(a));  
			
			delay_ms(5000);
		}
	}

	///* Replace with your application code */
	//while (1) {
		//I2C_0_example();
		//USART_0_example();
		////usart_sync_write(&USART_0,(uint8_t*)"Hello World!", 12);
		//delay_ms(700);
		//gpio_set_pin_level(LED,
		//// <y> Initial level
		//// <id> pad_initial_level
		//// <false"> Low
		//// <true"> High
		//false);
		//delay_ms(700);
		//gpio_set_pin_level(LED,
		//// <y> Initial level
		//// <id> pad_initial_level
		//// <false"> Low
		//// <true"> High
		//true);
	//}
}
