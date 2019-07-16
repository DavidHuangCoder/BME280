#include <stdlib.h>
#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"
#include "BME280/BME280.h"

#define dig_T1 bmp280->T1
#define dig_T2 bmp280->T2
#define dig_T3 bmp280->T3
#define dig_P1 bmp280->P1
#define dig_P2 bmp280->P2
#define dig_P3 bmp280->P3
#define dig_P4 bmp280->P4
#define dig_P5 bmp280->P5
#define dig_P6 bmp280->P6
#define dig_P7 bmp280->P7
#define dig_P8 bmp280->P8
#define dig_P9 bmp280->P9
#define dig_H1 bmp280->H1
#define dig_H2 bmp280->H2
#define dig_H3 bmp280->H3
#define dig_H4 bmp280->H4
#define dig_H5 bmp280->H5
#define dig_H6 bmp280->H6


uint8_t bmp280_read_register(struct i2c_m_sync_desc *const i2c, uint8_t reg_addr)
{
	uint8_t reg_data;
	struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(i2c, &I2C_0_io);
	i2c_m_sync_enable(i2c);
	i2c_m_sync_set_slaveaddr(i2c, 0x76, I2C_M_SEVEN);
	io_write(I2C_0_io, (uint8_t *)&reg_addr, 1);
	io_read(I2C_0_io,(uint8_t*)&reg_data,1);
	
	return reg_data;
}

void bmp280_write_register(struct i2c_m_sync_desc *const i2c, uint8_t reg_addr, uint8_t reg_data)
{
	uint8_t tx_data[2] = {reg_addr,reg_data};
    struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(i2c, &I2C_0_io);
	i2c_m_sync_enable(i2c);
	i2c_m_sync_set_slaveaddr(i2c, 0x76, I2C_M_SEVEN);
	io_write(I2C_0_io, (uint8_t *)&tx_data, 2);
}

/**
* 在bmp280_init()函数里默认初始化t_standby为0.5ms，
* 温度和气压的采样精度设为最低，
* 滤波器系数设为最低，
* 并且进入sleep mode。
*/
struct bmp280 *bmp280_init(struct i2c_m_sync_desc *const i2c)
{
	uint8_t bmp280_id;
	uint8_t lsb, msb;
	uint8_t ctrlmeas_reg, config_reg;
	struct bmp280 *bmp280;
	
	bmp280_id = bmp280_read_register(i2c, BMP280_CHIPID_REG);
	if(bmp280_id == 0x60) {
		bmp280 = malloc(sizeof(struct bmp280));
		
		bmp280->i2c = i2c;
		bmp280->mode = BMP280_NORMAL_MODE;
		bmp280->t_sb = BMP280_T_SB1;
		bmp280->p_oversampling = BMP280_P_MODE_1;
		bmp280->t_oversampling = BMP280_T_MODE_1;
		bmp280->filter_coefficient = BMP280_FILTER_MODE_1;
		} else {
		return NULL;
	}
	
	/* read the temperature calibration parameters */
	lsb = bmp280_read_register(i2c, BMP280_DIG_T1_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_T1_MSB_REG);
	dig_T1 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_T2_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_T2_MSB_REG);
	dig_T2 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_T3_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_T3_MSB_REG);
	dig_T3 = msb << 8 | lsb;
	
	/* read the pressure calibration parameters */
	lsb = bmp280_read_register(i2c, BMP280_DIG_P1_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P1_MSB_REG);
	dig_P1 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P2_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P2_MSB_REG);
	dig_P2 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P3_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P3_MSB_REG);
	dig_P3 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P4_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P4_MSB_REG);
	dig_P4 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P5_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P5_MSB_REG);
	dig_P5 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P6_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P6_MSB_REG);
	dig_P6 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P7_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P7_MSB_REG);
	dig_P7 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P8_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P8_MSB_REG);
	dig_P8 = msb << 8 | lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_P9_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_P9_MSB_REG);
	dig_P9 = msb << 8 | lsb;
	
	/* read the humidity calibration parameters */
	lsb = bmp280_read_register(i2c, BMP280_DIG_H1_REG);
	dig_H1 = lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_H2_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_H2_MSB_REG);
	dig_H2 = ((int16_t)msb) *256 + ((int16_t)lsb);
	lsb = bmp280_read_register(i2c, BMP280_DIG_H3_REG);
	dig_H3 = lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_H4_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_H4_MSB_REG);
	msb &= 0x0f;
	dig_H4 = ((int16_t)msb) + ((int16_t)lsb*16);
	lsb = bmp280_read_register(i2c, BMP280_DIG_H5_LSB_REG);
	msb = bmp280_read_register(i2c, BMP280_DIG_H5_MSB_REG);
	lsb &= 0xf0; 
	lsb = lsb >> 4;
	msb = msb << 4;
	dig_H5 = msb+lsb;
	lsb = bmp280_read_register(i2c, BMP280_DIG_H6_REG);
	dig_H6 = (int16_t)lsb;
	
	bmp280_reset(bmp280);

	uint8_t Temp = 0x11;												//hum
	bmp280_write_register(i2c, BMP280_CTRL_HUM, Temp);
	
	ctrlmeas_reg = 0X93; // bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
	config_reg = 0; // config_reg = bmp280->t_sb << 5 | bmp280->filter_coefficient << 2;
		
	bmp280_write_register(i2c, BMP280_CTRLMEAS_REG, ctrlmeas_reg);
	bmp280_write_register(i2c, BMP280_CONFIG_REG, config_reg);
	
	delay_ms(100);
	
	bmp280->valid = true;
	
	return bmp280;
}

void bmp280_reset(struct bmp280 *bmp280)
{
	bmp280_write_register(bmp280->i2c, BMP280_RESET_REG, BMP280_RESET_VALUE);
}

void bmp280_set_standby_time(struct bmp280 *bmp280, BMP280_T_SB t_standby)
{
	uint8_t config_reg;
	
	bmp280->t_sb = t_standby;
	config_reg = bmp280->t_sb << 5 | bmp280->filter_coefficient << 2;
	
	bmp280_write_register(bmp280->i2c, BMP280_CONFIG_REG, config_reg);
}

void bmp280_set_work_mode(struct bmp280 *bmp280, BMP280_WORK_MODE mode)
{
	uint8_t ctrlmeas_reg;
	
	bmp280->mode = mode;
	ctrlmeas_reg = bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
	
	bmp280_write_register(bmp280->i2c, BMP280_CTRLMEAS_REG, ctrlmeas_reg);
}

void bmp280_set_temperature_oversampling_mode(struct bmp280 *bmp280, BMP280_T_OVERSAMPLING t_osl)
{
	uint8_t ctrlmeas_reg;
	
	bmp280->t_oversampling = t_osl;
	ctrlmeas_reg = bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
	
	bmp280_write_register(bmp280->i2c, BMP280_CTRLMEAS_REG, ctrlmeas_reg);
}

void bmp280_set_pressure_oversampling_mode(struct bmp280 *bmp280, BMP280_P_OVERSAMPLING p_osl)
{
	uint8_t ctrlmeas_reg;
	
	bmp280->t_oversampling = p_osl;
	ctrlmeas_reg = bmp280->t_oversampling << 5 | bmp280->p_oversampling << 2 | bmp280->mode;
	
	bmp280_write_register(bmp280->i2c, BMP280_CTRLMEAS_REG, ctrlmeas_reg);
}

void bmp280_set_filter_mode(struct bmp280 *bmp280, BMP280_FILTER_COEFFICIENT f_coefficient)
{
	uint8_t config_reg;
	
	bmp280->filter_coefficient = f_coefficient;
	config_reg = bmp280->t_sb << 5 | bmp280->filter_coefficient << 2;
	
	bmp280_write_register(bmp280->i2c, BMP280_CONFIG_REG, config_reg);
}

/* Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC. */
static double bmp280_compensate_temperature_double(struct bmp280 *bmp280, int32_t adc_T)
{
	double var1, var2, temperature;
	
	var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0)
	* ((double) dig_T2);
	var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)
	* (((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0))
	* ((double) dig_T3);
	bmp280->t_fine = (int32_t) (var1 + var2);
	temperature = (var1 + var2) / 5120.0;
	
	return temperature;
}


/* Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa */
static double bmp280_compensate_pressure_double(struct bmp280 *bmp280, int32_t adc_P)
{
	double var1, var2, pressure;
	
	var1 = ((double) bmp280->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double) dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0
	+ ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);
	
	if (var1 == 0.0) {
		return 0; // avoid exception caused by division by zero
	}
	
	pressure = 1048576.0 - (double) adc_P;
	pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * pressure * pressure / 2147483648.0;
	var2 = pressure * ((double) dig_P8) / 32768.0;
	pressure = pressure + (var1 + var2 + ((double) dig_P7)) / 16.0;
	
	return pressure;
}

#if 0
static int32_t bmp280_compensate_temperature_int32(struct bmp280 *bmp280, int32_t adc_T)
{
	int32_t var1, var2, temperature;
	
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	bmp280->t_fine = var1 + var2;
	temperature = (bmp280->t_fine * 5 + 128) >> 8;
	
	return temperature;
}
static uint32_t bmp280_compensate_pressure_int64(struct bmp280 *bmp280, int32_t adc_P)
{
	int64_t var1, var2, pressure;
	
	var1 = ((int64_t)bmp280->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	
	pressure = 1048576-adc_P;
	pressure = (((pressure<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (pressure>>13) * (pressure>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * pressure) >> 19;
	pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	
	return (uint32_t)pressure;
}
#endif

/* Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC. */
double bmp280_get_temperature(struct bmp280 *bmp280)
{
	uint8_t lsb, msb, xlsb;
	int32_t adc_T;
	double temperature;
	
	xlsb = bmp280_read_register(bmp280->i2c, BMP280_TEMPERATURE_XLSB_REG);
	lsb = bmp280_read_register(bmp280->i2c, BMP280_TEMPERATURE_LSB_REG);
	msb = bmp280_read_register(bmp280->i2c, BMP280_TEMPERATURE_MSB_REG);
	
	adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4);
	temperature = bmp280_compensate_temperature_double(bmp280, adc_T);
	
	return temperature;
}
double bmp280_get_humidity(struct bmp280 *bmp280)
{
	uint8_t lsb, msb, xlsb;
	int32_t adc_T;
	double temperature;
	
	lsb = bmp280_read_register(bmp280->i2c, BMP280_HUMIDITY_LSB_REG);
	msb = bmp280_read_register(bmp280->i2c, BMP280_HUMIDITY_MSB_REG);
	
	adc_T = (msb << 8) | lsb;
	temperature = bme280_compensate_H_double(bmp280, adc_T);
	
	return temperature;
}
/* Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa */
double bmp280_get_pressure(struct bmp280 *bmp280)
{
	uint8_t lsb, msb, xlsb;
	int32_t adc_P;
	double pressure;
	
	
	xlsb = bmp280_read_register(bmp280->i2c, BMP280_PRESSURE_XLSB_REG);
	lsb = bmp280_read_register(bmp280->i2c, BMP280_PRESSURE_LSB_REG);
	msb = bmp280_read_register(bmp280->i2c, BMP280_PRESSURE_MSB_REG);
	
	adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4);
	pressure = bmp280_compensate_pressure_double(bmp280, adc_P);
	
	return pressure;
}
// Returns humidity in %RH as unsigned 32 bit integer Q22.10 format (22 and 10 fractional bits).
// Output value of “47 445” represents 47445/1024 = 46.333 %RH
double bme280_compensate_H_double(struct bmp280 *bmp280, int32_t adc_H)
{
	double var_H;
	var_H = ((( double )bmp280->t_fine) - 76800.0);
	var_H = (adc_H- ((( double )dig_H4) * 64.0 + (( double )dig_H5) / 16384.0 * var_H)) * ((( double )dig_H2) / 65536.0 * (1.0 + (( double )dig_H6) / 67108864.0 * var_H *(1.0 + (( double )dig_H3) / 67108864.0 * var_H)));
	var_H = var_H * (1.0 - (( double )dig_H1) * var_H / 524288.0);
	
	if (var_H > 100.0)
	var_H = 100.0;
	else if (var_H < 0.0)
	var_H = 0.0;
	return var_H;
}
/**
* 仅在BMP280被设置为normal mode后，
* 可使用该接口直接读取温度和气压。
*/
void bmp280_get_temperature_and_pressure(struct bmp280 *bmp280, double *temperature, double *pressure)
{
	*temperature = bmp280_get_temperature(bmp280);
	*pressure = bmp280_get_pressure(bmp280);
}

/**
* 当BMP280被设置为forced mode后，
* 可使用该接口直接读取温度和气压。
*/
void bmp280_forced_mode_get_temperature_and_pressure(struct bmp280 *bmp280, double *temperature, double *pressure)
{
	bmp280_set_work_mode(bmp280, BMP280_FORCED_MODE);
	
	delay_ms(100);
	
	bmp280_get_temperature_and_pressure(bmp280, temperature, pressure);
}

/**
* 此demo使用forced mode以1s为周期，
* 对温度和气压进行数据采集并打印。
*/