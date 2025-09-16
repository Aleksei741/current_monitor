//******************************************************************************
//include
//******************************************************************************
#include "ina226.h"
#include "i2c1_bsp.h"
#include "stm32f4xx_hal.h"
//******************************************************************************
// Секция определения констант
//******************************************************************************
#define REG_CONFIG       0x00
#define REG_SHUNT_VOLT   0x01
#define REG_BUS_VOLT     0x02
#define REG_POWER        0x03
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05
//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------
static uint8_t Addr[INA226_COUNT] = { 0x40, 0x41, 0x44, 0x45 };
static float Rshunt[INA226_COUNT] = { 0.1f, 0.1f, 0.1f, 0.1f };
static float CurrentLSB[INA226_COUNT] =
		{ 0.00025f, 0.00025f, 0.00025f, 0.00025f }; // 0.00025 A/бит
static double current[INA226_COUNT] = { 0 }; // Текущий ток в А
static uint32_t current_na[INA226_COUNT] = { 0 }; // Текущий ток в А
static uint8_t ina226_cnt = 0; // Текущий номер опрашиваемой микросхемы
static int ina226_stage = 0; // Состояние автомата опроса
static ina226_status_t ina226_status = INA226_OK; // Статус последней операции
static uint8_t rx_buf[INA226_COUNT][2];
static uint32_t timeWatchdog = 0; // Таймаут для watchdog
uint8_t reg;
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
static void callback(i2c1_ret_t status);
//******************************************************************************
// Секция описания функций
//******************************************************************************
// Функция для инициализации одного INA226
void INA226_Init(void) {
	I2C1_Init();
	I2C1_SetCallback(callback);
}
//------------------------------------------------------------------------------
// Функция для установки калибровки всех INA226
int INA226_SetCalibration(const uint8_t *addr, const float *rshunt,
		const float *current_lsb) {
	int status = 0;
	for (int i = 0; i < INA226_COUNT; i++) {
		Addr[i] = addr[i];
		Rshunt[i] = rshunt[i];
		CurrentLSB[i] = current_lsb[i];

		uint16_t cal = (uint16_t) ((0.00512f) / (Rshunt[i] * CurrentLSB[i]));
		uint8_t buf[3];
		buf[0] = REG_CALIBRATION;
		buf[1] = (uint8_t) (cal >> 8);
		buf[2] = (uint8_t) (cal & 0xFF);
		int res = I2C1_Write(Addr[i], buf, 3, 100);
		if (res != 0)
			status = res; // сохраняем ошибку, если была
	}
	return status;
}
//------------------------------------------------------------------------------
void INA226_start_reade(void) {
	ina226_cnt = 0;
	ina226_stage = 0;
	reg = REG_CURRENT;
	ina226_status = INA226_OK; // Сброс статуса
	timeWatchdog = HAL_GetTick() + INA226_I2C_TIMEOUT_MS; // Сброс таймаута
	I2C1_Write_IT(Addr[ina226_cnt], &reg, 1);
}
//------------------------------------------------------------------------------
ina226_status_t INA226_get_status(void) {
	if (timeWatchdog < HAL_GetTick() && ina226_status == INA226_OK) {
		ina226_status = INA226_TIMEOUT; // Таймаут
		I2C1_DeInit();
		I2C1_Init();
	}
	return ina226_status;
}
//------------------------------------------------------------------------------
// Чтение данных
void INA226_ReadCurrents(float *current_) {
	for (int i = 0; i < INA226_COUNT; i++) {
		current_[i] = (float) current[i];
	}
}
//------------------------------------------------------------------------------
void INA226_ReadCurrents_na(uint32_t *current_) {
	for (int i = 0; i < INA226_COUNT; i++) {
		current_[i] = current_na[i];
	}
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Колбэк для обработки событий I2C
static void callback(i2c1_ret_t status) {
	if (status == I2C1_ERR) {
		ina226_stage = 0;
		ina226_status = INA226_ERR; // Ошибка
		return;
	} else if (status == I2C1_TX_CPLT) {
		// Завершена передача, начинаем прием
		if (ina226_stage == 0) {
			ina226_stage = 1;
			I2C1_Read_IT(Addr[ina226_cnt], rx_buf[ina226_cnt], 2);
		}
	} else if (status == I2C1_RX_CPLT) {
		// Завершен прием, обрабатываем данные
		if (ina226_stage == 1) {
			int16_t raw = (rx_buf[ina226_cnt][0] << 8) | rx_buf[ina226_cnt][1];
			current[ina226_cnt] = (double) raw * (double) CurrentLSB[ina226_cnt]
					* 1000.0; // в мА
			current_na[ina226_cnt] = (uint32_t) (current[ina226_cnt]
					* 1000000.0f); // в нА

			// Переходим к следующему INA226
			ina226_cnt++;
			if (ina226_cnt < INA226_COUNT) {
				ina226_stage = 0;
				reg = REG_CURRENT;
				I2C1_Write_IT(Addr[ina226_cnt], &reg, 1);
			} else {
				// Все микросхемы опрошены
				ina226_stage = 0;
				ina226_cnt = 0;
				ina226_status = INA226_CPLT; // Опрос завершен
			}
		}
	}
}
