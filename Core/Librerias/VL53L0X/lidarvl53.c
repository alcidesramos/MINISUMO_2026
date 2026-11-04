/*
 * lidarvl53.c
 *
 *  Created on: Jul 20, 2023
 *      Author: alcid
 */


#include "lidarvl53.h"

int16_t lidar_mm[LIDAR_N];
uint8_t lidar_ok[LIDAR_N];
uint8_t lidar_nuevo[LIDAR_N];

static VL53L0X_Dev_t lidar_dev[LIDAR_N];   // un estado de la API por sensor
static uint8_t  lidar_presente[LIDAR_N];   // detectado en el arranque
static uint8_t  lidar_err[LIDAR_N];        // errores I2C consecutivos
static uint32_t lidar_t_ultimo[LIDAR_N];   // uwTick del ultimo dato (o reintento)
static uint8_t  mux_canal = 0xFF;          // canal activo del mux (0xFF = desconocido)

HAL_StatusTypeDef lidar_mux(uint8_t canal)
{
	if (canal == mux_canal) return HAL_OK;   // ya esta seleccionado

	uint8_t mask = (1 << canal);
	HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&LIDAR_I2C, LIDAR_MUX_ADDR, &mask, 1, 5);
	mux_canal = (st == HAL_OK) ? canal : 0xFF;
	return st;
}

// Si el periferico quedo en error (timeout, bus error, arbitraje) lo reinicia.
static void lidar_i2c_recupera(void)
{
	uint32_t e = LIDAR_I2C.ErrorCode;
	if ((LIDAR_I2C.State != HAL_I2C_STATE_READY) ||
	    (e & (HAL_I2C_ERROR_TIMEOUT | HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)))
	{
		HAL_I2C_DeInit(&LIDAR_I2C);
		HAL_I2C_Init(&LIDAR_I2C);   // Init se conserva en el handle
		mux_canal = 0xFF;
	}
}

uint8_t lidar_init_canal(uint8_t canal)
{
	VL53L0X_DEV Dev = &lidar_dev[canal];
	VL53L0X_Error st = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount;
	uint8_t isApertureSpads, VhvSettings, PhaseCal;

	lidar_ok[canal] = 0;
	lidar_err[canal] = 0;
	lidar_t_ultimo[canal] = uwTick;

	if (lidar_mux(canal) != HAL_OK) return 0;
	if (HAL_I2C_IsDeviceReady(&LIDAR_I2C, LIDAR_ADDR, 2, 5) != HAL_OK) return 0;

	// Si el sensor seguia midiendo (reinicio en caliente) primero se detiene.
	if (Dev->I2cHandle != NULL) VL53L0X_StopMeasurement(Dev);

	Dev->I2cHandle = &LIDAR_I2C;
	Dev->I2cDevAddr = LIDAR_ADDR;
	Dev->comms_type = 1;
	Dev->comms_speed_khz = 400;

	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_DataInit(Dev);
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_StaticInit(Dev);
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);

	// Limites de validez (sigma y senal) segun el perfil. VCSEL se deja en 14/10 (por defecto),
	// que es el adecuado para distancias cortas y no requiere recalibrar la fase.
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(LIDAR_SIGNAL_MCPS * 65536));
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(LIDAR_SIGMA_MM * 65536));
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, LIDAR_BUDGET_US);

	// Continuo back-to-back: una medida nueva cada timing budget, sin esperas extra.
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	if (st == VL53L0X_ERROR_NONE) st = VL53L0X_StartMeasurement(Dev);

	if (st != VL53L0X_ERROR_NONE) return 0;

	lidar_mm[canal] = lidar_max_mm;
	lidar_ok[canal] = 1;
	lidar_t_ultimo[canal] = uwTick;
	return 1;
}

uint8_t lidar_init_todos(void)
{
	uint8_t mascara = 0;

	for (uint8_t c = 0; c < LIDAR_N; c++)
	{
		lidar_mm[c] = lidar_max_mm;
		lidar_nuevo[c] = 0;
		lidar_presente[c] = lidar_init_canal(c);
		if (lidar_presente[c]) mascara |= (1 << c);
	}
	return mascara;
}

// Lee un sensor si tiene dato listo. 1 = dato nuevo, 0 = aun no, -1 = error I2C.
static int8_t lidar_lee_canal(uint8_t c)
{
	VL53L0X_DEV Dev = &lidar_dev[c];
	VL53L0X_RangingMeasurementData_t data;
	uint8_t listo = 0;
	int16_t mm;

	if (lidar_mux(c) != HAL_OK) return -1;
	if (VL53L0X_GetMeasurementDataReady(Dev, &listo) != VL53L0X_ERROR_NONE) return -1;
	if (!listo) return 0;
	if (VL53L0X_GetRangingMeasurementData(Dev, &data) != VL53L0X_ERROR_NONE) return -1;
	if (VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR) != VL53L0X_ERROR_NONE) return -1;

	// Medida invalida (sin objetivo, senal baja, sigma alto) -> "sin objetivo".
	if ((data.RangeStatus != 0) || (data.RangeMilliMeter == 0) || (data.RangeMilliMeter >= lidar_max_mm))
		mm = lidar_max_mm;
	else
		mm = (int16_t)data.RangeMilliMeter;

	// Filtro: los saltos grandes (aparece/desaparece el objetivo) pasan directo,
	// los cambios pequenos se suavizan con EMA para quitar ruido.
	int16_t ant = lidar_mm[c];
	int16_t dif = mm - ant;
	if ((mm == lidar_max_mm) || (ant == lidar_max_mm) || (dif > LIDAR_SALTO_MM) || (dif < -LIDAR_SALTO_MM))
		lidar_mm[c] = mm;
	else
		lidar_mm[c] = ant + (int16_t)(LIDAR_ALPHA * (float)dif);

	return 1;
}

uint8_t lidar_actualiza(void)
{
	uint8_t nuevos = 0;

	for (uint8_t c = 0; c < LIDAR_N; c++)
	{
		if (!lidar_presente[c]) continue;

		if (lidar_ok[c])
		{
			int8_t r = lidar_lee_canal(c);
			if (r > 0)
			{
				lidar_err[c] = 0;
				lidar_t_ultimo[c] = uwTick;
				lidar_nuevo[c] = 1;
				nuevos++;
				continue;
			}
			if (r < 0)
			{
				lidar_i2c_recupera();
				if (++lidar_err[c] >= LIDAR_MAX_ERR) lidar_ok[c] = 0;
			}
			if ((uwTick - lidar_t_ultimo[c]) > LIDAR_TIMEOUT_MS) lidar_ok[c] = 0;
		}

		// Sensor colgado o desconectado: reintento como maximo cada LIDAR_TIMEOUT_MS.
		if (!lidar_ok[c] && ((uwTick - lidar_t_ultimo[c]) > LIDAR_TIMEOUT_MS))
		{
			lidar_mm[c] = lidar_max_mm;
			lidar_init_canal(c);   // actualiza lidar_t_ultimo aunque falle
		}
	}
	return nuevos;
}

float lidar_cm(uint8_t canal)
{
	return ((float)lidar_mm[canal] / 10.0f);
}
