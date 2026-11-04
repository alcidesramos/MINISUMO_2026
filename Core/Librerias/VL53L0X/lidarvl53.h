/*
 * lidarvl53.h
 *
 *  Created on: Jul 20, 2023
 *      Author: alcid
 *
 *  8 sensores VL53L0X (todos en 0x52) detras de un multiplexor I2C TCA9548A (0x70).
 *  Los sensores trabajan en modo continuo: los 8 miden en paralelo y el MCU solo
 *  hace un barrido rapido (sin retardos) recogiendo los datos que esten listos.
 */

#ifndef LIBERRIAS_VL53L0X_LIDARVL53_H_
#define LIBERRIAS_VL53L0X_LIDARVL53_H_

#include "vl53l0x_api.h"
#include "main.h"

#define  LIDAR_I2C  hi2c2
extern I2C_HandleTypeDef  LIDAR_I2C;

#define LIDAR_N         8            // numero de canales del multiplexor
#define LIDAR_ADDR      0x52         // direccion (8 bits) de todos los VL53L0X
#define LIDAR_MUX_ADDR  (0x70 << 1)  // direccion (8 bits) del TCA9548A

// ================== PERFIL DE MEDICION ==================
// El timing budget es tambien el periodo de actualizacion de cada sensor.
#define LIDAR_PERFIL_RAPIDO      0   // 20 ms (~50 Hz), mas ruido
#define LIDAR_PERFIL_BALANCEADO  1   // 33 ms (~30 Hz), recomendado < 1 m
#define LIDAR_PERFIL_PRECISO     2   // 50 ms (~20 Hz), menos ruido

#ifndef LIDAR_PERFIL
#define LIDAR_PERFIL  LIDAR_PERFIL_BALANCEADO
#endif

#if   LIDAR_PERFIL == LIDAR_PERFIL_RAPIDO
#define LIDAR_BUDGET_US   20000
#define LIDAR_SIGNAL_MCPS 0.25f      // limite minimo de senal de retorno
#define LIDAR_SIGMA_MM    32         // limite maximo de sigma (incertidumbre)
#elif LIDAR_PERFIL == LIDAR_PERFIL_PRECISO
#define LIDAR_BUDGET_US   50000
#define LIDAR_SIGNAL_MCPS 0.25f
#define LIDAR_SIGMA_MM    18
#else
#define LIDAR_BUDGET_US   33000
#define LIDAR_SIGNAL_MCPS 0.25f
#define LIDAR_SIGMA_MM    18
#endif

#define LIDAR_PERIODO_MS  (LIDAR_BUDGET_US / 1000)

// ================== PROCESADO ==================
#define lidar_max_mm      1000   // sin objetivo o mas lejos de 1 m -> 100 cm
#define LIDAR_ALPHA       0.6f   // filtro EMA: 1.0 = sin filtro, menor = mas suave
#define LIDAR_SALTO_MM    150    // saltos mayores se aceptan sin filtrar (deteccion inmediata)
#define LIDAR_TIMEOUT_MS  250    // sin datos en este tiempo -> se reinicia el sensor
#define LIDAR_MAX_ERR     5      // errores I2C seguidos -> se reinicia el sensor

extern int16_t lidar_mm[LIDAR_N];    // distancia filtrada en mm (lidar_max_mm = sin objetivo)
extern uint8_t lidar_ok[LIDAR_N];    // 1 = sensor presente e inicializado
extern uint8_t lidar_nuevo[LIDAR_N]; // 1 = llego un dato nuevo (lo borra el usuario)

HAL_StatusTypeDef lidar_mux(uint8_t canal);
uint8_t lidar_init_canal(uint8_t canal);
uint8_t lidar_init_todos(void);
uint8_t lidar_actualiza(void);
float   lidar_cm(uint8_t canal);

#endif /* LIBERRIAS_VL53L0X_LIDARVL53_H_ */
