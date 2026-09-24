/*
 * BNO055.H
 *
 *  Created on: Oct 25, 2023
 *      Author: alcid
 */

#ifndef LIBRERIAS_BNO055_H_
#define LIBRERIAS_BNO055_H_
#include "math.h"
#include "main.h"


#define  BNO055_I2C   hi2c2



// Dirección I2C del sensor BNO055
#define BNO055_I2C_ADDRESS 0x28<<1
//#define BNO055_I2C_ADDRESS 0x29<<1

/*

 * en la linea 155 de este archivo
 //ojo programar aqui orientacion del sensor
// Valores predeterminados para configuración y signo de ejes
#define BNO055_AXIS_MAP_CONFIG_DEFAULT  0x24
#define BNO055_AXIS_MAP_SIGN_DEFAULT    0x00

 antes del while  llamar a   BNO055_INIT();
 y si desea BNO055_Calibra_Pitch_Roll();  o  BNO055_Calibra();
 en el while
 llamar funciones deseadas, ejemplo
 BNO055_ReadEuler();


void BNO055_ReadAcceleration();
BNO055_ReadMagnetometro();
BNO055_ReadGyro();

BNO055_ReadAccelerationlineal();
BNO055_ReadAccelerationgravedad();

BNO055_ReadQuaternion();
QuaternionToEulerAngles();

abajo mirar las variables donde quedan guardado cada informacion.

 */

//aceleciones
extern float ax,ay,az;
//variables de los giros
extern float gx,gy,gz;

//magnetometros
extern float max,may,maz;
//aceleraciones lineal
extern float alx,aly,alz;

//aceleracion teniendo en cuneta la gravedad
extern float agx,agy,agz;


//angulos de euler
extern float pitch,roll,yaw;
//quaternion
extern float q1,q2,q3,q4;
extern float pitch1,roll1,yaw1;

extern float pitch_offset,roll_offset;


// Registros de control
#define BNO055_PAGE_ID          0x07
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_PWR_MODE         0x3E

// Registros de calibración
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
// Continúa con los registros de calibración para el magnetómetro y el giroscopio.

// Registros de datos de sensores
#define BNO055_TEMP             0x34
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D


//registro para la aceleracion lineal
#define BNO055_LINEAR_ACC_DATA_X_LSB   0x28
#define BNO055_LINEAR_ACC_DATA_X_MSB   0x29
#define BNO055_LINEAR_ACC_DATA_Y_LSB   0x2A
#define BNO055_LINEAR_ACC_DATA_Y_MSB   0x2B
#define BNO055_LINEAR_ACC_DATA_Z_LSB   0x2C
#define BNO055_LINEAR_ACC_DATA_Z_MSB   0x2D

//registro para las gravedades
#define BNO055_GRAVITY_DATA_X_LSB   0x2E
#define BNO055_GRAVITY_DATA_X_MSB   0x2F
#define BNO055_GRAVITY_DATA_Y_LSB   0x30
#define BNO055_GRAVITY_DATA_Y_MSB   0x31
#define BNO055_GRAVITY_DATA_Z_LSB   0x32
#define BNO055_GRAVITY_DATA_Z_MSB   0x33


// Registros de datos del giroscopio
#define BNO055_GYRO_DATA_X_LSB 0x14
#define BNO055_GYRO_DATA_X_MSB 0x15
#define BNO055_GYRO_DATA_Y_LSB 0x16
#define BNO055_GYRO_DATA_Y_MSB 0x17
#define BNO055_GYRO_DATA_Z_LSB 0x18
#define BNO055_GYRO_DATA_Z_MSB 0x19

// Registros de datos del magnetómetro
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13


// Registros de orientación (Euler)
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F

// Registros de datos de cuaterniones
#define BNO055_QUATERNION_DATA_W_LSB 0x20
#define BNO055_QUATERNION_DATA_W_MSB 0x21
#define BNO055_QUATERNION_DATA_X_LSB 0x22
#define BNO055_QUATERNION_DATA_X_MSB 0x23
#define BNO055_QUATERNION_DATA_Y_LSB 0x24
#define BNO055_QUATERNION_DATA_Y_MSB 0x25
#define BNO055_QUATERNION_DATA_Z_LSB 0x26
#define BNO055_QUATERNION_DATA_Z_MSB 0x27

// registros de estados
#define BNO055_CALIB_STAT       0x35
#define BNO055_SELFTEST = 0X36
#define BNO055_INTR_STAT = 0X37
#define BNO055_SYS_STATUS       0x39

// Registros de configuración de ejes
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
//ojo programar aqui orientacion del sensor
// Valores predeterminados para configuración y signo de ejes
#define BNO055_AXIS_MAP_CONFIG_DEFAULT  0x24
#define BNO055_AXIS_MAP_SIGN_DEFAULT    0x00


//registos de configuracion imu
#define  BNO055_ACC_CONFIG 0x08
#define  BNO055_OPR_MODE 0x3D

#define BNO055_GYRO_CONFIG0 0x0A
#define BNO055_GYRO_CONFIG1 0x0B

// Definiciones de escalas para el acelerómetro
#define ACCEL_SCALE_2G  0x00  // ±2g
#define ACCEL_SCALE_4G  0x01  // ±4g
#define ACCEL_SCALE_8G  0x02  // ±8g
#define ACCEL_SCALE_16G 0x03  // ±16g

// Definiciones de escalas para el giroscopio
#define GYRO_SCALE_2000DPS  0x00  // ±2000 grados por segundo
#define GYRO_SCALE_1000DPS  0x01  // ±1000 grados por segundo
#define GYRO_SCALE_500DPS   0x02  // ±500 grados por segundo
#define GYRO_SCALE_250DPS   0x03  // ±250 grados por segundo
#define GYRO_SCALE_125DPS   0x04  // ±125 grados por segundo
//offset
#define BNO055_ACC_OFFSET_X_LSB 0x55 // Registros para offset del acelerómetro
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A

#define BNO055_GYR_OFFSET_X_LSB 0x67 // Registros para offset del giroscopio
#define BNO055_GYR_OFFSET_X_MSB 0x68
#define BNO055_GYR_OFFSET_Y_LSB 0x69
#define BNO055_GYR_OFFSET_Y_MSB 0x6A
#define BNO055_GYR_OFFSET_Z_LSB 0x6B
#define BNO055_GYR_OFFSET_Z_MSB 0x6C

#define BNO055_MAG_OFFSET_X_LSB 0x71 // Registros para offset del magnetómetro
#define BNO055_MAG_OFFSET_X_MSB 0x72
#define BNO055_MAG_OFFSET_Y_LSB 0x73
#define BNO055_MAG_OFFSET_Y_MSB 0x74
#define BNO055_MAG_OFFSET_Z_LSB 0x75
#define BNO055_MAG_OFFSET_Z_MSB 0x76


extern I2C_HandleTypeDef  BNO055_I2C;


void BNO055_INIT();
void BNO055_ReadAcceleration();
void BNO055_ReadAccelerationlineal();
void BNO055_ReadAccelerationgravedad();
void BNO055_ReadMagnetometro();
void BNO055_ReadGyro();
void BNO055_ReadEuler();
void BNO055_ReadQuaternion();
void QuaternionToEulerAngles();
void BNO055_Offset_Carga();
void BNO055_Offset_Carga_Promedio(uint16_t cuantos);



#endif /* LIBRERIAS_BNO055_H_ */
