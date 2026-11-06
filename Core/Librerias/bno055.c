/*
 * BNO055.C
 *
 *  Created on: Oct 24, 2023
 *      Author: alcid
 */


#include "bno055.h"
#define M_PI 3.14159265358979323846


//varibles de acelelacion
float ax,ay,az;

//aceleraciones lineal
 float alx,aly,alz;

//aceleracion teniendo en cuneta la gravedad
 float agx,agy,agz;

//variables de los giros
 float gx,gy,gz;
//magnetometros
extern float max,may,maz;

//angulos de euler
float pitch,roll,yaw;


//quaternion
extern float q1,q2,q3,q4;
extern float pitch1,roll1,yaw1;

// Variables globales para los offsets de los sensores
int16_t acc_offset_x;
int16_t acc_offset_y;
int16_t acc_offset_z;

int16_t gyr_offset_x;
int16_t gyr_offset_y;
int16_t gyr_offset_z;

int16_t mag_offset_x;
int16_t mag_offset_y;
int16_t mag_offset_z;


typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t acc_radius;
    int16_t	mag_radius;
} BNO055_CalibrationData;

BNO055_CalibrationData calibrame;

void BNO055_INIT()
{

    // Configurar axis_map_config y axis_map_sign
    uint8_t axis_map_config = BNO055_AXIS_MAP_CONFIG_DEFAULT;
    uint8_t axis_map_sign = BNO055_AXIS_MAP_SIGN_DEFAULT;

    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_AXIS_MAP_CONFIG, I2C_MEMADD_SIZE_8BIT, &axis_map_config, 1, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_AXIS_MAP_SIGN, I2C_MEMADD_SIZE_8BIT, &axis_map_sign, 1, 100);

    //escala del acelelometro
    uint8_t escala= ACCEL_SCALE_4G<< 3;
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_CONFIG, I2C_MEMADD_SIZE_8BIT, &escala, 1, HAL_MAX_DELAY);

     //escala del giro


    // Configurar el sensor para el modo fusion
    uint8_t power_mode = 0x00;  // Modo normal
    uint8_t operation_mode = 0x08;  // Modo fusion IMU

    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_PWR_MODE, I2C_MEMADD_SIZE_8BIT, &power_mode, 1, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &operation_mode, 1, 100);

}




// Función para leer aceleraciones en m/s^2
void BNO055_ReadAcceleration()
{
    uint8_t data[6];
    //si falla la lectura conserva los valores anteriores (data quedaria con basura)
    if (HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK) return;

    int16_t raw_ax = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_ay = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_az = (int16_t)(data[5] << 8 | data[4]);

    // Convertir datos en bruto a m/s^2

    ax = (float)raw_ax / 100.0;
    ay = (float)raw_ay / 100.0;
    az = (float)raw_az / 100.0;
}

// Función para leer aceleraciones lineales
void BNO055_ReadAccelerationlineal()
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_LINEAR_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

    int16_t raw_alx = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_aly = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_alz = (int16_t)(data[5] << 8 | data[4]);

    // Convertir datos en bruto a m/s^2
    alx = (float)raw_alx / 100.0;
    aly = (float)raw_aly / 100.0;
    alz = (float)raw_alz / 100.0;
}


void BNO055_ReadAccelerationgravedad()
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GRAVITY_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 1000);

    int16_t raw_agx = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_agy = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_agz = (int16_t)(data[5] << 8 | data[4]);

    agx = (float)raw_agx / 100.0;
    agy = (float)raw_agy / 100.0;
    agz = (float)raw_agz / 100.0;
}



// Función para leer velocidades angulares de los giroscopios en grados por segundo
void BNO055_ReadGyro()
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYRO_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

    int16_t raw_gx = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_gy = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_gz = (int16_t)(data[5] << 8 | data[4]);

    gx = (float)raw_gx / 16.0;
    gy = (float)raw_gy / 16.0;
    gz = (float)raw_gz / 16.0;
}


// Función para leer magnetometros
void BNO055_ReadMagnetometro()
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);

    int16_t raw_max = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_may = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_maz = (int16_t)(data[5] << 8 | data[4]);

    max = (float)raw_max / 16.0;
    may = (float)raw_may / 16.0;
    maz = (float)raw_maz / 16.0;
}

void BNO055_LEE_DATOS_CRUDOS()
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    int16_t raw_ax = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_ay = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_az = (int16_t)(data[5] << 8 | data[4]);

    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYRO_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    int16_t raw_gx = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_gy = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_gz = (int16_t)(data[5] << 8 | data[4]);

    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
    int16_t raw_max = (int16_t)(data[1] << 8 | data[0]);
    int16_t raw_may = (int16_t)(data[3] << 8 | data[2]);
    int16_t raw_maz = (int16_t)(data[5] << 8 | data[4]);


}

void BNO055_LEE_DATOS_CRUDOS_Prom(int cuantos)
{
    int16_t sum_raw_ax = 0;
    int16_t sum_raw_ay = 0;
    int16_t sum_raw_az = 0;

    int16_t sum_raw_gx = 0;
    int16_t sum_raw_gy = 0;
    int16_t sum_raw_gz = 0;

    int16_t sum_raw_max = 0;
    int16_t sum_raw_may = 0;
    int16_t sum_raw_maz = 0;

    uint8_t data[6];

    for (int i = 0; i < cuantos; i++)
    {
        HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
        sum_raw_ax += (int16_t)(data[1] << 8 | data[0]);
        sum_raw_ay += (int16_t)(data[3] << 8 | data[2]);
        sum_raw_az += (int16_t)(data[5] << 8 | data[4]);

        HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYRO_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
        sum_raw_gx += (int16_t)(data[1] << 8 | data[0]);
        sum_raw_gy += (int16_t)(data[3] << 8 | data[2]);
        sum_raw_gz += (int16_t)(data[5] << 8 | data[4]);

        HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100);
        sum_raw_max += (int16_t)(data[1] << 8 | data[0]);
        sum_raw_may += (int16_t)(data[3] << 8 | data[2]);
        sum_raw_maz += (int16_t)(data[5] << 8 | data[4]);
    }

    // Calcula el promedio dividiendo la suma por 'cuantos'
    int16_t promedio_raw_ax = sum_raw_ax / cuantos;
    int16_t promedio_raw_ay = sum_raw_ay / cuantos;
    int16_t promedio_raw_az = sum_raw_az / cuantos;

    int16_t promedio_raw_gx = sum_raw_gx / cuantos;
    int16_t promedio_raw_gy = sum_raw_gy / cuantos;
    int16_t promedio_raw_gz = sum_raw_gz / cuantos;

    int16_t promedio_raw_max = sum_raw_max / cuantos;
    int16_t promedio_raw_may = sum_raw_may / cuantos;
    int16_t promedio_raw_maz = sum_raw_maz / cuantos;

    // Ahora tienes los valores promedio en las variables promedio_raw_ax, promedio_raw_ay, etc.
}

// Función para leer ángulos de Euler
void BNO055_ReadEuler()
{
    uint8_t data[6];
    //si falla la lectura conserva los valores anteriores (data quedaria con basura)
    if (HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_EUL_HEADING_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 100) != HAL_OK) return;

    yaw = (float)((int16_t)(data[1] << 8 | data[0])) / 16.0;
    roll = (float)((int16_t)(data[3] << 8 | data[2])) / 16.0;
    pitch = (float)((int16_t)(data[5] << 8 | data[4])) / 16.0;
}




// Función para leer cuaterniones
void BNO055_ReadQuaternion()
{
    uint8_t data[8];
    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_QUATERNION_DATA_W_LSB, I2C_MEMADD_SIZE_8BIT, data, 8, 100);

    q1 = (float)((int16_t)(data[1] << 8 | data[0])) / 16384.0;
    q2 = (float)((int16_t)(data[3] << 8 | data[2])) / 16384.0;
    q3 = (float)((int16_t)(data[5] << 8 | data[4])) / 16384.0;
    q4 = (float)((int16_t)(data[7] << 8 | data[6])) / 16384.0;
}

// Función para convertir un cuaternión a ángulos de orientación (pitch, roll y yaw)
// Función para convertir cuaterniones a ángulos de Euler (pitch, roll y yaw)
void QuaternionToEuler()
{
   // Normaliza el cuaternión
    float norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
    q4 /= norm;

    // Cálculo de los ángulos de Euler
    pitch1 = atan2(2 * (q2 * q3 + q1 * q4), 1 - 2 * (q3 * q3 + q4 * q4));
    roll1 = asin(2 * (q1 * q3 - q2 * q4));
    yaw1 = atan2(2 * (q1 * q2 + q3 * q4), 1 - 2 * (q2 * q2 + q3 * q3));
}

void BNO055_Offset_Carga()
{
    uint8_t data[18]; // Cada sensor tiene 6 bytes de offset, y hay 3 sensores en total

    // Cambiar el sensor al modo de configuración
        uint8_t config_mode = 0x00; // 0x00 para el modo de configuración
        HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &config_mode, 1, 100);


    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 18, 100);

    // Procesa los datos de offset para cada sensor
    acc_offset_x = (int16_t)(data[1] << 8 | data[0]);
    acc_offset_y = (int16_t)(data[3] << 8 | data[2]);
    acc_offset_z = (int16_t)(data[5] << 8 | data[4]);

    mag_offset_x = (int16_t)(data[7] << 8 | data[6]);
    mag_offset_y = (int16_t)(data[9] << 8 | data[8]);
    mag_offset_z = (int16_t)(data[11] << 8 | data[10]);

    gyr_offset_x = (int16_t)(data[13] << 8 | data[12]);
    gyr_offset_y = (int16_t)(data[15] << 8 | data[14]);
    gyr_offset_z = (int16_t)(data[17] << 8 | data[16]);
   //lo guarda en el registo offset
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&acc_offset_x, 2, 100);
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&acc_offset_y, 2, 100);
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&acc_offset_z, 2, 100);

       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYR_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&gyr_offset_x, 2, 100);
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&gyr_offset_y, 2, 100);
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&gyr_offset_z, 2, 100);

       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&mag_offset_x, 2, 100);
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&mag_offset_y, 2, 100);
       HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&mag_offset_z, 2, 100);


}

void BNO055_Offset_Carga_Promedio(uint16_t cuantos)
{
    int32_t sum_acc_offset_x = 0;
    int32_t sum_acc_offset_y = 0;
    int32_t sum_acc_offset_z = 0;
    int32_t sum_gyr_offset_x = 0;
    int32_t sum_gyr_offset_y = 0;
    int32_t sum_gyr_offset_z = 0;
    int32_t sum_mag_offset_x = 0;
    int32_t sum_mag_offset_y = 0;
    int32_t sum_mag_offset_z = 0;

    // Cambiar el sensor al modo de configuración
    uint8_t config_mode = 0x00; // 0x00 para el modo de configuración
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &config_mode, 1, 100);


    for (uint16_t i = 0; i < cuantos; i++)
    {
        uint8_t data[18];
        HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 18, 100);

        int16_t acc_offset_x = (int16_t)(data[1] << 8 | data[0]);
        int16_t acc_offset_y = (int16_t)(data[3] << 8 | data[2]);
        int16_t acc_offset_z = (int16_t)(data[5] << 8 | data[4]);
        int16_t mag_offset_x = (int16_t)(data[7] << 8 | data[6]);
        int16_t mag_offset_y = (int16_t)(data[9] << 8 | data[8]);
        int16_t mag_offset_z = (int16_t)(data[11] << 8 | data[10]);
        int16_t gyr_offset_x = (int16_t)(data[13] << 8 | data[12]);
        int16_t gyr_offset_y = (int16_t)(data[15] << 8 | data[14]);
        int16_t gyr_offset_z = (int16_t)(data[17] << 8 | data[16]);

        sum_acc_offset_x += acc_offset_x;
        sum_acc_offset_y += acc_offset_y;
        sum_acc_offset_z += acc_offset_z;
        sum_gyr_offset_x += gyr_offset_x;
        sum_gyr_offset_y += gyr_offset_y;
        sum_gyr_offset_z += gyr_offset_z;
        sum_mag_offset_x += mag_offset_x;
        sum_mag_offset_y += mag_offset_y;
        sum_mag_offset_z += mag_offset_z;
    }

    // Calcula el promedio dividiendo la suma por 'cuantos'
    int16_t avg_acc_offset_x = (int16_t)(sum_acc_offset_x / cuantos);
    int16_t avg_acc_offset_y = (int16_t)(sum_acc_offset_y / cuantos);
    int16_t avg_acc_offset_z = (int16_t)(sum_acc_offset_z / cuantos);
    int16_t avg_gyr_offset_x = (int16_t)(sum_gyr_offset_x / cuantos);
    int16_t avg_gyr_offset_y = (int16_t)(sum_gyr_offset_y / cuantos);
    int16_t avg_gyr_offset_z = (int16_t)(sum_gyr_offset_z / cuantos);
    int16_t avg_mag_offset_x = (int16_t)(sum_mag_offset_x / cuantos);
    int16_t avg_mag_offset_y = (int16_t)(sum_mag_offset_y / cuantos);
    int16_t avg_mag_offset_z = (int16_t)(sum_mag_offset_z / cuantos);

    // Guarda los valores promedio en los registros de offset del BNO055
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_acc_offset_x, 2, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_acc_offset_y, 2, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_acc_offset_z, 2, 100);

    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYR_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_gyr_offset_x, 2, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_gyr_offset_y, 2, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_gyr_offset_z, 2, 100);

    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_OFFSET_X_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_mag_offset_x, 2, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_mag_offset_y, 2, 100);
    HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&avg_mag_offset_z, 2, 100);

}


int dios()
{
	uint8_t system_status;

    HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDRESS,BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT, &system_status, 1, 100);
    HAL_Delay(100);
   return(	system_status);

}

