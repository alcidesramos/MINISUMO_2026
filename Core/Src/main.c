/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h5xx_hal.h"
#include "uart.h"
#include "puertos.h"
#include "delays.h"
#include "pinout1.h"
#include "adc_x.h"
#include "bits.h"
#include "built_in.h"
#include "servos.h"
#include "PWM_IN.h"
#include "UARTRX.h"
#include "bno055.h"
#include "ircodigo.h"
#include "parametros.h"

#include "lidarvl53.h"
#include "IR_ANA_SEN.h"


#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"

#include "I2C_SCAN_BUS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_NodeTypeDef Node_GPDMA2_Channel0;
DMA_QListTypeDef List_GPDMA2_Channel0;
DMA_HandleTypeDef handle_GPDMA2_Channel0;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
extern uint16_t pulso1,pulso2,pulso3,pulso4;
float ancho1=0,ancho2=0,ancho3=0,ancho4=0;   //en us. 0 = sin señal del radio
#define RC_MIN_US      800   //pulso valido del radio (TIM2 cuenta en us)
#define RC_MAX_US      2200
#define RC_TIMEOUT_MS  200   //sin pulsos validos este tiempo = señal perdida
#define RC_PULSOS_OK   5     //pulsos validos seguidos para aceptar la señal (unos 100 ms a 50 Hz)
#define RC_PERIODO_MAX_MS 60 //mas separados que esto no cuentan como seguidos
#define RC3_BAJO       1300  //canal 3 por debajo = manda el radio (interruptor abajo = 970)
#define RC3_ALTO       1700  //canal 3 por encima = para el radio y manda python (interruptor arriba = 2050). Entre los dos = transmisor apagado
#define RC3_MEDIO_MS   300   //tiempo en el medio para darlo por apagado
#define RC_ZONA_MUERTA 30    //us alrededor de 1500 que cuentan como mando centrado
#define PRUEBA_SOLO_RADIO 1  //1 = prueba: canal 3 elige reposo o control, canales 1 y 2 a los motores. 0 = programa normal
char texto[500];

//variables lidar

uint16_t milimetros;
uint16_t codigo;
float voltaje;
float distancia[LIDAR_N] = {0.0f};   // distancia en cm (lidar_max_mm/10 = sin objetivo)

uint32_t TICK_RC;

//kgiro y vgiro ahora estan en PARAM (parametros.h): se guardan en la flash
uint8_t imu_ok=0;   //1 = el BNO055 respondio al arrancar

enum estados{stop,run,test,automatico};   //MISMO orden que ESTADOS en minisumo_panel.py

uint8_t estado=stop;
uint8_t  flancoestado=run;
uint8_t  flancoestado_=run;   //al encender no arranca: hay que subir y bajar el canal 3 para mover



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_GPDMA2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    uint32_t tick_pulso;    //ultimo pulso con ancho valido
    uint32_t tick_valido;   //ultimo pulso aceptado como señal
    uint8_t  seguidos;      //pulsos validos seguidos
} CANAL_RC;

//lee un canal del radio. Un radio real manda pulsos validos sin parar; el ruido da pulsos
//sueltos que pueden caer en el rango. Por eso solo se acepta tras RC_PULSOS_OK pulsos seguidos.
//sin señal aceptada durante RC_TIMEOUT_MS: ancho=0 = señal perdida
static void Lee_canal_RC(uint8_t *flag, uint16_t pulso, CANAL_RC *c, float *ancho)
{
     if (*flag==1)
     {
    	 if ((pulso>=RC_MIN_US)&&(pulso<=RC_MAX_US))
    	 {
    		 if ((uwTick-c->tick_pulso)>RC_PERIODO_MAX_MS) c->seguidos=0;   //muy separado del anterior
    		 c->tick_pulso=uwTick;
    		 if (c->seguidos<RC_PULSOS_OK) c->seguidos++;

    		 if (c->seguidos>=RC_PULSOS_OK)
    		 {
    			 *ancho=pulso;
    			 c->tick_valido=uwTick;
    		 }
    	 }
    	 else c->seguidos=0;

    	 *flag=0;
     }

     if ((uwTick-c->tick_valido)>RC_TIMEOUT_MS) *ancho=0;
}

void LEE_PWM()
{
     static CANAL_RC rc1,rc2,rc3,rc4;
     static uint8_t senal_ok=0;   //1 = llegan los canales 1, 2 y 3
     static uint32_t tick_c3_fuera=0;   //ultima vez que el canal 3 NO estaba en el medio

     //canales cruzados: ancho1 (velocidad) se lee de la entrada 2, ancho2 (giro) de la entrada 1
     Lee_canal_RC(&flag_PWM_in2,pulso2,&rc1,&ancho1);
     Lee_canal_RC(&flag_PWM_in1,pulso1,&rc2,&ancho2);
     Lee_canal_RC(&flag_PWM_in3,pulso3,&rc3,&ancho3);
     Lee_canal_RC(&flag_PWM_in4,pulso4,&rc4,&ancho4);

     //transmisor apagado: el receptor sigue mandando cerca de 1500 en todos los canales.
     //el canal 3 nunca se queda en el medio con el radio encendido: eso delata el apagado.
     //se exige un tiempo minimo porque al mover el interruptor puede pasar un instante por el medio
     uint8_t c3_medio=((ancho3>RC3_BAJO)&&(ancho3<RC3_ALTO)) ? 1 : 0;
     if (!c3_medio) tick_c3_fuera=uwTick;

     if ((ancho1==0)||(ancho2==0)||(ancho3==0)||((uwTick-tick_c3_fuera)>RC3_MEDIO_MS))
     {
    	 if (senal_ok==1)                //se acaba de perder: para el robot una sola vez
    	 {                               //(asi no anula un RUN serial con el radio apagado)
    		 estado=stop;
    		 uartx_write_text(&huart1, "RADIO: apagado o sin senal\r\n");
    	 }
    	 senal_ok=0;
    	 flancoestado=run;
    	 flancoestado_=run;              //al volver no arranca: hay que subir y bajar el canal 3
     }
     else
     {
    	 senal_ok=1;

    	 if (ancho3<=RC3_BAJO) flancoestado=run;          //abajo = manda el radio
    	 else if (ancho3>=RC3_ALTO) flancoestado=stop;    //arriba = para el radio y manda python
    	 //en el medio (un instante, moviendo el interruptor) conserva el valor anterior

    	 if (flancoestado!=flancoestado_)
    	 {
    		 if(flancoestado==run)
    		 {
    			 estado=run;
    			 //queda en TRAMA: si el robot se mueve solo, muestra si el firmware lo ordeno
    			 sprintf(texto,"RADIO: arranca C1=%.0f C2=%.0f C3=%.0f\r\n",ancho1,ancho2,ancho3);
    			 uartx_write_text(&huart1, texto);
    		 }
    		 else estado=stop;

    		 flancoestado_=flancoestado;
    	 }
     }



/*
	  sprintf(texto,"PULSO1=%.1f\r\n\r\n",ancho1);
	  uartx_write_text(&huart1, texto);

	*/
	// el envio de ancho1..4 (teleplot) ahora esta en Manda_basicos, 5 veces por segundo
}

//aplica velocidad y giro a los motores, en us respecto al neutro (-500 a +500).
//giro se limita a vgiro y cada motor a vmax (tambien en los test); kgiro frena el motor que va mas rapido:
//  kgiro<1 frena el izquierdo, kgiro>1 frena el derecho, 1 = sin correccion
static void Motores_mueve(int16_t vel, int16_t giro)
{
    //limito velocidad giro
    if (giro>PARAM.vgiro)  giro=PARAM.vgiro;
    if (giro<-PARAM.vgiro) giro=-PARAM.vgiro;

    // aplica giro
    int16_t pwm_left  = 1500 + vel + giro;
    int16_t pwm_right = 1500 + vel - giro;

    // Limito cada motor a 1500 +- vmax (vmax=500: 1000-2000)
    int16_t pwm_min = 1500 - PARAM.vmax;
    int16_t pwm_max = 1500 + PARAM.vmax;

    if (pwm_left < pwm_min)  pwm_left = pwm_min;
    if (pwm_left > pwm_max)  pwm_left = pwm_max;

    if (pwm_right < pwm_min) pwm_right = pwm_min;
    if (pwm_right > pwm_max) pwm_right = pwm_max;

    //aplica factor de correccion a un solo motor (asi nunca se sale del tope)
    if (PARAM.kgiro<1.0f) pwm_left  = 1500 + (int16_t)((pwm_left-1500)*PARAM.kgiro);
    else                  pwm_right = 1500 + (int16_t)((pwm_right-1500)/PARAM.kgiro);

    SERVO_MICRO(&SERVO1, pwm_left);
    SERVO_MICRO(&SERVO2, pwm_right);
}

//=============================================================================
//             TEST: mueve el robot sin radio para calibrar kgiro y vgiro
// TRECTO=vel,ms/  TGIRO=dir,ms/  (STOP o el radio lo cortan en cualquier momento)
//=============================================================================
#define TEST_VEL_MAX   500     //us respecto al neutro
#define TEST_MS_MIN    100
#define TEST_MS_MAX    10000   //si se pierde el enlace, el robot para solo en este tiempo
#define TEST_RAMPA_MS  200     //arranque suave: un tiron hace patinar y desvia el robot

static int16_t  test_vel=0;
static int16_t  test_giro=0;
static uint32_t test_ms=0;
static uint32_t test_t0=0;

static void Test_inicia(int16_t vel, int16_t giro, uint32_t ms)
{
    test_vel=vel;
    test_giro=giro;
    test_ms=ms;
    test_t0=uwTick;
    estado=test;
}

//corre en el while mientras estado==test. No bloquea: kgiro y vgiro se pueden cambiar en vivo
static void Test_tarea(void)
{
    uint32_t t=uwTick-test_t0;

    if (t>=test_ms)
    {
        estado=stop;
        SERVO_MILI(&SERVO1,1.5);
        SERVO_MILI(&SERVO2,1.5);
        uartx_write_text(&huart1, "TEST: fin\r\n");
        return;
    }

    float f=(t<TEST_RAMPA_MS) ? (float)t/TEST_RAMPA_MS : 1.0f;
    Motores_mueve((int16_t)(test_vel*f), (int16_t)(test_giro*f));
}

//modo automatico: corre en el while mientras estado==automatico.
//Por ahora no hace nada: motores en neutro. Aqui se programa la rutina
static void Control_Automatico(void)
{
    SERVO_MILI(&SERVO1,1.5);
    SERVO_MILI(&SERVO2,1.5);
}

//busca "clave" en la trama y lee el numero que sigue (ej. "KGIRO=0.95/" -> 0.95)
static uint8_t lee_num(const char *trama, const char *clave, float *valor)
{
    const char *p=strstr(trama,clave);
    char *fin;
    if (p==0) return 0;
    p+=strlen(clave);
    float v=strtof(p,&fin);
    if (fin==p) return 0;   //no habia numero
    *valor=v;
    return 1;
}

//lee "clave" seguida de dos enteros separados por coma (ej. "TRECTO=150,2000/")
static uint8_t lee_dos(const char *trama, const char *clave, long *a, long *b)
{
    const char *p=strstr(trama,clave);
    char *fin;
    if (p==0) return 0;
    p+=strlen(clave);
    *a=strtol(p,&fin,10);
    if ((fin==p)||(*fin!=',')) return 0;
    p=fin+1;
    *b=strtol(p,&fin,10);
    return (fin==p) ? 0 : 1;
}

void procesa_rx()
{
	const char *trama=(const char *)UARTRX1.trama_rx;
	float v;
	long a,b;

	//STOP primero: nada puede taparlo
	if (strstr(trama,"STOP"))  //orden de parada por serial
	    {
	        estado=stop;
	    }

	else if (strstr(trama,"LED1ON")) PIN_ON(LED2);
	else if (strstr(trama,"LED1OFF")) PIN_OFF(LED2);

	else if (lee_num(trama,"KGIRO=",&v))
	    {
	        if (!Param_set_kgiro(v)) uartx_write_text(&huart1, "KGIRO: fuera de rango\r\n");
	    }

	else if (lee_num(trama,"VGIRO=",&v))
	    {
	        if (!Param_set_vgiro((int32_t)v)) uartx_write_text(&huart1, "VGIRO: fuera de rango\r\n");
	    }

	else if (lee_num(trama,"VMAX=",&v))
	    {
	        if (!Param_set_vmax((int32_t)v)) uartx_write_text(&huart1, "VMAX: fuera de rango\r\n");
	    }

	//---------------- test (solo con el robot parado o en otro test)
	else if (lee_dos(trama,"TRECTO=",&a,&b))   //velocidad en us, tiempo en ms
	    {
	        if (estado==run) uartx_write_text(&huart1, "TEST: manda el radio, suba el canal 3\r\n");
	        else if ((a<-TEST_VEL_MAX)||(a>TEST_VEL_MAX)||(b<TEST_MS_MIN)||(b>TEST_MS_MAX))
	            uartx_write_text(&huart1, "TEST: valores invalidos\r\n");
	        else
	        {
	            Test_inicia((int16_t)a,0,(uint32_t)b);
	            uartx_write_text(&huart1, "TEST: recto\r\n");
	        }
	    }

	else if (lee_dos(trama,"TGIRO=",&a,&b))    //direccion (+1 derecha, -1 izquierda), tiempo en ms
	    {
	        if (estado==run) uartx_write_text(&huart1, "TEST: manda el radio, suba el canal 3\r\n");
	        else if (((a!=1)&&(a!=-1))||(b<TEST_MS_MIN)||(b>TEST_MS_MAX))
	            uartx_write_text(&huart1, "TEST: valores invalidos\r\n");
	        else
	        {
	            Test_inicia(0,(int16_t)(a*500),(uint32_t)b);   //Motores_mueve lo limita a vgiro
	            uartx_write_text(&huart1, "TEST: giro\r\n");
	        }
	    }

	else if (strstr(trama,"AUTO"))   //modo automatico, igual que el test: no le quita el mando al radio
	    {
	        if (estado==run) uartx_write_text(&huart1, "AUTO: manda el radio, suba el canal 3\r\n");
	        else
	        {
	            estado=automatico;
	            uartx_write_text(&huart1, "AUTO: activado\r\n");
	        }
	    }

	//---------------- parametros
	else if (strstr(trama,"GUARDAFLASH"))
	    {
	        if (estado!=stop) uartx_write_text(&huart1, "FLASH: detener el robot\r\n");
	        else uartx_write_text(&huart1, Param_guardar() ? "FLASH: guardado\r\n" : "FLASH: error al guardar\r\n");
	    }

	else if (strstr(trama,"LEEFLASH"))
	    {
	        uartx_write_text(&huart1, Param_leer_flash() ? "FLASH: parametros leidos\r\n" : "FLASH: no hay parametros guardados\r\n");
	    }

	else if (strstr(trama,"DEFECTO"))   //en RAM; GUARDAFLASH para grabarlos
	    {
	        Param_defecto();
	        uartx_write_text(&huart1, "DEFECTO: aplicado, sin guardar\r\n");
	    }

	else if (strstr(trama,"RUN"))  //orden de inicio por serial
	    {
	        if (ancho3>=RC3_ALTO) uartx_write_text(&huart1, "RUN: canal 3 arriba, no arranca\r\n");
	        else estado=run;
	    }
}

//lidars, radio, bateria, piso y parametros: 5 veces por segundo. imu = lo que se reporta al panel
//sobre la imu (0 = el panel no espera sus datos). Lleva su propio tiempo: llamarla siempre en el lazo
static void Manda_lentos(uint8_t imu)
{
	static uint32_t t_lento=0;

	if ((uwTick-t_lento)>=200)
	{
		uint8_t lidok=0;
		int n=0;

		t_lento=uwTick;
		for (uint8_t c=0; c<LIDAR_N; c++)
		{
			if (lidar_ok[c]) lidok|=(1u<<c);
			n+=sprintf(&texto[n],">L%u:%.1f\n",c+1,distancia[c]);
		}
		n+=sprintf(&texto[n],">C1:%.0f\n>C2:%.0f\n>C3:%.0f\n>C4:%.0f\n",ancho1,ancho2,ancho3,ancho4);
		//codigos crudos del ADC (0 a 4095): buffer 0 = bateria, 1 a 4 = sensores de piso
		n+=sprintf(&texto[n],">BAT:%u\n>S1:%u\n>S2:%u\n>S3:%u\n>S4:%u\n",
				adc1_codigo[0],adc1_codigo[1],adc1_codigo[2],adc1_codigo[3],adc1_codigo[4]);
		sprintf(&texto[n],"*KGIRO%.3f**VMAX%ld**VGIRO%ld**EST%u**GUARD%u**IMU%u**LIDOK%u*\n",
				PARAM.kgiro,(long)PARAM.vmax,(long)PARAM.vgiro,estado,Param_guardado(),imu,lidok);
		uartx_write_text(&huart1, texto);
	}
}

//datos para el panel y para Teleplot (formato ">nombre:valor"). Hay que llamarla siempre en
//el while: lleva sus propios tiempos para no llenar el serial, que bloquea el lazo al enviar
void  Manda_basicos()
{
	static uint32_t t_rapido=0;

	//imu: 20 veces por segundo
	if (imu_ok && ((uwTick-t_rapido)>=50))
	{
		t_rapido=uwTick;
		sprintf(texto,">PITCH:%.1f\n>ROLL:%.1f\n>YAW:%.1f\n>AX:%.2f\n>AY:%.2f\n>AZ:%.2f\n",pitch,roll,yaw,ax,ay,az);
		uartx_write_text(&huart1, texto);
	}

	Manda_lentos(imu_ok);
}
 void LEE_RX()
 {
	 if (UARTRX1.flag_rx==1)
	 	  {
	 	      uartx_write_text(&huart1, UARTRX1.trama_rx);
	 	      uartx_write_text(&huart1, "\r\n");
	 	      procesa_rx(); //procedimeinto donde se procesan los datos
	 		  uartRX_DMA_Re_init(&UARTRX1);
	 	  }

 }
#if USA_IR
void lee_ir()
{
	uint32_t codigo_ir;
	if (IR_leer(&codigo_ir))   //1 = llego un codigo (ya queda rearmado)
	{
	 PIN_BLINK(LED1);   //LED3 no: lo cambia PWM_IN.c con cada flanco del canal 1
	 sprintf(texto,"codigo=0x%08lX/\r\n",(unsigned long)codigo_ir);
	 uartx_write_text(&huart1, texto);

	 if ((codigo_ir==IRSTART)||(codigo_ir==IRSTART2))
		{
		 uartx_write_text(&huart1, "START\r\n");
		 estado=run;
		}
	 if ((codigo_ir==IRSTOP)||(codigo_ir==IRSTOP2))
		{
		 uartx_write_text(&huart1, "STOP\r\n");
		 estado=stop;
		}
	 if (codigo_ir==IRPROG)  uartx_write_text(&huart1, "PROG1\r\n");
	 if (codigo_ir==IRPROG2) uartx_write_text(&huart1, "PROG2\r\n");
	}
}
#endif
void LEE_IMU()
{
	 BNO055_ReadEuler();
	 BNO055_ReadAcceleration();

}
void Manda_IMU()
{
	sprintf(texto,"PITCH=%.2f   ROLL=%.2f    YAW=%.2f\r\n\r\n" ,pitch,roll,yaw);
	 uartx_write_text(&huart1, texto);
}

void Manda_Lidars()
{
	   sprintf(texto,"Dis1=%.1f\r\n",distancia[0]);
	   	uartx_write_text(&huart1, texto);
		sprintf(texto,"Dis2=%.1f\r\n",distancia[1]);
		uartx_write_text(&huart1, texto);
	    sprintf(texto,"Dis3=%.1f\r\n",distancia[2]);
	    uartx_write_text(&huart1, texto);
		sprintf(texto,"Dis4=%.1f\r\n",distancia[3]);
		uartx_write_text(&huart1, texto);
		sprintf(texto,"Dis5=%.1f\r\n",distancia[4]);
		uartx_write_text(&huart1, texto);
		sprintf(texto,"Dis6=%.1f\r\n",distancia[5]);
		uartx_write_text(&huart1, texto);
		sprintf(texto,"Dis7=%.1f\r\n",distancia[6]);
		uartx_write_text(&huart1, texto);
		sprintf(texto,"Dis8=%.1f\r\n",distancia[7]);
		uartx_write_text(&huart1, texto);
		uartx_write_text(&huart1,"\r\n");
}
void INIT_LIDARS_FAST()
{
	uint8_t mascara = lidar_init_todos();
	sprintf(texto,"LIDARS OK mask=0x%02X\r\n",mascara);   // bit n = sensor del canal n
	uartx_write_text(&huart1, texto);
}

// Envia los 8 lidars en un solo frame por interrupcion (no bloquea).
// Si el frame anterior aun se esta enviando, este se salta.
// Ojo: mientras transmite, un uartx_write_text bloqueante recibe HAL_BUSY.
void Manda_Lidars_Teleplot()
{
	static char tx_lidar[LIDAR_N * 16];
	int n = 0;

	if (huart1.gState != HAL_UART_STATE_READY) return;

	for (uint8_t c = 0; c < LIDAR_N; c++)
		n += snprintf(&tx_lidar[n], sizeof(tx_lidar) - n, ">L%u:%.1f\n", c + 1, distancia[c]);

	HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_lidar, n);
}

// Barrido rapido sin retardos: los sensores miden solos en modo continuo.
// Retorna cuantos sensores entregaron dato nuevo.
uint8_t LEE_LIDARS_FAST()
{
	uint8_t nuevos = lidar_actualiza();

	for (uint8_t c = 0; c < LIDAR_N; c++)
		distancia[c] = lidar_cm(c);
	return nuevos;
}




//procedimeintos a testear para meustreo
void  VOIDTEST()
{
	 LEE_PWM();
	     LEE_RX();
	    LEE_IMU();
	  //  uartx_write_text(&huart1, "OPERANDO\r\n");
	  LEE_LIDARS_FAST();   // barrido de los 8 lidars, debe quedar en pocos ms
}

void test_muestreo()
{

	  HAL_TIM_Base_Start(&htim15) ;
	   TIM15->CNT=0;
  // delay_ms(5);
  VOIDTEST();
		 		 uint16_t CONTA= TIM15->CNT;
		 		 float timues=CONTA/10.0;
			   	 sprintf(texto,"MUESTREO=%.1f\r\n\r\n",timues);
			   	 uartx_write_text(&huart1, texto);
			  while(1);
}



void Control_Sumo()
{
    //sin señal en acelerador o giro: motores parados
    if ((ancho1==0)||(ancho2==0))
    {
        SERVO_MILI(&SERVO1,1.5);
        SERVO_MILI(&SERVO2,1.5);
        return;
    }

    //leo valores respecto al cero =1500
    int16_t vel  = (int16_t)ancho1 - 1500;   // -500 a +500
    int16_t giro = (int16_t)ancho2 - 1500;   // -500 a +500

    //mando centrado o transmisor apagado (cerca de 1500): quieto, sin arrastrarse
    if (abs(vel)<RC_ZONA_MUERTA)  vel=0;
    if (abs(giro)<RC_ZONA_MUERTA) giro=0;

    Motores_mueve(vel,giro);
}

#if PRUEBA_SOLO_RADIO
static uint8_t prueba_stop=0;   //1 = STOP del panel: reposo hasta subir el canal 3 (y bajarlo para volver)

//ordenes de la prueba: STOP y parametros. Lo demas (TRECTO, AUTO...) se ignora y lo avisa
static void Prueba_procesa_rx(void)
{
	const char *trama=(const char *)UARTRX1.trama_rx;
	float v;

	//STOP primero: nada puede taparlo. El panel lo manda varias veces: avisa solo la primera
	if (strstr(trama,"STOP"))
	    {
	        if (!prueba_stop) uartx_write_text(&huart1, "PRUEBA: STOP, suba y baje el canal 3 para volver\r\n");
	        prueba_stop=1;
	    }

	else if (lee_num(trama,"KGIRO=",&v))
	    {
	        if (!Param_set_kgiro(v)) uartx_write_text(&huart1, "KGIRO: fuera de rango\r\n");
	    }

	else if (lee_num(trama,"VGIRO=",&v))
	    {
	        if (!Param_set_vgiro((int32_t)v)) uartx_write_text(&huart1, "VGIRO: fuera de rango\r\n");
	    }

	else if (lee_num(trama,"VMAX=",&v))
	    {
	        if (!Param_set_vmax((int32_t)v)) uartx_write_text(&huart1, "VMAX: fuera de rango\r\n");
	    }

	else if (strstr(trama,"GUARDAFLASH"))   //grabar frena la cpu: solo con los mandos centrados
	    {
	        uint8_t quieto=(estado==stop)||(ancho1==0)||(ancho2==0)||
	                       ((abs((int16_t)ancho1-1500)<RC_ZONA_MUERTA)&&(abs((int16_t)ancho2-1500)<RC_ZONA_MUERTA));

	        if (!quieto) uartx_write_text(&huart1, "FLASH: pase a reposo o centre los mandos para guardar\r\n");
	        else uartx_write_text(&huart1, Param_guardar() ? "FLASH: guardado\r\n" : "FLASH: error al guardar\r\n");
	    }

	else if (strstr(trama,"LEEFLASH"))
	    {
	        uartx_write_text(&huart1, Param_leer_flash() ? "FLASH: parametros leidos\r\n" : "FLASH: no hay parametros guardados\r\n");
	    }

	else if (strstr(trama,"DEFECTO"))   //en RAM; GUARDAFLASH para grabarlos
	    {
	        Param_defecto();
	        uartx_write_text(&huart1, "DEFECTO: aplicado, sin guardar\r\n");
	    }

	else uartx_write_text(&huart1, "PRUEBA: orden ignorada, solo STOP, KGIRO, VGIRO, VMAX y flash\r\n");
}

//prueba: lee los canales 1 y 2 y los aplica a los motores. No sale nunca de aqui.
//canal 3 abajo = control. Arriba, en el medio (transmisor apagado) o sin señal = reposo.
//sigue la posicion del interruptor: al volver abajo retoma el control, no hay que subirlo y bajarlo.
//STOP del panel (boton, Esc o espacio) = reposo hasta subir el canal 3; al bajarlo vuelve el control.
//lee los lidars y manda bateria, lidars y piso; la imu no. Por serial solo atiende STOP, kgiro, vgiro, vmax y la flash.
//sin señal en el canal 1 o 2, Control_Sumo deja los motores en neutro
static void Prueba_solo_radio(void)
{
	static CANAL_RC rc1,rc2,rc3;
	uint32_t t=uwTick;

	uartx_write_text(&huart1, "PRUEBA: solo radio, canales 1, 2 y 3\r\n");

	while (1)
	{
		if (UARTRX1.flag_rx==1)
		{
			uartx_write_text(&huart1, UARTRX1.trama_rx);
			uartx_write_text(&huart1, "\r\n");
			Prueba_procesa_rx();
			uartRX_DMA_Re_init(&UARTRX1);
		}

		//canales cruzados: ancho1 (velocidad) se lee de la entrada 2, ancho2 (giro) de la entrada 1
		Lee_canal_RC(&flag_PWM_in2,pulso2,&rc1,&ancho1);
		Lee_canal_RC(&flag_PWM_in1,pulso1,&rc2,&ancho2);
		Lee_canal_RC(&flag_PWM_in3,pulso3,&rc3,&ancho3);

		//solo el canal 3 arriba de verdad borra el STOP: un corte de señal o el transmisor apagado no
		if (ancho3>=RC3_ALTO) prueba_stop=0;

		uint8_t nuevo=((ancho3>0)&&(ancho3<=RC3_BAJO)&&(!prueba_stop)) ? run : stop;
		if (nuevo!=estado)
		{
			estado=nuevo;
			//queda en el log del panel: si pasa a reposo solo, muestra con que valor del canal 3
			sprintf(texto,"PRUEBA: %s C3=%.0f\r\n",(estado==run) ? "CONTROL" : "REPOSO",ancho3);
			uartx_write_text(&huart1, texto);
		}

		if (estado==run) Control_Sumo();
		else
		{
			SERVO_MILI(&SERVO1,1.5);
			SERVO_MILI(&SERVO2,1.5);
		}

		if ((uwTick-t)>=50)
		{
			t=uwTick;
			LEE_LIDARS_FAST();

			//reiniciar un lidar colgado bloquea el lazo: los motores siguen con la ultima orden
			uint32_t parado=uwTick-t;
			if (parado>=RC_PERIODO_MAX_MS)
			{
				sprintf(texto,"LIDAR: lazo parado %lu ms\r\n",(unsigned long)parado);
				uartx_write_text(&huart1, texto);
			}
		}

		Manda_lentos(0);
	}
}
#endif

//lee lidars e imu 20 veces por segundo
void Lee_sensores_tarea()
{
	static uint32_t t=0;

	if ((uwTick-t)<50) return;
	t=uwTick;

	LEE_LIDARS_FAST();
	if (imu_ok) LEE_IMU();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_GPDMA2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_ICACHE_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
 //INICIA PWM: lo primero, los ESC (bidireccionales) deben ver neutro = 1500us desde el arranque
 SERVO_init(&SERVO1);
 SERVO_init(&SERVO2);
 SERVO_init(&SERVO3);
 SERVO_init(&SERVO4);
 SERVO_MILI(&SERVO1,1.5);
 SERVO_MILI(&SERVO2,1.5);
 SERVO_MILI(&SERVO3,1.5);
 SERVO_MILI(&SERVO4,1.5);

 //cargo los parametros guardados en flash (o los de defecto si no hay)
 if (Param_init()) uartx_write_text(&huart1,"PARAMETROS: flash\r\n");
 else              uartx_write_text(&huart1,"PARAMETROS: defecto\r\n");

 Delay_Init();
 I2C_SCAN(hi2c2);
HAL_Delay(100);
//inicia lecturas de PWM
 PWM1_IN_Init();

  HAL_Delay(1000);
 //INICIA LIDAR
// INIT_LIDARS();
INIT_LIDARS_FAST();
Sensores_init();//inia adc DMA
 //inicio recepcion serial INT
 uartRX_it_idle_dma_init( &UARTRX1);
 uartx_write_text(&huart1, "INICINADO\r\n");
 //si quiero probar tiempo de muestreo
 //test_muestreo();

 //mas tiempo en neutro para que los ESC se armen antes de aceptar ordenes
 HAL_Delay(1000);



 //TEST  SALIDA PWM  1MS A 2M
/*
 SERVO_MILI(&SERVO1,2.0);
 SERVO_MILI(&SERVO2,2.0);
 SERVO_MILI(&SERVO3,2.0);
 SERVO_MILI(&SERVO4,2.0);

*/

  //INICIO IMU
 //solo si responde: sin sensor, las lecturas no sirven
 imu_ok=(HAL_I2C_IsDeviceReady(&BNO055_I2C,BNO055_I2C_ADDRESS,2,5)==HAL_OK) ? 1 : 0;
 if (imu_ok) BNO055_INIT();
 uartx_write_text(&huart1, imu_ok ? "IMU: OK\r\n" : "IMU: no responde\r\n");
 TICK_RC=uwTick;
//inicia el muestro
 //HAL_TIM_Base_Start_IT(&htim6);
#if USA_IR
//control IR (necesita EXTI6 y TIM7 con sus interrupciones en el .ioc)
 IR_init();
 //IR_calibrar(10);   //SOLO para sacar los codigos: imprime 10 y sigue. Luego comentar
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if PRUEBA_SOLO_RADIO
  Prueba_solo_radio();
#endif

  uint32_t tick_teleplot = uwTick;
/*
  while (1)
  {
   sprintf(texto,">BAT:%u\n",adc1_codigo[0]);
    uartx_write_text(&huart1, texto);
   sprintf(texto,">S1:%u\n",adc1_codigo[1]);
    uartx_write_text(&huart1, texto);
   sprintf(texto,">S2:%u\n",adc1_codigo[2]);
    uartx_write_text(&huart1, texto);
   sprintf(texto,">S3:%u\n",adc1_codigo[3]);
    uartx_write_text(&huart1, texto);
   sprintf(texto,">S4:%u\n",adc1_codigo[4]);
    uartx_write_text(&huart1, texto);
    HAL_Delay(100);
    


  }
  */
  
  
  /*
  while (1)
  {
    // Barrido sin esperas; se envia a Teleplot como maximo una vez por periodo de medida.
    if (LEE_LIDARS_FAST() && ((uwTick - tick_teleplot) >= LIDAR_PERIODO_MS))
    {
      tick_teleplot = uwTick;
      Manda_Lidars_Teleplot();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
  }*/
/*
  while(1)
  {
 LEE_PWM();
 uartx_write_text(&huart1, "\r\n\r\n");
 HAL_Delay(1000);
    
  }
*/
 while (1)
 {
#if USA_IR
	 lee_ir();
#endif
	 LEE_RX();
	 LEE_PWM();
	 Lee_sensores_tarea();
	 switch (estado)
	 {
	 case stop:

		  SERVO_MILI(&SERVO1,1.5);
		  SERVO_MILI(&SERVO2,1.5);

		       break;


	 case run:

	     	 Control_Sumo();
   	 		  break;


	 case automatico:

		 Control_Automatico();
		 break;


	 case test:

	     	 Test_tarea();
   	 		  break;


	 }


	// LEE_IMU();
	 Manda_basicos();
	// Manda_IMU();
	// delay_ms(20);
	 //SERVO_MICRO(&SERVO1,(uint16_t)ancho1);
	 //SERVO_MICRO(&SERVO2,(uint16_t)ancho2);


	// HAL_Delay(1);
	// HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
 }


 while (1)
 {
	 Lee_sensores();
	 float voltaje= adc1_codigo[0]*3.3/4095.0;
	    sprintf(texto,"BAT=%.1f\r\n",voltaje);
	    uartx_write_text(&huart1, texto);

     voltaje= lecturas_ir_an[0]*3.3/4095.0;
    sprintf(texto,"V1=%.1f\r\n",voltaje);
    uartx_write_text(&huart1, texto);

    voltaje= lecturas_ir_an[1]*3.3/4095.0;
        sprintf(texto,"V2=%.1f\r\n",voltaje);
        uartx_write_text(&huart1, texto);

        voltaje= lecturas_ir_an[2]*3.3/4095.0;
                sprintf(texto,"V3=%.1f\r\n",voltaje);
                uartx_write_text(&huart1, texto);

                voltaje= lecturas_ir_an[3]*3.3/4095.0;
                        sprintf(texto,"V4=%.1f\r\n\r\n",voltaje);
                        uartx_write_text(&huart1, texto);
            HAL_Delay(1500);

 }

  while (1)
  {

  Manda_IMU();
  Manda_Lidars_Teleplot();

    PIN_BLINK(LED3);
 // PIN_BLINK(LED2);
 // PIN_BLINK(LED3);

	  delay_ms(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief GPDMA2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA2_Init(void)
{

  /* USER CODE BEGIN GPDMA2_Init 0 */

  /* USER CODE END GPDMA2_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA2_CLK_ENABLE();

  /* GPDMA2 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA2_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA2_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA2_Init 1 */

  /* USER CODE END GPDMA2_Init 1 */
  /* USER CODE BEGIN GPDMA2_Init 2 */

  /* USER CODE END GPDMA2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10C043E5;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 250-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 250-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 25000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 200;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 250-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 8000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 25000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED3_Pin|PH1D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|PH2D_Pin|PH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin PH1D_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|PH1D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : STOP_Pin */
  GPIO_InitStruct.Pin = STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin PH2D_Pin PH_EN_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|PH2D_Pin|PH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_in_Pin */
  GPIO_InitStruct.Pin = IR_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SW_Pin */
  GPIO_InitStruct.Pin = SD_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
