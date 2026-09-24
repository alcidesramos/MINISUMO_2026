/*
 * IR_ANA_SEN.c
 *
 *  Created on: Oct 10, 2025
 *      Author: ALCIDES_RAMOS
 */


#include "IR_ANA_SEN.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
uint8_t  linea_color= negro_;

uint16_t lecturas_ir_an[ir_n_sensores];

float lecturas_filtradas[ir_n_sensores]= {0};

float Normalizados[ir_n_sensores];
bool linea=false;
uint16_t ir_minimos[]={65535,65535,65535,65535}; //Lectura minima   inicia con un valor alto
uint16_t ir_maximos[]={0,0,0,0}; // Lectura de maximo  inicia con un valor bajo

//lineal
float ir_pesos[]={-2.0,-1.0,1.0,2.0}; // Lectura normalizadas de 0 a 1
//float pesos[]={-16.0,-8.0,-4.0,-2.0,-0.5,0.5,2.0,4.0,8.0,16.0}; // Lectura normalizadas de 0 a 1
//cuadrarico
//float ir_pesos[]={-20.25, -12.25, -6.25, -2.25, -0.25, 0.25, 2.25, 6.25, 12.25, 20.25};

float ir_peso_anterior;// almacena el sentido de giro anterior
uint8_t flag_linea=0;
float peso,peso_anterior;

void Sensores_init()
{
	 //calibra adc
	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	    HAL_Delay(100);
     //inicia lectura dma
       	      ADC_Read_DMA(&hadc1, adc1_canales, adc1_codigo);
       	 	  HAL_Delay(10);

}

void Lee_sensores()
{
	lecturas_ir_an[0]=adc1_codigo[1];
	lecturas_ir_an[1]=adc1_codigo[2];
	lecturas_ir_an[2]=adc1_codigo[3];
	lecturas_ir_an[3]=adc1_codigo[4];
	}

void Inicia_filtro()
{
    for (int i = 0; i < ir_n_sensores; i++)
    {
        lecturas_filtradas[i] = (float)lecturas_ir_an[i];
    }
}
void  filtra_sensores()
{
	Lee_sensores();
	 for (int i = 0; i <ir_n_sensores; i++)
	    {
	        lecturas_filtradas[i] = alpha_sen * lecturas_ir_an[i] +
	                                (1.0f -alpha_sen ) * lecturas_filtradas[i];
	    }



}

void  Manda_Lecturas()
{
	char sen[200];
sprintf(sen,"%u   %u   %u   %u\r\n\r\n",
lecturas_ir_an[0],lecturas_ir_an[1],lecturas_ir_an[2],lecturas_ir_an[3]);
uartx_write_text(&IR_AN_UART, sen);

}

void  Manda_Lecturas_filtrada()
{
	char sen[200];
sprintf(sen,"%.1f   %.1f   %.1f   %.1f\r\n\r\n",lecturas_filtradas[0],lecturas_filtradas[1],lecturas_filtradas[2],lecturas_filtradas[3]);
uartx_write_text(&IR_AN_UART, sen);

}

void Calibrar_sensores()
{
	int16_t cuenta2;
	uint8_t cont;
	uint8_t sentido=0;
	uint32_t milis_QTR=uwTick;
	#define velo 4080
	//letar inicial de 400 ms
	while(uwTick-milis_QTR<400);
	milis_QTR=uwTick;//inia un nuevo temporizador

	//si se queire que sea automatico

    	 milis_QTR=uwTick;
    	 //tiempo de calibracion
while (uwTick-milis_QTR<5000)
{
     //le los sensores
    for (cont=0; cont<ir_n_sensores;cont++)
        {
    	 Lee_sensores();
    	  HAL_Delay(1);//delay entre lectira ya que es automatico por dma
           if (lecturas_ir_an[cont]>=ir_maximos[cont]) ir_maximos[cont]=lecturas_ir_an[cont];
           if (lecturas_ir_an[cont]<=ir_minimos[cont]) ir_minimos[cont]=lecturas_ir_an[cont];
         }
}

//por si se hace automatico
//APAGA MOTORES
  //ojo poner esto ARZ
    //MD_Stop();
    //MI_Stop();



}
void Manda_Limites()
{
char textoir[300];
  sprintf(textoir,"min1=%u\r\nmin2=%u\r\nmin3=%u\r\nmin4=%u\r\n\r\n",ir_minimos[0],ir_minimos[1],ir_minimos[2],ir_minimos[3]);
  uartx_write_text(&IR_AN_UART, textoir);
  sprintf(textoir,"max1=%u\r\nmax2=%u\r\nmax3=%u\r\nmax4=%u\r\n\r\n",ir_maximos[0],ir_maximos[1],ir_maximos[2],ir_maximos[3]);
  uartx_write_text(&IR_AN_UART, textoir);
}

//normalizo datos  en rango de 0 a 1;
void Normaliza_sensores(void)
{
    for (uint8_t k = 0; k < ir_n_sensores; k++)
    {
        float rango = (float)(ir_maximos[k] - ir_minimos[k]);
        float y = 0.0f;

        if (rango != 0.0f)
            y = ((float)lecturas_ir_an[k] - (float)ir_minimos[k]) / rango;

        if (linea_color==blanco_)  y=1.0-y;

        if (y < 0.0f) y = 0.0f;
        else if (y > 1.0f) y = 1.0f;

        Normalizados[k] = y;
    }
}

//normalizo datos  en rango de 0 a 1;
void Normaliza_sensores_filtrado(void)
{
    for (uint8_t k = 0; k < ir_n_sensores; k++)
    {
        float rango = (float)(ir_maximos[k] - ir_minimos[k]);
        float y = 0.0f;

        if (rango != 0.0f)
            y = (lecturas_filtradas[k] - (float)ir_minimos[k]) / rango;

        if (linea_color==blanco_)  y=1.0-y;

        if (y < 0.0f) y = 0.0f;
        else if (y > 1.0f) y = 1.0f;

        Normalizados[k] = y;
    }
}

void Manda_Normalizados()
{
	char sen[200];
	sprintf(sen,"Nor=%.2f/ %.2f/ %.2f/ %.2f/\r\n",
	Normalizados[0],Normalizados[1],Normalizados[2],Normalizados[3]);
	uartx_write_text(&IR_AN_UART, sen);
}


float Calcula_Pesos()
{
  char cuenta;
  float numerador=0,denominador=0;
  float valor;
  float peso_min=ir_pesos[0];
  float peso_max=ir_pesos[ir_n_sensores-1];

  flag_linea=0;
  for (cuenta=0;cuenta<ir_n_sensores; cuenta++)
  {
    //linea blanca
	 peso=Normalizados[cuenta];


	 //este valor depende de sus sensores
	 	 if(peso>linea_blanco)  flag_linea=1;  //indica que al menos un sensor esta en la linea

			if(peso>0.1)//elimina reflejos irrelevantes  es el umbral u offset
			{
			 numerador+=Normalizados[cuenta]*ir_pesos[cuenta];
			 denominador+=Normalizados[cuenta];
			}

	    }//fin del for

	if(!flag_linea)//sino esta en la line  corrige
		//auna velocidad media
		{
			if (peso_anterior>0) return( peso_max); // si estaba muy a la derecha antes retorna el maximo valor a la derecha
			else  if (peso_anterior<0) return( peso_min);  // si estaba muy a la izquierda antes retorna el maximo valor a la izquierda

		}
	//si tod esta en la linea
	//if (numerador<0.001) return(peso_anterior);



 //si esta en la linea
	if (denominador!=0) valor=numerador/denominador;
 	   peso_anterior=valor;//  almacena el valor por si se llega a salir de la linea
       return(valor);
}

void Manda_pesos()
{
	char sen[200];
	sprintf(sen,"Peso=%.2f\r\n\r\n",peso);
	uartx_write_text(&IR_AN_UART, sen);
}
