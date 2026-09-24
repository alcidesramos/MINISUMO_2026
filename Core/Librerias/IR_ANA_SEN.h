/*
 * IR_ANA_SEN.h
 *
 *  Created on: Oct 10, 2025
 *      Author: ALCIDES_RAMOS
 */

#ifndef LIBRERIAS_IR_ANA_SEN_H_
#define LIBRERIAS_IR_ANA_SEN_H_

#include "IR_ANA_SEN.h"
#include "main.h"
#include "adc_x.h"
#include "uart.h"


#define blanco_  0
#define negro_   1



#define ir_n_sensores 4   //numero de sensores
#define linea_blanco  0.4f //umbrar  para saber si no esta en la linea

//#define  linea_color  negro_ //1= negro   0= blanco
extern uint8_t  linea_color; //1= negro   0= blanco



#define  alpha_sen  0.3f  //filtro media movil sensores


#define IR_AN_UART huart1
extern UART_HandleTypeDef IR_AN_UART;


extern uint16_t lecturas_ir_an[];
extern float peso;


void Sensores_init();
void Lee_sensores();
void Manda_Lecturas();
void  Manda_Lecturas_filtrada();
void Calibrar_sensores();
void Inicia_filtro();
void  filtra_sensores();
void Manda_Limites();
void Normaliza_sensores();
void Manda_Normalizados();
float Calcula_Pesos();
void Manda_pesos();

#endif /* LIBRERIAS_IR_ANA_SEN_H_ */
