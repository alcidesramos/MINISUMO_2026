/*
 * PWM_IN.H
 *
 *  Created on: Jul 20, 2023
 *      Author: alcid
 */

#ifndef LIBRERIAS_PWM_IN_H_
#define LIBRERIAS_PWM_IN_H_

#include "main.h"

#define  manual 0
#define  RC 1



extern uint8_t sumo_modo;

//definir si es neceario
#define TIM2_PWM_IN htim2


extern uint8_t flag_PWM_in1;
extern uint8_t flag_PWM_in2;
extern uint8_t flag_PWM_in3;
extern uint8_t flag_PWM_in4;

extern uint32_t TICK_PWM_IN1;
extern uint32_t TICK_PWM_IN2;
extern uint32_t TICK_PWM_IN3;
extern uint32_t TICK_PWM_IN4;










extern TIM_HandleTypeDef TIM2_PWM_IN;

void  PWM1_IN_Init();
void PWM1_IN_Interrupt();



#endif /* LIBRERIAS_PWM_IN_H_ */
