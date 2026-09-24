/*
 * PWM_IN.C
 *
 *  Created on: Jul 20, 2023
 *      Author: alcid
 */


#include <PWM_IN.h>
uint8_t  sumo_modo=RC;


/*  colocar variables en main
extern uint16_t pulso1,pulso2,pulso3,pulso4;
float ancho1,ancho2;


//iniciar
 PWM1_IN_Init();

 //   y en el while
    ancho1=pulso1/10.0;  //pasa a cm
   ancho2=pulso4/10.0;  //pasa a cm

   //limpia los pulsos
   pulso1=0;
   pulso2=0;
   pulso3=0;
   pulso4=0;

  sprintf(texto,"PULSO1=%.1f\r\n\r\n",ancho1);
  uartx_write_text(&huart1, texto);
  HAL_Delay(1000);


//y en el archivo stm32f4xx_it.c debe quedar asi este procedimiento

void TIM2_CC_IRQHandler(void)
{
  #include "PWM_IN.h"  //debe incluirlo
  PWM1_IN_Interrupt();
  HAL_TIM_IRQHandler(&hTIM2);

  }
  */



uint8_t caida1 = 1, caida2 = 1, caida3 = 1, caida4 = 1; //INICIA CON FLANCO DE SUBIDA
uint16_t pulso1, pulso2, pulso3, pulso4;
uint16_t pulso1b, pulso1s, pulso2b, pulso2s;
uint16_t pulso3b, pulso3s, pulso4b, pulso4s;
uint8_t flag_PWM_in1=0;
uint8_t flag_PWM_in2=0;
uint8_t flag_PWM_in3=0;
uint8_t flag_PWM_in4=0;

uint32_t TICK_PWM_IN1,TICK_PWM_IN2,TICK_PWM_IN3,TICK_PWM_IN4;

void  PWM1_IN_Init()
{
	//inicia el TIM2_PWM_IN con interrupcion de flanco y canales desados
	 HAL_TIM_IC_Start_IT(&TIM2_PWM_IN,TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&TIM2_PWM_IN,TIM_CHANNEL_2);
	  HAL_TIM_IC_Start_IT(&TIM2_PWM_IN,TIM_CHANNEL_3);
	  HAL_TIM_IC_Start_IT(&TIM2_PWM_IN,TIM_CHANNEL_4);
	  TICK_PWM_IN1=uwTick;
	  TICK_PWM_IN2=uwTick;
	  TICK_PWM_IN3=uwTick;
	  TICK_PWM_IN4=uwTick;

}

//se debe incluir   #include "PWM_IN.h"  en la int
void PWM1_IN_Interrupt()
{
	//SI EL EL CANAL 1

	if (__HAL_TIM_GET_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC1) ==1)
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		//si el flanco es de subida
		if (caida1 == 1) {
			//CONFIGURA FLANCO DE SUBIDA
			__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
			caida1 = 0;
			pulso1s = TIM2->CCR1;

		} //si

		//si capturo el flaco de bajada
		else {
			__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			caida1 = 1;
			pulso1b = TIM2->CCR1;
			flag_PWM_in1=1;
			TICK_PWM_IN1=uwTick;
			if (pulso1b > pulso1s)
				pulso1 = (pulso1b - pulso1s);
			else
				pulso1 = (TIM2->ARR - pulso1s) + pulso1b;
		} //Fin else

	    __HAL_TIM_CLEAR_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC1);//limpia la bandera

	} //fin si es el canal 1


	//SI EL EL CANAL 2
	if (__HAL_TIM_GET_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC2) ==1)

	{
		//si el flanco es de subida
		if (caida2 == 1) {
			//CONFIGURA FLANCO DE SUBIDA
			__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
			caida2 = 0;
			pulso2s = TIM2->CCR2;
		} //si

		//si capturo el flaco de bajada
		else {
			__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
			caida2 = 1;
			pulso2b = TIM2->CCR2;
			flag_PWM_in2=1;
			TICK_PWM_IN2=uwTick;
			if (pulso2b > pulso2s)
				pulso2 = (pulso2b - pulso2s);
			else
				pulso2 = (TIM2->ARR - pulso2s) + pulso2b;

		} //Fin else
		   __HAL_TIM_CLEAR_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC2);//limpia la bandera
	}

	//SI EL EL CANAL 3
	if (__HAL_TIM_GET_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC3) ==1)

	{
		//si el flanco es de subida
		if (caida3 == 1) {
			//CONFIGURA FLANCO DE SUBIDA
			__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			caida3 = 0;
			pulso3s = TIM2->CCR3;
		} //si

		//si capturo el flaco de bajada
		else {
			__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_RISING);
			caida3 = 1;
			pulso3b = TIM2->CCR3;
			flag_PWM_in3=1;
			TICK_PWM_IN3=uwTick;
			if (pulso3b > pulso3s)
				pulso3 = (pulso3b - pulso3s);
			else
				pulso3 = (TIM2->ARR - pulso3s) + pulso3b;

		} //Fin else
		   __HAL_TIM_CLEAR_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC3);//limpia la bandera
	}


	//SI EL EL CANAL 4
	if (__HAL_TIM_GET_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC4) ==1)

		{
			//si el flanco es de subida
			if (caida4 == 1) {
				//CONFIGURA FLANCO DE SUBIDA
				__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_FALLING);
				caida4 = 0;
				pulso4s = TIM2->CCR4;
			} //si

			//si capturo el flaco de bajada
			else {
				__HAL_TIM_SET_CAPTUREPOLARITY(&TIM2_PWM_IN, TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_RISING);
				caida4 = 1;
				pulso4b = TIM2->CCR4;
				flag_PWM_in4=1;
				TICK_PWM_IN4=uwTick;
				if (pulso4b > pulso4s)
					pulso4 = (pulso4b - pulso4s);
				else
					pulso4 = (TIM2->ARR - pulso4s) + pulso4b;

			} //Fin else
			   __HAL_TIM_CLEAR_FLAG(&TIM2_PWM_IN, TIM_FLAG_CC4);//limpia la bandera
		}

}


//COLOCAR  LOS MISMOS PROCEDIMEINTOS PERO ASOCIADOS A OTROS TIMERS SI ES NECESRIO
