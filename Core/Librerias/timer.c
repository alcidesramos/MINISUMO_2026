
/*
 * timer.h
 *
 *  Created on: May 1, 2025
 *      Author: ALCIDES_RAMOS
 */
#include "timer.h"


//DEFINA TIMER A USAR
#define TIM_INT  TIM6

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//mira si el timer desead0
	 if (htim->Instance == TIM_INT)
	 {

   //  if (R_START==1)
       {

/*     	    PIN_BLINK(LED2);
    		 LEE_PWM();
    	     LEE_RX();
    	    LEE_IMU();
    	  LEE_LIDARS_FAST();
 */


	}


	 }



	 /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

