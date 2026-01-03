/*
 * ADC_task.c
 *
 *  Created on: Dec 26, 2025
 *      Author: thiba
 */

#include "tasks.h"
#include "main.h"


/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
	// Attendre que le MasterTask donne l'autorisation (tu utilises déjà SemaphoreMasterHandle)
	  osSemaphoreWait(SemaphoreMasterHandle, osWaitForever);

	  uint32_t sumX = 0;
	  uint32_t sumY = 0;
	  uint32_t sumZ = 0;

	  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
	  	{
	  	    log_message("ERROR: TIM2 start failed!\r\n");
	  	    vTaskDelete(NULL);
	  	}
	  	else
	  	{
	  	    log_message("TIM2 started.\r\n");
	  	}
	  // Démarrer le timer si tu veux trig ADC par timer (ici on lance en mode software continous DMA)
	  // Démarrer ADC en DMA (circular)
	  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK)
	  {
	    log_message("ADC DMA start failed!\r\n");
	    vTaskDelete(NULL);
	  }
	  else
	  {
	    log_message("ADC DMA started (len=%d)\r\n", ADC_BUF_LEN);
	  }

	  for(;;)
	  {
		  //if (osSemaphoreWait(adcSemaphoreHandle, 1000) == osOK)
		  //{
		  	  sumX=sumY=sumZ=0;
			  for (uint32_t i = 0; i < SAMPLES; i++)
			  {
			     sumX += adc_buf[i * 3 + 0];
			     sumY += adc_buf[i * 3 + 1];
			     sumZ += adc_buf[i * 3 + 2];
			  }
		      // 1) Moyenne des 64 échantillons DMA
		      float avgX = sumX / (float)SAMPLES;
		      float avgY = sumY / (float)SAMPLES;
		      float avgZ = sumZ / (float)SAMPLES;

		      // Conversion -> tensions -> acceleration
		      float ax = (3.3f * avgX / 4095.0f) - 1.65f;  // centrage
		      float ay = (3.3f * avgY / 4095.0f) - 1.65f;
		      float az = (3.3f * avgZ / 4095.0f) - 1.65f;

		      // --------------------------------------------------------
		      // 2) MOYENNE GLISSANTE
		      // --------------------------------------------------------
		      maX = movingAverage(maX, ax);
		      maY = movingAverage(maY, ay);
		      maZ = movingAverage(maZ, az);

		      // --------------------------------------------------------
		      // 3) RMS SUR 1 SECONDE (100 échantillons)
		      // --------------------------------------------------------
		      rmsBufX[rmsIndex] = ax;
		      rmsBufY[rmsIndex] = ay;
		      rmsBufZ[rmsIndex] = az;

		      rmsIndex++;
		      if (rmsIndex >= RMS_WINDOW)
		      {
		          rmsIndex = 0;
		          rmsFilled = 1;  // indique que la fenêtre est pleine
		      }

		      if (rmsFilled)
		      {
		          rmsX = computeRMS(rmsBufX, RMS_WINDOW);
		          rmsY = computeRMS(rmsBufY, RMS_WINDOW);
		          rmsZ = computeRMS(rmsBufZ, RMS_WINDOW);

		          // --------------------------------------------------------
		          // 4) DÉTECTION DE SECOUSSE
		          // --------------------------------------------------------
		          float ampX = rmsX - baselineX;
		          float ampY = rmsY - baselineY;
		          float ampZ = rmsZ - baselineZ;

		          bool event =
		              (ampX > threshold) ||
		              (ampY > threshold) ||
		              (ampZ > threshold);

		          if (event)
		          {
		              log_message("SECOUSSE !  Amp: X=%.3f  Y=%.3f  Z=%.3f  (RMS X=%.3f Y=%.3f Z=%.3f)\r\n", ampX, ampY, ampZ, rmsX, rmsY, rmsZ);
		          }
		          else
		          {
		             // log_message("Calme : RMS X=%.3f  Y=%.3f  Z=%.3f\r\n",rmsX, rmsY, rmsZ);
		          }
		      }
		  //}


	    osDelay(100); // laisse un peu de place pour scheduler
	  }

  /* USER CODE END StartADCTask */
}
