/*
 * ADC_task.c
 *
 *  Created on: Dec 26, 2025
 *      Author: thiba
 */

#include "tasks.h"
#include "main.h"
#include <math.h>
bool event=false;

void calibrate_baseline(void) {
    uint32_t sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 200; // 2 secondes à 100 Hz
    for (int n = 0; n < 10; n++)   // 10 buffers complets
    {
        osSemaphoreWait(adcSemaphoreHandle, 1000);

        for (int i = 0; i < SAMPLES; i++)
        {
            sumX += adc_buf[i * 3 + 0];
            sumY += adc_buf[i * 3 + 1];
            sumZ += adc_buf[i * 3 + 2];
        }
    }
    baselineX = sumX / (float)(SAMPLES *10);
    baselineY = sumY / (float)(SAMPLES *10);
    baselineZ = sumZ / (float)(SAMPLES *10);

    log_message("Baselines: X=%.2f Y=%.2f Z=%.2f\r\n", baselineX, baselineY, baselineZ);
}

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
	  osDelay(2000);
	  calibrate_baseline();
	  for(;;)
	  {
		  if (osSemaphoreWait(adcSemaphoreHandle, 1000) == osOK)
		  {
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

		      // --------------------------------------------------------
		      // BASELINE ADAPTATIVE LENTE (anti-dérive)
		      // --------------------------------------------------------
		      baselineX = 0.995f * baselineX + 0.005f * avgX;
		      baselineY = 0.995f * baselineY + 0.005f * avgY;
		      baselineZ = 0.995f * baselineZ + 0.005f * avgZ;

		      // Conversion -> tensions -> acceleration
		     /* float ax = (3.3f * avgX / 4095.0f) - 1.65f;  // centrage
		      float ay = (3.3f * avgY / 4095.0f) - 1.65f;
		      float az = (3.3f * avgZ / 4095.0f) - 1.65f;*/
		      float ax =avgX;
		      float ay = avgY;
		      float az = avgZ;


		      // --------------------------------------------------------
		      // 2) MOYENNE GLISSANTE
		      // --------------------------------------------------------
		      maX = movingAverage(maX, ax);
		      maY = movingAverage(maY, ay);
		      maZ = movingAverage(maZ, az);

		      // --------------------------------------------------------
		      // 3) RMS SUR 1 SECONDE (100 échantillons)
		      // --------------------------------------------------------
		      rmsBufX[rmsIndex] = ax - baselineX;
		      rmsBufY[rmsIndex] = ay - baselineY;
		      rmsBufZ[rmsIndex] = az - baselineZ;

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


		          /* Horodatage */
		          RTC_extern now;
		          RTC_ReadTime(&now);



		          // --------------------------------------------------------
		          // 4) DÉTECTION DE SECOUSSE
		          // --------------------------------------------------------
		          float ampX = rmsX ;
		          float ampY = rmsY ;
		          float ampZ = rmsZ ;
		          rmsX=rmsX+baselineX;
		          rmsY = rmsY+ baselineY;
		          rmsZ = rmsZ + baselineZ;
		          event =
		              (fabsf(ampX) > threshold) ||
		              (fabsf(ampY) > threshold) ||
		              (fabsf(ampZ) > threshold);
		          /* Top10 */
		          fram_update_top10(rmsX, rmsY, rmsZ, &now,event);
		          if (event)
		          {
		        	  osDelay(10);
		              log_message("SECOUSSE !  Amp: X=%.3f  Y=%.3f  Z=%.3f  (RMS X=%.3f Y=%.3f Z=%.3f)\r\n", ampX, ampY, ampZ, rmsX, rmsY, rmsZ);
		              osDelay(10);
		          }
		          else
		          {
		             // log_message("Calme : RMS X=%.3f  Y=%.3f  Z=%.3f\r\n",rmsX, rmsY, rmsZ);
		          }
		      }
		  }


	   // osDelay(100); // laisse un peu de place pour scheduler
	  }

  /* USER CODE END StartADCTask */
}
