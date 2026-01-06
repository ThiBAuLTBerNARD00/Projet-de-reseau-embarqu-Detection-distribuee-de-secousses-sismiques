/*
 * Client_task.c
 *
 *  Created on: Dec 26, 2025
 *      Author: thiba
 */

/* USER CODE BEGIN Header_StartClientTask */
#include "tasks.h"
#include "main.h"
/**
* @brief Function implementing the clientTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClientTask */
void StartClientTask(void const * argument)
{
  /* USER CODE BEGIN StartClientTask */
  /* Infinite loop */
	RTC_extern rtc;
	FramRMS_t last;
	for(;;)
 	{
		if (ip_count == 0)
		{
		    log_message("[CLIENT] Aucun serveur connu. Attente broadcast...\r\n");
		    osDelay(1000);
		    continue;
		}
		struct netconn *conn;
		ip_addr_t server_ip;
		err_t err;
		char *target_ip = broadcast_ip_list[ip_index];
		ip_index = (ip_index + 1) % ip_count;

		//IP4_ADDR(&server_ip,192,168,1,41);
		ip4addr_aton(target_ip, &server_ip);
		conn=netconn_new(NETCONN_TCP);
		if(conn!=NULL){
			err=netconn_connect(conn,&server_ip,1234);
			if(err==ERR_OK){
				char txbuf[512];
                float ax = rmsX;
                float ay = rmsY;
                float az = rmsZ;
                RTC_ReadTime(&rtc);
                if (!fram_get_latest_entry(&last))
                {
                	log_message("[CLIENT] FRAM vide ou invalide\r\n");
                	netconn_close(conn);
                	netconn_delete(conn);
                	osDelay(1000);
                	continue;
                }
                rtc_get_timestamp(ts, &last.time);

                const char *status = (last.shake) ? "secousse" : "normal";
                snprintf(txbuf, sizeof(txbuf),"{\"type\": \"data\","
                		"\"id\": \"nucleo-01\","
                		"\"timestamp\": \"%s\","
                		"\"acceleration\": {\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                		"\"status\": \"%s\"}",
						ts,last.rms_x, last.rms_y, last.rms_z,status);
				netconn_write(conn, txbuf, strlen(txbuf), NETCONN_COPY);
				log_message("[CLIENT] Sending : %s... à %s\r\n",txbuf,ipaddr_ntoa(&server_ip));
			}
			else{
				log_message("[CLIENT] Could not reach server.\r\n");
			}
			netconn_close(conn);
			netconn_delete(conn);
		}
		else{
		log_message("[CLIENT] No connection available.\r\n");
		}

		osDelay(8000/(ip_count+1));

	}
  /* USER CODE END StartClientTask */
}
