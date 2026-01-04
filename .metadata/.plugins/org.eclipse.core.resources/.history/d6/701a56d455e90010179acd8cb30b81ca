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
				//const char *json="{\"type\": data_request, \"payload\":\"1.1;1.2;1.3;10.9;10.8;10.7 je tenmerde\"}";
				/*RTC_ReadTime(&rtc);
				rtc_get_timestamp(ts, &rtc);
				const char *json ="{"
			      		   "\"type\":\"data_request\","
			       		   "\"from\":\"nucleo-01\","
			       		   "\"to\":\"nucleo-14\","
			       		   "\"timestamp\":\"2025-10-02T08:20:00Z\""
			       		 "}";
				log_message("[CLIENT] Sending : %s...\r\n",json);
				netconn_write(conn,json,strlen(json),NETCONN_COPY);*/
				char txbuf[512];
                float ax = rmsX;
                float ay = rmsY;
                float az = rmsZ;
                RTC_ReadTime(&rtc);
                rtc_get_timestamp(ts, &rtc);
                snprintf(txbuf, sizeof(txbuf),"{\"type\":\"data_response\","
                		"\"id\":\"nucleo-01\","
                		"\"timestamp\":\"%s\","
                		"\"acceleration\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                		"\"status\":\"normal\"}",
						ts,ax, ay, az);
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
		osDelay(8000);
	}
  /* USER CODE END StartClientTask */
}
