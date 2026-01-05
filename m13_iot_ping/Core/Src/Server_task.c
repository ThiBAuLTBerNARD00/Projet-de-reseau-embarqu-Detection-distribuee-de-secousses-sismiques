/*
 * Server_task.c
 *
 *  Created on: Dec 26, 2025
 *      Author: thiba
 */

#include "tasks.h"
#include "main.h"

/* USER CODE BEGIN Header_StartServerTask */
/**
* @brief Function implementing the serverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServerTask */
void StartServerTask(void const * argument)
{
  /* USER CODE BEGIN StartServerTask */
	struct netconn *conn, *newconn;
	err_t err;
	struct netbuf *buf;
	char *data;
	u16_t len;
	ip_addr_t client_ip;
	RTC_extern rtc;
	LWIP_UNUSED_ARG(argument);
	osSemaphoreWait(SemaphoreMasterHandle, osWaitForever);
	/* create a new TCP netconn */
	conn = netconn_new(NETCONN_TCP);
	if (!conn) {
	    printf("netconn_new TCP failed\n");
	    vTaskDelete(NULL);
	    return;
	}

	err = netconn_bind(conn, IP_ADDR_ANY, TCP_SERVER_PORT);
	if (err != ERR_OK) {
	    printf("netconn_bind TCP failed: %d\n", err);
	    netconn_delete(conn);
	    vTaskDelete(NULL);
	    return;
	}

	netconn_listen(conn);
	printf("TCP server listening on port %d\n", TCP_SERVER_PORT);
    for (;;) {
        /* accept (blocking) but managed by netconn/tcpip thread */
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK && newconn) {
            //ip_addr_copy(client_ip, netconn_getpeer(newconn));
            printf("TCP connection accepted\n");

	            /* set a recv timeout so we don't block forever */
            netconn_set_recvtimeout(newconn, 3000); /* ms */

            while ((err = netconn_recv(newconn, &buf)) == ERR_OK) {
                do {
                    netbuf_data(buf, (void**)&data, &len);
                    if (len > 0) {
                        /* assure null-terminated for strstr usage */
                        char msg[512];
                        size_t copylen = (len < sizeof(msg)-1) ? len : sizeof(msg)-1;
                        memcpy(msg, data, copylen);
                        msg[copylen] = '\0';
	                        /* debug print */
                        log_message("TCP recv: %s\n", msg);
                        osDelay(30);
                        if (strstr(msg, "data_request")) {
                            char txbuf[256];
                            float ax = rmsX;
                            float ay = rmsY;
                            float az = rmsZ;
                            RTC_ReadTime(&rtc);
                            rtc_get_timestamp(ts, &rtc);
                            snprintf(txbuf, sizeof(txbuf),
                                     "{\"type\":\"data_response\","
                                     "\"id\":\"nucleo-01\","
                                     "\"timestamp\":\"%s\","
                                     "\"acceleration\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                                     "\"status\":\"normal\"}",
									 ts,ax, ay, az);
                            netconn_write(newconn, txbuf, strlen(txbuf), NETCONN_COPY);
                        }/* -------- DATA RESPONSE -------- */
                        else if (strstr(msg, "data"))
                        {
                        	FramRMS_t recep;
                            if (extract_data(msg, &recep))
                            {
                            	if(recep.shake && event && (recep.time)){
                            		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
                            	}
                            	else{HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);}
                                uint8_t node = get_node_index(msg);

                                fram_update_rms(node, &recep);

                                log_message("[SERVER] RMS received from node %d : X=%.3f Y=%.3f Z=%.3f\r\n",
                                            node, recep.rms_x, recep.rms_y, recep.rms_z);

                                osDelay(15);
                            }
                            else
                            {
                                log_message("[SERVER] Invalid data_response format\r\n");
                            }
                        }


                    }
                } while (netbuf_next(buf) >= 0);
                netbuf_delete(buf);
            }

            /* close connection */
            netconn_close(newconn);
            netconn_delete(newconn);
            printf("TCP connection closed\n");
        }
        /* small delay to yield */
        osDelay(10);
    }
  /* USER CODE END StartServerTask */
}
