/*
 * Broardcast_task.c
 *
 *  Created on: Dec 26, 2025
 *      Author: thiba
 */

#include "tasks.h"
#include "main.h"
/* USER CODE BEGIN Header_StartBroadCast */
/**
* @brief Function implementing the BroardCast thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBroadCast */
void StartBroadCast(void const * argument)
{
  /* USER CODE BEGIN StartBroadCast */
	/* Infinite loop */
	struct udp_pcb *udp_send;
	struct udp_pcb *udp_rec;
	struct pbuf *p;
	RTC_extern udprtc;
	const char *device_id = "nucleo-01";
	const char *my_ip = "192.168.1.180";   //
	uint16_t len=0;
	//uint16_t err=0;
	ip_addr_t dest_ip;
	RTC_extern rtc;
	IP4_ADDR(&dest_ip, 192,168,1,255);       //

	// Attendre autorisation de la MasterTask
	osSemaphoreWait(SemaphoreMasterHandle, osWaitForever);

	log_message("Broadcast task started.\r\n");

	udp_send = udp_new();
	//udp_setflags(udp, UDP_FLAGS_BROADCAST);
	ip_set_option(udp_send, SOF_BROADCAST);
	printf("Flags netif: 0x%X\n", netif_default->flags);
	if (!udp_send) {
	   log_message("UDP alloc failed!\r\n");
	   vTaskDelete(NULL);
	}
	//err=udp_connect(udp, &dest_ip, 50000);
	udp_bind(udp_send, IP_ADDR_ANY, 0);     // port source aléatoire

	for(;;)
	{
	    char json_msg[256];
	    RTC_ReadTime(&rtc);
	    rtc_get_timestamp(ts, &rtc);
        len=snprintf(json_msg, sizeof(json_msg),
       		"{"
      		   "\"type\":\"presence\","
       		   "\"id\":\"%s\","
       		   "\"ip\":\"%s\","
       		   "\"timestamp\":\"%s\""
       		 "}",
			 device_id,
	         my_ip,
			 ts
	         //get_timestamp() // A faire avec la RTC //"\"timestamp\":\"%s\""
	        );


	     p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	     if (!p) continue;
	     pbuf_take(p, json_msg, len);
	     udp_sendto(udp_send, p, &dest_ip, UDP_BROADCAST_PORT);

	     pbuf_free(p);


	     osDelay(10000);
	 }
  /* USER CODE END StartBroadCast */
}
