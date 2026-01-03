/*
 * BroadcastServer_task.c
 *
 *  Created on: Dec 26, 2025
 *      Author: thiba
 */

#include "tasks.h"
#include "main.h"
/* USER CODE BEGIN Header_StartServerBroadcastTask */
/**
* @brief Function implementing the ServBroadcast thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServerBroadcastTask */
void StartServerBroadcastTask(void const * argument)
{
  /* USER CODE BEGIN StartServerBroadcastTask */
  /* Infinite loop */
	struct netconn *udp_conn;
	struct netbuf *buf;
	err_t err;

	/* Créer une connexion UDP en mode écoute */
	udp_conn = netconn_new(NETCONN_UDP);
	netconn_bind(udp_conn, IP_ADDR_ANY, UDP_LISTEN_PORT);   // 1234
	netconn_set_recvtimeout(udp_conn,15000);
	for(;;)
	{
	    /* Attendre un paquet UDP (bloquant jusqu'à réception) */
	    err = netconn_recv(udp_conn, &buf);

	    if (err == ERR_OK)
	    {
	        char data[256];
	        ip_addr_t *addr = netbuf_fromaddr(buf);  // IP source
	        u16_t port      = netbuf_fromport(buf);  // Port source
	        store_broadcast_ip(addr);
	        u16_t len = buf->p->tot_len;
	        if (len >= sizeof(data)) len = sizeof(data)-1;

	        netbuf_copy(buf, data, len);
	        data[len] = '\0';

	        log_message("[NETCONN UDP] Reçu de %s:%d : %s\r\n",ipaddr_ntoa(addr), port, data);

	        netbuf_delete(buf);
	    }
	}
  /* USER CODE END StartServerBroadcastTask */
}
