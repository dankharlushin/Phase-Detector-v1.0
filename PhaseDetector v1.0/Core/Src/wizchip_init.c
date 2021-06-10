/*
 * wizchip_init.c
 *
 *  Created on: 12 янв. 2020 г.
 *      Author: Александр
 */


#include "wizchip_init.h"

uint8_t freesize;
extern uint16_t version;

/* Mac, IP ... W5500*/
wiz_NetInfo gWIZNETINFO = { .mac = {0x18,0xCF,0x5E,0x53,0xBF,0x3D},
							.ip = {169, 254, 153, 204},
                            .sn = {255,255,0,0},
                            .gw = {0,0,0,0},
                            .dns = {0,0,0,0},
                            .dhcp = NETINFO_STATIC };

/* UDP Destanation adr */
udp_NetInfo dstNetInfo = {.ip = {169, 254, 153, 217},
							.port = 5000 };

/** @brief Main W5500 Function
 *  @param SPI_HandleTypeDef* spi - dialog with w5500
 *  GPIO_TypeDef *GPIO_CS - port CS
 *  uint16_t GPIO_PIN_CS - Pin CS
 *  UART_HandleTypeDef* uart - debug info in console (can be deleted)
 *  @retval None
 *  */
void WizchIP_main(SPI_HandleTypeDef* spi,GPIO_TypeDef *GPIO_CS, uint16_t GPIO_PIN_CS, UART_HandleTypeDef* uart)
{
	SPI_WIZCHIP = spi;
	GPIO_SPI_CS_WIZCHIP = GPIO_CS;
	GPIO_Pin_SPI_CS_WIZCHIP = GPIO_PIN_CS;

	UART_WIZCHIP = uart;

	uint8_t gDATABUF[DATA_BUF_SIZE] = {1, 0, 1, 0};

	uint8_t tmp;
	int32_t ret = 0;
	uint8_t memsize[2][8] = {{16,0,0,0,0,0,0,0},{16,0,0,0,0,0,0,0}}; //

	 // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP //
	   ////////////////////////////////////////////////////////////////////////////////////////////////////
	   /* Critical section callback - No use in this example */
	   reg_wizchip_cris_cbfunc(0, 0);
	   /* Chip selection call back */
	#if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
	    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	#elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
	    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_select);  // CS must be tried with LOW.
	#else
	   #if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
	      #error "Unknown _WIZCHIP_IO_MODE_"
	   #else
	      reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	   #endif
	#endif
	    /* SPI Read & Write callback function */
	    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
	    reg_wizchip_spiburst_cbfunc(wizchip_read_burst, wizchip_write_burst);

	    uint8_t ret_version = 0;
	    ret_version = getVERSIONR();
	    if(ret_version != 0x4)
	    {
	    	//Error read SPI
	    	while(1);
	    }

	    ////////////////////////////////////////////////////////////////////////
	    /* WIZCHIP SOCKET Buffer initialize */
	        if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
	        {
	        	//UART_Printf("WIZCHIP Initialized fail.\r\n");

	           while(1);
	        }

	        /* PHY link status check */
	        do
	        {
	           if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1);

	        	   //UART_Printf("Unknown PHY Link stauts.\r\n");

	        }while(tmp == PHY_LINK_OFF);
	        /* Network initialization */

	        network_init();

	        /* Main loop */
	        while(1)
	        {
	        	if( (ret = loopback_tcps_server(0, gDATABUF, 5000)) < 0)
	        	{
	        		UART_Printf("SOCKET ERROR : %ld\r\n", ret);
	        	}

	        	// Uncomment if want TCP client
	        	/*if( (ret = loopback_tcps_client(0, gDATABUF, 5000)) < 0)
	        	{
	        		UART_Printf("SOCKET ERROR : %ld\r\n", ret);
	        	}*/


	        	/* Uncomment if want UDP
	        	if( (ret = loopback_udp(0, gDATABUF, 5000)) < 0)
	        	{
	        		//UART_Printf("SOCKET ERROR : %ld\r\n", ret);
	        	}
	        	*/

	        } // end of Main loop

}


/* Set Low CS */
void  wizchip_select(void)
{
   GPIO_SPI_CS_WIZCHIP->BSRR = GPIO_Pin_SPI_CS_WIZCHIP << 16U;
}
/* Set High CS*/
void  wizchip_deselect(void)
{
	GPIO_SPI_CS_WIZCHIP->BSRR = GPIO_Pin_SPI_CS_WIZCHIP;
}

void  wizchip_write(uint8_t wb)    //Write SPI
{
	HAL_SPI_Transmit( SPI_WIZCHIP, &wb, 	1 , HAL_MAX_DELAY);
}

uint8_t wizchip_read() //Read SPI
{
	uint8_t spi_read_buf;
    HAL_SPI_Receive (SPI_WIZCHIP, &spi_read_buf, 1, HAL_MAX_DELAY);
    return spi_read_buf;

}

void wizchip_read_burst(uint8_t* pBuf, uint16_t len) //Read SPI
{
	HAL_SPI_Receive (SPI_WIZCHIP, pBuf, len, HAL_MAX_DELAY);
}

void wizchip_write_burst(uint8_t* pBuf, uint16_t len) //Read SPI
{
	HAL_SPI_Transmit(SPI_WIZCHIP, pBuf, len, HAL_MAX_DELAY);
}

/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
	uint8_t rx_tx_buff_sizes[] = {16, 0, 0, 0, 0, 0, 0, 0};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
    uint8_t tmpstr[6];
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

//	HAL_Delay(50);
//	UART_Printf("\r\n=== %s NET CONF ===\r\n",(char*)tmpstr);
//	HAL_Delay(10);
//	UART_Printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],
//			  gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
//	HAL_Delay(10);
//	UART_Printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
//	HAL_Delay(10);
//	UART_Printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
//	HAL_Delay(10);
//
//	UART_Printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
//	HAL_Delay(10);
//	UART_Printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
//	UART_Printf("======================\r\n");

}
/////////////////////////////////////////////////////////////

int32_t loopback_tcps_server(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint16_t size = 0;

   switch(getSn_SR(sn))   //Проверить состояние сокета sn
   {

      case SOCK_ESTABLISHED :

    	 /* Check physical state cabel Ethernet */
    	 if((getPHYCFGR() & PHYCFGR_LNK_ON)== 0)
    	 {
    	     close(sn);
    	 }
		 /* Check If new client Connected */
         if(getSn_IR(sn) & Sn_IR_CON)
         {
        	// UART_Printf("%d:Connected\r\n",sn);
            setSn_IR(sn,Sn_IR_CON);
         }
//------------------------------------------------------------//
//-------------- Example Mirror ------------------------------//
//------------------------------------------------------------//
         /* Receive data */
         if((size = getSn_RX_RSR(sn)) > 0)
         {
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recv(sn,buf,size);	//function receive data
            if(ret <= 0) return ret;
            ret = send(sn,buf,size);	//function transmit data
            if(ret <= 0) return ret;
         }


//------------------------------------------------------------//
         break;

      case SOCK_CLOSE_WAIT :

         if((ret=disconnect(sn)) != SOCK_OK) return ret;

         break;

      case SOCK_INIT :

         if( (ret = listen(sn)) != SOCK_OK) return ret;  //слушаем сокет

         break;
      case SOCK_CLOSED:
    	  if((ret=socket(sn,Sn_MR_TCP,port,0x00)) != sn)  return ret;
    	  // UART_Printf("%d:Opened\r\n",sn);
		  //UART_Printf("%d:LBTStart\r\n");

		  setSn_KPALVTR(sn, 10);

         break;
      default:
         break;
   }
   return 1;
}

int32_t loopback_tcps_client(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint16_t size = 0;

   switch(getSn_SR(sn))
   {

   case SOCK_ESTABLISHED :
	   /* Check physical state cabel Ethernet */
	   if((getPHYCFGR() & PHYCFGR_LNK_ON)== 0)
	   {
		   close(sn);
	   }
	   /* Check If new client Connected */
	   if(getSn_IR(sn) & Sn_IR_CON)
	   {
		   // UART_Printf("%d:Connected\r\n",sn);
		   setSn_IR(sn,Sn_IR_CON);
	   }

//------------------------------------------------------------//
//-------------- Example Mirror ------------------------------//
//------------------------------------------------------------//
	   /* Receive data */
	   if((size = getSn_RX_RSR(sn)) > 0)
	   {
		   if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
		   ret = recv(sn,buf,size);	//function receive data
		   if(ret <= 0) return ret;
	   }
	   ret = send(sn,buf,size);	//function transmit data
	   if(ret <= 0) return ret;
//------------------------------------------------------------//

	   break;

   case SOCK_CLOSE_WAIT :	//Состояние закрытия сокета
	   //UART_Printf("%d:CloseWait\r\n",sn_dst);
	   if((ret=disconnect(sn)) != SOCK_OK) return ret;
	   //UART_Printf("%d:Closed\r\n",sn_dst);

	   break;

   case SOCK_INIT :
	   ;
	   //UART_Printf("%d:Try connect to the server, port [%d]\r\n",sn, port);
	   uint8_t ipServer[4] = {169, 254, 153, 203};   // Dst IP Address
	   //Connect to the Server
	   if( (ret = connect(sn, ipServer, port)) != SOCK_OK)
	   {
		   return ret;
	   }
	   //UART_Printf("%d:Connect, port [%d]\r\n",sn, port);
	   break;

   case SOCK_CLOSED:

	   /* Create socket 10800 - port this socket, range (5000 - ...)*/
	   if((ret=socket(sn,Sn_MR_TCP,10800,0x00)) != sn)  return ret;
	   break;

   default:
	   break;
   }

   return 1;
}



int32_t loopback_udp (uint8_t sn, uint8_t* buf, uint16_t port)
{
	int32_t ret = 0;
	uint16_t size = 0;

	switch(getSn_SR(sn))
	{
	case SOCK_UDP :
		if((size = getSn_RX_RSR(sn)) > 0)
		{
			if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
			ret = recvfrom(sn,buf,size,dstNetInfo.ip,(uint16_t*)&dstNetInfo.port);
			if(ret <= 0)
			{
				//UART_Printf("%d: recvfrom error. %ld\r\n",sn,ret);
				return ret;
			}

			ret = sendto(sn,buf, size, dstNetInfo.ip, dstNetInfo.port);

		}
		break;

	case SOCK_CLOSED:
		if((ret=socket(sn,Sn_MR_UDP,port,0x00)) != sn)
			return ret;
		break;

	default :
		break;
	}

	return ret;

}


/* Аналог printf, только все выводится через UART*/
void UART_Printf(const char* fmt, ...) {
    char buff[256]; //сюда пишем рез-т
    va_list args;  //переменные параметры
    va_start(args, fmt); //макрос, который считает, что все параметры
    //после fmt - переменные параметры
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(UART_WIZCHIP, (uint8_t*)buff, strlen(buff),
                      1000);
    va_end(args);
}



