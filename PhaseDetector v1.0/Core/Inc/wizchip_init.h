/*
 * wizchip_init.h
 *
 *  Created on: 12 ���. 2020 �.
 *      Author: ���������
 */

#ifndef WIZCHIP_INIT_H_
#define WIZCHIP_INIT_H_

#include "socket.h"
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>  //��� atoi()
#include "wizchip_conf.h"
///////////////////////////////////////////////
//Select the library for your stm32fxxx_hal.h//
///////////////////////////////////////////////

#include "stm32f4xx_hal.h"

//#include "write_flash.h"
#include "main.h"

/////////////////////////////
//Write the necessary ports//
/////////////////////////////

SPI_HandleTypeDef* SPI_WIZCHIP;
UART_HandleTypeDef* UART_WIZCHIP;
GPIO_TypeDef* GPIO_SPI_CS_WIZCHIP;
uint16_t  GPIO_Pin_SPI_CS_WIZCHIP;

//-----------------------------------------------------------------------------//
/////////////////////////////////////////
// SOCKET NUMBER DEFINION for Examples //
/////////////////////////////////////////
#define SOCK_TCPS        0
#define SOCK_UDPS        1

////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   2048


////////////////////////////////////////////////
//-------------------------------------------------------------------//
//      _Ethernet - ��� ������ ���������� �������� � Ethernet        //
//		_UART - ��� ������ ���������� �������� � UART
//-------------------------------------------------------------------//

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
////////////////////////////////////////////////

/* Private function prototypes -----------------------------------------------*/
void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();
void wizchip_read_burst(uint8_t* pBuf, uint16_t len);
void wizchip_write_burst(uint8_t* pBuf, uint16_t len);

void w5500_writeReg(uint8_t op, uint16_t addres, uint8_t data);

void UART_Printf(const char* fmt, ...);
void WizchIP_main(SPI_HandleTypeDef* spi,GPIO_TypeDef *GPIO_CS, uint16_t GPIO_PIN_CS, UART_HandleTypeDef* uart);
//////////////////////////////////
// For example of ioLibrary_BSD //
//////////////////////////////////
void network_init(void);								// Initialize Network information and display it
int32_t loopback_tcps_server(uint8_t sn, uint8_t* buf, uint16_t port);	// Loopback TCP server
int32_t loopback_tcps_client(uint8_t sn, uint8_t* buf, uint16_t port);	// Loopback TCP client
int32_t loopback_udp (uint8_t sn, uint8_t* buf, uint16_t port);
#endif /* WIZCHIP_INIT_H_ */
