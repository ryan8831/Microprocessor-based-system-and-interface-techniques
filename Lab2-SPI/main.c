/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 8 $
 * $Date: 15/01/16 1:45p $
 * @brief
 *           Implement SPI Master loop back transfer.
 *           This sample code needs to connect SPI0_MISO0 pin and SPI0_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC131.h"

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
void ADLX345_setup(void);
void registerWrite(char address,char value);
int8_t registerRead(uint8_t address);
int16_t Read_RawDataX(void);
int16_t Read_RawDataY(void);
int16_t Read_RawDataZ(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
	float x1,y1,z1,x,y,z;
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();
    /* Init ADLX345 */
   	ADLX345_setup();
   	uint8_t deviceID;
    deviceID = registerRead(0x00);

    printf("\nStart");
    if (deviceID == 0xE5) {
         printf("Device ID read successfully: 0x%X\n", deviceID);
     } else {
         printf("Device ID read failed: 0x%X\n", deviceID);
     }
   while(1)
   {
	        x1 =  Read_RawDataX();
	        y1 =  Read_RawDataY();
	        z1 =  Read_RawDataZ();
   			x = x1/32.0;
   			y = y1/32.0;
   			z = z1/32.0;
   			printf("\nx: %.2f, y: %.2f, z: %.2f", x,y,z);
   			CLK_SysTickDelay(50000000);
   	}

    SPI_Close(SPI0);
    while(1);
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to HXT and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select HCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL1_SPI0_S_HCLK, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
    SYS->GPC_MFP |= (SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP3_PC0_Msk | SYS_ALT_MFP3_PC1_Msk | SYS_ALT_MFP3_PC2_Msk | SYS_ALT_MFP3_PC3_Msk);
    SYS->ALT_MFP |= (SYS_ALT_MFP3_PC0_SPI0_SS0 | SYS_ALT_MFP3_PC1_SPI0_CLK | SYS_ALT_MFP3_PC2_SPI0_MISO0 | SYS_ALT_MFP3_PC3_SPI0_MOSI0);
    //27SCL        25SDA            26SDO        28 CS
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS_ACTIVE_LOW);
}
void registerWrite(char address,char value){
		SPI_WRITE_TX(SPI0,address); //((spi)->TX = (u32TxData))
		SPI_TRIGGER(SPI0);         //((spi)->CNTRL |= SPI_CNTRL_GO_BUSY_Msk)
		while(SPI_IS_BUSY(SPI0));  //檢查 GO_BUSY bit=0代表傳輸完成
		SPI_WRITE_TX(SPI0,value);
		SPI_TRIGGER(SPI0);
		while(SPI_IS_BUSY(SPI0));
}
void ADLX345_setup(void){
	 /*POWER_CTL(0x2D): 0x08*/
	 registerWrite(0x2D,0x08);  //啟動測量模式
	  /*DATA_FORMAT(0x31): 0x0B*/
	  registerWrite(0x31,0x0B); // ±16g 測量範圍 1011
	 /*FIFO_CTL(0x38): 0x80*/
	  registerWrite(0x38,0x80);
      printf("ADXL  Init\n");
}

int8_t registerRead(uint8_t address) {
    int8_t value;
    // 設置地址，讀取操作需要將最高位元設為 1
    address |= 0x80;

    SPI_WRITE_TX(SPI0, address);
    SPI_TRIGGER(SPI0);
    while (SPI_IS_BUSY(SPI0));

    SPI_WRITE_TX(SPI0, 0x00);
    SPI_TRIGGER(SPI0);
    while (SPI_IS_BUSY(SPI0));
    value = SPI_READ_RX(SPI0);   //  ((spi)->RX)
    return value;
}
int16_t Read_RawDataX(void){

	int8_t LoByte,HiByte;
    //#define SPI_WRITE_TX(spi, u32TxData)   ((spi)->TX = (u32TxData))
	SPI_WRITE_TX(SPI0, 0xB2);    //0x32  0011 0010 -> B2 1011  0011
	//觸發SCLK
    SPI_TRIGGER(SPI0);          //SPI0->CNTRL |= (1 << 0); 設GO_BUSY_MASK=1
    while(SPI_IS_BUSY(SPI0));   //檢查 GO_BUSY bit=0代表傳輸完成
    LoByte = SPI_READ_RX(SPI0);

	SPI_WRITE_TX(SPI0, 0xB3);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    HiByte = SPI_READ_RX(SPI0);
    return (int16_t)((HiByte << 8) | LoByte);
}

int16_t Read_RawDataY(void){

	int8_t LoByte,HiByte;

	SPI_WRITE_TX(SPI0, 0xB4);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    LoByte = SPI_READ_RX(SPI0);

	SPI_WRITE_TX(SPI0, 0xB5);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    HiByte = SPI_READ_RX(SPI0);
		return((HiByte<<8) | LoByte);
}

int16_t Read_RawDataZ(void){

	int8_t LoByte,HiByte;

	SPI_WRITE_TX(SPI0, 0xB6);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    LoByte = SPI_READ_RX(SPI0);

	SPI_WRITE_TX(SPI0, 0xB7);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0));
    HiByte = SPI_READ_RX(SPI0);


	return((HiByte<<8) | LoByte);
}
/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

