/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/06/30 4:51p $
 * @brief    Software Development Template.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC131.h"
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_I2C0HandlerFn = NULL;
void SYS_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();
    CLK_EnableModuleClock(I2C0_MODULE);
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    //SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA8_Msk | SYS_GPA_MFP_PA9_Msk);
    SYS->GPA_MFP = SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;
}

void ADXL345_Init(void);
void I2C0_Init(void);
void I2C0_Close(void);
void I2C0_Write(uint8_t DeviceAddr, uint8_t ADDRESS, uint8_t DATA);
int8_t I2C0_Read(uint8_t DeviceAddr, uint8_t ADDRESS);
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);       //避免I2C超時
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);   //沒有超時則回傳I2C的狀態
    }
}

int main()
{
	uint8_t DEVID, X0, X1, Y0, Y1, Z0, Z1;
	float X_axis, Y_axis, Z_axis ;
	uint8_t temp;
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    I2C0_Init();
    /* Lock protected registers */
    SYS_LockReg();
    ADXL345_Init();
    printf( "I2C ADXL\n");
    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
	temp = I2C0_Read(0x53, 0x2D );
	/*printf ( "TEMP==%X\n  ", temp);
	printf("0x31 value  is (0x%X)\n", I2C0_Read(0x53, 0x31));
	printf("0x38 value  is (0x%X)\n", I2C0_Read(0x53, 0x38));*/
	   uint8_t devid;
	   devid = I2C0_Read(0x53, 0x00); // 0x53 是 ADXL345 的 I2C 地址，0x00 是 DEVID 寄存器地址

	    /* 打印結果 */
	    if (devid == 0xE5) {
	        printf("ADXL345 DEVID read successfully: 0x%X\n", devid);
	    } else {
	        printf("ADXL345 DEVID read failed: 0x%X\n", devid);
	    }

	while(1){
		/* Read registers Data  */

		X0 = I2C0_Read(0x53, 0x32);
		X1 = I2C0_Read(0x53, 0x33);
		Y0 = I2C0_Read(0x53, 0x34);
		Y1 = I2C0_Read(0x53, 0x35);
		Z0 = I2C0_Read(0x53, 0x36);
		Z1 = I2C0_Read(0x53, 0x37);
		/* XYZ Data offset */

		/*X_axis = ((X1 << 8) & 0xFF00) | (X0 & 0x00FF);
		Y_axis = ((Y1 << 8) & 0xFF00) | (Y0 & 0x00FF);
		Z_axis = ((Z1 << 8) & 0xFF00) | (Z0 & 0x00FF);
		X_axis = (float)X_axis / 256.0;
		Y_axis = (float)Y_axis / 256.0;
		Z_axis = (float)Z_axis / 256.0;*/
		/*X X_axis = (((X1 << 8) | X0) /(256)) ;
		Y_axis = (((Y1 << 8) | Y0) /(256)) ;
		Z_axis = (((Z1 << 8) | Z0) /(256)) ; */
		X_axis = (float)((int16_t)((X1 << 8) | X0)) / 256.0;
		Y_axis = (float)((int16_t)((Y1 << 8) | Y0)) / 256.0;
		Z_axis = (float)((int16_t)((Z1 << 8) | Z0)) / 256.0;

		/*printf("%X  %X\n", X1, X0);
		printf("%X  %X\n", Y1, Y0);
		printf("%X  %X\n", Z1, Z0);*/
		//printf("X: %.2f, Y: %.2f, Z: %.2f \n", X_axis, Y_axis, Z_axis);
		printf("X: %.2f, Y: %.2f, Z: %.2f \n", X_axis, Y_axis, Z_axis);
		//getchar();
		//printf("X: %d, Y: %d, Z: %d \n", X_axis, Y_axis, Z_axis);
		/* delay */
		CLK_SysTickDelay(5000000);
		CLK_SysTickDelay(5000000);
	}
	/* Close I2C0 */
	   I2C0_Close();
    while(1);
}
void I2C0_Write(uint8_t DeviceAddr, uint8_t ADDRESS, uint8_t DATA)
{
				/* I2C as master sends START signal */
				I2C_START(I2C0);
				/*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write DeviceAddr to Register I2CDAT */
				I2C_SET_DATA(I2C0, DeviceAddr << 1);
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	      /*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write ADDRESS to Register I2CDAT */
				I2C_SET_DATA(I2C0, ADDRESS);
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	      /*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write DATA to Register I2CDAT */
				I2C_SET_DATA(I2C0, DATA);
	      /* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
				/*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write STOP to Register I2CDAT */
				I2C_STOP(I2C0);
}

int8_t I2C0_Read(uint8_t DeviceAddr, uint8_t ADDRESS)
{
		        int8_t TEMP;
				/* I2C as master sends START signal */
				I2C_START(I2C0);
				/*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write DeviceAddr to Register I2CDAT */
				I2C_SET_DATA(I2C0, DeviceAddr  << 1);
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	      /*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write ADDRESS to Register I2CDAT */
				I2C_SET_DATA(I2C0, ADDRESS);
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	      /*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* I2C as master sends START signal */
				I2C_START(I2C0);
				/*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write DeviceAddr to Register I2CDAT */
				I2C_SET_DATA(I2C0, ((DeviceAddr << 1) | 0x01));
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	      /*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write 0xFF to Register I2CDAT */
				I2C_GET_DATA(I2C0) = 0xFF;
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
	      /*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/*  I2C GET DATA */
				TEMP = I2C_GET_DATA(I2C0);
				/* I2C as master sends SI = 0 */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
				/*  I2C bus status get ready */
				I2C_WAIT_READY(I2C0);
				/* Write STOP to Register I2CDAT */
				I2C_STOP(I2C0);

				return TEMP;
}

void ADXL345_Init(void)
{
    /* Init  ADXL345 module  */
	  printf( "I2C ADXL\n");
      I2C0_Write(0x53, 0x2D, 0x08);
	  I2C0_Write(0x53, 0x31, 0x0B);
	  I2C0_Write(0x53, 0x38, 0x80);
	  printf("0x2D value  is (0x%X)\n", I2C0_Read(0x53, 0x2D));
	  printf("0x31 value  is (0x%X)\n", I2C0_Read(0x53, 0x31));
	  printf("0x38 value  is (0x%X)\n", I2C0_Read(0x53, 0x38));
	  printf("DEVID value is (0x%X)\n", I2C0_Read(0x53, 0x00));
	  printf("ADXL345 Init is OK\n");
	  printf("Start!\n");
	  CLK_SysTickDelay(5000000);
	  CLK_SysTickDelay(5000000);

}
void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Enable I2C interrupt */
		/* I2C_EnableInt(I2C0); */
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
