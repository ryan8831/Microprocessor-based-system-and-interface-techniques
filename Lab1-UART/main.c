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

void SYS_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();// ��delay�M

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    //(1UL<<0) | (1UL<<1)
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
}


int main()
{
    int8_t ch;

    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
    printf("Please choose LED ON or OFF:\n");
    GPIO_SetMode(PB, BIT13, GPIO_PMD_OUTPUT);
    do
    {
        printf("Input: ");
        ch = getchar();
        printf("%c\n", ch);
        if(ch=='1'){
        	PB13=1;
        	printf("LED ON\n");
        }
        else if(ch=='0'){
            PB13=0;
            printf("LED OFF\n");
        }
    }
    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
