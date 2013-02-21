/**
  ******************************************************************************
  * @file    SDIO/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ff.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_spi.h"
#include "sdcard.h"
#include "led_485.h"
#include "stm32f10x_usart.h"
void ProcessData(void);
extern  int TmCnt;
struct SENDTYPE Team485[]; 
extern  struct SENDTYPE Data485;
extern int TeamLeft;
extern int TeamRead;
extern int TeamWrite;
extern int MYADDR;
extern unsigned char NeedNewData;
extern unsigned char bNeedNewData;
extern u8 UartBuffer[];
extern int CurDataMode;
extern struct _SPIPacket SPIData[TEAMLENGTH];
extern  long CurDataPoint;		
extern int m_Mode;
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles SDIO global interrupt request.
  * @param  None
  * @retval None
  */
 void SPI1_IRQHandler(void)
{
  unsigned char t;
  static int CurPa=0;
  struct _SPIPacket s;
   unsigned char *p;
   SPI_I2S_ClearITPendingBit(SPI1,SPI_I2S_IT_RXNE);
  p=(unsigned char *)&s;
  if (m_Mode==0)
  {
	  t=SPI_I2S_ReceiveData(SPI1);				  //receive data
	  if (CMD_SPI_ANG0==t)						 // 0 angle  command received
	   {
	      if (1==bNeedNewData)					  //if the previous data not finished then assert an error
		   {
		     //error  speed is not enough ,let can to tell
			 bNeedNewData=1;
		   }
		  else
		  {
	        bNeedNewData=1;					     
			s.Cmd=CMD_SYN;
			PostMsg(s);							 //post the SYN command to read and put data into fifo
			
		  }
	   }
	  if (t==2)						 //used to test, nothing happend
	  { 
	    t=1;
	  }
  }
  if (m_Mode==1)
  {
      p[CurPa]=SPI_I2S_ReceiveData(SPI1);
	  CurPa++;
	  if (CurPa>=16)
	  {
	     CurPa=0;
		 if (SPIData[CurDataPoint].Adr!=MYADDR && SPIData[CurDataPoint].Adr!=0)		   //if the data is not for my , then IGNORE it
            return;
		 PostMsg(s);	
	  }
  }
}
void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

void TIM4_IRQHandler(void)							  //NOT USED
{
  	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	TmCnt++;
}
void DMA1_Channel5_IRQHandler(void)
{
  
  DMA_ClearITPendingBit(DMA1_IT_TC5);
  ProcessData();
}

void DMA1_Channel2_IRQHandler(void)
{
  
  DMA_ClearITPendingBit(DMA1_IT_TC2);
  if (SPIData[CurDataPoint].Adr!=MYADDR && SPIData[CurDataPoint].Adr!=0)		   //if the data is not for my , then IGNORE it
    return;	 
  if (SPIData[CurDataPoint].Cmd==CMD_SYN && SPIData[CurDataPoint].pData.pData8[0]!=0)	//not used (because we have changed the SYN way)
   return;
  CurDataPoint++;
  if (CurDataPoint>=TEAMLENGTH) CurDataPoint=0;		  //NOTICE that the DMA will put the data into the team automatically
  TeamLeft++;			 							 // DMA will put the data into the team follow the "CurDataPoint" 							   
  InitDMA(); 							     //restart DMA
}										   

void USART1_IRQHandler(void)
{
  u8 *pp;
  static int CurDCount;
  //pp=(u8*)&Data485;

    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
   pp=(u8*)&Data485;
    if (CurDCount==0)
	{
	   
      *pp=USART_ReceiveData(USART1);
	  CurDCount=1;
	  return;
 
    }
	else
	{
	   pp++;
	  *pp=USART_ReceiveData(USART1);
	  CurDCount=0;
	}

 // u16 temp1,temp2,temp3,temp4;
/*
  *pp=USART_ReceiveData(USART1);
   USART_ClearITPendingBit(USART1,USART_IT_RXNE);
   pp++;
  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {}
  *pp =USART_ReceiveData(USART1);
  */
  ProcessData();
//  TESTLED1OFF;
//  TESTLED2OFF;
//	 TESTLED3OFF;
 // USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}
void ProcessData()
{
  long t;
  long t1=0;
  static int jj;
  
 
  if ((Data485.HOST==TOALL) && (Data485.ADR==CMD_TESTCOM) && (Data485.CMD==CMD_TESTCOM))
   {				  //for test
    if (jj==0)
	{
     TESTLED1ON;
	 TESTLED2ON;
	 TESTLED3ON;
	 jj=1;
	 }
	 else
	 {
	 jj=0;
	 TESTLED1OFF;
	 TESTLED2OFF;
	 TESTLED3OFF;
	 }
	 return;
   }
  //if (Data485.HOST==TOALL || Data485.HOST==TOMIDLAYER || (Data485
 if (Data485.HOST==0 && Data485.ADR==0 && Data485.CMD==0)
  {
    ShowErrMsg(3);
	return;
  }

  if (Data485.HOST==TODISPLAY) return;
  if ((Data485.HOST==TOADR) && (Data485.ADR!=MYADDR)) return;
  //以上忽略数据

  //在中断里面处理优先数据
  if (Data485.CMD==CMD_SYN ||Data485.CMD==CMD_SYN2) 	//同步命令
   {
      if (Data485.ADR==0)
	    {
		    if(NeedNewData>0)
			  ShowErrMsg(3);   //速度不够
			if (NeedNewData!=2)  //2为暂停模式
		    NeedNewData=1;
		}
	  return ;  //同步命令不进入消息队列
   }
   else if (Data485.CMD==CMD_LEDMODEGRAY)
    {
	  if (Data485.ADR>0)
	    {  //该命令带表模拟数据
		   CurDataMode=2+Data485.ADR;
		}
	   else
	    {
		   CurDataMode=1;
		}
	  return;
	}
	else if (Data485.CMD==CMD_LEDMODEMONO)
	 {
	    CurDataMode=0;
	 }

  //其它命令以队列方式逐个执行
  if (TeamLeft<TEAMLENGTH)
  {
	  TeamLeft++;
	  //memcpy(Team485[TeamWrite],Data485,UARTSIZE);
	  Team485[TeamWrite].HOST=Data485.HOST;
	  Team485[TeamWrite].ADR=Data485.ADR;
	  Team485[TeamWrite].CMD=Data485.CMD;
	  TeamWrite++;
	  if (TeamWrite>=TEAMLENGTH) TeamWrite=0;
  }

}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
