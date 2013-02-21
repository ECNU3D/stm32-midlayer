#include "stm32f10x.h"
#include "led_485.h"
#define USART_FLAG_TXE                       ((uint16_t)0x0080)

__inline void DMA_UARTTX(struct SENDTYPE* p);
__inline void Send485(struct SENDTYPE* p);
__inline void Send485_2(struct SENDTYPE *p);
struct SENDTYPE m_SendBuffer;

__inline void GPIODelay(u16 t)
{
  volatile long a;
  for (a=0;a<t;a++)
   {}
}
void SendDataAll(u16 dd)
{
  CSALLOFF();
  GPIOE->ODR=dd;
  GPIODelay(DELAYTIME);
  CSALLON();
  GPIODelay(DELAYTIME);
  CSALLOFF();
  GPIODelay(DELAYTIME);
}
__inline void SendDataCS0(u8 dd)
{
  CS0OFF();
  GPIOE->ODR =dd;
  GPIODelay(DELAYTIME);
  CS0ON();
  GPIODelay(DELAYTIME);
}
__inline void SendDataCS1(u8 dd)
{
  u16 t;
  t=dd;
  t=t<<8;
  CS1OFF();
  GPIOE->ODR =t;//(((u16)dd)<<8);
  GPIODelay(DELAYTIME);
  CS1ON();
  GPIODelay(DELAYTIME);
}

__inline void SetAdr485(struct SENDTYPE *p,u16 a)
{
  p->ADR=a;  
}

__inline void SetMode485(struct SENDTYPE *p,u8 a)
{
  p->HOST=a;  
}
__inline void SetCMD485(struct SENDTYPE *p,u8 a)
{
  p->CMD=a;  
}
 void SetData(struct SENDTYPE *p,u8 a,u16 b,u8 c)
{

  p->HOST=a; 
  p->ADR=b ;
  p->CMD=c;
  Send485_2(p);
}
void Send485_2(struct SENDTYPE *p)
{
  u8 *p1;
  p1=(u8*)p;
   USART_SendData(USART2, *p1);
   //while (!(USART1->SR & USART_FLAG_TXE));
       while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
    }
    
   p1++	;
     USART_SendData(USART2, *p1);
	     while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
    }
    
  // while (!(USART1->SR & USART_FLAG_TXE));
}
__inline void Send485(struct SENDTYPE* p)
{

    DMA_Cmd(DMA1_Channel4, DISABLE);
   // DMA_Configuration2(p);
	DMA_Cmd(DMA1_Channel4, ENABLE);	
    while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);	  
}
__inline void FillHeader(struct SENDTYPE* p)
{
// p->HEADER=THEADER;
// p->TAIL=TTAIL;
}
__inline void SetAdr(struct SENDTYPE* p,int adr)
{
  p->ADR=adr;
}
__inline void SetCmd(struct SENDTYPE* p,int Cmd1,int Cmd2)
{
//  p->CMD1=Cmd1;
//  p->CMD2=Cmd2;
}

__inline void FillData(struct SENDTYPE*p,u32 p1)
{

// p->DATA=p1;
}
__inline void LedOn(struct SENDTYPE*p,int adr)
{
  FillHeader(p);
  SetAdr(p,adr);
  SetCmd(p,CMD_LEDON,0);
  Send485(p);
}
__inline void LedOff(struct SENDTYPE*p,int adr)
{
   FillHeader(p);
  SetAdr(p,adr);
  SetCmd(p,CMD_LEDOFF,0);
  Send485(p); 
}
 void DMA_UARTTX(struct SENDTYPE* p)
{
  DMA_InitTypeDef DMA_InitStructure;
 // DMA_DeInit(DMA1_Channel4);  
 // DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)p;
 // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
 // DMA_InitStructure.DMA_BufferSize = SENDSIZE;
 // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
 // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
 // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
 // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
 // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
 // DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
 // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
}
/*---end of file led_485.c------------2010/3/1------------*/
