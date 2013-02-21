
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_rcc.h"

#include "display.h"



void ResetFIFO( unsigned short usTab[] )
{
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  GPIOD->ODR &= (~(1 << 7));		// /WE	->0
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  GPIOB->ODR |= (1 << 5);			// /WST	->1
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  GPIOB->ODR &= (~(1 << 5));		// /WST	->0
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  GPIOD->ODR |= (1 << 7);			// /WE	->1	
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
  GPIOB->ODR |= (1 << 5);			// /WST	->1
  usTab[0] = 0x0;
  usTab[1] = 0x0;
  usTab[2] = 0x0;
  usTab[3] = 0x0;
}

void DisplayPic( unsigned short usTab[], unsigned char ucPicData[] )
{
  unsigned int i;

  usTab[0] = 0xffff;
  GPIOD->ODR &= (~(1 << 7));		// /WE	->0
  for( i = 0; i < 184320/2; i++ )
  {
	   usTab[i] = ((unsigned short*)ucPicData)[i%256];
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   usTab[i] = ((unsigned short*)ucPicData)[i%256];
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   usTab[i] = ((unsigned short*)ucPicData)[i%256];
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   usTab[i] = ((unsigned short*)ucPicData)[i%256];
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   usTab[i] = ((unsigned short*)ucPicData)[i%256];
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   usTab[i] = ((unsigned short*)ucPicData)[i%256];
  }
  GPIOD->ODR |= (1 << 7);			// /WE	->1
  usTab[0] = ((unsigned short*)ucDotData)[0];			// Last valid data
}


void DisplayNum( unsigned short usTab[], unsigned char ucNum )
{
  unsigned int i,j,uiTensDigi,uiOneDigi,uiTmpX,uiTmpY;
  unsigned short usTmp;

  usTab[0] = 0xffff;
  GPIOD->ODR &= (~(1 << 7));		// /WE	->0
  for( i = 0; i < 184320/2; i++ )
  {
	   uiTensDigi = ucNum / 10;
	   uiOneDigi = ucNum % 10;
	   j = i & ((1 << 8) - 1);
	   uiTmpX = ucDotDataNum[uiOneDigi][j<<1] & 0x0f;
	   if( uiTmpX == 6 )
	   {
	   	   uiTmpX = 0;
	   }
	   uiTmpY = (ucDotDataNum[uiTensDigi][j<<1] >> 4);
	   if( uiTmpY == 6 )
	   {
	   	   uiTmpY = 0;
	   }
	   usTmp = ( ( uiTmpX << 4 ) | ( uiTmpY & 0x0f) ) & 0xff;
	   usTab[j] = usTmp;
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   uiTensDigi = ucNum / 10;
	   uiOneDigi = ucNum % 10;
	   j = i & ((1 << 8) - 1);
	   uiTmpX = ucDotDataNum[uiOneDigi][j<<1] & 0x0f;
	   if( uiTmpX == 6 )
	   {
	   	   uiTmpX = 0;
	   }
	   uiTmpY = (ucDotDataNum[uiTensDigi][j<<1] >> 4);
	   if( uiTmpY == 6 )
	   {
	   	   uiTmpY = 0;
	   }
	   usTmp = ( ( uiTmpX << 4 ) | ( uiTmpY & 0x0f) ) & 0xff;
	   usTab[j] = usTmp;
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   uiTensDigi = ucNum / 10;
	   uiOneDigi = ucNum % 10;
	   j = i & ((1 << 8) - 1);
	   uiTmpX = ucDotDataNum[uiOneDigi][j<<1] & 0x0f;
	   if( uiTmpX == 6 )
	   {
	   	   uiTmpX = 0;
	   }
	   uiTmpY = (ucDotDataNum[uiTensDigi][j<<1] >> 4);
	   if( uiTmpY == 6 )
	   {
	   	   uiTmpY = 0;
	   }
	   usTmp = ( ( uiTmpX << 4 ) | ( uiTmpY & 0x0f) ) & 0xff;
	   usTab[j] = usTmp;
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   uiTensDigi = ucNum / 10;
	   uiOneDigi = ucNum % 10;
	   j = i & ((1 << 8) - 1);
	   uiTmpX = ucDotDataNum[uiOneDigi][j<<1] & 0x0f;
	   if( uiTmpX == 6 )
	   {
	   	   uiTmpX = 0;
	   }
	   uiTmpY = (ucDotDataNum[uiTensDigi][j<<1] >> 4);
	   if( uiTmpY == 6 )
	   {
	   	   uiTmpY = 0;
	   }
	   usTmp = ( ( uiTmpX << 4 ) | ( uiTmpY & 0x0f) ) & 0xff;
	   usTab[j] = usTmp;
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   uiTensDigi = ucNum / 10;
	   uiOneDigi = ucNum % 10;
	   j = i & ((1 << 8) - 1);
	   uiTmpX = ucDotDataNum[uiOneDigi][j<<1] & 0x0f;
	   if( uiTmpX == 6 )
	   {
	   	   uiTmpX = 0;
	   }
	   uiTmpY = (ucDotDataNum[uiTensDigi][j<<1] >> 4);
	   if( uiTmpY == 6 )
	   {
	   	   uiTmpY = 0;
	   }
	   usTmp = ( ( uiTmpX << 4 ) | ( uiTmpY & 0x0f) ) & 0xff;
	   usTab[j] = usTmp;
  }
  for( i = 0; i < 184320/2; i++ )
  {
	   uiTensDigi = ucNum / 10;
	   uiOneDigi = ucNum % 10;
	   j = i & ((1 << 8) - 1);
	   uiTmpX = ucDotDataNum[uiOneDigi][j<<1] & 0x0f;
	   if( uiTmpX == 6 )
	   {
	   	   uiTmpX = 0;
	   }
	   uiTmpY = (ucDotDataNum[uiTensDigi][j<<1] >> 4);
	   if( uiTmpY == 6 )
	   {
	   	   uiTmpY = 0;
	   }
	   usTmp = ( ( uiTmpX << 4 ) | ( uiTmpY & 0x0f) ) & 0xff;
	   usTab[j] = usTmp;
  }
  GPIOD->ODR |= (1 << 7);			// /WE	->1
  usTab[0] = 0x00;					// Last valid data
}
