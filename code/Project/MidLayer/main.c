/**
  ******************************************************************************
  * @file    SDIO/main.c
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main program body
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
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_rcc.h"
#include "sdcard.h"
#include "led_485.h"
#include "ff.h"

#include "stdio.h"
#include "display.h"


/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
long CurDataPoint = 0;
long TData = 0;
uint16_t Tab[184512 / 184512] __attribute__((at(0x68000000))) ;
struct _SPIPacket tempStruct;
void PostMsg(struct _SPIPacket sp);
/* Private define ------------------------------------------------------------*/
//#define BlockSize            512 /* Block Size in Bytes */
//#define BufferWordsSize      (BlockSize >> 2)

//#define NumberOfBlocks       2  /* For Multi Blocks operation (Read/Write) */
//#define MultiBufferWordsSize ((BlockSize * NumberOfBlocks) >> 2)
int TmCnt = 0;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SD_CardInfo SDCardInfo;
//uint32_t Buffer_Block_Tx[BufferWordsSize], Buffer_Block_Rx[BufferWordsSize];
//uint32_t Buffer_MultiBlock_Tx[MultiBufferWordsSize], Buffer_MultiBlock_Rx[MultiBufferWordsSize];
volatile TestStatus EraseStatus = FAILED, TransferStatus1 = FAILED, TransferStatus2 = FAILED;
SD_Error Status = SD_OK;
void InitSD(void);
void InitUart(void);
void InitDMA(void);
void FSMC_Config(void);
void  ReadAndDispatch(void);
void Init_SPI(void);
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void  GPIO_Configuration(void);
USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
void Fill_Buffer(uint32_t *pBuffer, uint16_t BufferLenght, uint32_t Offset);
TestStatus Buffercmp(uint32_t *pBuffer1, uint32_t *pBuffer2, uint16_t BufferLength);
TestStatus eBuffercmp(uint32_t *pBuffer, uint16_t BufferLength);
u16 SDBuffer[5 * 2880 / 2]; //10K
long SDWritePoint = 0;
volatile u8 TestMode = 0;
char bDownloadMode = 0;
void ShowErrMsg(u8 a);
void EnterO0O(void);
int nosd;
int CurDataMode = 0;
unsigned long LoopCount = 1;
unsigned long CurLoop = 0;
struct _SPIPacket SPIData[TEAMLENGTH];
void updata(void);
unsigned char bNeedNewData = 0;
unsigned long FLength = 0;
unsigned long CurFPoint = 0;
struct _SPIPacket spp;
unsigned char *p;
u32 data[10 * 1024 / 4];
u8  *pdata;
int m_Mode = 1; //0 = normal mode  1 =  download mode
// long CurDataPoint=0;
void GDelay(u32 t)
{
    volatile u32 i;
    for (i = 0; i < t; i++)
    {}
}
u8 TestBuff[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program.
  * @param  None
  * @retval None
  */
FATFS fs;            // Work area (file system object) for logical drive
FRESULT res;         // FatFs function common result code
FRESULT fresult;
FIL g_sFileObject;
FIL g_Save;
// FIL g_sSD;
unsigned int nRead;
#define READSIZE 2880	  //8*180=1440*2=2880
char ReadBuff[READSIZE * 4];
unsigned char CurBuff = 0;
long CurBlock = 0;
int CurReadPoint = 0;
double Speed;
long tm;
long i, j;
struct SENDTYPE Data485;
int TeamLeft = 0;
int TeamRead = 0;
int TeamWrite = 0;
unsigned char NeedNewData = 0;
// struct SENDTYPE Team485[TEAMLENGTH];

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
int MYADDR = 0;
u16 temp;
int temp2 = 0;
int CurPa = 0;
char NameShow[20];
unsigned char bb;
#ifdef DEBUGJJ
u8  OutMsg[20];
int CA;
int c;

void DebugMsg(char *s);
int jjtemp = 0;
#endif
u16 Test1 = 1;
unsigned int uiData[10 * 1024 / 4];
unsigned char *ucData;
void GET_Address(void);


#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}

int main(void)
{
	union{

struct
{
unsigned short s1:3;
unsigned short s2:3;
unsigned short s3:3;
}x;
char c;
}v;
    int i;
v.x.s1=200;


    RCC_Configuration();
    NVIC_Configuration();
	printf("AD value ");
    GPIO_Configuration();
    GPIOA->ODR |= (1 << 8);
    Init_SPI();
    // InitDMA();

    //  GPIOA->ODR &= (~(1 << 8));
    //	while(1);
    InitSD();
    FSMC_Config();
    // ShowErrMsg(2);

    InitUart();

    //GET_Address();

    //MYADDR=1;  //assume address is 1
    TestMode = 0;

    GDelay(200000);

    //#pragma Otime
    //#pragma O0
    ResetFIFO( Tab );
    //#pragma Ospace
    //#pragma O0

    res = f_mount(0, &fs);


    GPIOD->ODR |= (1 << 7);		// /WE	->1
    Tab[0] = 0xffff;
    Tab[1] = 0xffff;

    temp = 0 ;

    //#pragma Otime
    //#pragma O3

    //GPIOA->ODR&=~(1<<8);
    RESET_FPGA060();

    fresult = f_open(&g_sFileObject, "Address.txt", FA_READ );			//open file
    if( fresult == FR_OK )
    {
        fresult = f_read(&g_sFileObject, (unsigned char *)&MYADDR, 1, &nRead);
    }
    else
    {
        MYADDR = 0;
    }
    f_close(&g_sFileObject);

    DisplayNum( Tab, MYADDR );

    //SET_FPGA060();

    //#pragma Otime
    //#pragma O3
    //Tab[255] = ((unsigned short *)uiData)[255];
    p = (unsigned char *)&spp;
    while(1)
    {
        if( m_Mode == 1 )
        {
            while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET )
            {
                Tab[0] = 0xffff;
            }
            p[CurPa] = SPI_I2S_ReceiveData(SPI1);
            CurPa++;
            if( CurPa >= 16 )
            {
                CurPa = 0;
                PostMsg(spp);
                ReadAndDispatch();
            }
        }
        else if( m_Mode == 0 )
        {
            unsigned int uiRecData;

            while( SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET )
            {
                Tab[0] = 0xffff;
            }
            uiRecData = SPI_I2S_ReceiveData(SPI1);
            if(	CMD_SPI_ANG0 == uiRecData )
            {
                spp.Cmd = CMD_SYN;
                PostMsg(spp);
                ReadAndDispatch();
            }
            else if( CMD_SPI_SWITCH == uiRecData )
            {
                m_Mode = 1;
                RESET_FPGA060();
            }

        }//End else if

        //read and process the message
    }// End while(1)

}




/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
    /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
       initialize the PLL and update the SystemFrequency variable. */
    SystemInit();
}

/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


}


void InitSD()
{
    /*-------------------------- SD Init ----------------------------- */
    Status = SD_Init();

    if (Status == SD_OK)
    {
        /*----------------- Read CSD/CID MSD registers ------------------*/
        Status = SD_GetCardInfo(&SDCardInfo);
    }

    if (Status == SD_OK)
    {
        /*----------------- Select Card --------------------------------*/
        Status = SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16));
    }

    if (Status == SD_OK)
    {
        Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
    }

    /* Set Device Transfer Mode to DMA */
    if (Status == SD_OK)
    {
        Status = SD_SetDeviceMode(SD_DMA_MODE);
    }
}

void InitUart()
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE);

    /* USART Configuration*/
    USART_InitStructure.USART_BaudRate = 7200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* Configure USART2 */
    USART_Init(USART2, &USART_InitStructure);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
}

void InitDMA()
{

    // #define DMA_Channel1        ((DMA_Channel_TypeDef *) DMA_Channel1_BASE)
    //  #define DMA_Channel2        ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define SPI1_DR_Address    0x4001300C

    static  DMA_InitTypeDef    DMA_InitStructure;
    /* DMA1 and DMA2 clock enable */

    //DMA_DeInit(DMA1_Channel2); 		 //DMA1--CHANNEL2 DEFINES FOR SPI RX

    //  CurDataPoint=0;
    if (CurDataPoint == 0)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
        DMA_DeInit(DMA1_Channel2);
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SPI1_DR_Address;
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&SPIData[CurDataPoint];
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = 16;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel2, &DMA_InitStructure);
        DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
    }
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&SPIData[CurDataPoint];		//put the data into the team directly
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel2, ENABLE);				     //DMA1_CH2 used to SPI1
}


void GPIO_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd((uint32_t)0x0000FFFD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Pa0---CS0  Pa1--CS1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //PB8-PB15 并行
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 | GPIO_Pin_15 ;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);
    //DATA线
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 | GPIO_Pin_15 ;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    //PG0,PG1	PF15--测试LED
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 ;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    //  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // GPIO_Init(GPIOF, &GPIO_InitStructure);
#ifdef DEBUGJJ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}
void ShowErrMsg(u8 a)
{

}

int Address;
void GET_Address(void)
{
    int t2, a;
    int la;

    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* ---------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 3600, TIM2 counter clock = 100Hz
    --------------------------------------------------------------- */

    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Prescaler configuration TIM2的时钟设定为100Hz，计数器的计量单位：1ms*/
    TIM_PrescalerConfig(TIM2, 35999, TIM_PSCReloadMode_Immediate);

    /*  */
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* TIM2 enable counter [允许tim2计数]*/
    TIM_Cmd(TIM2, ENABLE);

    while(1)
    {
        t2 = TIM_GetCounter(TIM2);
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != SET)		 // gai
        {
        }
        {
            Address = USART_ReceiveData(USART1);
            if(Address != 0)
            {
                MYADDR = Address;
                la = Address + 1;
                USART_SendData(USART2, la);
                while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
                {}
                break;
            }
        }
        //   if(t2>10000) break;
    }


}

void  ReadAndDispatch()
{
    u32 temp, i, j;

    if( TeamLeft > 0 )				   				//see whether there is message to process
    {
        TeamLeft--; 				 					//kick off the message
        switch( SPIData[TeamRead].Cmd )				//修改 process the msg
        {
        case CMD_DISTRIBUTEADR:
            if( (SPIData[TeamRead].Adr != MYADDR) && (SPIData[TeamRead].Adr != 0) )
            {
                break;
            }
            GET_Address();
            RESET_FPGA060();
            if( MYADDR != 0 )
            {
                fresult = f_open(&g_sFileObject, "Address.txt", FA_CREATE_ALWAYS | FA_WRITE );	//open file
                if( fresult == FR_OK )
                {
                    fresult = f_write(&g_sFileObject, (unsigned char *)&MYADDR, 1, &nRead);
                    if( fresult == FR_OK )
                    {
                        DisplayNum( Tab, MYADDR );
                    }
                }
                f_close(&g_sFileObject);
            }
            SET_FPGA060();
            break;
        case CMD_SYN:			   					//synchron command
            //#pragma Ospace
            //#pragma O0

#pragma O3

            GPIOD->ODR &= (~(1 << 7));		// /WE	->0
            fresult = f_read(&g_sFileObject, (unsigned char *)Tab, 184320, &nRead);		 //read 184320 BYTES and put it into the fifo
            GPIOD->ODR |= (1 << 7);			// /WE	->1
            Tab[1] = uiData[0];				// Last valid data

#pragma O0

            CurFPoint += 184320;

            if( nRead == 0 ) 						 //the same function (if reach the end ,goto head), not used
            {
                f_lseek(&g_sFileObject, 0);
                GPIOD->ODR &= (~(1 << 7));			 // /WE	->0
                fresult = f_read(&g_sFileObject, &Tab, 184320, &nRead);
                GPIOD->ODR |= (1 << 7);				 // /WE	->1
                Tab[1] = uiData[0];					 // Last valid data
                if( nRead == 0 )
                {
                    //error
                    nRead = 1;
                }
            }//End if
            bNeedNewData = 0;						 //data process complete
            break;
        case CMD_OPENFILE:					   		 //open the data file command, the data will be picked out from this file
            //		   if( SPIData[TeamRead].Adr!=MYADDR && SPIData[TeamRead].Adr != 0 )
            //		   {
            //		      break;
            //		   }
            if( SPIData[TeamRead].Adr != MYADDR && SPIData[TeamRead].Adr != 0 )
            {
                break;
            }
            else if( SPIData[TeamRead].Adr == 0 )
            {
                for( i = 0; i < 1; i++ )
                {
                    NameShow[i] = SPIData[TeamRead].pData.pData8[i];	   	//copy the name to open
                }
                NameShow[1] = 0x30 + (MYADDR / 10);
                NameShow[2] = 0x30 + (MYADDR % 10);
                NameShow[3] = '.';
                NameShow[4] = 'd';
                NameShow[5] = 'a';
                NameShow[6] = 't';
                fresult = f_open(&g_sFileObject, NameShow, FA_WRITE | FA_READ );	//open file
                FLength = g_sFileObject.fsize;		       						//log the file size
                CurFPoint = 0;							   							//current file point zero
                break;
            }

            for( i = 0; i < 10; i++ )
            {
                NameShow[i] = SPIData[TeamRead].pData.pData8[i];	   		//copy the name to open
            }
            fresult = f_open(&g_sFileObject, NameShow, FA_WRITE | FA_READ );	//open file
            FLength = g_sFileObject.fsize;		       						//log the file size
            CurFPoint = 0;							   							//current file point zero
            break;
        case CMD_SDDATASTART:				   		 //ready to fill the SD data
            nosd = 0;								 //Not
            if( SPIData[TeamRead].Adr != MYADDR && SPIData[TeamRead].Adr != 0 )
            {
                nosd = 1;
            }
            bDownloadMode = 1;				   		 //put the flag to notice into download data mode
            SDWritePoint = 0;
            TData = 0;
            //f_close(&g_sFileObject);
            for( i = 0; i < 10; i++)					  //copy the file name
            {
                NameShow[i] = SPIData[TeamRead].pData.pData8[i];
            }
            //sprintf (NameShow,"%_F15",MYADDR);
            if( nosd == 0 )
                fresult = f_open( &g_Save, NameShow, FA_WRITE | FA_READ );	  //open the file to append data
            if( fresult != FR_OK )
            {
                if( nosd == 0 )
                {
                    fresult = f_open(&g_Save, NameShow, FA_WRITE  | FA_READ | FA_CREATE_NEW ); //if the file doesn't exist then create it
                }
            }
            else
            {
                if( nosd == 0 )
                {
                    f_lseek(&g_Save, g_Save.fsize);	 //if the file already exists move the write point to the end of this file so as to append data
                }
            }
            RESET_FPGA060();
            updata();
            break;
        case  CMD_SDDATAEND:						  //SD data fill END command
            bDownloadMode = 0;
            // SDWritePoint=2880;
            for( i = 0; i < SPIData[TeamRead].Reserve[0]; i++ )	//to see how many bytes left (as the protocal the number is put in the first byte of the reserve data)
            {
                SDBuffer[SDWritePoint] = SPIData[TeamRead].pData.pData8[i];
                SDWritePoint++;							 		//continue to put the last datas
            }

            //DMA_Cmd(DMA1_Channel2, DISABLE);
            //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);
            //GDelay(20000);

            fresult = f_write(&g_Save, SDBuffer, SDWritePoint, &nRead);	 //write the data
            f_close(&g_Save);						//close the file

            //GDelay(20000);
            //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
            //DMA_Cmd(DMA1_Channel2, ENABLE);
            SDWritePoint = 0;						//write point zero
            break;
        case CMD_SDDATAING:						// filling the SD data command
            if( bDownloadMode == 1 )				// in fact this flag is not used now  (it is used in NINGBO version)
            {
                //是当前模块进入了烧写SD卡模式
                //   SDBuffer[SDWritePoint]=((Team485[TeamRead].ADR) & 0xff);	修改
                for( i = 0; i < 10; i++ )
                {
                    SDBuffer[SDWritePoint] = SPIData[TeamRead].pData.pData8[i];
                    SDWritePoint++;			   									//copy 10 BYTES into buffer
                }
                TData += 10;
                if( SDWritePoint >= 10220 )										//if buffer full then write once
                {
                    //缓冲区满了，写入一次数据
                    //DMA_Cmd(DMA1_Channel2, DISABLE);
                    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);
                    //GDelay(20000);
                    fresult = f_write(&g_Save, SDBuffer, SDWritePoint, &nRead);	 //write 10K buffer

                    //GDelay(20000);
                    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
                    //DMA_Cmd(DMA1_Channel2, ENABLE);

                    SDWritePoint = 0;					 							 //zero the write point
                }
            }//End if
            break;
        case CMD_ENTERSYN:			   	//enter synchron mode command
            if( (SPIData[TeamRead].Adr != MYADDR) && (SPIData[TeamRead].Adr != 0) )
            {
                break;
            }
            EnterO0O();			  		//close DMA  and open SPI interrupt to  enter synchron mode
            f_lseek(&g_sFileObject, 0);

            //ResetFIFO( Tab );
#pragma O3

            GPIOD->ODR &= (~(1 << 7));		// /WE	->0
            fresult = f_read(&g_sFileObject, &Tab, 184320, &nRead);
            GPIOD->ODR |= (1 << 7);			// /WE	->1
            Tab[0] = 0x0;

#pragma O0

            //GPIOA->ODR|=(1<<8);
            SET_FPGA060();
            break;
        }//End switch

        TeamRead++;					 			//read point ++
        if( TeamRead >= TEAMLENGTH )
        {
            TeamRead = 0;							//looop team
        }
    }//End if
}



void Init_SPI()
{
    SPI_InitTypeDef    	SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);


    /* Enable SPI1 */
    SPI_Cmd(SPI1, ENABLE);
}



void updata(void)
{
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t k = 0;
    uint32_t csize = 0;
    uint32_t size;
    u8 fsize[11];
    u8 l = 0, dou10 = 0;

    pdata = (u8 *)data;

mhead:
    i = 0;
    j = 0;
    k = 0;
    l = 0;
    dou10 = 0;

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI1);
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(SPI1);
    j++;
    j++;
    csize++;
    csize++;

    while(1)
    {
        //if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==SET)
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

        //{
        fsize[i++] = SPI_I2S_ReceiveData(SPI1);
        j++;
        csize++;
        //}
        if( i == 10 ) break;
    }

    size = fsize[3];
    size = fsize[2] | (size << 8);
    size = fsize[1] | (size << 8);
    size = fsize[0] | (size << 8);

    i = 0;

    while(1)
    {
        //if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==SET)
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

        {
            SPI_I2S_ReceiveData(SPI1);
            i++;
            j++;
            csize++;
        }
        if( i == 4 ) break;
    }

    i = 0;
    while(1)
    {
        //if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==SET)
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        {
            pdata[i++] = SPI_I2S_ReceiveData(SPI1);
            j++;
            csize++;
            //		if((j>10*1024)&&((j%(10*1024))>0)&&((j%(10*1024))<17))
            //		i--;
            //      if((j>10240&&j<10257)||(j>20480&&j<20497)||(j>30720&&j<30736))
            //		i--;

        }
        if( j >= 10 * 1024 )
        {
            if( nosd == 0 )
            {
                fresult = f_write(&g_Save, (unsigned char *)data, i, &nRead);
            }
            if( fresult != 0 )
            {
                SET_FPGA060();
                DisplayPic( Tab, (unsigned char *)ucDotDataKu );
                while(1)
                {}
            }
            if( csize < size )
            {
                goto mhead ;
            }
            i = 0;
            // break;
        }//End if
        if( csize >= size )
        {
            GDelay(2000);
            if( nosd == 0)
            {
                fresult = f_write(&g_Save, (unsigned char *)data, i, &nRead);
                fresult = f_close(&g_Save);
                SET_FPGA060();
                DisplayPic( Tab, (unsigned char *)ucDotDataXiao );
            }
            //		  if( nosd == 0 )
            //		  {
            //		 	  fresult=f_close(&g_Save);
            //		  }
            return;
        }//End if
    }//End while(1)
}


void FSMC_Config()
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOF, ENABLE);

    /*-- GPIO Configuration ------------------------------------------------------*/
    /* SRAM Data lines configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                  GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* SRAM Address lines configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 |
                                  GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* NOE and NWE configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //  /* NE3 configuration */
    //  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    //  GPIO_Init(GPIOG, &GPIO_InitStructure);

    /* NBL0, NBL1 configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /*-- FSMC Configuration ------------------------------------------------------*/
    p.FSMC_AddressSetupTime = 0;
    p.FSMC_AddressHoldTime = 0;
    p.FSMC_DataSetupTime = 4;
    p.FSMC_BusTurnAroundDuration = 0;
    p.FSMC_CLKDivision = 0;
    p.FSMC_DataLatency = 4;
    p.FSMC_AccessMode = FSMC_AccessMode_A;

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    /* Enable FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);

}
void EnterO0O()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);

    NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
    m_Mode = 0;

}
void PostMsg(struct _SPIPacket sp)
{
    memcpy(&SPIData[CurDataPoint], &sp, sizeof(sp));			//put the message into the team
    CurDataPoint++;
    if (CurDataPoint >= TEAMLENGTH) CurDataPoint = 0;
    TeamLeft++;
}
#ifdef DEBUGJJ
void DebugMsg(char *s)					 //not used
{
    while(1)
    {
        if ((*s == 0) || (*s) == '\0')
        {
            // USART_SendData(USART2, '\n');

            //		 while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
            {
            }
            return ;
        }
        USART_SendData(USART2, *s++);

        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
    }

}
#endif

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
