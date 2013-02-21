

#ifndef		RESET_FPGA060
#define		RESET_FPGA060()			GPIOA->ODR &= (~(1 << 8))		
#endif

#ifndef		SET_FPGA060
#define		SET_FPGA060()			GPIOA->ODR |= (1 << 8)		
#endif

extern const unsigned char ucDotData[512];
extern const unsigned char ucDotDataXiao[512];
extern const unsigned char ucDotDataKu[512];
extern const unsigned char ucDotDataNum[][512];

void DisplayNum( unsigned short usTab[], unsigned char ucNum );

void ResetFIFO( unsigned short usTab[] );

void DisplayPic( unsigned short usTab[], unsigned char ucPicData[] );
