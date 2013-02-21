#define NODEBUGJJ					  //�Ƿ����õ���ģʽ

#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404

#define UARTSIZE 2 //2�ֽ� ÿ��ͬ��������2�ֽ�
#define BAUDRATE  115200				//ÿ��2.7k��ͬ�� ÿ��ͬ��2�ֽ�  ��������Ҫ 43200bps

#define TEAMLENGTH 100
#define SENDSIZE 1500   //180*64/8=1440  reserve 60�ֽ�	 ��1500	 2��һ����������


#define TESTLED1ON GPIOB->ODR |=1<<12
#define TESTLED1OFF GPIOB->ODR &=~(1<<12)
#define TESTLED2ON GPIOB->ODR |= 1<<13
#define TESTLED2OFF GPIOB->ODR &= ~(1<<13)
#define TESTLED3ON GPIOB->ODR |=1<<14
#define TESTLED3OFF GPIOB->ODR &=~(1<<14)

#define DATAHANG  8
#define DATAANGLE 180

#define RECSIZE DATAHANG*DATAANGLE*2 //8���ֽ�1�� ��180��	 ˫����

#define CR1_CEN_Set                 ((uint16_t)0x0001)		  //��ʱ2
#define CR1_CEN_Reset               ((uint16_t)0x03FE)

#define  Bit8To16(x,y)  ((u16)x<<8 | y)

#define LEDLINE1DATA(x)  	GPIOF->ODR = (GPIOF->ODR & 0x00ff) | ((u16)x<<8)		  //PF8-PF15
#define LEDLINE2DATA(x)		GPIOF->ODR = (GPIOF->ODR & 0xff00) | x		 			  //PF0-PF7
#define LEDLINE3DATA(x)		GPIOG->ODR = (GPIOG->ODR & 0x00ff) | ((u16)x<<8)		  //PG8-PG15
#define LEDLINE4DATA(x)		GPIOG->ODR = (GPIOG->ODR & 0xff00) | x		 			  //PG0-PG7
#define LEDLINE5DATA(x)		GPIOB->ODR = (GPIOB->ODR & 0x00ff) | ((u16)x<<8)		  //PB8-PB15
#define LEDLINE6DATA(x)		GPIOB->ODR = (GPIOB->ODR & 0xff00) | x		  			  //PB0-PB7
#define LEDLINE7DATA(x)		GPIOD->ODR = (GPIOD->ODR & 0x00ff) | ((u16)x<<8)		  //PD8-PD15
#define LEDLINE8DATA(x)		GPIOD->ODR = (GPIOD->ODR & 0xff00) | x		  			  //PD0-PD7


 
#define THEADER 0x55
#define TTAIL 0Xaa

#define CMD_LEDON  		0x0001
#define CMD_LEDOFF 		0X0002
#define CMD_TESTMODE	0X0003
#define CMD_IAPMID    	0X0004  //��д�м��оƬ����
#define CMD_IAPSHOW		0X0005  //��д��ʾ��оƬ����
   //��доƬ�����еĿ���������CMD2����
#define CMD_FILLDATA    0x0006	//������  ,	����������������
#define CMD_RESET		0x0007
#define CMD_SYNANG		0X0008  //�Ƕ�ͬ��
#define CMD_DATAANG		0x0009  //��ͬ���Ƕȵ�ͬʱ�������� �Ƕ���ϢӦ��CMD2��
#define CMD_SELECTANG	0x000a  //ѡ��Ƕȣ�����ѡ��Ƕ�����
#define CMD_FILLDATAANG	0x000b  //����ָ���Ƕ����ݣ��Ƕ���Ϣ��CMD2��
#define CMD_WAIT		0X000C  //�ȴ�����
#define CMD_MODEMONO    0X000D  //��ɫģʽ
#define CMD_MODEGRAY	0X000E   //��ɫģʽ


/*
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
*/
#define DELAYTIME  10  //�ߵ͵�ƽά��ʱ��
#define CS0ON()   GPIOG->ODR |=(1<<7)
#define CS0OFF()  GPIOG->ODR &= ~(1<<7)
#define CS1ON()   GPIOG->ODR |=(1<<6)
#define CS1OFF()  GPIOG->ODR &= ~(1<<6)

#define CSALLON() GPIOA->ODR |=((1<<1) | (1<<0))
#define CSALLOFF() GPIOA->ODR &=~((1<<1) | (1<<0))

/*
struct SENDTYPE
{
 int HEADER; //4 BYTES
 int ADR;  //4BYTES
 int CMD1;  //4BYTES
 int CMD2; //4BYTES
 u8  DATA[1440]; //1440 BYTES
 int TAIL;  //4BYTES
 void (*LEDON)(void);
 void (*LEDOFF)(void);
};
*/
#define CMD_DOWNLOADMODE	2  //IAPģʽ
#define CMD_RUNMODE  		3  //��������ģʽ
#define CMD_DISTRIBUTEADR	19  //��ʼ�����ַ
#define CMD_DATAPROCESSON	50  //��ʼ�м���ṩ����
#define CMD_DATAPROCESSOFF  60  //ֹͣ�м���ṩ����
#define CMD_ONLED  			7  //LED����
#define CMD_OFFLED			8  //LED����
#define CMD_LEDSTILL        9  //ά�ֵ�ǰ����
#define CMD_SYN				10 //ͬ������
#define CMD_DATACIRCULAR	11 //�����������ģʽ
#define CMD_SYN2			12 //ͬ������(���������Ѱַ�Ƕ���Ϣ)
#define CMD_LEDMODEMONO		13 //��ɫģʽ(��ģʽ��Ĭ������)
#define CMD_LEDMODEGRAY		14 //�Ҷ�ģʽ(��ģʽ��Ҫ����֧��)(ADR��=0������ȫ�Ҷ����� >0 ����ɫ��������)
#define CMD_TEST			15 //����ģʽ
#define CMD_TESTCOM			160 //����ģʽ (������������,��˸��)
#define CMD_SDDATASTART		4  //��ʼ����SD������
#define CMD_SDDATAING       5 //���ڴ���SD������
#define CMD_SDDATAEND		6 //SD�����ݴ������
#define CMD_RAMMODE			20 //�ڴ�ģʽ
#define CMD_RAMSTART		21 //�ڴ�ģʽ����ʼ
#define CMD_RAMING			22 //�ڴ�ģʽ��������
#define CMD_RAMEND			23 //�ڴ�ģʽ������
#define CMD_RAMMODEEXTEND	24 //�ڴ�ģʽ��չ����ʱ��ͬ������
#define CMD_RAMEXTENDSTART	25 //��ʼ��ͬ���ͷ�485
#define CMD_SETLOOPTIMEL	26 //�����ظ�ʱ�� ��λ
#define CMD_SETLOOPTIMEH	27 //�����ظ�ʱ�䡡��λ
#define CMD_ENTERSYN        16 //ȡ��SPI DMAģʽ����Ϊ�����ж�
#define CMD_OPENFILE        17 //���ļ�
#define CMD_SPI_SWITCH      18 //���ļ�
#define CMD_SPI_ANG0        1  //0�Ƕ�ͬ��




#define TODISPLAY 1
#define TOMIDLAYER 2
#define TOALL 0
#define TOADR 3
struct SENDTYPE
{
 u16   HOST :2 ;  //2BIT  		   //0x1 �����͸���ʾ�� 0x02�����͸��м�� 0x00 ������Ҫ���� 0x03������ַ����
 u16  ADR  :9;  //9BIT		      //����176��ʾ��+88�м�� ��264 	�����Ҫ���Ե���Ѱַ  0����㲥
 u16   CMD   :5;  //5BIT		      //����32�����������
};
struct _SPIPacket {
unsigned char Cmd;
unsigned char Adr;
union {
unsigned char pData8[10];
//unsigned short pData16[4];
//unsigned long pData32[2];
}pData;
unsigned char Reserve[4];
};//Packet;
enum C_STATUS{C_RESET,C_CONFIG,C_DOWNLOADING,C_RUNNING,C_IDLE,C_LEDON,C_LEDOFF,C_RAM};  
 __inline void Set485(struct SENDTYPE *p,u8 a,u16 b,u8 c);
void SendDataAll(u16 dd);

