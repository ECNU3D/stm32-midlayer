#define NODEBUGJJ					  //是否启用调试模式

#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404

#define UARTSIZE 2 //2字节 每次同步数据有2字节
#define BAUDRATE  115200				//每秒2.7k次同步 每次同步2字节  则至少需要 43200bps

#define TEAMLENGTH 100
#define SENDSIZE 1500   //180*64/8=1440  reserve 60字节	 ＝1500	 2度一切面的情况下


#define TESTLED1ON GPIOB->ODR |=1<<12
#define TESTLED1OFF GPIOB->ODR &=~(1<<12)
#define TESTLED2ON GPIOB->ODR |= 1<<13
#define TESTLED2OFF GPIOB->ODR &= ~(1<<13)
#define TESTLED3ON GPIOB->ODR |=1<<14
#define TESTLED3OFF GPIOB->ODR &=~(1<<14)

#define DATAHANG  8
#define DATAANGLE 180

#define RECSIZE DATAHANG*DATAANGLE*2 //8个字节1度 共180度	 双缓冲

#define CR1_CEN_Set                 ((uint16_t)0x0001)		  //定时2
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
#define CMD_IAPMID    	0X0004  //烧写中间层芯片命令
#define CMD_IAPSHOW		0X0005  //烧写显示层芯片命令
   //烧写芯片过程中的控制命令由CMD2决定
#define CMD_FILLDATA    0x0006	//发数据  ,	单独发送数据命令
#define CMD_RESET		0x0007
#define CMD_SYNANG		0X0008  //角度同步
#define CMD_DATAANG		0x0009  //在同步角度的同时发送数据 角度信息应在CMD2中
#define CMD_SELECTANG	0x000a  //选择角度，单独选择角度命令
#define CMD_FILLDATAANG	0x000b  //发送指定角度数据，角度信息在CMD2中
#define CMD_WAIT		0X000C  //等待命令
#define CMD_MODEMONO    0X000D  //单色模式
#define CMD_MODEGRAY	0X000E   //灰色模式


/*
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
*/
#define DELAYTIME  10  //高低电平维持时间
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
#define CMD_DOWNLOADMODE	2  //IAP模式
#define CMD_RUNMODE  		3  //正常工作模式
#define CMD_DISTRIBUTEADR	19  //开始分配地址
#define CMD_DATAPROCESSON	50  //开始中间层提供数据
#define CMD_DATAPROCESSOFF  60  //停止中间层提供数据
#define CMD_ONLED  			7  //LED灯亮
#define CMD_OFFLED			8  //LED灯灭
#define CMD_LEDSTILL        9  //维持当前数据
#define CMD_SYN				10 //同步命令
#define CMD_DATACIRCULAR	11 //进入接收数据模式
#define CMD_SYN2			12 //同步命令(该命令可以寻址角度信息)
#define CMD_LEDMODEMONO		13 //单色模式(该模式是默认设置)
#define CMD_LEDMODEGRAY		14 //灰度模式(该模式需要数据支持)(ADR域=0代表完全灰度数据 >0 代表单色调节亮度)
#define CMD_TEST			15 //测试模式
#define CMD_TESTCOM			160 //测试模式 (仅调试中有用,闪烁灯)
#define CMD_SDDATASTART		4  //开始传输SD卡数据
#define CMD_SDDATAING       5 //正在传输SD卡数据
#define CMD_SDDATAEND		6 //SD卡数据传输结束
#define CMD_RAMMODE			20 //内存模式
#define CMD_RAMSTART		21 //内存模式１开始
#define CMD_RAMING			22 //内存模式１进行中
#define CMD_RAMEND			23 //内存模式１结束
#define CMD_RAMMODEEXTEND	24 //内存模式扩展（定时器同步）　
#define CMD_RAMEXTENDSTART	25 //开始自同步释放485
#define CMD_SETLOOPTIMEL	26 //设置重复时间 低位
#define CMD_SETLOOPTIMEH	27 //设置重复时间　高位
#define CMD_ENTERSYN        16 //取消SPI DMA模式，改为单次中断
#define CMD_OPENFILE        17 //打开文件
#define CMD_SPI_SWITCH      18 //打开文件
#define CMD_SPI_ANG0        1  //0角度同步




#define TODISPLAY 1
#define TOMIDLAYER 2
#define TOALL 0
#define TOADR 3
struct SENDTYPE
{
 u16   HOST :2 ;  //2BIT  		   //0x1 代表送给显示层 0x02代表送给中间层 0x00 代表都需要接收 0x03代表按地址接收
 u16  ADR  :9;  //9BIT		      //共有176显示层+88中间层 ＝264 	如果需要可以单独寻址  0代表广播
 u16   CMD   :5;  //5BIT		      //定义32种命令见上面
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

