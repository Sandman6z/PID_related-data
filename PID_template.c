#include<reg51.h>
#include<intrins.h>
#include<math.h>
#include<string.h>

struct PID 
{
    unsigned int SetPoint; // 设定目标 Desired Value
    unsigned int Proportion; // 比例常数 Proportional Const
    unsigned int Integral; // 积分常数 Integral Const
    unsigned int Derivative; // 微分常数 Derivative Const
    unsigned int LastError; // Error[-1]
    unsigned int PrevError; // Error[-2]
    unsigned int SumError; // Sums of Errors
};
struct PID spid; // PID Control Structure
unsigned int rout; // PID Response (Output)
unsigned int rin; // PID Feedback (Input)

sbit data1 =P1^0;
sbit clk = P1^1;
sbit plus = P2^0;
sbit subs = P2^1;
sbit stop = P2^2;
sbit output = P3^4;
sbit DQ = P3^3;

unsigned char flag,flag_1=0;
unsigned char high_time,low_time,count=0;//占空比调节参数
unsigned char set_temper=35;
unsigned char temper;
unsigned char i;
unsigned char j=0;
unsigned int s;

/***********************************************************
延时子程序,延时时间以12M晶振为准,延时时间为30us×time
***********************************************************/
void delay(unsigned char time)
{
unsigned char m,n;
for(n=0;n<time;n++)
for(m=0;m<2;m++){}
}

/***********************************************************
获取温度子程序
***********************************************************/
void get_temper()
{
	unsigned char i,j;
	do
	{
		i=reset();  /*复位*/
	}
	while(i!=0); 	/*1为无反馈信号*/
	i=0xcc; 		/*发送设备定位命令*/
	write_byte(i);
	i=0x44; 		/*发送开始转换命令*/
	write_byte(i);
	delay(180); 	/*延时*/
	do
	{
		i=reset();  /*复位*/
	}
	while(i!=0);
	i=0xcc; 		/*设备定位*/
	write_byte(i);
	i=0xbe; 		/*读出缓冲区内容*/
	write_byte(i);
	j=read_byte();
	i=read_byte();
	i=(i<<4)&0x7f;
	s=(unsigned int)(j&0x0f);
	s=(s*100)/16;
	j=j>>4;
	temper=i|j; 	/*获取的温度放在temper中*/
}

/*====================================================================================================
Initialize PID Structure
=====================================================================================================*/
void PIDInit (struct PID *pp)
{
	memset ( pp,0,sizeof(struct PID));
}
/*=====================================================================
PID计算部分
=====================================================================*/
unsigned int PIDCalc( struct PID *pp, unsigned int NextPoint )
{
	unsigned int dError,Error;
    Error = pp->SetPoint - NextPoint; // 偏差
    pp->SumError += Error; // 积分
    dError = pp->LastError - pp->PrevError; // 当前微分
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
    return (pp->Proportion * Error//比例
    + pp->Integral * pp->SumError //积分项
    + pp->Derivative * dError); // 微分项
}

/***********************************************************
温度比较处理子程序
***********************************************************/
compare_temper()
{
	unsigned char i;
	if(set_temper>temper)
	{
		if(set_temper-temper>1)
		{
			high_time=100;
			low_time=0;
		}
		else
		{
			for(i=0;i<10;i++)
			{ 
				get_temper();
				rin = s; // Read Input
				rout = PIDCalc ( &spid,rin ); // Perform PID Interation
			}
		if (high_time<=100)
			high_time=(unsigned char)(rout/800);
		else
			high_time=100;
		low_time= (100-high_time);
		}
	}
	else if(set_temper<=temper)
	{
		if(temper-set_temper>0)
		{
			high_time=0;
			low_time=100;
		}
		else
		{
			for(i=0;i<10;i++)
			{ 
				get_temper();
				rin = s; // Read Input
				rout = PIDCalc ( &spid,rin ); // Perform PID Interation
				if (high_time<100)
					high_time=(unsigned char)(rout/10000);
				else
					high_time=0;
					low_time= (100-high_time);
			}
		}
		// else
		// {}
	}
}

/*****************************************************
T0中断服务子程序，用于控制电平的翻转 ,40us*100=4ms周期
******************************************************/
void serve_T0() interrupt 1 using 1
{
if(++count<=(high_time))
output=1;
else if(count<=100)
{
output=0;
}
else
count=0;
TH0=0x2f;
TL0=0xe0;
}

/****************显示子程序***************
功能：将占空比温度转化为单个字符，显示占空比和测得到的温度
******************************************************/
void display()
{
	unsigned char code number[]={0xfc,0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6};
	unsigned char disp_num[6];
	unsigned int k,k1;
	k=high_time;
	k=k%1000;
	k1=k/100;
	if(k1==0)
		disp_num[0]=0;
	else
		disp_num[0]=0x60;
	k=k%100;
	disp_num[1]=number[k/10];
	disp_num[2]=number[k%10];
	k=temper;
	k=k%100;
	disp_num[3]=number[k/10];
	disp_num[4]=number[k%10]+1;
	disp_num[5]=number[s/10];
	disp_1(disp_num);
}

/*中断初始化*/
void Initialize()
{
	TMOD=0x21;
	TH0=0x2f;
	TL0=0x40;
	SCON=0x50;
	PCON=0x00;
	TH1=0xfd;
	TL1=0xfd;
	PS=1;
	EA=1;
	EX1=0;
	ET0=1;
	ES=1;
	TR0=1;
	TR1=1;
}
/****************************
主程序
****************************/
main()
{
	unsigned char z;
	unsigned char a,b,flag_2=1,count1=0;
	unsigned char phil[]={2,0xce,0x6e,0x60,0x1c,2};

	Initialize();

	high_time=50;
	low_time=50;
	PIDInit ( &spid ); // Initialize Structure
	spid.Proportion = 10; // Set PID Coefficients
	spid.Integral = 8;
	spid.Derivative =6;
	spid.SetPoint = 100; // Set PID Setpoint
	while(1)
	{
		if(plus==0)
		{
			EA=0;
			for(a=0;a<5;a++)
				for(b=0;b<102;b++)
				{}
			if(plus==0)
			{
				set_temper++;
				flag=0;
			}
		}
		else if(subs==0)
		{
			for(a=0;a<5;a++)
				for(b=0;a<102;b++)
				{}
			if(subs==0)
			{
				set_temper--;
				flag=0;
			}
		}
		else if(stop==0)
		{
			for(a=0;a<5;a++)
				for(b=0;b<102;b++)
				{}
			if(stop==0)
			{
				flag=0;
				break;
			}
			EA=1;
		}
		get_temper();
		b=temper;
		if(flag_2==1)
			a=b;
		if((abs(a-b))>5)
			temper=a;
		else
			temper=b;
		a=temper;
		flag_2=0;
		if(++count1>30)
		{
			display();
			count1=0;
		}
		compare_temper();
	}
	TR0=0;
	z=1;
	while(1)
	{
		EA=0;
		if(stop==0)
		{
			for(a=0;a<5;a++)
				for(b=0;b<102;b++)
				{}
			if(stop==0)
				disp_1(phil);
				// break;
		}
		EA=1;
	}
}




//DS18b20 子程序
#include <REG52.H>
sbit DQ=P2^1; //定义端口
typedef unsigned char byte;
typedef unsigned int word;
//延时
void delay(word useconds)
{
	for(;useconds>0;useconds--);
}
//复位
byte ow_reset(void)
{
	byte presence;
	DQ=0; //DQ低电平
	delay(29); //480us
	DQ=1; //DQ高电平
	delay(3); //等待
	presence=DQ; //presence信号
	delay(25);

	return(presence);
} //0允许，1禁止
//从1-wire 总线上读取一个字节
byte read_byte(viod)
{
	byte i;
	byte value=0;
	for (i=8;i>0;i--)
	{
		value>>=1;
		DQ=0;
		DQ=1;
		delay(1);
		if(DQ)value|=0x80;
		delay(6);
	}
	return(value);
}
//向1-wire总线上写一个字节
void write_byte(char val)
{
	byte i;
	for (i=8;i>0;i--) //一次写一个字节
	{
		DQ=0;
		DQ=val&0x01;
		delay(5);
		DQ=1;
		val=val/2;
	}
	delay(5);
}
//读取温度
char Read_Temperature(void)
{
	union
	{
		byte c[2];
		int x;
	}temp;
	ow_reset();
	write_byte(0xcc);
	write_byte(0xBE);
	temp.c[1]=read_byte();
	temp.c[0]=read_byte();
	ow_reset();
	write_byte(0xCC);
	write_byte(0x44);

	return temp.x/2;
}