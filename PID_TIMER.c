
unsigned char m,n,p;  			//温度的十位 个位 小数
unsigned char test_temp;		//温度检定标志

unsigned char Change_step=1;	//温度设置步进

int PID_MAX;
unsigned int out,PWMT,counter;
int time;	//可控硅脉冲触发时刻

sbit PWM=P2^2;	//PWM控制脚

int PID(int Set_value,int Real_value) //标准PID温度控制算法
{
	int error;
	float P_term, D_term;
	int pid_out;

	error=Set_value - Real_value;//误差量

	P_term =Kp*error;	//比例量
	
	I_term+=Ki*error;	//积分量
	if(I_term>PID_MAX) I_term=PID_MAX;	//限定积分量上限
	else if(I_term<0) I_term=0;			//限定积分量下限

	D_term =Kd*(error - last_error); 	//微分量

	last_error=error;	//缓存当前误差量

	pid_out=(signed int)(P_term+I_term+D_term);	//PID控制量计算
	
	if(pid_out>PID_MAX) 
		pid_out=PID_MAX;		//控制量上限=PID_MAX
	else if(pid_out<0) 
		pid_out=0;				//控制量下限=0

	return(pid_out);	
}

void Init0(void) interrupt 0
{
	TH0=time>>8;
	TL0=time&0x00FF;
	
	TF0=0;
	ET0=1;

	counter++;
}

void T0_int(void) interrupt 1
{
	TH0=0xFF;
	TL0=0x80;

	if(PWM) 
	{	PWM=0;
		TH0=0xDA;
		TL0=0x00;		
	}
	else	PWM=1;
}
void main()
{	
	PWMT=128;			//128级步进PWM控制		
	PID_MAX=PWMT;
	counter=0;
	out=0;
	PWM=0;

	I_term=0;
	last_error=0;

	Set_temp=41;		//初始设定温度为41度
	Set_temp<<=4;
	Real_temp=Set_temp;

  
	while(1)
	{	if(counter>=5) 
		{
			test_temp=1;	//5*20ms=100ms 进行一次温度检定
			counter=0;
		}
		if(test_temp) 	//温度检定标志置位，进入温度PID调节
		{
			Real_temp=ReadTemperature(); //采集当前实际温度
			out=PID(Set_temp,Real_temp); //PID程序
			
		time =~((9500 / PID_MAX) * (PID_MAX - out) + 250); //可控硅触发时刻计数
			
			test_temp=0;				 //检定完成，清温度检定标志
		}

//	 	Real_temp=ReadTemperature();	//读取当前温度	
//		out=PID(Set_temp,Real_temp); 	//PID程序
		
		//显示温度(设定状态显示设定温度，非设定状态显示实际温度)		
		if(!key_set_flag) Disp_temp=Real_temp;
		display(Disp_temp);	

	}
}
