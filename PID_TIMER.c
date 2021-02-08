
unsigned char m,n,p;  			//�¶ȵ�ʮλ ��λ С��
unsigned char test_temp;		//�¶ȼ춨��־

unsigned char Change_step=1;	//�¶����ò���

int PID_MAX;
unsigned int out,PWMT,counter;
int time;	//�ɿع����崥��ʱ��

sbit PWM=P2^2;	//PWM���ƽ�

int PID(int Set_value,int Real_value) //��׼PID�¶ȿ����㷨
{
	int error;
	float P_term, D_term;
	int pid_out;

	error=Set_value - Real_value;//�����

	P_term =Kp*error;	//������
	
	I_term+=Ki*error;	//������
	if(I_term>PID_MAX) I_term=PID_MAX;	//�޶�����������
	else if(I_term<0) I_term=0;			//�޶�����������

	D_term =Kd*(error - last_error); 	//΢����

	last_error=error;	//���浱ǰ�����

	pid_out=(signed int)(P_term+I_term+D_term);	//PID����������
	
	if(pid_out>PID_MAX) 
		pid_out=PID_MAX;		//����������=PID_MAX
	else if(pid_out<0) 
		pid_out=0;				//����������=0

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
	PWMT=128;			//128������PWM����		
	PID_MAX=PWMT;
	counter=0;
	out=0;
	PWM=0;

	I_term=0;
	last_error=0;

	Set_temp=41;		//��ʼ�趨�¶�Ϊ41��
	Set_temp<<=4;
	Real_temp=Set_temp;

  
	while(1)
	{	if(counter>=5) 
		{
			test_temp=1;	//5*20ms=100ms ����һ���¶ȼ춨
			counter=0;
		}
		if(test_temp) 	//�¶ȼ춨��־��λ�������¶�PID����
		{
			Real_temp=ReadTemperature(); //�ɼ���ǰʵ���¶�
			out=PID(Set_temp,Real_temp); //PID����
			
		time =~((9500 / PID_MAX) * (PID_MAX - out) + 250); //�ɿع败��ʱ�̼���
			
			test_temp=0;				 //�춨��ɣ����¶ȼ춨��־
		}

//	 	Real_temp=ReadTemperature();	//��ȡ��ǰ�¶�	
//		out=PID(Set_temp,Real_temp); 	//PID����
		
		//��ʾ�¶�(�趨״̬��ʾ�趨�¶ȣ����趨״̬��ʾʵ���¶�)		
		if(!key_set_flag) Disp_temp=Real_temp;
		display(Disp_temp);	

	}
}
