/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14			//圆周率
#define r  1.5			//编码器所带轮子的半径
#define n  11
#define i  50
#define rr  1.5
#define t  100			//中断一次时间

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float  act_m_speed2;
float  act_m_distance2;			//底部绞盘
float  act_m_speed3;
float  act_m_distance3;			//升降梯


int myabs(int a)	//绝对值函数
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
//pwm驱动电机
//参数（第几个轮子，运动模式，占空比）
void Set_Pwm(uint8_t who,uint8_t mode,int motor_pwm)		
{
    if(who==1)
	{
		if(mode==1)
		{	
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_9 ,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_11 ,GPIO_PIN_SET );
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,0);
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_2 ,motor_pwm);			
		}
		else if(mode==2)
		{				
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_9 ,GPIO_PIN_SET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_11,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_1 ,myabs(motor_pwm));
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_2 ,0);
		}
		else if(mode==3)
		{		
			if(motor_pwm>0)
			{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_9 ,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_11 ,GPIO_PIN_SET );
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,0);
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_2 ,motor_pwm);			
			}
			else if(motor_pwm<0)
			{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_9 ,GPIO_PIN_SET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_11,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,myabs(motor_pwm));
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_2 ,0);
			}
			else 
			{
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_9 ,GPIO_PIN_RESET );  
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_11,GPIO_PIN_RESET );
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_1,0);
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_2,0);
			}
		}
		else 
		{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_9 ,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_11,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,0);
			__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_2 ,0);
		}
	}
	if(who==2)							//电机1
	{
		if(mode==1)						//正转
		{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_SET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,motor_pwm);
		}
		else if(mode==2)				//反转
		{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_SET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,myabs(motor_pwm));
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
		}
		else if(mode==3)				//pid调节模式
		{		
			if(motor_pwm>0)				//正
			{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14,GPIO_PIN_SET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,motor_pwm);
			}
			else if(motor_pwm<0)		//反
			{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_SET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,myabs(motor_pwm));
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
			}
			else 						//停
			{
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
			}
		}
		else 							//停
		{
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
		}
	}
}

//定义pid结构体存放pid数据
typedef struct
{
   	float kp,ki,kd;
    float error,err_next,err_last;//误差，上次误差，上上次误差
    float integral,maxIntegral;//积分，积分限幅
    float output,maxOutput;//输出，输出限幅
}pid;

pid add2;//内环
pid pos2;//外环
pid add3;//内环
pid pos3;//外环

//初始化位置环pid参数
void PID_Init_pos(pid *ppid,float p,float ii,float d,float maxI,float maxOut)
{
    ppid->kp=p;
    ppid->ki=ii;
    ppid->kd=d;
    ppid->maxIntegral=maxI;
    ppid->maxOutput=maxOut;
}
//初始化速度环pid参数
void PID_Init_add(pid *ppid,float p,float ii,float d)
{
    ppid->kp=p;
    ppid->ki=ii;
    ppid->kd=d;
}


//位置环
//执行一次pid计算
//参数为（pid结构体，目标值，反馈值），计算结果存放output成员中
float PID_Calc_pos(pid *ppid,float target,float back)
{
 	//更新数据
    ppid->err_next=ppid->error;//??error???
    ppid->error=target-back;//???error
    //计算微分
    float dout=(ppid->error-ppid->err_next)*ppid->kd;
    //计算比例
    float pout=ppid->error*ppid->kp;
    //计算积分
    ppid->integral+=ppid->error*ppid->ki;
    //积分限幅
    if(ppid->integral > ppid->maxIntegral) ppid->integral=ppid->maxIntegral;
    else if(ppid->integral < -ppid->maxIntegral) ppid->integral=-ppid->maxIntegral;
    //计算输出
    ppid->output=pout+dout+ppid->integral;
    //输出限幅
    if(ppid->output > ppid->maxOutput) ppid->output=ppid->maxOutput;
    else if(ppid->output < -ppid->maxOutput) ppid->output=-ppid->maxOutput;
    return ppid->output;
}
//速度环
//执行一次pid计算
//参数为（pid结构体，目标值，反馈值），计算结果存放output成员中
float PID_Calc_add(pid *ppid,float target,float back)
{
 	//更新数据
    ppid->error=target-back;//???error
    //计算微分
    float dout=(ppid->error-2*ppid->err_next+ppid->err_last)*ppid->kd;        //pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    //计算比例
    float pout=(ppid->error-ppid->err_next)*ppid->kp;                           //pid.Kp*(pid.err-pid.err_next)
    //计算积分
    ppid->integral=ppid->error*ppid->ki;                                        //pid.Ki*pid.err
    //计算输出
	float vas=pout+dout+ppid->integral;
    ppid->output+=vas ;
 	ppid->err_last=ppid->err_next;
    ppid->err_next=ppid->error;//??error???
    return ppid->output;
}

// 获取add/pos当前误差值
float get_error(pid *ppid) 
{
  return ppid->error;   
}
// 获取add/pos上次误差值
float get_last_error(pid *ppid) 
{
  return ppid->err_next ;   
}

//串级pid计算
//参数（外环目标值，外环反馈值，内环反溃值）
//外环pid输出目标速度，作为内环输入目标值，限制外环输出，也就是限制车目标速度最大值
float PID_CascadeCalc_2(float pos_targ,float pos_back,float add_back)
{
    float a=PID_Calc_pos(&pos2,pos_targ,pos_back);//计算外环   
    float end_value=PID_Calc_add(&add2,a,add_back);//计算内环
    return end_value;
}
float PID_CascadeCalc_3(float pos_targ,float pos_back,float add_back)
{
    float a=PID_Calc_pos(&pos3,pos_targ,pos_back);//计算外环   
    float end_value=PID_Calc_add(&add3,a,add_back);//计算内环
    return end_value;
}

//编码器计数	motor_2
int16_t Encoder_Get_2(void)			
{
	int16_t Temp;
	Temp =  __HAL_TIM_GetCounter(&htim3);
	TIM3->CNT =0;
	return Temp;
}
//编码器计数	motor_3
int16_t Encoder_Get_3(void)			
{
	int16_t Temp;
	Temp =  __HAL_TIM_GetCounter(&htim4);
	TIM4 ->CNT =0;
	return Temp;
}

//得到每次单位时间内行走的距离（轮子2）   cm
float tool_distance_2(void)		   
{
	int16_t b;
	float s;
    b=Encoder_Get_2();
	s=(b*2*pi*rr)/(4*n*i);
	return s;
}
//得到每次单位时间内行走的距离（轮子3）   cm
float tool_distance_3(void)		   
{
	int16_t b;
	float s;
    b=Encoder_Get_3();
	s=(b*2*pi*rr)/(4*n*i);
	return s;
}

float target_dis_2(int16_t dis)			//设置轮2目标位移
{
	return dis;
}
float target_dis_3(int16_t dis)			//设置轮3目标位移
{
	return dis;
}

typedef struct 
{
	float sx,s2,s3;   //各轮所需要目标位移变量
}dis;
dis dd;
void liner_dis(dis *ddd,float v22,float v33)		//底盘分解公式，得出各轮所需要目标位移
{
	float v2,v3;
	v2=v22;
	v3=v33;
	ddd->s2 =target_dis_2(v2);
	ddd->s3 =target_dis_3(v3);
}

//计算轮子2
float  distance_2;
float  speed_pos_2;
void caculate_2()
{
//	act_m_distance2+=tool_distance_2();   //actual  轮子行径距离cm  
//    distance_2=PID_Calc_pos(&pos2,3,act_m_distance2);

    act_m_distance2+=tool_distance_2();   //actual  轮子行径距离cm 
	act_m_speed2=tool_distance_2()*10;		//actual  轮子转速cm/s 
	speed_pos_2=PID_CascadeCalc_2(dd.s2 ,act_m_distance2,act_m_speed2);
	Set_Pwm(1,3,speed_pos_2);
//	Set_Pwm (1,3,distance_2);

}
//计算轮子3
float  distance_3;
float  speed_pos_3;
void caculate_3()
{
//	act_m_distance3+=tool_distance_3();   //actual  轮子行径距离cm  
//   distance_3=PID_Calc_pos(&pos3,10,act_m_distance3);
    act_m_distance3+=tool_distance_3();   //actual  轮子行径距离cm 
	act_m_speed3=tool_distance_3()*10;		//actual  轮子转速cm/s 
	speed_pos_3=PID_CascadeCalc_3(dd.s3 ,act_m_distance3,act_m_speed3);
	Set_Pwm(2,3,speed_pos_3);
//	Set_Pwm (3,3,distance_3);

}

void angle_take_on(float angle)	//爪子张开  100
{
	__HAL_TIM_SET_COMPARE (&htim2 ,TIM_CHANNEL_2 ,angle/180*2000+500);	
}
void angle_take_off(float angle)	//爪子闭合  0
{
	__HAL_TIM_SET_COMPARE (&htim2 ,TIM_CHANNEL_2 ,angle/180*2000+500);	
}

//中断回调函数  
uint16_t a=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim ->Instance ==TIM6 )
  {
	  a++;			//测试中断函数正常否
	  caculate_2();
	  caculate_3();
  }
}

//类似于lcd显示串口接收数据
char arr_send[20];			//定义发送数组				
void send_dat_1()
{
	sprintf (arr_send,"dis:%.2fcm\r\n",act_m_distance2);
	HAL_UART_Transmit(&huart1 ,(uint8_t  *)arr_send,sizeof (arr_send ),50);
}
void send_dat_2()
{
	sprintf (arr_send,"dis:%dcm\r\n",Encoder_Get_2());
	HAL_UART_Transmit(&huart1 ,(uint8_t  *)arr_send,sizeof (arr_send ),50);
}

char send_buff[20];
uint8_t rec_buff[20];

uint8_t rec_data, count;
uint8_t rec_flag;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		//HAL_UART_Transmit(huart, &rec_data, 1, 50);
		TIM7->CNT = 0;
		rec_flag = 1;
		rec_buff[count] = rec_data;
		count++;
		HAL_UART_Receive_IT(huart, &rec_data, 1);
	}
}

uint8_t turn_on_flag=0;
uint8_t turn_off_flag=0;
uint8_t take_on_flag=0;
uint8_t take_off_flag=0;
void uart_data_rec()		//接收函数
{
	if(rec_flag)
	{
		if(TIM7->CNT > 15)  //数据接收完成
		{
			//处理数据
			if(rec_buff[0] == 'A') //爪子张开
			{
		        angle_take_on(20);
			}
			else if(rec_buff[0] == 'B' ) //爪子初始化闭合 
			{			
				angle_take_off(100);
			}
			else if(rec_buff[0] == 'C')  //测试
			{
		        HAL_GPIO_WritePin (GPIOB,GPIO_PIN_8,GPIO_PIN_SET );
			}
			else if(rec_buff[0] == 'D' ) //爪子抓取后闭合
			{
				angle_take_off(100);
			}	
			else if(rec_buff[0] == 'E' ) //底部绞盘旋转到目标
			{
				liner_dis (&dd,6,0);
			}	
			else if(rec_buff[0] == 'F' ) //底部绞盘旋转回正
			{
				liner_dis (&dd,0,1);
			}	
			else if(rec_buff[0] == 'G' ) //升降梯上升
			{
				liner_dis (&dd,4,1);
			}	
			else if(rec_buff[0] == 'H' ) //升降梯下降
			{
				liner_dis (&dd,0,0);
			}	
			else
			{
				sprintf(send_buff, "error!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t *)send_buff, sizeof(send_buff), 50);			
			}
			
			rec_flag = 0;
			for(int j=0; j<count; j++)
				rec_buff[j] = 0;
			count = 0;
		}
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */ 
  HAL_UART_Receive_IT (&huart1,&rec_data ,1);
  HAL_TIM_Base_Start(&htim7);

  OLED_Init ();
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);		
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);		//底部绞盘

  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1 );
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2 );		//升降梯
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);			//电机
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);          //舵机

  HAL_TIM_Base_Start_IT (&htim6);					//中断

  PID_Init_add(&add2,10,0,2);//初始化内环参数   速度环
  PID_Init_add(&add3,10,0,1);//初始化内环参数   速度环
  
  PID_Init_pos(&pos2,10,0.1,3,1000,2000);//初始化外环参数    位置环
  PID_Init_pos(&pos3,10,0,3,1000,2000);//初始化外环参数    位置环

//  liner_dis (&dd,6,6);

//  angle_take_on(10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
//	send_dat_1();
	uart_data_rec();
	OLED_ShowString(0,29,(uint8_t *)rec_buff,16,1);//14	
	OLED_DisplayTurn(0);
	OLED_Refresh();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
