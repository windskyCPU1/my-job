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
#define pi 3.14			//Բ����
#define r  1.5			//�������������ӵİ뾶
#define n  11
#define i  50
#define rr  1.5
#define t  100			//�ж�һ��ʱ��

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
float  act_m_distance2;			//�ײ�����
float  act_m_speed3;
float  act_m_distance3;			//������


int myabs(int a)	//����ֵ����
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
//pwm�������
//�������ڼ������ӣ��˶�ģʽ��ռ�ձȣ�
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
	if(who==2)							//���1
	{
		if(mode==1)						//��ת
		{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_SET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,motor_pwm);
		}
		else if(mode==2)				//��ת
		{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_SET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,myabs(motor_pwm));
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
		}
		else if(mode==3)				//pid����ģʽ
		{		
			if(motor_pwm>0)				//��
			{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14,GPIO_PIN_SET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,motor_pwm);
			}
			else if(motor_pwm<0)		//��
			{
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_SET );  
			HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,myabs(motor_pwm));
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
			}
			else 						//ͣ
			{
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
			}
		}
		else 							//ͣ
		{
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_13,GPIO_PIN_RESET );  
				HAL_GPIO_WritePin (GPIOE,GPIO_PIN_14 ,GPIO_PIN_RESET );
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,0);
				__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,0);
		}
	}
}

//����pid�ṹ����pid����
typedef struct
{
   	float kp,ki,kd;
    float error,err_next,err_last;//���ϴ������ϴ����
    float integral,maxIntegral;//���֣������޷�
    float output,maxOutput;//���������޷�
}pid;

pid add2;//�ڻ�
pid pos2;//�⻷
pid add3;//�ڻ�
pid pos3;//�⻷

//��ʼ��λ�û�pid����
void PID_Init_pos(pid *ppid,float p,float ii,float d,float maxI,float maxOut)
{
    ppid->kp=p;
    ppid->ki=ii;
    ppid->kd=d;
    ppid->maxIntegral=maxI;
    ppid->maxOutput=maxOut;
}
//��ʼ���ٶȻ�pid����
void PID_Init_add(pid *ppid,float p,float ii,float d)
{
    ppid->kp=p;
    ppid->ki=ii;
    ppid->kd=d;
}


//λ�û�
//ִ��һ��pid����
//����Ϊ��pid�ṹ�壬Ŀ��ֵ������ֵ�������������output��Ա��
float PID_Calc_pos(pid *ppid,float target,float back)
{
 	//��������
    ppid->err_next=ppid->error;//??error???
    ppid->error=target-back;//???error
    //����΢��
    float dout=(ppid->error-ppid->err_next)*ppid->kd;
    //�������
    float pout=ppid->error*ppid->kp;
    //�������
    ppid->integral+=ppid->error*ppid->ki;
    //�����޷�
    if(ppid->integral > ppid->maxIntegral) ppid->integral=ppid->maxIntegral;
    else if(ppid->integral < -ppid->maxIntegral) ppid->integral=-ppid->maxIntegral;
    //�������
    ppid->output=pout+dout+ppid->integral;
    //����޷�
    if(ppid->output > ppid->maxOutput) ppid->output=ppid->maxOutput;
    else if(ppid->output < -ppid->maxOutput) ppid->output=-ppid->maxOutput;
    return ppid->output;
}
//�ٶȻ�
//ִ��һ��pid����
//����Ϊ��pid�ṹ�壬Ŀ��ֵ������ֵ�������������output��Ա��
float PID_Calc_add(pid *ppid,float target,float back)
{
 	//��������
    ppid->error=target-back;//???error
    //����΢��
    float dout=(ppid->error-2*ppid->err_next+ppid->err_last)*ppid->kd;        //pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    //�������
    float pout=(ppid->error-ppid->err_next)*ppid->kp;                           //pid.Kp*(pid.err-pid.err_next)
    //�������
    ppid->integral=ppid->error*ppid->ki;                                        //pid.Ki*pid.err
    //�������
	float vas=pout+dout+ppid->integral;
    ppid->output+=vas ;
 	ppid->err_last=ppid->err_next;
    ppid->err_next=ppid->error;//??error???
    return ppid->output;
}

// ��ȡadd/pos��ǰ���ֵ
float get_error(pid *ppid) 
{
  return ppid->error;   
}
// ��ȡadd/pos�ϴ����ֵ
float get_last_error(pid *ppid) 
{
  return ppid->err_next ;   
}

//����pid����
//�������⻷Ŀ��ֵ���⻷����ֵ���ڻ�����ֵ��
//�⻷pid���Ŀ���ٶȣ���Ϊ�ڻ�����Ŀ��ֵ�������⻷�����Ҳ�������Ƴ�Ŀ���ٶ����ֵ
float PID_CascadeCalc_2(float pos_targ,float pos_back,float add_back)
{
    float a=PID_Calc_pos(&pos2,pos_targ,pos_back);//�����⻷   
    float end_value=PID_Calc_add(&add2,a,add_back);//�����ڻ�
    return end_value;
}
float PID_CascadeCalc_3(float pos_targ,float pos_back,float add_back)
{
    float a=PID_Calc_pos(&pos3,pos_targ,pos_back);//�����⻷   
    float end_value=PID_Calc_add(&add3,a,add_back);//�����ڻ�
    return end_value;
}

//����������	motor_2
int16_t Encoder_Get_2(void)			
{
	int16_t Temp;
	Temp =  __HAL_TIM_GetCounter(&htim3);
	TIM3->CNT =0;
	return Temp;
}
//����������	motor_3
int16_t Encoder_Get_3(void)			
{
	int16_t Temp;
	Temp =  __HAL_TIM_GetCounter(&htim4);
	TIM4 ->CNT =0;
	return Temp;
}

//�õ�ÿ�ε�λʱ�������ߵľ��루����2��   cm
float tool_distance_2(void)		   
{
	int16_t b;
	float s;
    b=Encoder_Get_2();
	s=(b*2*pi*rr)/(4*n*i);
	return s;
}
//�õ�ÿ�ε�λʱ�������ߵľ��루����3��   cm
float tool_distance_3(void)		   
{
	int16_t b;
	float s;
    b=Encoder_Get_3();
	s=(b*2*pi*rr)/(4*n*i);
	return s;
}

float target_dis_2(int16_t dis)			//������2Ŀ��λ��
{
	return dis;
}
float target_dis_3(int16_t dis)			//������3Ŀ��λ��
{
	return dis;
}

typedef struct 
{
	float sx,s2,s3;   //��������ҪĿ��λ�Ʊ���
}dis;
dis dd;
void liner_dis(dis *ddd,float v22,float v33)		//���̷ֽ⹫ʽ���ó���������ҪĿ��λ��
{
	float v2,v3;
	v2=v22;
	v3=v33;
	ddd->s2 =target_dis_2(v2);
	ddd->s3 =target_dis_3(v3);
}

//��������2
float  distance_2;
float  speed_pos_2;
void caculate_2()
{
//	act_m_distance2+=tool_distance_2();   //actual  �����о�����cm  
//    distance_2=PID_Calc_pos(&pos2,3,act_m_distance2);

    act_m_distance2+=tool_distance_2();   //actual  �����о�����cm 
	act_m_speed2=tool_distance_2()*10;		//actual  ����ת��cm/s 
	speed_pos_2=PID_CascadeCalc_2(dd.s2 ,act_m_distance2,act_m_speed2);
	Set_Pwm(1,3,speed_pos_2);
//	Set_Pwm (1,3,distance_2);

}
//��������3
float  distance_3;
float  speed_pos_3;
void caculate_3()
{
//	act_m_distance3+=tool_distance_3();   //actual  �����о�����cm  
//   distance_3=PID_Calc_pos(&pos3,10,act_m_distance3);
    act_m_distance3+=tool_distance_3();   //actual  �����о�����cm 
	act_m_speed3=tool_distance_3()*10;		//actual  ����ת��cm/s 
	speed_pos_3=PID_CascadeCalc_3(dd.s3 ,act_m_distance3,act_m_speed3);
	Set_Pwm(2,3,speed_pos_3);
//	Set_Pwm (3,3,distance_3);

}

void angle_take_on(float angle)	//צ���ſ�  100
{
	__HAL_TIM_SET_COMPARE (&htim2 ,TIM_CHANNEL_2 ,angle/180*2000+500);	
}
void angle_take_off(float angle)	//צ�ӱպ�  0
{
	__HAL_TIM_SET_COMPARE (&htim2 ,TIM_CHANNEL_2 ,angle/180*2000+500);	
}

//�жϻص�����  
uint16_t a=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim ->Instance ==TIM6 )
  {
	  a++;			//�����жϺ���������
	  caculate_2();
	  caculate_3();
  }
}

//������lcd��ʾ���ڽ�������
char arr_send[20];			//���巢������				
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
void uart_data_rec()		//���պ���
{
	if(rec_flag)
	{
		if(TIM7->CNT > 15)  //���ݽ������
		{
			//��������
			if(rec_buff[0] == 'A') //צ���ſ�
			{
		        angle_take_on(20);
			}
			else if(rec_buff[0] == 'B' ) //צ�ӳ�ʼ���պ� 
			{			
				angle_take_off(100);
			}
			else if(rec_buff[0] == 'C')  //����
			{
		        HAL_GPIO_WritePin (GPIOB,GPIO_PIN_8,GPIO_PIN_SET );
			}
			else if(rec_buff[0] == 'D' ) //צ��ץȡ��պ�
			{
				angle_take_off(100);
			}	
			else if(rec_buff[0] == 'E' ) //�ײ�������ת��Ŀ��
			{
				liner_dis (&dd,6,0);
			}	
			else if(rec_buff[0] == 'F' ) //�ײ�������ת����
			{
				liner_dis (&dd,0,1);
			}	
			else if(rec_buff[0] == 'G' ) //����������
			{
				liner_dis (&dd,4,1);
			}	
			else if(rec_buff[0] == 'H' ) //�������½�
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
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);		//�ײ�����

  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1 );
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2 );		//������
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);			//���
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);          //���

  HAL_TIM_Base_Start_IT (&htim6);					//�ж�

  PID_Init_add(&add2,10,0,2);//��ʼ���ڻ�����   �ٶȻ�
  PID_Init_add(&add3,10,0,1);//��ʼ���ڻ�����   �ٶȻ�
  
  PID_Init_pos(&pos2,10,0.1,3,1000,2000);//��ʼ���⻷����    λ�û�
  PID_Init_pos(&pos3,10,0,3,1000,2000);//��ʼ���⻷����    λ�û�

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
