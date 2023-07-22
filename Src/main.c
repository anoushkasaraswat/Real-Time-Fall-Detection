
#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include "MY_LIS3DSH.h"
#include "TJ_MPU6050.h"


	char txData1[30]="Possible Fall\r\n";
	char txData2[30]="Fall Confirmed\r\n";
	char txData3[30]="No Fall\r\n";

int data1,data2,kk=0; 

LIS3DSH_DataScaled Datascaled;
LIS3DSH_DataScaled myData;
LIS3DSH_DataScaled read_acc_data (void);

static float lis3dsh_Sensitivity = LIS3DSH_SENSITIVITY_0_12G;

int32_t SER_PutChar (int32_t ch){

	while (!(UART4->SR& 0X0080));

	UART4->SR&= 0XFFBF;
		UART4->DR=(ch &0xFF);

	return(ch);
}

int32_t UART4_possibleFall(void){   
//int32_t UART4func(void){   
  // RX IRQ part 
		kk=0;
	//TX IRQ part
		while(kk<strlen(txData1)){
			data1=SER_PutChar((int32_t)txData1[kk]);
			data2=data1;
			kk++;
		}
		return 0;
}

int32_t UART4_noFall(void){   
//int32_t UART4func(void){   
  // RX IRQ part 
		kk=0;
	//TX IRQ part
		while(kk<strlen(txData3)){
			data1=SER_PutChar((int32_t)txData3[kk]);
			data2=data1;
			kk++;
		}
		return 0;
}

int32_t UART4_confirmedFall(void)     {   
  // RX IRQ part 
		kk=0;
	//TX IRQ part
		while(kk<strlen(txData2)){
			data1=SER_PutChar((int32_t)txData2[kk]);
			data2=data1;
			kk++;
		}
		return 0;
}


void SystemClock_Config(void);
static void I2C1_Init(void);
void SPI_ACCEL_INIT(void);
void write_fifo(void);
void read_calibrate(void);
unsigned char read_status(void);
void write_calibrate(void);
void read_acc_test (void);
void read_acc_test11 (void);
void read_acc_test1 (void);
void read_acc_test2 (void);
void read_acc_test3 (void);
void i2c_read_write(void);
I2C_HandleTypeDef I2C1_InitStruct;


int i=0,j=0, flag=0;
float svm_internal,svm_external;
float inte[10000],exte[10000];
uint32_t startTick,startTick1;
uint32_t msTimeout;

RawData_Def myAccelRaw;
ScaledData_Def myAccelScaled;
float lpfint_a[30],lpfint_b[30],lpfint_c[30],lpfext_a[30],lpfext_b[30],lpfext_c[30];
float LPF_Beta = 0.5; // 0<ß<1
float SMAint=0 ,SMAext=0;
float thresh=80;


bool get_SMA(void){
		j=0;
		startTick = HAL_GetTick();
		msTimeout=1000;
		SMAint=0;
		SMAext=0;
		while((HAL_GetTick() - startTick) < msTimeout)
	{
		  read_acc_test3();
			myData = read_acc_data();
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
				
    		// LPF: Y(n) = (1-ß)*Y(n-1) + (ß*X(n))) = Y(n-1) - (ß*(Y(n-1)-X(n)))
     			  
    lpfint_a[j+1] = lpfint_a[j] - (LPF_Beta * (lpfint_a[j] - myData.x));
   	lpfint_b[j+1] = lpfint_b[j] - (LPF_Beta * (lpfint_b[j] - myData.y));
		lpfint_c[j+1] = lpfint_c[j] - (LPF_Beta * (lpfint_c[j] - myData.z));
		
//Scaled data
		MPU6050_Get_Accel_Scale(&myAccelScaled);
		
		lpfext_a[j+1] = lpfext_a[j] - (LPF_Beta * (lpfext_a[j] - myAccelScaled.x));
   	lpfext_b[j+1] = lpfext_b[j] - (LPF_Beta * (lpfext_b[j] - myAccelScaled.y));
		lpfext_c[j+1] = lpfext_c[j] - (LPF_Beta * (lpfext_c[j] - myAccelScaled.z));
		SMAint= SMAint + (0.04)*(fabs(myData.x - lpfint_a[j+1])+ fabs(myData.y - lpfint_b[j+1])+ fabs(myData.z   - lpfint_c[j+1]));
		SMAext= SMAext + (0.04)*(fabs(myAccelScaled.x - lpfext_a[j+1])+ fabs(myAccelScaled.y - lpfext_b[j+1])+ fabs(myAccelScaled.z   - lpfext_c[j+1]));
		
		j++;
		HAL_Delay(100);
	}

	if(((SMAint + SMAext)/2)>thresh){
		return 1;
	}else{
		return 0;
	}
}
int main(void)

{
  SystemClock_Config();
  SPI_ACCEL_INIT();
  I2C1_Init();
  write_fifo();
  write_calibrate();
	read_calibrate();
	MPU_ConfigTypeDef myMpuConfig;
  HAL_Init(); 

  
	//1. Initialise the MPU6050 module and I2C
	MPU6050_Init(&I2C1_InitStruct);
	//2. Configure Accel and Gyro parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
	MPU6050_Config(&myMpuConfig);
	
//I2C_Write8(PWR_MAGT_1_REG, 0x80);
//HAL_Delay(100);
//I2C_Write8(PWR_MAGT_1_REG, 0x00);
//I2C_Write8(CONFIG_REG, 0x01);
//I2C_Write8(ACCEL_CONFIG_REG, 0x08);
	
  while (1)
  {
    i=0;
		
		startTick = HAL_GetTick();
		msTimeout=1000;
		while((HAL_GetTick() - startTick) < msTimeout)
	{
		  read_acc_test3();
			myData = read_acc_data();
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
				
		svm_internal=sqrt(pow(myData.x,2) + pow(myData.y,2) + pow(myData.z,2));
		inte[i]=svm_internal;
		
//Scaled data
		MPU6050_Get_Accel_Scale(&myAccelScaled);
		svm_external=sqrt(pow(myAccelScaled.x,2) + pow(myAccelScaled.y,2) + pow(myAccelScaled.z,2));
		exte[i]=svm_external;
		i++;
		
		if (svm_internal>1800)
		{
			GPIOD->BSRR=0x8000;
		}
		else
		{
		   GPIOD->BSRR=0x80000000;
		}
		if (svm_external>1800)
		{
			GPIOD->BSRR=0x4000;
		}
		else
		{
		   GPIOD->BSRR=0x40000000;
		}
		if (((svm_internal+svm_external)/2)>1800)
		{
			UART4_possibleFall();
			HAL_Delay(2000);
			startTick1=HAL_GetTick();
			//GPIOD->BSRR=0x2000;
			flag=0;
			while((HAL_GetTick() - startTick1) < 20*msTimeout)
			{
				if(get_SMA()){
					flag=1;
					UART4_noFall();
					break;
				}
								
			}
			if (flag==0)
			   {
				   GPIOD->BSRR=0x2000;
					 UART4_confirmedFall();
					 HAL_Delay(10000);
			   }			

		}
		else
		{
		   GPIOD->BSRR=0x20000000;
		}
		HAL_Delay(100);
  }
  
}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void I2C1_Init(void)
{

  I2C1_InitStruct.Instance = I2C1;
  I2C1_InitStruct.Init.ClockSpeed = 100000;
  I2C1_InitStruct.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2C1_InitStruct.Init.OwnAddress1 = 0;
  I2C1_InitStruct.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C1_InitStruct.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C1_InitStruct.Init.OwnAddress2 = 0;
  I2C1_InitStruct.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C1_InitStruct.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&I2C1_InitStruct) != HAL_OK)
  {
    Error_Handler();
  }
	
//	
//	RCC->APB1ENR |=1<<21;//Enable I2C1
//	I2C1->CR1 |=1<<0;   //peripheral enable
//	I2C1->CR2 |= 0x03<<0;  
//	
//	
//	I2C1->OAR1 |=1<<14;  //should always be set by software
////	//I2C1->OAR2 |=0X0000000;
//     I2C1->CCR |=0X00000010;
//	I2C1->TRISE &=~(0X00000002);
//	I2C1->TRISE |=0X00000004;
//	//I2C1->OAR1 &= 0;
//	//I2C1->OAR1 |=0x68<<1; //slave address
////  I2C1->CR1= 1<<9; //STOP
////	while (I2C1->CR1 & 1<<9); //confirming stop
}


void i2c_read_write(void){
	I2C1->CR1 |= 1<<8; //start bit
	while (I2C1->CR1 & 1<<8); //confirming the start condition
	I2C1->DR = 0x77;  //reading from 3B 
	while ( !(I2C1->SR1 & 1<<7) );   //confirming transmit by making sure trasmission is complete. 
	I2C1->CR1 |= 1<<10; //enable ack
	I2C1->CR1 &= ~(1<<1); // clear addr
	while ( !(I2C1->SR1 & 1<<2)) ; //wait until BTF is set
	I2C1->CR1 &= ~(1<<10); //disable ack
	//I2C1->CR1 |= 1<<8; //start bit 
	//while (I2C1->CR1 & 1<<8); //confirming the start condition)
	while (!(I2C1->SR1 & 1<<6));  
	uint32_t x_h= I2C1->DR;
	while ( !(I2C1->SR1 & 1<<2)) ; //wait until BTF is set 
	while ( !(I2C1->SR1 & 1<<6)); 
	uint32_t x_l= I2C1->DR;
}

static   unsigned char spi_send (unsigned char byte) {
SPI1->DR = byte;
while (!(SPI1->SR & SPI_FLAG_RXNE)); /* Wait for send to finish */
return (SPI1->DR);
}
unsigned char test,test_id, x_MSB,x_LSB,y_MSB,y_LSB,z_MSB,z_LSB,test_2,test_id_2,A1,A2,A3,A4,A5,A6,ctrl_4,ctrl_3,status;
int16_t tempDataRaw_x,tempDataRaw_y,tempDataRaw_z;

//float Datascaled_x,Datascaled_y,Datascaled_z;

LIS3DSH_DataScaled read_acc_data (void) {

	tempDataRaw_x = ((x_MSB << 8) + x_LSB);
	tempDataRaw_y = ((y_MSB << 8) + y_LSB);
	tempDataRaw_z = ((z_MSB << 8) + z_LSB);
	Datascaled.x = tempDataRaw_x * lis3dsh_Sensitivity ;
	Datascaled.y = tempDataRaw_y* lis3dsh_Sensitivity ;
	Datascaled.z = tempDataRaw_z * lis3dsh_Sensitivity ;
	return Datascaled;
} 

unsigned char read_status(void){
GPIOE->ODR &= ~(1<<3);
 test = spi_send(0xA7);
 status = spi_send(0);                  //FIFO CTRL
 GPIOE->ODR |= 1<<3;
	return status;
}
void write_fifo (void){
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0x2E);
 test_id = spi_send(0x40);                  //FIFO CTRL
 GPIOE->ODR |= 1<<3;
}
void read_acc_test (void){
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0x8F);
 test_id = spi_send(0);                       //WHO AM I?
 GPIOE->ODR |= 1<<3;
}
void read_acc_test1 (void){
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0x10);
 test_id = spi_send(0x11);                     //OFFSET X
 GPIOE->ODR |= 1<<3;
 
}

void write_calibrate (void){
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0x20);
 ctrl_4 = spi_send(0x47);                     //calibration   (CTRL_4=0x47)
 GPIOE->ODR |= 1<<3;
 
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0x23);
 ctrl_3 = spi_send(0xC8);                     //calibration   (CTRL_3=0xC8)
 GPIOE->ODR |= 1<<3;
	
GPIOE->ODR &= ~(1<<3);
 test = spi_send(0x24);
 ctrl_3 = spi_send(0xC8);                     //calibration   (CTRL_5=0xC8)
 GPIOE->ODR |= 1<<3;
}

void read_calibrate (void){
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0xA0);
 ctrl_4 = spi_send(0x67);                     //calibration          read ctrl_4
 //GPIOE->ODR |= 1<<3;
 
 //GPIOE->ODR &= ~(1<<3);
 test = spi_send(0xA3);
 ctrl_3 = spi_send(0xC8);                     //calibration          read ctrl_3
 GPIOE->ODR |= 1<<3;
	
}

void read_acc_test3 (void)
{
 GPIOE->ODR &= ~(1<<3);
 test = spi_send(0xA8);
 x_LSB = spi_send(0);
 x_MSB = spi_send(0);
 y_LSB = spi_send(0);                       
 y_MSB = spi_send(0);                    //read data from acc
 z_LSB = spi_send(0);
 z_MSB = spi_send(0);
 GPIOE->ODR |= 1<<3;
}


void SPI_ACCEL_INIT (void){
RCC->AHB1ENR |= 1 << 4;	//enable clock to GPIOE
RCC->AHB1ENR |= 1 << 0;	//enable clock to GPIOA
RCC->AHB1ENR |= 1 << 3;	//enable clock to GPIOD
RCC->AHB1ENR |=(1UL<<2);//Enable GPIOC clock
RCC->APB1ENR |=(1UL<<19);//Enable USART 4 clock
RCC->APB2ENR |= 1 << 12;	//clock to SPI1
GPIOE->MODER |= 1 << 6; 	//MODER3[1:0] = 01 bin

// Debugging
GPIOD->MODER &= ~(3 << 24);	
GPIOD->MODER |= 01 << 24; 	
GPIOD->MODER &= ~(3 << 26);	
GPIOD->MODER |= 01 << 26; 	
GPIOD->MODER &= ~(3 << 28);	
GPIOD->MODER |= 01 << 28; 	
GPIOD->MODER &= ~(3 << 30);	
GPIOD->MODER |= 01 << 30;
GPIOD->PUPDR &= ~(0xFF << 24);
GPIOD->OTYPER &= ~(0xF << 12);
	
	//RCC->APB1ENR|=(1UL<<19);//Enable USART 4 clock
	//RCC->AHB1ENR|=(1UL<<2);//Enable GPIOC clock
	GPIOC->MODER &=0XFF0FFFFF;
	GPIOC->MODER |=0X00A00000;
	GPIOC->AFR[1]|=0X00008800;//PC10 UART4_Tx, PC11 UART4_Rx (AF8)
	UART4->BRR=0x0145;
	UART4->CR1=0X200C;
	UART4->CR1 |= 0x00C0; // Enable TX interrupt
	//NVIC_EnableIRQ(UART4_IRQn); // Enable IRQ for UART4 in NVIC
	
GPIOA->MODER &= ~(3 << 10);	//clear bits 10 & 11
GPIOA->MODER |= 2 << 10; 	//MODER5[1:0] = 10 bin
GPIOA->MODER &= ~(3 << 12);	//clear bits 12 & 13
GPIOA->MODER |= 2 << 12; 	//MODER6[1:0] = 10 bin
GPIOA->MODER &= ~(3 << 14);	//clear bits 14 & 15
GPIOA->MODER |= 2 << 14; 	//MODER7[1:0] = 10 bin
GPIOA->AFR[0] |= (5<< 20); 	//enable SPI CLK to PA5
GPIOA->AFR[0] |= (5<< 24); 	//enable MISO to PA6
GPIOA->AFR[0] |= (5<< 28);	//enable MOSI to PA7
	
SPI1->CR1	= 0x0003;	//CPOL=1, CPHA=1
SPI1->CR1	|= 1 << 2;	//Master Mode
SPI1->CR1	&= ~(7<<3);	//Use maximum frequency
SPI1->CR1	|= 3<<8;	//Soltware disables slave function
SPI1->CR1	|= 1<<6;	//SPI enabled
GPIOE->ODR |= 1<<3;	
SPI1->CR2 = 0x0000;	//Motorola Format

}




void Error_Handler(void)
{
 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/