/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}
///////////////////////////// MIDPROJ CODE
void PORTH_Init(void){
    //Use PORTH pins for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                // activate clock for Port H
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};    // allow time for clock to stabilize
    GPIO_PORTH_DIR_R |= 0xFF;                                        // make PN0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;                                     // disable alt funct on PN0
  GPIO_PORTH_DEN_R |= 0xFF;                                        // enable digital I/O on PN0
                                                                                                    // configure P1 as GPIO
  //GPIO_PORTH_PCTL_R = (GPIO_PORTH_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0xFF;                                     // disable analog functionality on PN0        
    return;
}
void PORTN_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};//allow time for clock to stabilize
    GPIO_PORTN_DIR_R=0b00000010; //Make PN0 outputs, to turn on LED
    GPIO_PORTN_DEN_R=0b00000010;
    return;
}
void PORTM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};
    GPIO_PORTM_DIR_R=0b00000000; 
    GPIO_PORTM_DEN_R=0b00000001;
    return;
}
/////////////////////////////


//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){ //FOR STOP BUTTON
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){ //SHUTDOWN FUNCTION
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

int status=0;
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
volatile uint8_t stop = 1;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void PortM_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0xFF;        								// make PM out
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PM0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PM0
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PM0		
	return;
}


volatile long stepPosition = 0;
void spin_steps(int16_t steps){
	static uint16_t delay = 350;
	if (steps >0){
		for(int i=0; i<steps; i++){
			GPIO_PORTM_DATA_R = 0b00001001;
			SysTick_Wait10us(delay);
			GPIO_PORTM_DATA_R = 0b00000011;
			SysTick_Wait10us(delay);
			GPIO_PORTM_DATA_R = 0b00000110;
			SysTick_Wait10us(delay);
			GPIO_PORTM_DATA_R = 0b00001100;
			SysTick_Wait10us(delay);
			stepPosition += 1;
		}
	}
}

void spin_degrees(double degrees){
	double steps = degrees * (512.00/360.00);
	spin_steps((int16_t)steps);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////


void scan(uint16_t angle){
	
	VL53L1X_StartRanging(dev);
	
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;
	//spin_degrees(10);
	
	//wait until the ToF sensor's data is ready
	while (dataReady == 0){
		status = VL53L1X_CheckForDataReady(dev, &dataReady);
				FlashLED3(1);
				VL53L1_WaitMs(dev, 5);
	}
	dataReady = 0;
	
	//read the data values from ToF sensor
	status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
	
	FlashLED3(1);

	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	// print the resulted readings to UART
	if (stop == 1){
			return;
	}
	sprintf(printf_buffer,"%u, %u\n", Distance, angle); //SENDS INFO TO PY
	UART_printf(printf_buffer);
	SysTick_Wait10ms(50);
	
	VL53L1X_StopRanging(dev);
}

void scanArea(void){
	for (int i = 0; i < 360; i += 10){
		if (stop == 1){
			return;
		}
		scan(i);
		spin_degrees(10);
		if (stop == 1){
			return;
		}
	}
	spin_degrees(-360.0);
	UART_printf("END\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////


// give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 input
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}


// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
		PortJ_Init();
	
		GPIO_PORTJ_IS_R = 0x00;    	// (Step 1) PJ1 is edge-sensitive 
		GPIO_PORTJ_IBE_R = 0x00;    	//     			PJ1 is not both edges 
		GPIO_PORTJ_IEV_R = 0x00;    	//     			PJ1 falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;     // 					clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;     	// 					arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;        // (Step 2) enable interrupt 51 in NVIC
	
		NVIC_PRI12_R = 0xA0000000;	// (Step 4) set interrupt priority 5

		//EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}


void PortK_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){};	// allow time for clock to stabilize
  GPIO_PORTK_DIR_R &= ~0x01;    										// make PJ1 input
  GPIO_PORTK_DEN_R |= 0x01;     										// enable digital I/O on PJ1
	
	//GPIO_PORTK_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTK_AMSEL_R &= ~0x01;											//  disable analog functionality on PK1
	//GPIO_PORTK_PUR_R |= 0x01;													//	enable weak pull up resistor
}

void PortK_Interrupt_Init(void){
		PortK_Init();
	
		GPIO_PORTK_IS_R = 0x00;    	// (Step 1) PJ1 is edge-sensitive 
		GPIO_PORTK_IBE_R = 0x00;    	//     			PJ1 is not both edges 
		GPIO_PORTK_IEV_R = 0x00;    	//     			PJ1 falling edge event 
		GPIO_PORTK_ICR_R = 0x01;     // 					clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTK_IM_R = 0x01;     	// 					arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00100000;        // (Step 2) enable interrupt 51 in NVIC
	
		NVIC_PRI12_R = 0x80000000;	// (Step 4) set interrupt priority 4

		//EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

void GPIOJ_IRQHandler(void){
	
	//FlashAllLEDs();

	//UART_printf("Initiate measurement\n");
	stop=0;
	if (stop == 0){
		UART_printf("Scanning ...\n");
		//scan(0);
		scanArea();
	}
	stop=0;
	
	//UART_printf("exiting...\n");
	GPIO_PORTJ_ICR_R = 0x02;     					// acknowledge flag by setting proper bit in ICR register
}

void GPIOK_IRQHandler(void){
	UART_printf("ABORT\n");
	stop=1;
	spin_steps(-1*stepPosition);

	GPIO_PORTK_ICR_R = 0x01;     					// acknowledge flag by setting proper bit in ICR register
}


void initToF(void){
	
	uint8_t sensorState=0;
  uint16_t wordData;
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	PortM_Init();
	
	initToF();
	
	PortJ_Interrupt_Init();
	PortK_Interrupt_Init();
	EnableInt();

  while(1) {
		WaitForInt();
	}

}

