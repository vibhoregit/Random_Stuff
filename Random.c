/************************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates the application of a simple line follower robot. The 
 robot follows a white line over a black backround
 
 Concepts covered:  ADC, LCD interfacing, motion control based on sensor data

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#define mydelay 50

void port_init();
void reverse_path(void);
void timer5_init();
void velocity(unsigned char, unsigned char);
void check_angle(void);
void motors_delay();
void left(int deg);
void right(int deg);
int shortest_path(int source_node, int destination);
void reset_all(void);
void buzzer_pin_config ();
void line_follow();
void buzzer_on();
void buzzer_off();
unsigned int angle = 90;
unsigned int source = 1;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp = 0;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char i = 0;
unsigned char j = 0;
unsigned int temp = 0;
unsigned char path[48];
unsigned char parent[48];
int p = 0;
unsigned char exception_flag = 0;
unsigned char dist_arr[48] =   {255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255,
								255,255,255,255};

unsigned char sorted[48] = {0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0,0,0,
							0,0,0,0,0,0,0,0};
unsigned char sequence[] = {2 , 4 , 3 , 9 , 0};
unsigned int u_set[48][4] = {{205 , 2419, 0, 0},                      //1
							 {118 , 304, 0, 0},						  //2
							 {217, 403, 2800, 2720},				  //3
							 {316, 502, 0, 0},						  //4
							 {415, 601, 0, 0},						  //5
							 {514, 700, 0, 0},						  //6
							 {613, 823,3016, 3120},					  //7
							 {712, 922, 0, 0},						  //8
							 {811, 1021, 0, 0},						  //9
							 {910, 1120, 0, 0},						  //10
							 {1009, 1219, 3312, 3416},				  //11
							 {1108, 1318, 0, 0},				      //12
							 {1207, 1417,0 ,0},						  //13
							 {1306, 1516, 0, 0},					  //14
							 {1405, 1615, 3608, 3712},				  //15
							 {1504, 1714, 0, 0},					  //16
							 {1603, 1813, 0, 0},					  //17
							 {1702, 1912, 0, 0},					  //18
							 {1801, 2011, 3904, 4008},				  //19
							 {1900, 2110, 0,0},					      //20
							 {2023,2209,0,0},
							 {2122,2208,0,0},
							 {2221,2407,0,0},
							 {2320,100,0,0},
							 {2316,2604,0,0},						  //25
							 {4300,2708,4120,2516},
							 {308,2620,2804,0},
							 {312,2900,2716,0},
							 {2812,3004,3200,4420},
							 {2916,704,0,0},						 //30
							 {708,3220,0,0},						 
							 {3300,3108,2912,4516},
							 {1100,3212,3420,0},
							 {1104,3516,3308,0},
							 {4612,3404,3620,3816},					//35
							 {3508,1520,0,0},						  
							 {1500,3812,0,0},
							 {3700,4708,3504,3916},
							 {3804,1916,4012,0},
							 {3900,1920,4108,0},					 //40
							 {4020,4804,4212,2608},
							 {2312,4100,0,0},
							 {2612,4404,0,0},
							 {2908,4316,0,0},
							 {3204,4620,0,0},						 //45
							 {3500,4508,0,0},
							 {3820,4812,0,0},
							 {4116,4700,0,0}};
unsigned int R_set[48][2] = {{1,25},				//1
							 {2,0},
							 {3,0},
							 {4,0},
							 {5,27},				//5
							 {6,0},
							 {7,0},
							 {8,0},
							 {9,28},
							 {10,0},				//10
							 {11,0},
							 {12,0},
							 {13,30},
							 {14,0},
							 {15,0},				//15
							 {16,0},
							 {17,31},
							 {18,0},
							 {19,0},
							 {20,0},				//20
							 {21,33},
							 {22,0},
							 {23,0},
							 {24,0},
							 {25,0},				//25
							 {25,26},
							 {25,26},
							 {26,27},
							 {26,27},
							 {27,0},				//30
							 {28,0},
							 {28,29},
							 {28,29},
							 {29,30},
							 {29,30},				//35
							 {30,0},
							 {31,0},
							 {31,32},
							 {31,32},
							 {32,33},				//40
							 {32,33},
							 {33,0},
							 {26,0},
							 {26,0},
							 {29,0},				//45
							 {29,0},
							 {32,0},
							 {32,0} };							 								 								 								 								 								 								 								 								 								 							 							 							 								 								 								 								 								 							 								 								 								 														 								 								 								 								 								 							 						 								 								 								 							 								 								 								 							 								 								 								 																					 							 							 								 																 															 							 								 								 

//Function to configure buzzer
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
	buzzer_pin_config();
	adc_pin_config();
	motion_pin_config();	
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}




//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}
void left (int deg) //Left wheel backward, Right wheel forward
{
  char i = 0;
  motion_set(0x05);
  velocity(250,250);
  while(i++ < deg)
  _delay_ms(mydelay);
  stop();
}

void right (int deg) //Left wheel forward, Right wheel backward
{
  int i = 0;
  motion_set(0x0A);
  velocity(250,250);
  while(i++ < deg)
  _delay_ms(mydelay);
  stop();
}
void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	sei();   //Enables the global interrupts
}
void line_follow(void)
{
	unsigned char motion = 1;
	while(motion != 0)
	{
		sharp = ADC_Conversion(11);
		if(sharp < 0xD2)
		{
	    Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		if(((Left_white_line>0x1E) && (Center_white_line>0x1E))||((Right_white_line>0x1E) && (Center_white_line>0x1E)) )
		{
			stop();
			motion = 0;
		}
		else
		{ if(Center_white_line>0x48)
		{
			
			forward();
			velocity(250,250);
		}

		else{
			 if((Left_white_line>0x14) )
			{
			
			forward();
			velocity(60,200);
			}

			if((Right_white_line>0x14) )
			{
			
			forward();
			velocity(200,60);
			}
			}
		}				
		}
		else
		{
		stop();
		exception_flag = 1;
		motion = 0;			
		}			
		}	

}
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void check_angle(void)
{
	if(angle < 0)
	angle += 360;
	if(angle > 360)
	angle = angle-360;
}
void navigate()
{
	int i = 0;
	int j = 0;
	int h_angle = 0;
	unsigned int curr = 0, next = 0, x = 0;
	while(path[i+1]!=0)
	{for(j = 0; j < 4; j++)
		{
			curr = u_set[path[i]-1][j];
		    next = path[i];
			if(curr/100 == path[i+1])
			{
				h_angle = 15*(u_set[path[i]-1][j]%100);
				break;
			}
		}
		if((h_angle - angle) > -16 && (h_angle - angle) < 16)
		{
			line_follow();
			if(exception_flag != 1)
			angle = h_angle;
			
			
		}
		else{
			//printf("h_angle = %d, angle = %d\n", h_angle, angle);
			if(h_angle > angle)
			{
				left(h_angle-angle);//printf("Left turning by %d\n", h_angle-angle);
				line_follow();      //printf("Following Line\n");
				if(exception_flag != 1)
				angle = h_angle;
			}
			else
			{
				right(angle-h_angle);//printf("Right turning by %d\n", angle-h_angle);
				line_follow();//printf("Following Line\n");
				if(exception_flag != 1)
				angle = h_angle;
			}
			
		
		}
		check_angle();
		if(exception_flag == 1)
		{
			left(180);
			line_follow();
			left(180);
			 u_set[path[i]-1][j] = 0;
			 for(x = 0; x< 4; x++)
			 {
				 if(u_set[next-1][j]/100 == curr )
				    u_set[next-1][j] = 0;
			 }
			 p--;
			 exception_flag = 0;
			 
		}
		i++;
	}
	//printf("Beep! Beep!\n");
}
void reset_all()
{
	int i = 0;
	while(i < 48)
	{
		sorted[i] = 0;
		dist_arr[i] = 255;
		path[i] = 0;
		parent[i] = 0;
		i++;
	}
}	

void reverse_path()
{
	int i = 0, j = 0, temp = 0;
		while(path[i]!=0)
			i++;
		for(j = 0; j< i/2 ; j++)
		{
			temp = path[j];
			path[j] = path[i-1-j];
			path[i-1-j] = temp;
		}
}

int shortest_path(int source_node, int destination)
{
	
	reset_all();
	int i = 0, min_dist = 255, temp = 0, j = 0, k = 0, flag = 0;
	unsigned char  temp_parent = 1, min_node = 1;
	parent[source_node-1] = 200;
	sorted[source_node - 1] = 1;
	dist_arr[source_node - 1] = 0;
	while(i < 48)
	{
		min_dist = 255;
		flag = 0;
		for(j = 0; j < 48; j++)
		{
			if(sorted[j] == 1)
			{
				for(k = 0; k < 4; k++)
				{
					if(sorted[(u_set[j][k]/100)-1] == 1)
						continue;
					else{
						temp = dist_arr[j]+1;
						if(temp < min_dist)
						{
							flag = 1;
							temp_parent = j+1;
							min_node = u_set[j][k]/100;
							min_dist = temp;
						}
					}
				}
			}
		}
		if(flag == 1)
		{
			parent[min_node-1] = temp_parent;
			dist_arr[min_node-1] = min_dist;
			sorted[min_node-1] = 1;
			if(R_set[min_node - 1][0] == destination || R_set[min_node - 1][1] == destination)
				break;
		}
		i++;
	}
	
	i = min_node;
	j = 0;
	
	while((parent[i - 1] != 200))
	{
	
	path[j] = i;
	i = parent[i-1];
	j++;
	}
	reverse_path();
	
	return (int)min_node;
}



int main()
{
	init_devices();
	p = 0;
	int j = 47, x = sequence[0];
	while(sequence[p+1] != 0)
	{
		j = 47;
		x = shortest_path(x,sequence[p+1]);
		navigate();
		p++;		
	}
	return 0;
}

//Main Function
