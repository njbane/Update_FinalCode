#include "main.h"
#include "Pump_Valve.h"
#include "Recipe.h"
#include "I2C_Polling.h"
#include <stdio.h>

/*Initialize the desired register list filled with uint2pin*/
struct pinList REGA1; 
struct pinList REGB1;
struct pinList REGA2;
struct pinList REGB2;

void recipe_cmd(uint8_t ID[], int board, int Serial){
	/*
	Executes the recipe command based on the ID and board
	
	ID[0]: ID of Drink to be made
	ID[1]: Quanity of the Drink to be made
	board: Selects whether there are one ore two boards or not(0 - ONEBoard | 1 - TWOBoard)
	Serial: (1 - Turns Serial Data On)
	*/
	// Initization of Variables
	int quanity = (int) ID[1]; // Quanity of Drink being made - typecasted from uint8_t to int
	int len; // Length of Buffer data to be sent through UART
	int begin_time1; // Initialization of time to be used for pump timing
	uint8_t StopBuffer[2];
	char buffer[30]; // Buffer for UART transmission
	
	/* Flags */
	int stop_flag1 = 0, stop_flag2 = 0, stop_flag3 = 0, stop_flag4 = 0; // Flags to end while looop
	int count1,count2,count3,count4; // Makes sure the valves are only written to once to limit number of I2C transmissions
	
	while(quanity > 0){ // Allows multiple drinks to be made
		
		if(Serial == 1){
			len = sprintf(buffer,"RegA %d, RegB %d, RegC %d, RegD %d", recipe[(int)ID[0]].valv[0], recipe[(int)ID[0]].valv[1], recipe[(int)ID[0]].valv[2], recipe[(int)ID[0]].valv[3]);
			HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
		}
		uint2pin((int)ID[0]); // Converts the uint8_t to pins to be turned on
		for(int i = 0; i <=7; i++){ 
			/** 
			Each register has 8 valves that can be operated - for loop runs through each of
			the 8 valves. Implements turning on or off a valve - if the pin is '1' turns on
			and '0' turns off the valves.	
			**/
			int time1cout, time2cout, time3cout, time4cout;
			int total1time, total2time, total3time, total4time;
			stop_flag1 = 0, stop_flag2 = 0, stop_flag3 = 0, stop_flag4 = 0;
			count1 = 0, count2=0, count3=0, count4=0;
			
			if(board != 1){ // If only one board is in operation the flags are set to one to end the while loop
				stop_flag3 = 1;
				stop_flag4 = 1;
			}
			
			begin_time1 = (int)HAL_GetTick(); // Gets current time in milliseconds for pumps
			if(Serial == 1){
				len = sprintf(buffer, "\nBegin Time: %d ", begin_time1);
				HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
			}
			while((stop_flag1*stop_flag2*stop_flag3*stop_flag4) != 1){ 
				/** 
				The while looop checks the flags generated by pump time to make sure each 
				pump is on for the required period of time. - It allows for unequal timing
				of the pumps
				**/
				if(Serial == 1){
					len = sprintf(buffer, " ForLOOP %d ", i);
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
				}
				
				if(REGA1.pin[i] == 1){ // Turns on valve if REGA1.pin[] equals 1
					if(count1 == 0){ // Turns on the valve once that is connected to REGA1
						if(Serial == 1){ // Turns on or off serial data output
							len = sprintf(buffer, "\nREGA1 %d ", REGA1.pin[i]);
							HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
						}
						PWM_PumpControl(1,0,255); // Turns Pump 1 on at full speed 255
						I2C_MCP23_ValveControl(i,0,0,&regA1,&regB1,0); //Pump_Valve.h 
						count1 = 1; // Sets flag so that it will only execute once
					}
				}
				
				if(REGB1.pin[i] == 1){ // Turns on pin if REGB1.pin[] equals 1
					if(count2 == 0){
						if(Serial == 1){
							len = sprintf(buffer, "\nREGB1 %d ", REGB1.pin[i]);
							HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
						}
						PWM_PumpControl(2,0,255); // Turns Pump 2 on at full speed 255
						I2C_MCP23_ValveControl(i,0,0,&regA1,&regB1,0);
						count2 = 1;
					}
				}
				if(board == 1){ //If commanded valve control board two will execute the command
					if(REGA2.pin[i] == 1){
						if(count3 == 0){
							if(Serial == 1){
								len = sprintf(buffer, "\nREGA2 %d ", REGA2.pin[i]);
								HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
							}
							PWM_PumpControl(3,0,255); // Turns Pump 3 on at full speed 255
							I2C_MCP23_ValveControl(i,0,0,&regA2,&regB2,1);
							count3 = 1;
						}
					}
					if(REGB2.pin[i] == 1){
						if(count4 == 0){
							if(Serial == 1){
								len = sprintf(buffer, "\nREGB2 %d ", REGB2.pin[i]);
								HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
							}
							PWM_PumpControl(4,1,255); // Turns Pump 4 ON Pump_Valve.h
							I2C_MCP23_ValveControl(i,0,0,&regA2,&regB2,1);
							count4 = 1;
						}
					}
				}
//				begin_time1 = HAL_GetTick();
//				while(((int)HAL_GetTick - begin_time1) < (117*((int) recipe[(int) ID[0]].time[i]))){
//					HAL_I2C_Slave_Receive(&hi2c1,StopBuffer,2,HAL_MAX_DELAY);
//					if(StopBuffer[0] == 0x10){
//						break;
//					}
//				}
				if(REGA1.pin[i] == 1){
					time1cout = (int)HAL_GetTick - begin_time1;
					total1time = 117*((int) recipe[(int) ID[0]].time[i]);
					if(Serial == 1){
						len = sprintf(buffer, " TimeA-Cout:%d Total:%d ",time1cout,total1time);
						HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, 1000);
					}
					if(time1cout > total1time){
						/*
						((int)HAL_GetTick - begin_time1): [TIME ELAPSED] Gets current time in microseconds and subtracts beginning time
						
						(117*((int) recipe[(int)ID[0]].time[i]: [PUMP TIME] 0 to 255 represents from 0 to 30 seconds. Each increment
						up by 1 represents 117 ms (256(steps)/30(seconds) = .117) therefore we multiply the number stored
						in the recipe by the multiplier to get total time in milliseconds
						
						if TIME ELAPSED is greater than PUMP TIME statement executes and sets the stop_flagX = 1
						*/
						if(Serial == 1){ // OUTPUTS Serial Data to Monitor
							len = sprintf(buffer, "\nREGA1");
							HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
						}
						PWM_PumpControl(1,0,0); // Turns Pump 1 Off Pump_Valve.h
						I2C_MCP23_ValveControl(i,1,0,&regA1,&regB1,0); // Turns valves off
						stop_flag1 = 1; // Sets the stop_flag1 to 1 to allow for the loop to end
					}
				}
				else{stop_flag1 = 1;} // If REGA1.pin[i] == 0 then stop_flag1 is set to 1 to end the while loop
				if(REGB1.pin[i] == 1){
					time2cout = (int)HAL_GetTick - begin_time1;
					total2time = 117*((int) recipe[(int) ID[0]].time[i+8]);
					
					if(Serial == 1){
						len = sprintf(buffer, " TimeB-Cout:%d Total:%d ",time2cout, total2time);
						HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, 1000);
					}
					if(time2cout > total2time){
						/*
						((int)HAL_GetTick - begin_time1): [TIME ELAPSED] Gets current time in microseconds and subtracts beginning time
						
						(117*((int) recipe[(int)ID[0]].time[i]: [PUMP TIME] 0 to 255 represents from 0 to 30 seconds. Each increment
						up by 1 represents 117 ms (256(steps)/30(seconds) = .117) therefore we multiply the number stored
						in the recipe by the multiplier to get total time in milliseconds
						
						if TIME ELAPSED is greater than PUMP TIME statement executes and sets the stop_flagX = 1
						*/
						if(Serial == 1){ // OUTPUTS Serial Data to Monitor
							len = sprintf(buffer, "\nREGB1");
							HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
						}
						PWM_PumpControl(2,0,0); // Turns Pump 2 OFF Pump_Valve.h
						I2C_MCP23_ValveControl(i,1,0,&regA1,&regB1,0); // Turns valves off
						stop_flag2 = 1; // Sets the stopflag to 1 to end loop
					}
				}
				else{stop_flag2 = 1;} // If REGB1.pin[i] == 0 then stop_flag1 is set to 1 to end the while loop
				
				if(board == 1){
					if(REGA2.pin[i] == 1){
						time3cout = (int)HAL_GetTick - begin_time1;
						total3time = 117*((int) recipe[(int) ID[0]].time[i+16]);
						if(Serial == 1){
							len = sprintf(buffer, " TimeC-Cout:%d Total:%d ",time3cout,total3time);
							HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, 1000);
						}
						if(time3cout > total3time){
							/*
							((int)HAL_GetTick - begin_time1): [TIME ELAPSED] Gets current time in microseconds and subtracts beginning time
							
							(117*((int) recipe[(int)ID[0]].time[i]: [PUMP TIME] 0 to 255 represents from 0 to 30 seconds. Each increment
							up by 1 represents 117 ms (256(steps)/30(seconds) = .117) therefore we multiply the number stored
							in the recipe by the multiplier to get total time in milliseconds
							
							if TIME ELAPSED is greater than PUMP TIME statement executes and sets the stop_flagX = 1
							*/
							if(Serial == 1){
								len = sprintf(buffer, "\nREGA2");
								HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
							}
							PWM_PumpControl(3,0,0); // Turns Pump 3 off
							I2C_MCP23_ValveControl(i,1,0,&regA2,&regB2,1);
							stop_flag3 = 1; // Sets teh stopflag to 1 to end loop
						}
					}
					else{stop_flag3 = 1;}
					if(REGB2.pin[i] == 1){
						time4cout = (int)HAL_GetTick - begin_time1;
						total4time = 117*((int) recipe[(int) ID[0]].time[i+24]);
						if(Serial == 1){
							len = sprintf(buffer, "TimeD-Cout:%d Total:%d ",time4cout,total4time);
							HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, 1000);
						}
						if(time4cout > total4time){
							/*
							((int)HAL_GetTick - begin_time1): [TIME ELAPSED] Gets current time in microseconds and subtracts beginning time
							
							(117*((int) recipe[(int)ID[0]].time[i]: [PUMP TIME] 0 to 255 represents from 0 to 30 seconds. Each increment
							up by 1 represents 117 ms (256(steps)/30(seconds) = .117) therefore we multiply the number stored
							in the recipe by the multiplier to get total time in milliseconds
							
							if TIME ELAPSED is greater than PUMP TIME statement executes and sets the stop_flagX = 1
							*/
							if(Serial == 1){
								len = sprintf(buffer, "\nREGB2");
								HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
							}
							PWM_PumpControl(4,0,0); // Turns Pump 4 off
							I2C_MCP23_ValveControl(i,1,0,&regA2,&regB2,1);
							stop_flag4 = 1;
						}
					}
					else{stop_flag4 = 1;}
				}
				else{stop_flag3 = 1; stop_flag4 = 1;} // If there is only one board then the stopflags are set to 1
				
				// Emergency Stop
				HAL_I2C_Slave_Receive(&hi2c1,StopBuffer,2,HAL_MAX_DELAY);
				if(StopBuffer[0] == 0x10){
					uint8_t gpioA = 0x12, gpioB = 0x13;
					// ENDS WHILE LOOOP
					stop_flag1 = 1;
					stop_flag2 = 1;
					stop_flag3 = 1;
					stop_flag4 = 1;
					
					// Stops Pumping
					PWM_PumpControl(1,0,0);
					PWM_PumpControl(2,0,0);
					if(board == 1){ // If there are two boards it will end the other pumps
						PWM_PumpControl(3,0,0);
						PWM_PumpControl(4,0,0);
					}
					
					// Closes all the Valves
					I2C_MCP23018W(gpioA, 0x00, hi2c2);
					HAL_Delay(10);
					I2C_MCP23018W(gpioB, 0x00, hi2c2);
					if(board == 1){
						HAL_Delay(10);
						I2C_MCP23018W(gpioA, 0x00, hi2c3);
						HAL_Delay(10);
						I2C_MCP23018W(gpioB, 0x00, hi2c3);
					}
					i = 8; // ENDS FOR Loop 
				}
				if(Serial ==1){
					len = sprintf(buffer, "StopFlag 1: %d, 2: %d, 3: %d, 4: %d ", stop_flag1,stop_flag2,stop_flag3,stop_flag4);
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
				}
			}
			// Sets the count variables to zero so that more valves can be turned on or off
			count1 = 0;
			count2 = 0;
			count3 = 0;
			count4 = 0;
		}
		quanity--; // Decrements the quanity variable
	}
}

void uint2pin(int ID){
	/*
	Converts the uint8_t to char which will be used for pin/valve control
	
	ID: Selects which recipe to convert that is stored in the recipe struct array
	
	*/
	int k;
	for(int i = 0; i <= 3; i++){ // Handles the four Register structs for MCP23018
		uint8_t hexConvert = recipe[ID].valv[i]; // transfers data to make handling data easier
		k = 0;
		int conversion;
		for(uint8_t j = 0x80; j != 0; j >>= 1){ // Not sure how this for loop works but has been tested and does convert hex to bin
			conversion = (hexConvert&j)?1:0; // Conversion
			switch (i){ //Chooses which register the data is fed into 
				case 0:{
					REGA1.pin[k] = conversion;
					break;
				}
				case 1:{
					REGB1.pin[k] = conversion;
					break;
				}
				case 2:{
					REGA2.pin[k] = conversion;
					break;
				}
				case 3:{
					REGB2.pin[k] = conversion;
					break;
				}
			}
			k++;
		}
	}
}
