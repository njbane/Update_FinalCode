#include "main.h"
#include <stdio.h>
#include "I2C_Polling.h"
#include "Recipe.h"

struct data recipe[24]; // Initalizes the Data Recipe Struct Array with space for 24

int I2C_Slave(int board, int Serial){
	/*
	Polling I2C Mode - Data will come in one byte to select path for data to be received in.
	
	if(0x37) is recieved then it will expect to recieve an additional 9 bytes of data - 0 ID, 1-4 VALVES, 5-8 PumpTimes
	if(0x57) is recieved then it will expect to recieve two bytes of data - 0 ID, 1 Quanity
	
	int board: Selects whether there are one ore two boards or not(0 - ONEBoard | 1 - TWOBoard)
	int Serial: Turns on (1) or off(0) the serial communication for debugging
	*/
	int len = 0,RXBuf_Valv_Pump = 10, error = 0, incre = 0;
	uint8_t aRxBuffer[12];
	char buffer[15];
	HAL_StatusTypeDef error2;
	
	if(Serial == 1){
		len = sprintf(buffer, "Main");
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, HAL_MAX_DELAY);
	}
	error2 = HAL_I2C_Slave_Receive(&hi2c1, aRxBuffer, 11, HAL_MAX_DELAY);
	if( error2 == HAL_OK){
			if(Serial == 1){
					len = sprintf(buffer, "\nSlaveRecieve");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,11,1000);
			}
			if(aRxBuffer[0] == 0x37){
				int i = 0;
				if(Serial == 1){
					len = sprintf(buffer, "\nRecipe 0x37: ");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,11,1000);
				}
				for(int f = 0; f<8; f++){
					recipe[(int)aRxBuffer[1]].time[f] = aRxBuffer[2+f]; //Stores DataBuffer in the ID
				}
				recipe[(int)aRxBuffer[1]].valv[i] = aRxBuffer[10];
			}
			else if(aRxBuffer[0] == 0x38){
				int i = 1;
				incre = i*8;
				if(Serial == 1){
					len = sprintf(buffer, "\nRecipe 0x38: ");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,11,1000);
				}
				for(int f = 0; f<8; f++){
					recipe[(int)aRxBuffer[1]].time[f+incre] = aRxBuffer[2+f]; //Stores DataBuffer in the ID
				}
				recipe[(int)aRxBuffer[1]].valv[i] = aRxBuffer[10];
			}
			else if(aRxBuffer[0] == 0x39){
				int i = 2;
				incre = i*8;
				if(Serial == 1){
					len = sprintf(buffer, "\nRecipe 0x39: ");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,11,1000);
				}
				for(int f = 0; f<8; f++){
					recipe[(int)aRxBuffer[1]].time[f+incre] = aRxBuffer[2+f]; //Stores DataBuffer in the ID
				}
				recipe[(int)aRxBuffer[1]].valv[i] = aRxBuffer[10];
			}
			else if(aRxBuffer[0] == 0x40){
				int i = 3;
				incre = i*8;
				if(Serial == 1){
					len = sprintf(buffer, "\nRecipe 0x40: ");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,11,1000);
				}
				for(int f = 0; f<8; f++){
					recipe[(int)aRxBuffer[1]].time[f+incre] = aRxBuffer[2+f]; //Stores DataBuffer in the ID
				}
				recipe[(int)aRxBuffer[1]].valv[i] = aRxBuffer[10];
			}
			else if(aRxBuffer[0] == 0x57){
				if(Serial == 1){
					len = sprintf(buffer, "\nMake Drink 0x57: ");
					HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,3,1000);
//					for(int t = 0; t <4; t++){
//						incre = t*8;
//						for(int f = 0; f<8; f++){
//							aRxBuffer[incre+2+f] = recipe[(int)aRxBuffer[1]].time[incre+f]; //Stores DataBuffer in the ID
//						}
//						aRxBuffer[33+t] = recipe[(int)aRxBuffer[1]].valv[t];
//					}
					len = sprintf(buffer, "RecipeContents: ");
					HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
					HAL_UART_Transmit(&huart2,aRxBuffer,11,1000);
				}
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
				//recipe_cmd(aRxBuffer,board);
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
			}
			else{
				if(Serial == 1){
					len = sprintf(buffer, "\nErrorRegister");
					HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
					HAL_UART_Transmit(&huart2, &aRxBuffer[0],1,HAL_MAX_DELAY);
				}
				error = 5;
			}
		}
	else if(error2 == HAL_BUSY){
		if(Serial == 1){
			len = sprintf(buffer,"\nHAL_BUSY");
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
		}
	}
	else if(error2 == HAL_TIMEOUT){
		if(Serial == 1){
			len = sprintf(buffer,"\nHAL_TIMEOUT");
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
		}
	}
	else if(error2 == HAL_ERROR){
		error = 1;
		if(Serial == 1){
			len = sprintf(buffer,"\nERROR");
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
		}
	}
	
	return error;
}
