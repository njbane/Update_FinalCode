#include "main.h"
#include <stdio.h>
#include "I2C_Polling.h"

struct data recipe[24]; // Initalizes the Data Recipe Struct Array with space for 24

int I2C_Slave(uint8_t ID[], int Serial){
	/*
	Polling I2C Mode - Data will come in one byte to select path for data to be received in.
	
	if(0x37) is recieved then it will expect to recieve an additional 9 bytes of data - 0 ID, 1-4 VALVES, 5-8 PumpTimes
	if(0x57) is recieved then it will expect to recieve two bytes of data - 0 ID, 1 Quanity
	
	int ID[]: returns to the main program which drink is selected to be made and the quanity
	int Serial: Turns on (1) or off(0) the serial communication for debugging
	*/
	int len = 0,RXBuf_Valv_Pump = 9, error = 0;
	uint8_t DataBuffer[RXBuf_Valv_Pump];
	uint8_t aRxBuffer[1];
	char buffer[15];
	
	if(Serial == 1){
		len = sprintf(buffer, "Main");
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, HAL_MAX_DELAY);
	}
	if(HAL_I2C_Slave_Receive(&hi2c1, aRxBuffer, 1, HAL_MAX_DELAY) == HAL_OK){
		if(aRxBuffer[0] == 0x37){
			if(HAL_I2C_Slave_Receive(&hi2c1, DataBuffer, RXBuf_Valv_Pump, HAL_MAX_DELAY) == HAL_OK){
				if(Serial == 1){
					len = sprintf(buffer, "0x37");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,DataBuffer,RXBuf_Valv_Pump,1000);
				}
				for(int i = 0; i <4; i++){
					recipe[(int)DataBuffer[0]].time[i] = DataBuffer[i+1]; 
					recipe[(int)DataBuffer[0]].valv[i] = DataBuffer[i+5];
				}
				ID[0] = 0;
				ID[1] = 0;
			}
			else{error = 2;}
		}
		else if(aRxBuffer[0] == 0x57){
			if(HAL_I2C_Slave_Receive(&hi2c1,DataBuffer,2,HAL_MAX_DELAY) == HAL_OK){
				ID[0] = DataBuffer[0];
				ID[1] = DataBuffer[1];
				if(Serial == 1){
					len = sprintf(buffer, "0x57");
					HAL_UART_Transmit(&huart2,(uint8_t*)buffer,len,1000);
					HAL_UART_Transmit(&huart2,DataBuffer,2,1000);
				}
			}
			else{error = 2;}
		}
	}
	else{error = 1;}
	
	return error;
}