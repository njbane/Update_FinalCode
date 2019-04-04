#include "main.h"
//#include <stdlib.h>
#include "Pump_Valve.h"

int I2C_MCP23018W(uint8_t reg_addr, uint8_t data, I2C_HandleTypeDef multi_board){
	/*
	Function to write to MCP23018 GPIO Expander using I2C 
	GPIO Expander - Microchip MCP23018 16 PIN GPIO Expander
	
	reg_addr: Port A = 0x12
					  Port B = 0x13
	data: Hexadecimal Code to input the register
	multi_board: hi2c1, hi2c2, hi2c3 depending on which board it is using
	*/
	int error_flag = 0; //Initalize the error flag 
	uint8_t write_data[2]= {(uint8_t) reg_addr,(uint8_t) data}; //Combines Data Packet
	uint8_t address = 0x40; //I2C Address of the MCP23018
	
	if(HAL_I2C_Master_Transmit(&multi_board,address,write_data,2,HAL_MAX_DELAY) == HAL_OK){
		error_flag = 0;
	}
	else{
		error_flag = 2;
	}
	
	return error_flag;
}

int I2C_MCP23_Setup(uint8_t* reg_statA, uint8_t* reg_statB, I2C_HandleTypeDef board_sl){
	/*
	Initalizes the MCP23018 to allow it control the vales by setting up the registers
	necessary to control the valves
	
	*reg_statA: Status of register A currently
	*reg_statB: Status of register B currently
	board_sl: Selecting with board will be initialized
	
	returns whether a write error occured during the process;
	
	*/
	uint8_t IODIRA = 0x00, IODIRB = 0x01, GPPUA = 0x0C, GPPUB = 0x0D, gpioA = 0x12, gpioB = 0x13;
	uint8_t address = 0x40;
	int write_error;
	
	if(HAL_I2C_IsDeviceReady(&board_sl, address, 5, HAL_MAX_DELAY) == HAL_OK){
		write_error = I2C_MCP23018W(IODIRA, 0x00, board_sl); //Input Output Data Direction A - OUTPUT
		HAL_Delay(10);
		write_error = I2C_MCP23018W(IODIRB, 0x00, board_sl);
		HAL_Delay(10);
		write_error = I2C_MCP23018W(GPPUA, 0xFF, board_sl); // Pull Up Register - Set Pull Up Resistor
		HAL_Delay(10);
		write_error = I2C_MCP23018W(GPPUB, 0xFF, board_sl);
		HAL_Delay(10);
		write_error = I2C_MCP23018W(gpioA, 0x00, board_sl);
		HAL_Delay(10);
		write_error = I2C_MCP23018W(gpioB, 0x00, board_sl); // Turen the GPIO Register - Set 1 to turn off valves
		
		*reg_statA = 0x00;
		*reg_statB = 0x00;
	}
	else{write_error = 1;}
	return write_error;
}

int I2C_MCP23_ValveControl(int pin, int status, int reg, uint8_t *reg_statA, uint8_t *reg_statB, int board){
	/*
	Pin: Select Which Pin needs to be turned on or off (0-7)
	Status: 1 turns pin on(low) and 0 turns pin off(high)
	Reg: which reg to operate on 1 = regb | 0 = regA
	*reg_statA: pointer to keep track of changes made to registerA
	*reg_statB: pointer to keep track of changes made to registerB
	board: int 0 = hi2c2, int 1 = hi2c3 - Allows the selection of which board to control
	*/
	uint8_t gpioA = 0x12;
	uint8_t gpioB = 0x13;
	uint8_t pin_addr[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
	int error;
	
	
	if(board == 0){ // Controls the board on hi2c2 port of the STM32L476
		if(status == 1){ //Turns gate of the open drain on creating a logic low output
			if(reg == 1){ //Controls Pins 1-7 and 24 
			 //Prevent user from overflowing register 
				*reg_statB = *reg_statB - pin_addr[pin];
				error = I2C_MCP23018W(gpioB,*reg_statB,hi2c2);
			}
			else{ // Controls pins 14 - 22
				// Prevents user from overflowing value
				*reg_statA = *reg_statA - pin_addr[pin]; //Updates status of reg_statA by adding the value held in pin_addr(pin-14)
				error = I2C_MCP23018W(gpioA,*reg_statA,hi2c2); // Function to write to I2C addres of GPIO A and the value to that register to MCP23018
		
			}
		}
		else{ //Turns gate of the open drain off creating a logic high output
			if(reg == 1){
			 // If 
				*reg_statB = *reg_statB + pin_addr[pin];
				error = I2C_MCP23018W(gpioB,*reg_statB, hi2c2);
			}
			else{
			
				*reg_statA = *reg_statA + pin_addr[pin];
				error = I2C_MCP23018W(gpioA,*reg_statA, hi2c2);

			}
		}
	}
	else{ // Controls the board on hi2c3 port of the STM32L476
		if(status == 1){ //Turns gate of the open drain on creating a logic low output
			if(reg == 1){ //Controls Pins 1-7 and 24 
			 //Prevent user from overflowing register 
				*reg_statB = *reg_statB - pin_addr[pin];
				error = I2C_MCP23018W(gpioB,*reg_statB,hi2c3);
			}
			else{ // Controls pins 14 - 22
				// Prevents user from overflowing value
				*reg_statA = *reg_statA - pin_addr[pin]; //Updates status of reg_statA by adding the value held in pin_addr(pin-14)
				error = I2C_MCP23018W(gpioA,*reg_statA,hi2c3); // Function to write to I2C addres of GPIO A and the value to that register to MCP23018
		
			}
		}
		else{ //Turns gate of the open drain off creating a logic high output
			if(reg == 1){
			 // If 
				*reg_statB = *reg_statB + pin_addr[pin];
				error = I2C_MCP23018W(gpioB,*reg_statB, hi2c3);
			}
			else{
			
				*reg_statA = *reg_statA + pin_addr[pin];
				error = I2C_MCP23018W(gpioA,*reg_statA, hi2c3);
			}
		}
	}
	return error;
}

void PWM_PumpControl(int Pump_Num,int Direc,int Speed){
	/*
	PWM_PumpControl controls the pumps by choosing which pump, direction, and speed
	
	Pump_Num: Select which pump to use (1-6)
	Direc: Choose which Direction 0 - Full Stop, 1 - Forward, 2 - Backware
	Speed: Select speed of pump from 0 - 255
	*/
	if (Pump_Num == 1)
	{
		if(Direc ==1){ // Direction based off the data sheet
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0); // Sets compare register to set pulse width as part of a whole
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Speed);
		}
		else if(Direc == 2){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Speed);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
		}
	}
	else if (Pump_Num == 2)
	{
		if(Direc ==1){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Speed);
		}
		else if(Direc == 2){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Speed);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		}
	}
	else if (Pump_Num == 3){
		if(Direc ==1){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Speed);
		}
		else if(Direc == 2){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Speed);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		}
	}
	else if (Pump_Num == 4){
		if(Direc ==1){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,Speed);
		}
		else if(Direc == 2){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,Speed);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
		}
	}
	else if (Pump_Num == 5){
		if(Direc ==1){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Speed);
		}
		else if(Direc == 2){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Speed);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
		}
	}
	else if (Pump_Num == 6){
		if(Direc ==1){
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Speed);
		}
		else if(Direc == 2){
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Speed);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
		}
	}
}
