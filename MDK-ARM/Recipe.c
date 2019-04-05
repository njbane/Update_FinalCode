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

void recipe_cmd(uint8_t ID[], int board){
	/*
	Executes the recipe command based on the ID and board
	
	ID[0]: ID of Drink to be made
	ID[1]: Quanity of the Drink to be made
	board: Selects whether there are one ore two boards or not(0 - ONEBoard | 1 - TWOBoard)
	*/
	int quanity = (int) ID[1];
	while(quanity > 0){
		uint2pin((int)ID[0]);
		for(int i = 0; i <=7; i++){
			//Implements turn on or off a valves if the pin is '1' turns on or off the valve
			if(REGA1.pin[i] == 1){
				I2C_MCP23_ValveControl(i,0,0,&regA1,&regB1,0); //Pump_Valve.h
			}
			if(REGB1.pin[i] == 1){
				I2C_MCP23_ValveControl(i,0,0,&regA1,&regB1,0);
			}
			if(board == 1){ //If commanded valve control board two will execute the command
				if(REGA2.pin[i] == 1){
					I2C_MCP23_ValveControl(i,0,0,&regA2,&regB2,1);
				}
				if(REGB2.pin[i] == 1){
					I2C_MCP23_ValveControl(i,0,0,&regA2,&regB2,1);
				}
			}
			HAL_Delay(500); //Place holder for pump times until we figure out how pumping data will work
			if(REGA1.pin[i] == 1){
				I2C_MCP23_ValveControl(i,1,0,&regA1,&regB1,0);
			}
			if(REGB1.pin[i] == 1){
				I2C_MCP23_ValveControl(i,1,0,&regA1,&regB1,0);
			}
			if(board == 1){
				if(REGA2.pin[i] == 1){
					I2C_MCP23_ValveControl(i,1,0,&regA2,&regB2,1);
				}
				if(REGB2.pin[i] == 1){
					I2C_MCP23_ValveControl(i,1,0,&regA2,&regB2,1);
				}
			}
		}
		quanity--;
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
