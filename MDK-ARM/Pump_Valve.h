#ifndef Pump_Valve_H
#define Pump_Valve_H

#include "main.h"

int I2C_MCP23018W(uint8_t reg_addr, uint8_t data,I2C_HandleTypeDef multi_board);
int I2C_MCP23_Setup(uint8_t* reg_statA, uint8_t* reg_statB,I2C_HandleTypeDef board_sl);
int I2C_MCP23_ValveControl(int pin, int status, int reg, uint8_t *reg_statA, uint8_t *reg_statB, int board); 
void PWM_PumpControl(int Pump_Num,int Direc,int Speed);

#endif