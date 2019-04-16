#ifndef I2C_Polling_H
#define I2C_Polling_H

#include "main.h"

struct data{
	uint8_t valv[4];
	uint8_t time[32];
};

extern struct data recipe[24];

int I2C_Slave(int board, int Serial);

#endif
