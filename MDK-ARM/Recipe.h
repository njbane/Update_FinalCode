#ifndef Recipe_H
#define Recipe_H

#include "main.h"
#include "Pump_Valve.h"
#include <stdio.h>

struct pinList{
	int pin[8];
};

extern struct pinList REGA1;
extern struct pinList REGB1;
extern struct pinList REGA2;
extern struct pinList REGB2;

void recipe_cmd(uint8_t ID[], int board);
void uint2pin(int ID);


#endif
