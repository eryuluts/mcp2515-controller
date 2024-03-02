/*
 * CanInterface.c
 *
 * Created: 21.05.2020 17:54:24
 * Author : Eray Ulutaþ
 */ 

#include <avr/io.h>
#include "CAN.h"


int main(void)
{
	CanMessage_t *pMessage;
    while (1) 
    {	
		pMessage = canGetMessage();
		if (pMessage) {
			// UPDATE STATE
		}
    }
}

