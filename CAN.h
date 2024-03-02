/*
 * CAN.h
 *
 * Created: 22.05.2020 19:06:29
 *  Author: Eray Ulutas
 */ 


#ifndef CAN_H_
#define CAN_H_
#include "mcp2515.h"

void canInit();
void canSendMessage(CanMessage_t message);
CanMessage_t *canGetMessage();


#endif /* CAN_H_ */