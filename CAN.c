/*
 * CAN.c
 *
 * Created: 24.05.2020 01:32:46
 *  Author: Eray Ulutas
 */ 

#include "CAN.h"
#include "mcp2515.h"

#define QUEUE_SIZE 3
CanMessage_t MessageQueue[QUEUE_SIZE];
int iMessageQueue = 0;

void appendQueue(CanMessage_t message) {
	if (iMessageQueue == QUEUE_SIZE) return;
	
	MessageQueue[iMessageQueue++] = message;
}

CanMessage_t *canGetMessage() {
	if (iMessageQueue == 0) return 0;
	
	return &MessageQueue[--iMessageQueue];
}

void canSendMessage(CanMessage_t message) {
	mcpSendMessage(message);
}

/* when interrupt occurs get messages and write them to local buffer*/
void mcpInterrupt() {
	uint8_t rx_flags = mcpGetStatus() & 3;
	if (rx_flags & 1)
		appendQueue(*canGetMessage(RXB0));
	
	if (rx_flags & 2)
		appendQueue(*canGetMessage(RXB1));
}


void canInit() {
	mcpReset();
	
	mcpInit(mcpInterrupt);
	
	mcpSetMode(MCP_CONF_MODE);
	mcpSetMask(MASK0, 0, 1);
	mcpSetMask(MASK1, 0, 1);
	
	mcpSetBitTiming(0x04, 0xD2, 0x42);		// set bit-timing (check ds. of mcp2515)
	MCPRollover(1)							// enable rollover.
	
	mcpSetMode(MCP_NORMAL_MODE);
}