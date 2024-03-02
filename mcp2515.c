/*
 * mcp2515.c
 *
 * Created: 21.05.2020 19:43:21
 *  Author: Eray Ulutas
 */ 

#include "mcp2515.h"
#include "spiMaster.h"
#include <avr/interrupt.h>

void (*int_handler)(void);
void mcpInit(void (*fp)(void)) {
	int_handler = fp;
	
	MCUCR |= (1 << MCP_SENSE);	// Trigger interrupt on falling edge of INT0.
	GICR |= (1 << MCP_INT);
	
	spiMasterInit();
}

ISR(MCP_VECT) {
	int_handler();
}


uint8_t mcpReadRegister(uint8_t addr) {
	SPIMasterBegin
	
	spiMasterTx(READ_INSTRUCTION);
	spiMasterTx(addr);
	
	uint8_t data = spiMasterTx(0);
	
	SPIMasterEnd
	return data;
}

void mcpReadRegisters(uint8_t addr, uint8_t aData[], int num) {
	SPIMasterBegin
	
	spiMasterTx(READ_INSTRUCTION);
	spiMasterTx(addr);
	
	for(int i = 0; i < num; i++)
		aData[i] = spiMasterTx(0);	// mcp2515 has auto-increment of addr pointer.
		
	SPIMasterEnd
}

void mcpWriteRegister(uint8_t addr, uint8_t data) {
	SPIMasterBegin
		
	spiMasterTx(WRITE_INSTRUCTION);
	spiMasterTx(addr);
	spiMasterTx(data);
		
	SPIMasterEnd
}

void mcpWriteRegisters(uint8_t addr, uint8_t aData[], int num) {
	SPIMasterBegin
	
	spiMasterTx(WRITE_INSTRUCTION);
	spiMasterTx(addr);
	
	for(int i = 0; i < num; i++)
		spiMasterTx(aData[i]);	// mcp2515 has auto-increment of addr pointer.
	
	SPIMasterEnd
}

void mcpModifyRegister(uint8_t addr, uint8_t mask, uint8_t data) {
	SPIMasterBegin
	
	spiMasterTx(addr);
	spiMasterTx(mask);
	spiMasterTx(data);
	
	SPIMasterEnd
}

void mcpReadRx(uint8_t nBuffer, uint8_t aData[]) {
	SPIMasterBegin
	uint8_t ins = READRX_INSTRUCTION | (nBuffer << 2);
	spiMasterTx(ins);
	
	for(int i = 0; i < nMCP_BUFFER_BYTES; i++)
		aData[i] = spiMasterTx(0);
	
	SPIMasterEnd
}

void mcpLoadTx(uint8_t nBuffer, uint8_t aData[]) {
	SPIMasterBegin
	uint8_t ins = LOADTX_INSTRUCTION | (nBuffer << 1);
	spiMasterTx(ins);
	
	for(int i = 0; i < nMCP_BUFFER_BYTES; i++)
		spiMasterTx(aData[i]);
	
	SPIMasterEnd
}

void mcpRTS(uint8_t nBuffer) {
	SPIMasterBegin
	spiMasterTx(RTS_INSTRUCTION | (1 << nBuffer));
	SPIMasterEnd
}

uint8_t mcpGetStatus() {
	SPIMasterBegin
	uint8_t status = spiMasterTx(READSTAT_INSTRUCTION);
	SPIMasterEnd
	
	return status;
}

uint8_t mcpGetMode() {
	return mcpReadRegister(CANSTAT) >> 5;
}

void mcpReset() {
	SPIMasterBegin
	spiMasterTx(RESET_INSTRUCTION);
	SPIMasterEnd
}

void mcpSetMode(uint8_t mode) {
	SPIMasterBegin
	while(mcpGetMode() != mode)
		mcpModifyRegister(CANCTRL, 7 << 5, mode << 5);
	SPIMasterEnd
}

void mcpSetMask(uint8_t nMask, uint32_t id, uint8_t ext) {
	if (mcpGetMode() != MCP_CONF_MODE) return;	// for modifying mask registers you must switch to conf mode.
	
	uint8_t addr = (nMask == 1) ? RXM1SIDH : RXM0SIDH;
	if (nMask > 1) return;
	
	uint8_t mask[4];
	mcpIdToBuffer(id, ext, mask);
	
	mcpWriteRegisters(addr, mask, 4);
}

void mcpSetFilter(uint8_t nFilter, uint32_t id, uint8_t ext) {
	if (mcpGetMode() != MCP_CONF_MODE) return;	// for modifying filter registers you must switch to conf mode.
	
	uint8_t addr;
	switch(nFilter) {
		case 0: addr = RXF0SIDH; break;
		case 1: addr = RXF1SIDH; break;
		case 2: addr = RXF2SIDH; break;
		case 3: addr = RXF3SIDH; break;
		case 4: addr = RXF4SIDH; break;
		case 5: addr = RXF5SIDH; break;
		default: return;
	}
	
	uint8_t mask[4];
	mcpIdToBuffer(id, ext, mask);
	
	mcpWriteRegisters(addr, mask, 4);
}

void mcpSetBitTiming(uint8_t rCNF1, uint8_t rCNF2, uint8_t rCNF3) {
	if (mcpGetMode() != MCP_CONF_MODE) return;	// for modifying cnf registers you must switch to conf mode.
	
	uint8_t data[] = {rCNF3, rCNF2, rCNF1};
	mcpWriteRegisters(rCNF3, data, 3);
}

void mcpIdToBuffer(uint32_t id, uint8_t ext, uint8_t buffer[]) {
	if(ext) {
		buffer[MCP_EIDL] = (uint8_t) id; id >>= 8;
		buffer[MCP_EIDH] = (uint8_t) id; id >>= 8;
		buffer[MCP_SIDL] = (uint8_t) ((id << 5) | (id & 3)) | (1 << 3); id >>= 5;
		buffer[MCP_SIDH] = (uint8_t) id;
	} else {
		buffer[MCP_EIDL] = 0;
		buffer[MCP_EIDH] = 0;
		buffer[MCP_SIDL] = (uint8_t) ((id << 5) | (id & 3)); id >>= 5;
		buffer[MCP_SIDH] = (uint8_t) id;
	}
}

uint32_t mcpBufferToId(uint8_t buffer[]) {
	uint32_t id = 0;
	
	uint8_t ext = buffer[MCP_SIDL] & (1 << 3);
	if(ext) {
		id = buffer[MCP_SIDH] << 3;
		id |= (buffer[MCP_SIDL] >> 5); id <<= 2;
		id |= (buffer[MCP_SIDL] & 3); id <<= 8;
		id |= buffer[MCP_EIDH]; id <<= 8;
		id |= buffer[MCP_EIDL];
	} else {
		id = buffer[MCP_SIDH] << 3;
		id |= buffer[MCP_SIDL] >> 5;
	}
	
	return id;
}


// Return available buffer index or -1
int chkTxBuffer() {
	uint8_t mcp_stat = mcpGetStatus();
	
	int available_buffer = -1;
	if (mcp_stat & (1 << STATE_TX0REQ))
		available_buffer = TXB0;
	else if (mcp_stat & (1 << STATE_TX1REQ))
		available_buffer = TXB1;
	else if (mcp_stat & (1 << STATE_TX2REQ))
		available_buffer = TXB2;
	
	return available_buffer;
}

void mcpSendMessage(CanMessage_t canMessage) {
	uint8_t buffer[nMCP_BUFFER_BYTES] = {0};
	
	mcpIdToBuffer(canMessage.id, canMessage.extended, buffer);
	buffer[MCP_DLC] = canMessage.length | (canMessage.rtr << 6);
	
	for(int i = 0; i < canMessage.length; i++)
		buffer[MCP_D0 + i] = canMessage.data[i];
	
	int available_buffer = chkTxBuffer();
	if (available_buffer == -1) return;
	
	mcpLoadTx(available_buffer, buffer);
	mcpRTS(available_buffer);
}

CanMessage_t mcpGetMessage(uint8_t nBuffer) {
	uint8_t buffer[nMCP_BUFFER_BYTES];
	mcpReadRx(nBuffer, buffer);	// read mcp rx_buffer into buffer.
	
	uint32_t id = mcpBufferToId(buffer);
	uint8_t ext = buffer[MCP_SIDL] & (1 << 3);
	
	uint8_t rtr = 0;
	if (ext)
		rtr = buffer[MCP_SIDL] & (1 << 4);
	else
		rtr = buffer[MCP_DLC] & (1 << 6);
	
	uint8_t length = buffer[MCP_DLC] & (0b1111);
	
	CanMessage_t message;
	message.id = id;
	message.extended = ext;
	message.rtr = rtr;
	message.length = length;
	for(int i = 0; i < message.length; i++)
	message.data[i] = buffer[MCP_D0 + i];
	
	return message;
}