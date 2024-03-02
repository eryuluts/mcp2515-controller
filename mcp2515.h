/*
 * mcp2515.h
 *
 * Created: 21.05.2020 18:33:33
 *  Author: Eray Ulutas
 */ 


#ifndef MCP2515_H_
#define MCP2515_H_
#include <stdint.h>
#include <avr/interrupt.h>

#define MCPFREQ 32000000UL	// We are using a 32MHz oscillator on mcp2515.

/* CAN FRAME */
typedef struct CanMessage {
	uint32_t id;		// can message identification (arbitration)
	uint8_t extended;	// extended frame indicator bit.
	uint8_t length;		// length of data (last four bits in control field)
	uint8_t rtr;		// remote transmission request indicator bit.
	uint8_t data[8];	// data to transmit (data field)
} CanMessage_t;

/*	Interrupt settings */
#define MCP_INT INT0 // Interrupt used by mcp2515.
#define MCP_VECT INT0_vect	// INT vector.
#define MCP_SENSE ISC01	// Interrupt on falling edge.
void mcpInit(void (*fp)(void));							// Init spi interface and interrupt.

/* Rollover macro. When used RXB0 will overflow to RXB1 */
#define MCPRollover(s) mcpModifyRegister(RXB0CTRL, (1 << BUKT), (s << BUKT));

uint8_t mcpReadRegister(uint8_t addr);
void mcpReadRegisters(uint8_t addr, uint8_t aData[], int num);	
void mcpWriteRegister(uint8_t addr, uint8_t data);
void mcpWriteRegisters(uint8_t addr, uint8_t aData[], int num);
void mcpModifyRegister(uint8_t addr, uint8_t mask, uint8_t data);
void mcpReadRx(uint8_t nBuffer, uint8_t aData[]);					// Read RXnB and clear RXnIF
void mcpLoadTx(uint8_t nBuffer, uint8_t aData[]);
void mcpRTS(uint8_t nBuffer);										// Request to send. (nBuffer is buffer index)
uint8_t mcpGetStatus();
uint8_t mcpGetMode();
void mcpSetMask(uint8_t nMask, uint32_t id, uint8_t ext);
void mcpSetFilter(uint8_t nFilter, uint32_t id, uint8_t ext);
void mcpSetMode(uint8_t mode);
void mcpReset();
void mcpSetBitTiming(uint8_t rCNF1, uint8_t rCNF2, uint8_t rCNF3);	// Set bit timing (look mcp2515 ds. bit timing section)

void mcpIdToBuffer(uint32_t id, uint8_t ext, uint8_t buffer[]);		// Fill id bytes of buffer. (first 4 bytes)
uint32_t mcpBufferToId(uint8_t buffer[]);							// Get id from buffer.

void mcpSendMessage(CanMessage_t canMessage);
CanMessage_t mcpGetMessage(uint8_t nBuffer);



/* INSTRUCTION TABLE */
#define RESET_INSTRUCTION		0xC0			// Instruction for immediate reset
#define READ_INSTRUCTION		0x03			// Read register
#define WRITE_INSTRUCTION		0x02			// Write register
#define READRX_INSTRUCTION		0x90			// 10010mn0 - m,n is address of the RX buffer
#define LOADTX_INSTRUCTION		0x40			// 01000abc - a,b,c is address of the TX buffer
#define RTS_INSTRUCTION			0x80			// 10000abc - a,b,c is address of the TX buffer
#define READSTAT_INSTRUCTION	0xA0			// Read device status instruction
#define RXSTAT_INSTRUCTION		0xB0			// Read receive buffer status instruction
#define BITMODIFY_INSTRUCTION	0x05			// For specific bit modifications

/* Transmit Buffers */
#define TXB0					0				// Transmit buffer 0
#define TXB1					1				// Transmit buffer 1
#define TXB2					2				// Transmit buffer 2


/* Receive Buffers */
#define RXB0					0				// Receive buffer 0
#define RXB1					1				// Receive buffer 1


#define nMCP_BUFFER_BYTES		13				// nBytes of a rx and tx buffers.

/* bytes of buffers */
#define MCP_SIDH				0
#define MCP_SIDL				1				// IDE[3], SRR[4] if standart frame.
#define MCP_EIDH				2
#define MCP_EIDL				3
#define MCP_DLC					4				// extended RTR[6], data length DLC[3:0]
#define MCP_D0					5
#define MCP_D1					6
#define MCP_D2					7
#define MCP_D3					8
#define MCP_D4					9
#define MCP_D5					10
#define MCP_D6					11
#define MCP_D7					12

/* FILTERS */
#define	RXF0					0				// RXB0 filter 0 (or RXB1 filter 0, when rollover is active)
#define	RXF1					1				// RXB0 filter 1 (or RXB1 filter 1, when rollover is active)
#define	RXF2					2				// RXB1 filter 2
#define	RXF3					3				// RXB1 filter 3
#define	RXF4					4				// RXB1 filter 4
#define	RXF5					5				// RXB1 filter 5

/* MASKS */
#define	MASK0					0				// RXB0 Mask
#define	MASK1					1				// RXB1 Mask


/* READ STATE INS BITS */
#define STATE_RX0IF				0
#define STATE_RX1IF				1
#define STATE_TX0REQ			2
#define STATE_TX0IF				3
#define STATE_TX1REQ			4
#define STATE_TX1IF				5
#define STATE_TX2REQ			6
#define STATE_TX2IF				7


/*
	OP Modes
	When changing modes, change will not occur until all pending messages transmitted.
	It is recommended to verify change by checking CANSTAT register.
*/
#define MCP_NORMAL_MODE			0				// Only mode for transmitting messages.
#define MCP_SLEEP_MODE			1				// SLEEP. When wake up op mode will be LISTENONLY
#define MCP_LOOPBACK_MODE		2				// Silent mode. Allow transmission to itself wo. actually transmitting over can bus.
#define MCP_LISTENONLY_MODE		3				// Silent mode. All messages on the bus will be received (including errors).
#define MCP_CONF_MODE			4				// For setting masks, filters, bit-timing and pins.


#define CANSTAT					0x0E			// CAN status register
#define CANCTRL					0x0F			// CAN control register


/* Bit Timing registers */
#define CNF3					0x28
#define CNF2					0x29
#define CNF1					0x2A

/* Interrupt enable register */
#define CANINTE					0x2B
#define MERRE					7				// RW-0, Message error interrupt enable bit
#define WAKIE					6				// RW-0, Wakeup interrupt enable bit
#define ERRIE					5				// RW-0, Error interrupt enable bit
#define TX2IE					4				// RW-0, Transmit buffer 2 empty interrupt enable bit
#define TX1IE					3				// RW-0, Transmit buffer 1 empty interrupt enable bit
#define TX0IE					2				// RW-0, Transmit buffer 0 empty interrupt enable bit
#define RX1IE					1				// RW-0, Receive buffer 1 full interrupt enable bit
#define RX0IE					0				// RW-0, Receive buffer 0 full interrupt enable bit

/* Interrupt flag register */
#define CANINTF					0x2C
#define MERRF					7				// RW-0, Message error interrupt flag bit
#define WAKIF					6				// RW-0, Wakeup interrupt flag bit
#define ERRIF					5				// RW-0, Error interrupt flag bit
#define TX2IF					4				// RW-0, Transmit buffer 2 empty interrupt flag bit
#define TX1IF					3				// RW-0, Transmit buffer 1 empty interrupt flag bit
#define TX0IF					2				// RW-0, Transmit buffer 0 empty interrupt flag bit
#define RX1IF					1				// RW-0, Receive buffer 1 full interrupt flag bit
#define RX0IF					0				// RW-0, Receive buffer 0 full interrupt flag bit

/* Error flag register */
#define EFLG					0x2D
#define RX1OVR					7				// RW-0, Receive buffer 1 overflow flag bit
#define RX0OVR					6				// RW-0, Receive buffer 0 overflow flag bit
#define TXBO					5				// R-0, Bus-off error flag bit
#define TXEP					4				// R-0, Transmit error - passive flag bit
#define RXEP					3				// R-0, Receive error - passive flag bit
#define TXWAR					2				// R-0, Transmit error warning flag bit
#define RXWAR					1				// R-0, Receive error warning flag bit
#define EWARN					0				// R-0, Error warning flag bit

/* Error Counters */
#define TEC						0x1C			// Transmit error counter
#define REC						0x1D			// Receive error counter

/* Transmit buffer n control register */
#define TXBnCTRL(n)				0x30+(n*0x10)
#define TXB0CTRL				TXBnCTRL(0)
#define TXB1CTRL				TXBnCTRL(1)
#define TXB2CTRL				TXBnCTRL(2)
#define ABTF					6				// R-0, Message aborted flag bit
#define MLOA					5				// R-0, Message lost arbitration bit
#define TXERR					4				// R-0, Transmit error detected bit
#define TXREQ					3				// RW-0, Message transmit request bit
#define TXP1					1				// RW-0, Transmit buffer priority bit 1
#define TXP0					0				// RW-0, Transmit buffer priority bit 0


/* Receive buffer n control register */
#define RXBnCTRL(n)				0x60+(n*0x10)
#define RXB0CTRL				RXBnCTRL(0)
#define RXB1CTRL				RXBnCTRL(1)
#define RXM1					6				// RW-0, Receive buffer operating mode bit 1
#define RXM0					5				// RW-0, Receive buffer operating mode bit 0
#define RXRTR					3				// R-0, Receive remote transfer request bit
#define BUKT					2				// RW-0, Rollover enable bit (used only by RXB0CTRL)
#define FILHIT2					2				// R-0, Filter hit bit 2 (used only by RXB1CTRL)
#define FILHIT1					1				// R-0, Filter hit bit 1 (used only by RXB1CTRL)
#define FILHIT0					0				// R-0, Filter hit bit 0


/* Mask n standard identifier high */
#define RXMnSIDH(n)				0x20+(n*4)
#define RXM0SIDH				RXMnSIDH(0)
#define RXM1SIDH				RXMnSIDH(1)

/* Filter n standard identifier high */
#define RXFnSIDH(n)				0+(n*4)+((n>2)?4:0)
#define RXF0SIDH				RXFnSIDH(0)
#define RXF1SIDH				RXFnSIDH(1)
#define RXF2SIDH				RXFnSIDH(2)
#define RXF3SIDH				RXFnSIDH(3)
#define RXF4SIDH				RXFnSIDH(4)
#define RXF5SIDH				RXFnSIDH(5)

#endif /* MCP2515_H_ */