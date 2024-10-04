#ifndef MFRC522_h
#define MFRC522_h


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
// GPIO control for the reset pin

#define SYSFS_GPIO_PATH             "/sys/class/gpio"
#define SYSFS_GPIO_EXPORT_FN        "/export"
#define SYSFS_GPIO_UNEXPORT_FN      "/unexport"
#define SYSFS_GPIO_VALUE            "/value"
#define SYSFS_GPIO_DIRECTION        "/direction"
#define SYSFS_GPIO_EDGE             "/edge"

#define DIR_IN                      "in"
#define DIR_OUT                     "out"

#define VALUE_HIGH                  "1"
#define VALUE_LOW                   "0"

#define EDGE_RISING                 "rising"
#define EDGE_FALLING                "falling"

// Defined as 4MHz in original library but datasheet claims up to 10MHz
#define MFRC522_BIT_RATE	1000000

#define READ				0x80

#define WRITE				0x00

#define RESET_PIN			534

#define SPI_DEVICE          "/dev/spidev0.0"


static const uint8_t SELF_TEST_BYTES[] = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};

// Defined in MFRC522 9.2 must be bit shifted to the right by 1

enum PCD_Register {
	CommandReg 			= (uint8_t)((0x01 << 1) & 0x7E),
	ComLenReg			= (uint8_t)((0x02 << 1) & 0x7E),
	DivLenReg			= (uint8_t)((0x03 << 1) & 0x7E),
	ComIrqReg			= (uint8_t)((0x04 << 1) & 0x7E),
	DivIrqReg			= (uint8_t)((0x05 << 1) & 0x7E),
	ErrorReg			= (uint8_t)((0x06 << 1) & 0x7E),
	Status1Reg			= (uint8_t)((0x07 << 1) & 0x7E),
	Status2Reg			= (uint8_t)((0x08 << 1) & 0x7E),
	FifoDataReg			= (uint8_t)((0x09 << 1) & 0x7E),
	FifoLevelReg		= (uint8_t)((0x0A << 1) & 0x7E),
	WaterLevelReg		= (uint8_t)((0x0B << 1) & 0x7E),
	ControlReg			= (uint8_t)((0x0C << 1) & 0x7E),
	BitFramingReg		= (uint8_t)((0x0D << 1) & 0x7E),
	CollReg				= (uint8_t)((0x0E << 1) & 0x7E),
	ModeReg				= (uint8_t)((0x11 << 1) & 0x7E),
	TxModeReg			= (uint8_t)((0x12 << 1) & 0x7E),
	RxModeReg			= (uint8_t)((0x13 << 1) & 0x7E),
	TxControlReg		= (uint8_t)((0x14 << 1) & 0x7E),
	TxAskReg			= (uint8_t)((0x15 << 1) & 0x7E),
	TxSelReg			= (uint8_t)((0x16 << 1) & 0x7E),
	RxSelReg			= (uint8_t)((0x17 << 1) & 0x7E),
	RxThresholdReg		= (uint8_t)((0x18 << 1) & 0x7E),
	DemoDReg			= (uint8_t)((0x19 << 1) & 0x7E),
	MfTxReg				= (uint8_t)((0x1C << 1) & 0x7E),
	MfRxReg				= (uint8_t)((0x1D << 1) & 0x7E),
	SerialSpeedReg		= (uint8_t)((0x1F << 1) & 0x7E),
	CrcResultRegH		= (uint8_t)((0x21 << 1) & 0x7E),
	CrcResultRegL		= (uint8_t)((0x22 << 1) & 0x7E),
	ModWidthReg			= (uint8_t)((0X24 << 1) & 0x7E),
	RfcfgReg			= (uint8_t)((0X26 << 1) & 0x7E),
	GsnReg				= (uint8_t)((0x27 << 1) & 0x7E),
	CwgspReg			= (uint8_t)((0x28 << 1) & 0x7E),
	ModgspReg			= (uint8_t)((0x29 << 1) & 0x7E),
	TmodeReg			= (uint8_t)((0x2A << 1) & 0x7E),
	TPrescalerReg		= (uint8_t)((0x2B << 1) & 0x7E),
	TReloadRegH			= (uint8_t)((0x2C << 1) & 0x7E),
	TReloadRegL			= (uint8_t)((0x2D << 1) & 0x7E),
	TCounterValRegH		= (uint8_t)((0x2E << 1) & 0x7E),
	TCounterValRegL		= (uint8_t)((0x2F << 1) & 0x7E),
	TestSel1Reg			= (uint8_t)((0x31 << 1) & 0x7E),
	TestSel2Reg			= (uint8_t)((0x32 << 1) & 0x7E),
	TestPinenReg		= (uint8_t)((0x33 << 1) & 0x7E),
	TestPinValueReg		= (uint8_t)((0x34 << 1) & 0x7E),
	TestBusReg			= (uint8_t)((0x35 << 1) & 0x7E),
	AutoTestReg			= (uint8_t)((0x36 << 1) & 0x7E),
	VersionReg			= (uint8_t)((0x37 << 1) & 0x7E),
	AnalogTestReg		= (uint8_t)((0x38 << 1) & 0x7E),
	TestDac1Reg			= (uint8_t)((0x39 << 1) & 0x7E),
	TestDac2Reg			= (uint8_t)((0x3A << 1) & 0x7E),
	TestAdcReg			= (uint8_t)((0x3B << 1) & 0x7E)
};

// Defined in MFRC522 10.3 

typedef enum _PCD_Command{
	PCD_Idle					=0x00,	
	PCD_Mem						=0x01,
	PCD_GenerateRandomID		=0X02,
	PCD_CalcCRC					=0X03,
	PCD_Transmit				=0X04,
	PCD_NoCmdChange				=0X07,
	PCD_Receive					=0X08,
	PCD_SelfTest				=0x09,
	PCD_Transceive				=0X0C,
	PCD_MFAuthent				=0X0E,
	PCD_SoftReset				=0X0F,
}	PCD_Command;

// MFRC522 RxGain masks, defined in 9.3.3.6

typedef enum _PCD_RxGain {
	RxGain_18dB = 		0x00 << 4, // 000b - 18 dB, minimum
	RxGain_23dB = 		0x01 << 4, // 001b - 23 dB
	RxGain_18dB_2 =		0x02 << 4, // 010b - 18 dB, it seems 010b is a duplicate for 000b
	RxGain_23dB_2 =		0x03 << 4, // 011b - 23 dB, it seems 011b is a duplicate for 001b
	RxGain_33dB = 		0x04 << 4, // 100b - 33 dB, average, and typical default
	RxGain_38dB = 		0x05 << 4, // 101b - 38 dB
	RxGain_43dB = 		0x06 << 4, // 110b - 43 dB
	RxGain_48dB = 		0x07 << 4, // 111b - 48 dB, maximum
	RxGain_min =		0x00 << 4, // 000b - 18 dB, minimum, convenience for RxGain_18dB
	RxGain_avg =		0x04 << 4,		   // 100b - 33 dB, average, convenience for RxGain_33dB
	RxGain_max = 		0x07 << 4 // 111b - 48 dB, maximum, convenience for RxGain_48dB
}	PCD_RxGain;


// do not have access to ISO 14443-3 have to trust library

typedef enum _PICC_Command {
	// The commands used by the PCD to manage communication with several PICCs
	// (ISO 14443-3, Type A, section 6.4)
	PICC_CMD_REQA = 		0x26, 	// REQuest command, Type A. Invites PICCs in state
						  			// IDLE to go to READY and prepare for anti collision
						  			// or selection. 7 bit frame.
	PICC_CMD_WUPA = 		0x52, 	// Wake-UP command, Type A. Invites PICCs in state
						  			// IDLE and HALT to go to READY(*) and prepare for
						  			// anti collision or selection. 7 bit frame.
	PICC_CMD_CT = 			0x88,   // Cascade Tag. Not really a command, but used during
						  			// anti collision.
	PICC_CMD_SEL_CL1 = 		0x93, 	// Anti collision/Select, Cascade Level 1
	PICC_CMD_SEL_CL2 = 		0x95, 	// Anti collision/Select, Cascade Level 2
	PICC_CMD_SEL_CL3 = 		0x97, 	// Anti collision/Select, Cascade Level 3
	PICC_CMD_HLTA = 		0x50, 	// HaLT command, Type A. Instructs an ACTIVE PICC to
						  			// go to state HALT.
	// The commands used for MIFARE Classic (from
	// http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these
	// commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	PICC_CMD_MF_AUTH_KEY_A = 	0x60, 		// Perform authentication with Key A
	PICC_CMD_MF_AUTH_KEY_B = 	0x61, 		// Perform authentication with Key B
	PICC_CMD_MF_READ = 			0x30,	   	// Reads one 16 byte block from the
											// authenticated sector of the PICC. Also used for
											// MIFARE Ultralight.
	PICC_CMD_MF_WRITE = 		0xA0, 		// Writes one 16 byte block to the
							  				// authenticated sector of the PICC. Called
							  				// "COMPATIBILITY WRITE" for MIFARE Ultralight.
	PICC_CMD_MF_DECREMENT = 	0xC0, 		// Decrements the contents of a block and
								  			// stores the result in the internal data
								  			// register.
	PICC_CMD_MF_INCREMENT = 	0xC1, 		// Increments the contents of a block and
								  			// stores the result in the internal data
								  			// register.
	PICC_CMD_MF_RESTORE =		0xC2, 		// Reads the contents of a block into the internal data register.
	PICC_CMD_MF_TRANSFER =		0xB0, 		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from
	// http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE
	// Ultralight.
	PICC_CMD_UL_WRITE = 		0xA2 		// Writes one 4 byte page to the PICC.
} PICC_Command;

// Return codes from the functions in this class. Remember to update
// GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some
// optimizations are triggered
typedef enum _StatusCode {
	STATUS_OK,			  		// Success
	STATUS_ERROR,		   		// Error in communication
	STATUS_COLLISION,	  		// Collision detected
	STATUS_TIMEOUT,		   		// Timeout in communication.
	STATUS_NO_ROOM,		   		// A buffer is not big enough.
	STATUS_INTERNAL_ERROR, 		// Internal error in the code. Should not happen ;-)
	STATUS_INVALID,		   		// Invalid argument.
	STATUS_CRC_WRONG,	  		// The CRC_A does not match
	STATUS_MIFARE_NACK = 0xff 	// A MIFARE PICC responded with NAK.
} StatusCode;

typedef struct {
	uint8_t size;
	uint8_t uidByte[10];
	uint8_t sak;
} Uid;

/*******************************************************************************
*GPIO functions
*******************************************************************************/
void GPIO_init(int GPIOPin);
int GPIO_SetDirection(int GPIOPin, char* direction);
int GPIO_SetValue(int GPIOPin, char* value);
int _GPIO_SetFileValue(char *fname, char *value);
/*******************************************************************************
*Initialzation functions
*******************************************************************************/
int _MFRC522_SPIInit();
void MFRC522_Init();
/*******************************************************************************
*Communication with MFRC522
*******************************************************************************/
int MFRC522_Self_Test();
void _MFRC522_WriteRegister(uint8_t reg, uint8_t command);
uint8_t _MFRC522_ReadRegister(uint8_t reg);
void _MFRC522_AntennaOn();
void _MFRC522_ClearRegisterBitMask(uint8_t reg, uint8_t mask);
void _MFRC522_SetRegisterBitMask(uint8_t reg, uint8_t mask);
StatusCode _MFRC522_CalculateCRC(uint8_t *data, uint8_t data_length, uint8_t *result);
StatusCode _MFRC522_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *tx_data, uint8_t tx_length, uint8_t *rx_data, uint8_t *rx_length,  uint8_t *valid_bits, uint8_t rxAlign);
StatusCode _MFRC522_TransceiveData(uint8_t *tx_data, uint8_t tx_length, uint8_t *rx_data, uint8_t *rx_length, uint8_t *valid_bits, uint8_t rxAlign);
/*******************************************************************************
*Communication with PICC(card)
*******************************************************************************/
StatusCode PICC_REQA_or_WUPA(uint8_t command, uint8_t *buffer_ATQA, uint8_t *buffer_size);
StatusCode PICC_RequestA(uint8_t *buffer_ATQA, uint8_t *buffer_size);
StatusCode PICC_Select(Uid *uid, uint8_t valid_bits);
StatusCode PICC_ReadCardSerial(Uid *uid);
bool PICC_IsNewCardPresent();
#endif