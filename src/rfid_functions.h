// Register values provided by MFRC522 9.2 

#define READ				(1<<7)
#define	WRITE				0

#define COMMANDREG 			0x01
#define COMLENREG			0x02
#define DIVLENREG			0x03
#define COMIRQREG			0x04
#define DIVIRQREG			0x05
#define ERRORREG			0x06
#define STATUS1REG			0x07
#define STATUS2REG			0x08
#define FIFODATAREG			0x09
#define	FIFOLEVELREG		0x0A
#define WATERLEVELREG		0x0B
#define CONTROLREG			0x0C
#define	BITFRAMINGREG		0x0D
#define COLLREG				0x0E
#define	MODEREG				0x11
#define	TXMODEREG			0x12
#define	RXMODEREG			0x13
#define	TXCONTROLREG		0x14
#define	TXASKREG			0x15
#define TXSELREG			0x16
#define	RXSELREG			0x17
#define RXTHRESHOLDREG		0x18
#define DEMODREG			0x19
#define MFTXREG				0x1C
#define	MFRXREG				0x1D
#define	SERIALSPEEDREG		0x1F
#define	CRCRESULTREG_H		0x21
#define	CRCRESULTREG_L		0x22
#define	MODWIDTHREG			0X24
#define	RFCFGREG			0X26
#define	GSNREG				0x27
#define	CWGSPREG			0x28
#define	MODGSPREG			0x29
#define	TMODEREG			0x2A
#define	TPRESCALERREG		0x2B
#define	TRELOADREG_H		0x2C
#define	TRELOADREG_L		0x2D
#define	TCOUNTERVALREG_H	0x2E
#define	TCOUNTERVALREG_L	0x2F
#define	TESTSEL1REG			0x31
#define	TESTSEL2REG			0x32
#define	TESTPINENREG		0x33
#define	TESTPINVALUEREG		0x34
#define	TESTBUSREG			0x35
#define	AUTOTESTREG			0x36
#define	VERSIONREG			0x37
#define	ANALOGTESTREG		0x38
#define	TESTDAC1REG			0x39
#define	TESTDAC2REG			0x3A
#define	TESTADCREG			0x3B

// Command values given by MFRC522 10.3

#define IDLE					0x00	
#define MEM						0x01
#define GENERATE_RANDOM_ID		0X02
#define	CALCCRC					0X03
#define	TRANSMIT				0X04
#define	NOCMDCHANGE				0X07
#define	RECEIVE					0X08
#define	SELF_TEST				0x09
#define	TRANSCEIVE				0X0C
#define	MFAUTHENT				0X0E
#define	SOFT_RESET				0X0F

