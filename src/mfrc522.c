#include "mfrc522.h"
#include "time.h"

int fd;

void GPIO_init(int GPIOPin) {
	char gpio_str[5];
    sprintf(gpio_str,"%d",GPIOPin);
	_GPIO_SetFileValue(SYSFS_GPIO_PATH SYSFS_GPIO_UNEXPORT_FN, gpio_str);
	_GPIO_SetFileValue(SYSFS_GPIO_PATH SYSFS_GPIO_EXPORT_FN, gpio_str);
	return;
}

// sets GPIO to disired direction
int GPIO_SetDirection(int GPIOPin, char* direction) {
	char path_str[41];
    sprintf(path_str,"%s/gpio%d%s",SYSFS_GPIO_PATH,GPIOPin,SYSFS_GPIO_DIRECTION);
	return	_GPIO_SetFileValue(path_str,direction);
}

// sets GPIO pin to received value
int GPIO_SetValue(int GPIOPin, char* value) {
	char path_str[41];
    sprintf(path_str,"%s/gpio%d%s",SYSFS_GPIO_PATH,GPIOPin,SYSFS_GPIO_VALUE);
	return _GPIO_SetFileValue(path_str,value);
}

// Opens GPIO file in linux and sets value
int _GPIO_SetFileValue(char *fname, char *value) {
	int fd;
	fd = open(fname, O_WRONLY | O_NONBLOCK);
	if (fd < 0) {
		printf("Could not open file %s...%d\r\n",fname,fd);
		return -1;
	}
	write(fd,value,strlen(value));
    

    return 0;
}

// Initializes a linux fd with the spi tag
int _MFRC522_SPIInit(){
	
	uint32_t spi_speed = 1000000;
	int bpw = 8;
	int mode = SPI_MODE_0;


	if((fd = open(SPI_DEVICE, O_RDWR)) < 0) {
		printf("Failed to open SPI device\n");
		return fd;
	}

	if(ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		printf("Failed to write SPI mode\n");
		return -1;
	}

	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
		printf("Failed to write SPI max Speed\n");
		return -1;
	}

	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
		printf("Failed to write SPI BPW");
		return -1;
	}
	
	return 0;
	
}

uint8_t _MFRC522_ReadRegister(uint8_t reg){
	if(_MFRC522_SPIInit() < 0) {
		printf("failed to initializes SPI. \n");
		return 0;
	}
	uint8_t tx_buffer[2];
	uint8_t rx_buffer[2];
	struct spi_ioc_transfer spi;
	memset(&spi, 0, sizeof(spi));

	spi.tx_buf = (unsigned long) tx_buffer;
	spi.rx_buf = (unsigned long) rx_buffer;
	spi.bits_per_word = 8;
	spi.speed_hz = MFRC522_BIT_RATE;
	spi.delay_usecs = 0;
	spi.len = 2; 
	
	tx_buffer[0] = (READ | reg);
	
	if(ioctl(fd, SPI_IOC_MESSAGE(1), &spi) < 0) {
		printf("failed to read spi message : %s\n", strerror(errno));
		close(fd);
		return rx_buffer[1];
	}
	close(fd);
	return rx_buffer[1];

}

// Writes a value to MFRC522 register
void _MFRC522_WriteRegister(uint8_t reg, uint8_t command){
	if(_MFRC522_SPIInit() < 0) {
		printf("failed to initializes SPI. \n");
		return;
	}
	uint8_t buffer[2];
	struct spi_ioc_transfer spi;
	memset(&spi, 0, sizeof(spi));

	spi.tx_buf = (unsigned long) buffer;
	spi.rx_buf = (unsigned long) buffer;
	spi.bits_per_word = 8;
	spi.speed_hz = MFRC522_BIT_RATE;
	spi.delay_usecs = 0;
	spi.len = 2; 
	
	buffer[0] = (WRITE | reg);
	buffer[1] = command;

	if(ioctl(fd, SPI_IOC_MESSAGE(1), &spi) < 0) {
		printf("failed to send spi message : %s\n", strerror(errno));
		close(fd);
		return ;
	}
	close(fd);
	return;


}

void _MFRC522_SetRegisterBitMask(uint8_t reg, uint8_t mask){
	
	uint8_t val = _MFRC522_ReadRegister(reg);
	
	_MFRC522_WriteRegister(reg, (val | mask));
	
	return;
}

void _MFRC522_ClearRegisterBitMask( uint8_t reg, uint8_t mask) {
	uint8_t val = _MFRC522_ReadRegister(reg);
	
	_MFRC522_WriteRegister(reg, val & (~mask));

	return;
}


void _MFRC522_AntennaOn() {
	
	uint8_t value = _MFRC522_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		_MFRC522_WriteRegister( TxControlReg, value | 0x03);
	}

	return;
}


StatusCode _MFRC522_CalculateCRC(uint8_t *data, uint8_t data_length, uint8_t *result){
	
	_MFRC522_WriteRegister(CommandReg, PCD_Idle);
	_MFRC522_SetRegisterBitMask(FifoLevelReg, 0x80);
	
	for (int i = 0; i < data_length; i++) {
		_MFRC522_WriteRegister(FifoDataReg, data[i]);
	}

	_MFRC522_WriteRegister(CommandReg, PCD_CalcCRC);

	uint16_t i = 5000;
	uint8_t n;

	while (1)
	{
		printf("here \n");
		n = _MFRC522_ReadRegister(DivIrqReg);

		if ( n & 0x04 ) {
			break;
		}
		if (--i == 0){
			return STATUS_TIMEOUT;
		}
	}

	_MFRC522_WriteRegister(CommandReg, PCD_Idle);

	result[0] = _MFRC522_ReadRegister(CrcResultRegL);
	result[1] = _MFRC522_ReadRegister(CrcResultRegH);
	return STATUS_OK;
	
}

// Perform a self test as covered on MFRC522 16.1
int MFRC522_Self_Test() {
		
	GPIO_init(RESET_PIN);

	GPIO_SetDirection(RESET_PIN, DIR_OUT);

	GPIO_SetValue(RESET_PIN, VALUE_LOW);
	sleep(0.05);


	_MFRC522_WriteRegister( (CommandReg & 0x7E ), PCD_SoftReset);

	printf("command register is %02x\n",_MFRC522_ReadRegister((CommandReg & 0x7E )));

	_MFRC522_WriteRegister( (FifoLevelReg & 0x7E), 0x80);

	for (int i = 0; i < 25; i++) {
		_MFRC522_WriteRegister((FifoDataReg & 0x7E), 0x00);
	}

	_MFRC522_WriteRegister( (CommandReg & 0x7E), PCD_Mem);

	_MFRC522_WriteRegister( (AutoTestReg & 0x7E), PCD_SelfTest);

	_MFRC522_WriteRegister( (FifoDataReg & 0x7E), 0x00);
	printf("fifo  is %02x\n",_MFRC522_ReadRegister((FifoLevelReg & 0x7E)));
	_MFRC522_ReadRegister((FifoDataReg & 0x7E));
	printf("fifo is %02x\n",_MFRC522_ReadRegister((FifoLevelReg & 0x7E)));

	_MFRC522_WriteRegister( (CommandReg & 0x7E), PCD_CalcCRC);

	uint8_t buf = 0;
	printf("buffer  is %02x\n",_MFRC522_ReadRegister((FifoLevelReg & 0x7E)));
	
	while(buf < 64){
		buf = _MFRC522_ReadRegister((FifoLevelReg & 0x7E));
		
	}
	
	uint8_t result[64];

	for (int i = 0; i < 64; i++) {
		result[i] = _MFRC522_ReadRegister((FifoDataReg & 0x7E));
	}

	for (int i = 0; i < 64; i++) {
		if (result[i] != SELF_TEST_BYTES[i]){
			
			return -1;
		}
	}
	
	return 1;
}

// starts 
void MFRC522_Init(){

	GPIO_init(RESET_PIN);

	GPIO_SetDirection(RESET_PIN, DIR_OUT);

	GPIO_SetValue(RESET_PIN, VALUE_LOW);
	
	sleep(1);

	GPIO_SetValue(RESET_PIN, VALUE_HIGH);

	sleep(0.05);

	_MFRC522_WriteRegister( CommandReg, PCD_SoftReset);

	_MFRC522_WriteRegister( TmodeReg, 0x80);

	_MFRC522_WriteRegister( TPrescalerReg, 0xA9);

	_MFRC522_WriteRegister( TReloadRegH, 0x03);

	_MFRC522_WriteRegister( TReloadRegL, 0xE8);

	_MFRC522_WriteRegister( TxAskReg, 0x40);

	_MFRC522_WriteRegister( ModeReg, 0x3D);

	_MFRC522_AntennaOn();

	
	return;

}

StatusCode _MFRC522_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *tx_data, uint8_t tx_length, uint8_t *rx_data, uint8_t *rx_length,  uint8_t *valid_bits, uint8_t rxAlign){

	uint8_t n, _validBits;
	unsigned int i;

	uint8_t txLastBits = valid_bits ? *valid_bits : 0;

	uint8_t bitFraming = ((rxAlign << 4) | txLastBits);


	_MFRC522_WriteRegister(CommandReg, PCD_Idle);
	
	_MFRC522_WriteRegister(ComIrqReg, 0x7F);

	for (int j = 0; j < tx_length; j++){
		_MFRC522_WriteRegister(FifoDataReg, tx_data[j]);
	}

	_MFRC522_WriteRegister(BitFramingReg, bitFraming);

	_MFRC522_WriteRegister(CommandReg, command);

	if (command == PCD_Transceive) {
		_MFRC522_SetRegisterBitMask(BitFramingReg, 0x80);
	}

	i = 2000;

	while (1){
		n = _MFRC522_ReadRegister(ComIrqReg);

		if (n & waitIRq) {
			break;
		}

		if (n & 0x01) {
			
			return STATUS_TIMEOUT;
		}

		if (--i == 0){
			
			return STATUS_TIMEOUT;
		}
	}

	uint8_t errorval = _MFRC522_ReadRegister(ErrorReg);

	if (errorval & 0x13) {
		
		printf("error\n");
		return STATUS_ERROR;
	}

	if (rx_data && rx_length) {
		n = _MFRC522_ReadRegister(FifoLevelReg);

		if ( n > *rx_length) {
			
			printf("room\n");
			printf("fifo_reg:%02x\n",n);
			printf("rx_length: %02x\n", *rx_length);
			return STATUS_NO_ROOM;
		}

		*rx_length = n;

		for (int j = 0; j < *rx_length; j++) {
			rx_data[j] = _MFRC522_ReadRegister(FifoDataReg);
		}

		_validBits = _MFRC522_ReadRegister(ControlReg) & 0x07;

		if (valid_bits) {
			*valid_bits = _validBits;
		}	
	}

	if (errorval & 0x08) {
		
		printf("collision\n");
		return STATUS_COLLISION;
	}
	
	
	return STATUS_OK;
}

StatusCode _MFRC522_TransceiveData(uint8_t *tx_data, uint8_t tx_length, uint8_t *rx_data, uint8_t *rx_length, uint8_t *valid_bits, uint8_t rxAlign) {
	uint8_t waitIRq = 0x30;
	return _MFRC522_CommunicateWithPICC(PCD_Transceive, waitIRq, tx_data, tx_length, rx_data, rx_length, valid_bits, rxAlign);
}

StatusCode PICC_REQA_or_WUPA(uint8_t command, uint8_t *buffer_ATQA, uint8_t *buffer_size){
	uint8_t valid_bits;
	StatusCode status;


	if (buffer_ATQA == NULL || *buffer_size < 2) {
		return STATUS_NO_ROOM;
	}

	_MFRC522_ClearRegisterBitMask(CollReg, 0x80);
	
	valid_bits = 7;

	

	status = _MFRC522_TransceiveData(&command, 1, buffer_ATQA, buffer_size, &valid_bits, 0);

	if (status != STATUS_OK){
		return status;
	}
	
	if (*buffer_size != 2 || valid_bits != 0) {
		return STATUS_ERROR;
	}

	return STATUS_OK;
}

StatusCode PICC_RequestA(uint8_t *buffer_ATQA, uint8_t *buffer_size) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, buffer_ATQA, buffer_size);
}

StatusCode PICC_ReadCardSerial(Uid *uid){
	return PICC_Select(uid, 0);
}

bool PICC_IsNewCardPresent(){
	uint8_t buffer_ATQA[2];
	uint8_t buffer_size = sizeof(buffer_ATQA);
	StatusCode result = PICC_RequestA(buffer_ATQA, &buffer_size);
	return (result == STATUS_OK || result == STATUS_COLLISION);
}

StatusCode PICC_Select(Uid *uid, uint8_t valid_bits) {

	uint8_t status;   //[2:0] uid_complete select_done use_cascade_tag

	uint8_t cascade_level = 1;
	
	StatusCode result;

	uint8_t count;

	uint8_t index;

	uint8_t uid_index;

	int8_t current_level_known_bits;

	uint8_t buffer[9];

	uint8_t buffer_used;

	uint8_t rx_align;

	uint8_t tx_last_bits;

	uint8_t *response_buffer;

	uint8_t response_length;


	if (valid_bits > 80) {
		printf("over 80? really???\n");
		return STATUS_INVALID;
	}


	while (!(status & (1 << 2))) {  // uid_complete check
		
		switch(cascade_level){
			case 1:
			buffer[0] = PICC_CMD_SEL_CL1;
			uid_index = 0;
			if(valid_bits && uid->size > 4) status |= (1 << 0); // use_cascade_tag
			break;

			case 2:
			buffer[0] = PICC_CMD_SEL_CL2;
			uid_index = 3;
			if(valid_bits && uid->size > 7) status |= (1 << 0); // use_cascade_tag
			break;

			case 3:
			buffer[0] = PICC_CMD_SEL_CL3;
			uid_index = 6;
			if(valid_bits && uid->size > 4) status &= ~(0x01); // clears use_cascade_tag
			break;

			default:
			return STATUS_INTERNAL_ERROR;
		}
	

	current_level_known_bits = valid_bits - (8 * uid_index);
	if(current_level_known_bits < 0) current_level_known_bits = 0;

	index = 2;

	if(status & 0x01) buffer[index++] = PICC_CMD_CT; // checks use_cascade_tag

	uint8_t words_to_copy = current_level_known_bits / 8 + (current_level_known_bits % 8 ? 1 : 0);

	if (words_to_copy) {
		uint8_t max_words = (status & 0x01) ? 3 : 4;

		if (words_to_copy > max_words) {
			words_to_copy = max_words;
		}

		for (count = 0; count < words_to_copy; count++) {
			buffer[index++] = uid->uidByte[uid_index + count];
		}
	}

	if(status & 0x01) current_level_known_bits +=8;

	

	while(!(status & 0x02)) {
		if (current_level_known_bits >= 32) {
			
			buffer[1] = 0x70;
			buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
			printf("CRC. \n");
			result = _MFRC522_CalculateCRC(buffer,7,&buffer[7]);

			if (result != STATUS_OK)  return result;
			tx_last_bits = 0;
			buffer_used = 9;
			response_buffer = &buffer[6];
			response_length = 3;
		} else {
			
			tx_last_bits = current_level_known_bits % 8;
			count = current_level_known_bits / 8;
			index = 2 + count;
			buffer[1] = (index << 4) + tx_last_bits;
			buffer_used = index + (tx_last_bits ? 1:0);
			response_buffer = &buffer[index];
			response_length = sizeof(buffer) - index;
		}

		rx_align = tx_last_bits;

		printf("BitFraming \n");
		_MFRC522_WriteRegister(BitFramingReg, (rx_align << 4) + tx_last_bits);
		printf("result \n");
		result = _MFRC522_TransceiveData(buffer, buffer_used, response_buffer, &response_length, &tx_last_bits, rx_align);

		if (result == STATUS_COLLISION) {
			printf("col reg \n");
			uint8_t value_of_col_reg = _MFRC522_ReadRegister(CollReg);
			
			if ( value_of_col_reg & 0x20)  return STATUS_COLLISION;

			uint8_t col_pos = value_of_col_reg & 0x1F;

			if (col_pos == 0) col_pos = 32;

			if (col_pos <= current_level_known_bits)  return STATUS_INTERNAL_ERROR;

			current_level_known_bits = col_pos;

			count = (current_level_known_bits-1) % 8;

			index = 1 + (current_level_known_bits / 8) + (count ? 1 : 0);

			buffer[index] |= (1 << count);
		}else if (result != STATUS_OK) {
			
			return result;
		}else{
			if(current_level_known_bits >= 32) {
				status |= (1<<1);
			} else {
				current_level_known_bits = 32;
			}
		}
	}
	index = (buffer[2] == PICC_CMD_CT) ? 3 : 2;
	words_to_copy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
	for(count = 0; count < words_to_copy; count++){
		uid->uidByte[uid_index + count] = buffer[index++];
	}

	if (response_length != 3 || tx_last_bits != 0)  return STATUS_ERROR;
	
	printf("CRC2 \n");
	result = _MFRC522_CalculateCRC(response_buffer, 1, &buffer[2]);


	if (result != STATUS_OK)  return result;

	if ((buffer[2] != response_buffer[1]) || (buffer[3] != response_buffer[2]))  return STATUS_CRC_WRONG;

	if (response_buffer[0] & 0x04)
	{
		cascade_level++;
	}else{
		status |= (1<<2);
		uid->sak = response_buffer[0];
	}
	
}
	uid->size = 3 * cascade_level + 1;
	
	return STATUS_OK;
	
}