/*
 * bootloader_command_app.c
 *
 *  Created on: 10 Oca 2021
 *      Author: mfati
 */

#include "bootloader_command_app.h"
#include "main.h"

extern uint8_t supported_commands[];
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
void printMessage(char *format, ...);

extern CRC_HandleTypeDef hcrc;
const uint32_t go_to_address = 0;
void bootloader_get_ver_cmd(uint8_t *bl_rx_data) {
	uint8_t bl_Version = 0;

	printMessage("Bootloaer_Get_Ver_Cmd");
	printMessage(" BL_VER :%#x,%#x  ", bl_rx_data[1], bl_rx_data[2]);

	uint32_t command_packet_length = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) ((uint32_t*) bl_rx_data
			+ command_packet_length - 4));

	// crc control
	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_length - 4, host_crc)) {
		printMessage("Checksum success");
		bootloader_send_ack(1);
		bl_Version = bootloader_get_version();
		printMessage(" BL_VER : %d %#x  ", bl_Version, bl_Version);
		bootloader_uart_write_data(&bl_Version, 1);
	} else {
		printMessage("Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_get_help_cmd(uint8_t *bl_rx_data) {
	printMessage("bootloader_get_help_cmd");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage("Checksum success");
		bootloader_send_ack(strlen(supported_commands));
		bootloader_uart_write_data(supported_commands,
				strlen(supported_commands));
		for (int i = 0; i < strlen(supported_commands); i++) {
			printMessage("%#x ", supported_commands[i]);
		}
	} else {
		printMessage("Checksum fail");
		bootloader_send_nack();
	}
}

void bootloader_get_cid_cmd(uint8_t *bl_rx_data) {
	uint16_t cID = 0;

	printMessage("bootloader_get_cid_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage("Checksum succes ");
		bootloader_send_ack(2);
		cID = get_mcu_chip_id();
		printMessage("Chip Id: %#x ", cID, cID);
		bootloader_uart_write_data((uint8_t*) &cID, 2);
	} else {
		printMessage("Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_go_to_addr_cmd(uint8_t *bl_rx_data) {
	uint32_t go_to_address = 0;
	uint8_t addr_valid = ADDR_VALID;
	uint8_t addr_invalid = ADDR_INVALID;

	printMessage("bootlodaer_go_to_addr_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data[0] + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage("Checksum succes ");
		bootloader_send_ack(1);

		//go_to_address = ((uint32_t*)((uint32_t*)&bl_rx_data[2]));

		go_to_address |= ((bl_rx_data[2]) & 0xFFFFFFFF);
		go_to_address |= ((bl_rx_data[3]) & 0xFFFFFFFF) << 8;
		go_to_address |= ((bl_rx_data[4]) & 0xFFFFFFFF) << 16;
		go_to_address |= ((bl_rx_data[5]) & 0xFFFFFFFF) << 24;
		printMessage("GO Addr: %#x ", go_to_address);

		if (bootloader_verify_address(go_to_address) == ADDR_VALID) {
			bootloader_uart_write_data(&addr_valid, 1);
			printMessage("Going to Address ");
			SCB->VTOR = go_to_address;
			//__set_MSP(mspValue);	// Bu fonksiyon F407 De calisiyordu ama
			//L053 de çalışmıyor
			SysTick->CTRL = 0;

			SysTick->LOAD = 0;

			SysTick->VAL = 0;
			HAL_I2C_DeInit(&hi2c1);
			HAL_UART_MspDeInit(&huart2);
			HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);

			HAL_CRC_DeInit(&hcrc);
			HAL_RCC_DeInit();

			HAL_DeInit();
			 		// T Bit = 1
			uint32_t jump_address = *((volatile uint32_t*) (go_to_address + 4));
				void (*jump_to_app)(void) = (void *)jump_address;
				jump_to_app();
		} else {
			printMessage("Go Address Invalid ");
			bootloader_uart_write_data(&addr_invalid, 1);
		}
	} else {
		printMessage("Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_flash_erase_cmd(uint8_t *bl_rx_data) {
	uint8_t eraseStatus = 0;

	printMessage("bootloader_flash_erase_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc)) {
		printMessage("Checksum success ");
		bootloader_send_ack(1);
		printMessage("Initial Sector: %d Nubmer Of Secotrs: %d ", bl_rx_data[2],
				bl_rx_data[3]);

		eraseStatus = execute_flash_erase(bl_rx_data[2], bl_rx_data[3]);

		printMessage(" Flash Erase Status : %d ", eraseStatus);
		bootloader_uart_write_data(&eraseStatus, 1);
	}
	else {
		printMessage(" Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_mem_write_cmd(uint8_t *bl_rx_data) {
	uint8_t addrValid = ADDR_VALID;
	uint8_t writeStatus = 0x00;
	uint8_t checkSum = 0;
	uint8_t length = 0;

	length = bl_rx_data[0];

	uint8_t payloadLength = bl_rx_data[6];

	uint32_t memAddress = *((uint32_t*) (&bl_rx_data[2]));

	checkSum = bl_rx_data[length];

	printMessage(" bootloader_mem_write_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc)) {
		printMessage(" Checksum success ");
		bootloader_send_ack(1);

		printMessage(" Memory Write Address: %#x ", memAddress);

		if (bootloader_verify_address(memAddress) == ADDR_VALID) {
			printMessage(" Valid Memory Write Address ");

			writeStatus = execute_memory_write(&bl_rx_data[7], memAddress, payloadLength);

			bootloader_uart_write_data(&writeStatus, 1);
		} else {
			printMessage(" Invalid Memory Write Address ");
			writeStatus = ADDR_INVALID;
			bootloader_uart_write_data(&writeStatus, 1);
		}
	} else {
		printMessage(" Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_enable_read_write_protect_cmd(uint8_t *bl_rx_data) {
	uint8_t status = 0;

	printMessage(" bootloader_enable_read_write_protect_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) ((uint32_t*)bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage(" Checksum success ");
		bootloader_send_ack(1);

		status = configure_flash_sector_r_w_protection(bl_rx_data[2],
				bl_rx_data[3], 0);

		printMessage(" Status: %d", status);

		bootloader_uart_write_data(&status, 1);
	} else {
		printMessage(" Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_read_sector_protection_status_cmd(uint8_t *bl_rx_data) {
	uint16_t status = 0;

	printMessage(" bootloader_read_sector_protection_status_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage(" Checksum success ");
		bootloader_send_ack(1);

		status = read_OB_r_w_protection_status();

		printMessage(" nWRP status: %#", status);
		bootloader_uart_write_data((uint8_t*) &status, 2);
	} else {
		printMessage(" Checksum fail ");
		bootloader_send_nack();
	}

}

void bootloader_disable_read_write_protect_cmd(uint8_t *bl_rx_data) {
	uint8_t status = 0;

	printMessage(" bootloader_disable_read_write_protect_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage(" Checksum success ");
		bootloader_send_ack(1);

		status = configure_flash_sector_r_w_protection(0, 0, 1);

		printMessage(" Status: %d", status);

		bootloader_uart_write_data(&status, 1);
	} else {
		printMessage(" Checksum fail ");
		bootloader_send_nack();
	}
}

void bootloader_uart_write_data(uint8_t *Buffer, uint32_t len) {
	HAL_UART_Transmit(&huart2, Buffer, len, HAL_MAX_DELAY);
}

uint8_t bootloader_verify_crc(uint8_t *Buffer, uint32_t len, uint32_t crcHost) {
	uint32_t crcValue = 0xFF;
	uint32_t data = 0;

	for (uint32_t i = 0; i < len; i++) {
		data = Buffer[i];
		crcValue = HAL_CRC_Accumulate(&hcrc, &data, 1);
	}

	__HAL_CRC_DR_RESET(&hcrc);

	if (crcValue == crcHost) {
		return CRC_SUCCESS;
	}

	return CRC_SUCCESS;
}

void bootloader_get_rdp_cmd(uint8_t *bl_rx_data) {
	uint8_t rdpLevel = 0;

	printMessage(" bootloader_get_rdp_cmd ");

	uint32_t command_packet_len = bl_rx_data[0] + 1;

	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4,
			host_crc)) {
		printMessage(" Checksum succes ");
		bootloader_send_ack(1);
		rdpLevel = get_flash_rdp_level();
		printMessage(" STM32F4 RDP Level: %d %#x ", rdpLevel, rdpLevel);
		bootloader_uart_write_data(&rdpLevel, 1);
	} else {
		printMessage(" Checksum fail ");
		bootloader_send_nack();
	}

}

void bootloader_send_ack(uint8_t followLength) {
	uint8_t ackBuffer[2];
	ackBuffer[0] = BL_ACK_VALUE;
	ackBuffer[1] = followLength;

	HAL_UART_Transmit(&huart2, ackBuffer, 2, HAL_MAX_DELAY);
}

void bootloader_send_nack() {
	uint8_t nackValue = BL_NACK_VALUE;
	HAL_UART_Transmit(&huart2, &nackValue, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_get_version(void) {
	return BL_VER;
}

uint16_t get_mcu_chip_id(void) {
	uint16_t cID;
	cID = (uint16_t) (DBG->IDCODE) & 0x0FFF;
	return cID;
}

uint8_t get_flash_rdp_level(void) {
	uint8_t rdp_level = 0;

#if	1

	volatile uint32_t *OB_Addr = (uint32_t*) 0x1FFFC000;
	rdp_level = (uint8_t) (*OB_Addr >> 8);

#else

	FLASH_OBProgramInitTypeDef OB_InitStruct;
	HAL_FLASHEx_OBGetConfig(&OB_InitStruct);
	rdp_level = (uint8_t) OB_InitStruct.RDPLevel;

#endif

	return rdp_level;
}

uint8_t bootloader_verify_address(uint32_t goAddress) {
	if (goAddress >= FLASH_BASE && goAddress <= G0_FLASH_END)
		return ADDR_VALID;

}

uint8_t execute_flash_erase(uint8_t sectorNumber, uint8_t numberOfSector) {
	FLASH_EraseInitTypeDef FlashEraseInitStruct = { 0 };
	uint32_t SectorError = 0;
	HAL_StatusTypeDef status = { 0 };

	if (numberOfSector > 11)
		return INVALID_SECTOR;

	if ((sectorNumber <= 11) || (sectorNumber == 0xFF)) {
		if (sectorNumber == 0xFF) {
			FlashEraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		} else {
			uint8_t remainingSector = 11 - sectorNumber;

			if (sectorNumber > remainingSector)
				sectorNumber = remainingSector;

			FlashEraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
			//FlashEraseInitStruct.Sector = sectorNumber;
			//FlashEraseInitStruct.NbSectors = numberOfSector;
		}
		//FlashEraseInitStruct.Banks = FLASH_BANK_1;

		HAL_FLASH_Unlock();
		//FlashEraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t) HAL_FLASHEx_Erase(&FlashEraseInitStruct,
				&SectorError);
		HAL_FLASH_Lock();

		return status;
	}

	return INVALID_SECTOR;
}

uint8_t execute_memory_write(uint8_t *Buffer, uint32_t memAddress, uint32_t len) {
	uint8_t status = HAL_OK;

	HAL_FLASH_Unlock();

	for (uint32_t i = 0; i < len; i++) {
		//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, memAddress+i, Buffer[i]);
	}

	HAL_FLASH_Lock();

	return status;
}

uint8_t configure_flash_sector_r_w_protection(uint8_t sector_details,
		uint8_t protection_mode, uint8_t enableOrDisable) {
	volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	// enableOrDisable == 0 -> en_w_r_protect | enableOrDisable == 1 -> dis_r_w_protect

	if (enableOrDisable) {
		HAL_FLASH_OB_Unlock();

		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
			;

		*pOPTCR |= (0xFF << 16);

		*pOPTCR |= (1 << 1);

		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
			;

		HAL_FLASH_OB_Lock();

		return 0;
	}

	if (protection_mode == 1)	// write protection
			{
		HAL_FLASH_OB_Unlock();

		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
			;

		*pOPTCR &= ~(sector_details << 16);

		*pOPTCR |= (1 << 1);

		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
			;

		HAL_FLASH_OB_Lock();
	} else if (protection_mode == 2) // read / write protection
			{
		HAL_FLASH_OB_Unlock();

		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
			;

		*pOPTCR &= ~(0xFF << 16);				// write protecton all sector
		*pOPTCR |= (sector_details << 16);

		*pOPTCR |= (0xFF << 8);					// read protection all sector

		*pOPTCR |= (1 << 1);

		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
			;

		HAL_FLASH_OB_Lock();
	}

	return 0;
}

uint16_t read_OB_r_w_protection_status() {
	FLASH_OBProgramInitTypeDef flashOBInitStruct;

	HAL_FLASH_OB_Unlock();

	HAL_FLASHEx_OBGetConfig(&flashOBInitStruct);

	HAL_FLASH_OB_Lock();

	//return flashOBInitStruct.WRPSector;
}
