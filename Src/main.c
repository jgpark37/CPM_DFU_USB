/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define FLASH_USER_START_ADDR		ADDR_FLASH_SECTOR_3   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     	ADDR_FLASH_SECTOR_11   /* End @ of user Flash area */
#define FLASH_ENDURANCE			(10000-100)

#define FLASH_PAGE_SIZE1         		((uint32_t)(1024*1))   /* FLASH Page Size (1024 byte) */
#define FLASH_PAGE_SIZE16         		((uint32_t)(1024*16))   /* FLASH Page Size (16384 byte) */
#define FLASH_PAGE_SIZE64         		((uint32_t)(1024*64))   /* FLASH Page Size (16384 byte) */
#define FLASH_PAGE_SIZE128        	((uint32_t)(1024*128))   /* FLASH Page Size (16384 byte) */
#define FLASH_PI_DATA_SIZE				64 //patient info data size
#define FLASH_PI_TOTAL_NUM				512

#define FLASH_SYSTEM_INFO_ADDR		(uint32_t)FLASH_USER_START_ADDR
#define FLASH_SYSTEM_INFO2_ADDR		(uint32_t)(FLASH_SYSTEM_INFO_ADDR+FLASH_PAGE_SIZE16)	//for bad block after 100,000 write
#define FLASH_PATIENT_DATA_ADDR		(uint32_t)(FLASH_SYSTEM_INFO2_ADDR+FLASH_PAGE_SIZE16)
#define FLASH_PATIENT_DATA2_ADDR		(uint32_t)(FLASH_PATIENT_DATA_ADDR+ \
										(FLASH_PI_DATA_SIZE*FLASH_PI_TOTAL_NUM)*FLASH_PAGE_SIZE16)	//for bad block

#define SIZE_OF_U32						4
#define APP_BLOCK_TRANSFER_SIZE		512
#define PRINTF_BUF  100
#define FLASH_SPACE_FOR_FW			1024*1024-(128*2*1024+16*2*1024)
//HX 	: hexar
//_KR 	: model name
//50 	: model version and region
//P		: type
#define FW_FILE_NAME					"\\HX_KR20P.BIN"
//
//support function
//#define SUPPORT_BD_V21


enum _LANG_TYPE {
	LT_ENG,				//pjg<>180330 change position to kor
	LT_KOR,
	LT_CHINA,
	LT_MAX
};

enum _OS_ERROR_CODE {
	//드라이버 IC
	EC_MDIC_UNDER_VOL,			//저전압
	EC_MDIC_HIGH_TEMP,			//고온
	EC_MDIC_HOLL,					//홀 센싱 에러
	EC_MDIC_SHORT_GND,			//그라운드로 단락
	EC_MDIC_SHORT_VCC,			//전원으로 단락
	EC_MDIC_MOTOR_SHORT,			//모터 단락
	EC_MDIC_LOW_CUR,				//저전류
	//모터관련
	EC_MOTOR_OVERLOAD	= 20,	//모터 과부하
	//LCD		
	EC_LCD_COMM			= 40,	//LCD와 통신 에러
	//HOME-IN		
	EC_HOMEIN				= 60,	//홈인 에러
	EC_OVER_RANGE,				//구동 중 제한 범위 벗어난 경우 
	//USB		
	EC_USB_DISK_FAIL		= 80,	//USB 인식 안됨
	EC_USB_OPEN,					//파일 열기 안됨
	EC_USB_WRITE,					//파일 쓰기 안됨
	EC_USB_READ,					//파일 읽기 안됨
	//EEPROM		
	EC_EEP_READ			= 90,	//읽기 안됨
	EC_EEP_WRITE,					//쓰기 안됨
	//Login
	EC_LOGIN_PWD_ERR,				//암호5회 틀림
	//etc
	EC_EMPTY_SYSINFO,				//시스템 파라미터 없음. (Calibration 실행 )
	EC_NO_FIRMWARE,				//펌웨어 없음(DFU)
	Error_MAX
};

enum _KEY_TYPE {
	KEY_NONE,
	KEY_SHORT,
	KEY_LONG,
	KEY_LONG_10S,
	KEY_MAX
};

/* Private variables ---------------------------------------------------------*/
char *pSndInfo[][LT_MAX] = {	
	//eng                               kor                         china   etc..
	{"sp wav/1E.wav\r",	 	"sp wav/1.wav\r",  	"sp wav/1c.wav\r"},		//angle
	{"sp wav/2E.wav\r",		 "sp wav/2.wav\r",  	"sp wav/2c.wav\r"},		//go home
	{"sp wav/3.wav\r",		 "sp wav/3.wav\r",  	"sp wav/3.wav\r"},		// go first button
	{"sp wav/4E.wav\r",		 "sp wav/4.wav\r",  	"sp wav/4c.wav\r"},		//run stop
	{"sp wav/5E.wav\r",		 "sp wav/5.wav\r",  	"sp wav/5c.wav\r"},		//run start
	{"sp wav/6E.wav\r",		 "sp wav/6.wav\r",  	"sp wav/6c.wav\r"},		//run pause
	{"sp wav/7E.wav\r",		 "sp wav/7.wav\r",  	"sp wav/7c.wav\r"},		//patient info
	{"sp wav/8.wav\r", 		 "sp wav/8.wav\r",  	"sp wav/8.wav\r"},		//left/right button
	{"sp wav/9E.wav\r",		 "sp wav/9.wav\r",  	"sp wav/9c.wav\r"},		//speed/time
	{"sp wav/10E.wav\r",		 "sp wav/10.wav\r", 	"sp wav/10c.wav\r"},		//init system
	{"sp wav/11.wav\r",		 "sp wav/11.wav\r", 	"sp wav/11.wav\r"},		//up/down button
	{"sp wav/12E.wav\r", 		 "sp wav/12.wav\r", 	"sp wav/12c.wav\r"},		//motor overload
	{"sp wav/13.wav\r", 		 "sp wav/13.wav\r", 	"sp wav/13.wav\r"},		//go to setup
	{"sp wav/14E.wav\r", 	 	 "sp wav/14.wav\r", 	"sp wav/14c.wav\r"},		//train complete
	{"sp wav/15E.wav\r\0", 	 "sp wav/15.wav\r\0", 	"sp wav/15c.wav\r\0"},		//system error
	{"sp wav/16E.wav\r\0", 	 "sp wav/16E.wav\r\0", 	"sp wav/16E.wav\r\0"},		//calibration complete
};	

FIL fp; //file handle
DIR fat_dir;				    /* Directory object */
FILINFO fno;			  /* File information object */
FATFS fatfs; //structure with file system information
FRESULT FATFS_Status;
uint8_t appBuffer[FLASH_PAGE_SIZE16+1];
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t SectorError;
char USBH_Path[4];  /* USBH logical drive path */
uint32_t keyPressNum = 0;
uint32_t keyReleaseNum = 0;
char bufchk[100];
uint8_t fFWOk;
uint8_t fDispBootScreen;
uint32_t bootTime;
uint8_t drawPos;

extern ApplicationTypeDef Appli_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void uart_putstring(char *pbuf)
{
	int i;

	for (i = 0; i < PRINTF_BUF; i++) {
		bufchk[i] = pbuf[i];
		if (pbuf[i] == 0) break;
                    //printf("%c",pbuf[i]); 
	}
    HAL_UART_Transmit(&huart1, (uint8_t *)bufchk, i, i*10);
}

uint8_t UI_DisplayErrorCode(uint16_t code)
{	
	char buf[18];
	uint8_t errcode;
	//int i;
																									 						 
	buf[0] = 'i';
	buf[1] = ' ';
	buf[2] = 'r';
	//buf[3] = '0';
	buf[4] = '.';
	buf[5] = 'p';
	buf[6] = 'n';
	buf[7] = 'g';
	buf[8] = ',';
	buf[9] = '2';
	buf[10] = '9';		   
	buf[11] = '0';
	buf[12] = ',';
	buf[13] = '1';
	buf[14] = '1';
	buf[15] = '3';
	buf[16] = '\r';
	buf[17] = 0;
	
	errcode = (uint8_t)(code&0x00ff);
	uart_putstring("i error.png,0,0\r");
	
	buf[3] = '0' + errcode/10;
	uart_putstring(buf); //1st 
	buf[3] = '0' + errcode%10;
	buf[9] = '3';	//x coord
	buf[10] = '0';
	buf[11] = '5';
	uart_putstring(buf); //2st 

	return ((uint8_t)(code>>8));
}

char *UI_GetSndInfo(uint8_t num)
{
	return pSndInfo[num][0];
}

void UI_AniProgress2(uint32_t pos)
{
    if (pos >= 10) return;
	switch (pos%10) {
	case 0:
		uart_putstring("i lsndb.bmp,102,177\r");
		uart_putstring("i rsndb.bmp,107,177\r");
		//APP_SendMessage(hParent, WM_PAINT, 0, (LPARAM)"i fbar.bmp,99,173\r"); 
		break;	
	case 1:
		uart_putstring("i rsndb.bmp,132,177\r");
		break;
	case 2:
		uart_putstring("i rsndb.bmp,157,177\r");
		break;
	case 3:
		uart_putstring("i rsndb.bmp,182,177\r");
		break;
	case 4:
		uart_putstring("i rsndb.bmp,207,177\r");
		break;	
	case 5:
		uart_putstring("i rsndb.bmp,232,177\r");
		break;
	case 6:
		uart_putstring("i rsndb.bmp,260,177\r");
		break;
	case 7:
		uart_putstring("i rsndb.bmp,288,177\r");
		break;
	case 8:
		uart_putstring("i rsndb.bmp,316,177\r");
		break;	
	case 9:
		uart_putstring("i rsndb.bmp,344,177\r");
		break;
	}
}

void UI_DisplayProgress(int value, int max)
{
	int _1st, _2nd, _3rd;
	int per;
	int temp;
	char buf[18];
    char lang;
    
    lang = 1;
		
	buf[0] = 'i';
	buf[1] = ' ';
	buf[2] = '0';
	//buf[3] = '0';
	buf[4] = '.';
	buf[5] = 'b';
	buf[6] = 'm';
	buf[7] = 'p';
	buf[8] = ',';
    if (lang == 0) {
        buf[9] = '2';
        buf[10] = '1';
        buf[11] = '4';
    }
    else {
         buf[9] = '2';
        buf[10] = '1';
        buf[11] = '4';
   }
	buf[12] = ',';
	buf[13] = '1';
	buf[14] = '4';
	buf[15] = '2';
	buf[16] = '\r';
    buf[17] = 0;

	per = (value*100)/max;
    if (drawPos == per) return;
    drawPos = per;
	//per += 1;
	_3rd= per/100;
	temp = per%100;
	_2nd = temp/10;
	_1st = temp%10;
	buf[3] = '0'+_1st;
	uart_putstring(buf);
	buf[3] = '0'+_2nd;
	buf[10] = '0';//'1';
	buf[11] = '0';//'2';
	if(_2nd > 0 || _3rd > 0)
		uart_putstring(buf);
	buf[3] = '0'+_3rd;
	buf[9] = '1';
	buf[10] = '8';//'9';
	buf[11] = '6';//'8';
	if(_3rd > 0)
		uart_putstring(buf);
	UI_AniProgress2(per/10);

}

static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;  
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;  
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;  
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;  
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;  
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;  
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;  
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_SECTOR_7;  
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_SECTOR_8;  
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_SECTOR_9;  
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_SECTOR_10;  
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
	{
		sector = FLASH_SECTOR_11;
	}

	return sector;
}

int CopyAppToUserMemory(FSIZE_t appSize)
{
	uint32_t i, j, k;
	FSIZE_t appTailSize, appBodySize;
	FSIZE_t appAddrPointer;
	UINT readBytes;
    uint32_t total_per;

	uart_putstring("i Saving.png,0,0\r");
	UI_DisplayProgress(0, 100);
	
	if (f_lseek(&fp, 0) != FR_OK) return 0; //Go to the fist position of file
	appTailSize = appSize % APP_BLOCK_TRANSFER_SIZE;
	appBodySize = appSize - appTailSize;
	appAddrPointer = 0;
    total_per = ((appSize / FLASH_PAGE_SIZE16) + 1) + appBodySize/APP_BLOCK_TRANSFER_SIZE + 1;
    k = 0;
	HAL_FLASH_Unlock();
	for(i = 0; i < ((appSize / FLASH_PAGE_SIZE16) + 1); i++) //Erase n + 1 pages for new application
	{
		//while(FLASH_GetStatus() != FLASH_COMPLETE);
		EraseInitStruct.Sector = GetSector(FLASH_USER_START_ADDR + i * FLASH_PAGE_SIZE16);
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{
			//uSysStatus.all |= FLASH_ERASE_FAIL;
			HAL_FLASH_Lock();
			return 0;
		}
		UI_DisplayProgress(k++, total_per);
	}
	
	for(i = 0; i < appBodySize; i += APP_BLOCK_TRANSFER_SIZE)
	{
		/*
		* For example, size of File1 = 1030 bytes
		* File1 = 2 * 512 bytes + 6 bytes
		* "body" = 2 * 512, "tail" = 6
		* Let's write "body" and "tail" to MCU FLASH byte after byte with 512-byte blocks
		*/
		if (f_read(&fp, appBuffer, APP_BLOCK_TRANSFER_SIZE, &readBytes)  != FR_OK)
			return 0; //Read 512 byte from file
		if (APP_BLOCK_TRANSFER_SIZE != readBytes) return 0;
		
        if (i == 0) 
          ;//appBuffer[8] ^= 0xe0; //decode
		for(j = 0; j < APP_BLOCK_TRANSFER_SIZE; j += SIZE_OF_U32) //write 512 byte to FLASH
		{
			//while(FLASH_GetStatus() != FLASH_COMPLETE);
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
				FLASH_USER_START_ADDR + i + j, 
				*((uint32_t *)&appBuffer[j])) != HAL_OK)
			{
				//uSysStatus.all |= FLASH_WRITE_FAIL;
				HAL_FLASH_Lock();
				return 0;
			}
		}
		appAddrPointer += APP_BLOCK_TRANSFER_SIZE; //pointer to current position in FLASH for write
		UI_DisplayProgress(k++, total_per);
	}
	
	if (f_read(&fp, appBuffer, appTailSize, &readBytes) != FR_OK)
		return 0; //Read "tail" that < 512 bytes from file
		if (appTailSize != readBytes) return 0;
	while((appTailSize % SIZE_OF_U32) != 0)     //if appTailSize MOD 4 != 0 (seems not possible, but still...)
	{
		appTailSize++;              //increase the tail to a multiple of 4
		appBuffer[appTailSize - 1] = 0xFF;  //and put 0xFF in this tail place
	}
	
	for(i = 0; i < appTailSize; i += SIZE_OF_U32) //write "tail" to FLASH
	{
		//while(FLASH_GetStatus() != FLASH_COMPLETE);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
			FLASH_USER_START_ADDR + appAddrPointer + i, 
			*((volatile uint32_t*)(appBuffer + i))) != HAL_OK)
		{
			//uSysStatus.all |= FLASH_WRITE_FAIL;
			HAL_FLASH_Lock();
			return 0;
		}
	}
	UI_DisplayProgress(total_per, total_per);

	return 1;
}

void GoToUserApp(void)
{
	uint32_t appJumpAddress;
	void (*GoToApp)(void);
	
	/* Test if user code is programmed starting from address "ApplicationAddress" */
	if (((*(uint32_t*)FLASH_USER_START_ADDR) & 0x2FFF0000 ) == 0x20000000)
	{ /* Jump to user application */
		appJumpAddress = *((volatile uint32_t*)(FLASH_USER_START_ADDR + 4));
		GoToApp = (void (*)(void))appJumpAddress;
		//SCB->VTOR = FLASH_USER_START_ADDR;
		__set_MSP(*((volatile uint32_t*) FLASH_USER_START_ADDR)); //stack pointer (to RAM) for USER app in this address
		GoToApp();
    	}
	else {
		if (bootTime < 200) HAL_Delay(200-bootTime);
		uart_putstring("\r");
		UI_DisplayErrorCode(EC_NO_FIRMWARE);
		uart_putstring(UI_GetSndInfo(14));
	}
}

uint32_t idlecnt = 0;;
uint32_t startcnt = 0;
uint32_t disconcnt = 0;
uint32_t readycnt = 0;
int CheckUSBMemory(void)
{
	//uint32_t i;
	
	//while (1)
	{
		//MX_USB_HOST_Process();
		if (Appli_state == APPLICATION_IDLE) {
			idlecnt++;
			return 0;
		}
		else if (Appli_state == APPLICATION_START) {
			startcnt++;
			return 0;
		}
		else if (Appli_state == APPLICATION_DISCONNECT) {
			disconcnt++;
			return 0;
		}
		else if (Appli_state == APPLICATION_READY) {
			readycnt++;
			return 1;
			//break;
		}
		//i++;
		//if (i == 3400) {
		//	uart_putstring("i load.bmp,0,0\r");
		//}
		//if (i < 3500) return 0;
		//HAL_Delay(1);
	}
	return 1;
}

int CheckPressedKey(uint32_t loop, uint8_t flag)
{
	uint32_t i;
	  
    HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);                                // LCD PWR LOW 
   	keyReleaseNum = 0;
	keyPressNum = 0;
	bootTime = 0;
	if (HAL_GPIO_ReadPin(STOP_SW_GPIO_Port, STOP_SW_Pin) == GPIO_PIN_RESET) {
		for (i = 0; i < loop; i++) {
			MX_USB_HOST_Process();
			CheckUSBMemory();
			if (HAL_GPIO_ReadPin(STOP_SW_GPIO_Port, STOP_SW_Pin) == GPIO_PIN_RESET) {
				keyPressNum++;
				keyReleaseNum = 0;
			}
			else {
				if (keyPressNum) keyPressNum--;
				keyReleaseNum++;
				if (keyReleaseNum > 4) {
                    break;
                }
			}
			HAL_Delay(8);
			if (i == 20 && flag) {
				uart_putstring("\r");
				uart_putstring("i boot.bmp,0,0\r");
				fDispBootScreen = 1;
			}
			bootTime++;
		}
	}  

	if (keyPressNum > 900) return KEY_LONG_10S;
	else if (keyPressNum > 200) return KEY_LONG;
	else if (keyPressNum > 2 && keyPressNum < 50) return KEY_SHORT;
	else {
        HAL_Delay(50);
        uart_putstring("\r");
        uart_putstring("i boot.bmp,0,0\r");
    }
	return KEY_NONE;
}

uint8_t FileCheckAndCopy(void)
{
	uint32_t i;
	UINT readBytes;
	FRESULT FILE_Status;
	FSIZE_t appSize;
    uint8_t tempbuf;
	
	FILE_Status = f_open(&fp, FW_FILE_NAME, FA_READ);
	if(FILE_Status == FR_OK)
	{
		appSize = f_size(&fp);
		if (appSize > 1000 && appSize < FLASH_SPACE_FOR_FW) {
			fFWOk = 1;
			for(i = 0; i < appSize; i++) //Byte-to-byte compare files in MSD_MEM and USER_MEM
			{
				f_read(&fp, &appBuffer, 1, &readBytes);
                tempbuf = *((volatile uint8_t*)(FLASH_USER_START_ADDR + i));
				if(tempbuf != appBuffer[0]) 
				//if(*((volatile uint8_t*)(FLASH_USER_START_ADDR + i)) != appBuffer[0]) 
				{
					//if byte of USER_MEM != byte of MSD_MEM
                    tempbuf = 1;
					break;
				}
				if ( i == 2) {  //stack lsb address
					if (appBuffer[0] != 0x00) fFWOk = 0;
				}
				else if ( i == 3) {	//stack msb address
					if (appBuffer[0] != 0x20) fFWOk = 0;
				}
				else if ( i == 7) {	//f/w msb address
					if (appBuffer[0] != 0x08) fFWOk = 0;
				}
			}
			if (i != appSize && fFWOk)//=> was done "break" instruction in for(;;) cycle => new firmware in MSD_FLASH
			{
				//uart_putstring("i dfum.bmp,0,0\r");
				//HAL_Delay(10);
                drawPos = 200;
				if (!CopyAppToUserMemory(appSize)) {
					UI_DisplayErrorCode(EC_USB_WRITE);
					uart_putstring(UI_GetSndInfo(14));
					HAL_Delay(2000);
				}
                tempbuf = 1;
			}
			else {
                tempbuf = 2;
				if (fFWOk != 1) {
					UI_DisplayErrorCode(EC_EMPTY_SYSINFO);
					uart_putstring(UI_GetSndInfo(14));
					HAL_Delay(2000);
				}
			}
		}
	
		FILE_Status = f_close(&fp);
		//PeriphDeInit();
	}
	else //if FILE_Status != FR_OK
	{
		if(FILE_Status == FR_NO_FILE)
		{
			//No file error
		}
		else //if FILE_Status != FR_NO_FILE
		{
			//Other error
		}
		UI_DisplayErrorCode(EC_USB_OPEN);
		uart_putstring(UI_GetSndInfo(14));
		HAL_Delay(2000);
        tempbuf = 0;
	}
    return tempbuf;
}

void FLASH_SetParam(void)
{
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
}

int IsPressedButton(uint32_t time)
{
    keyPressNum = 0;
    keyReleaseNum = 0;
    
    while (time > 0) 
    {
        if (HAL_GPIO_ReadPin(STOP_SW_GPIO_Port, STOP_SW_Pin) == GPIO_PIN_RESET) {
            keyPressNum++;
            keyReleaseNum = 0;
        }
        else {
            if (keyPressNum) keyPressNum--;
            keyReleaseNum++;
            if (keyReleaseNum > 4 && keyPressNum > 10 && keyPressNum < 200) {
                break;
            }
        }
 		HAL_Delay(10);
        time--;
   }
   
   if (keyPressNum > 999) {
     return KEY_LONG_10S;
   }
   else if (keyPressNum > 200) {
     return KEY_LONG;
   }
	else {
      if (keyPressNum > 10 && keyPressNum < 200) {
        return KEY_SHORT;
      }
    }
	
    return KEY_NONE;
}

void IsReleaseButton(uint32_t time)
{    
    while (time > 0) 
    {
        if (HAL_GPIO_ReadPin(STOP_SW_GPIO_Port, STOP_SW_Pin) == GPIO_PIN_RESET) {
        }
        else {
            keyReleaseNum++;
            if (keyReleaseNum > 10) {
                break;
            }
        }
 		HAL_Delay(10);
   }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);                                // LCD PWR LOW 
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  	//HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);                                // LCD PWR LOW 
  	fDispBootScreen = 0;
	FLASH_SetParam();
	if (!CheckPressedKey(500, 1)) {
		GoToUserApp();
		while(1);
	}
	if (CheckUSBMemory()) {
	  	uart_putstring("i dfum.bmp,0,0\r");
        uart_putstring("f Release PLAY Button,100,220\r");
        IsReleaseButton(1);
        uart_putstring("fc 255,255,255\r");       
        uart_putstring("f Release PLAY Button,100,220\r");
        uart_putstring("fc 0,0,0\r");       
        uart_putstring("f Press PLAY Button,100,220\r");
        if (IsPressedButton(6000) == KEY_SHORT) {
            FATFS_Status = f_mount(&fatfs, (TCHAR const*)USBH_Path, 0);
            if(FATFS_Status == FR_OK)
            {
                drawPos = FileCheckAndCopy();
                FATFS_Status = f_mount(NULL, "0", 1);
                if (drawPos == 1) {
                    uart_putstring("fc 76,76,76\r");       
                    uart_putstring("f Press PLAY Button,100,220\r");
                }
                else  if (drawPos == 2) {
                    uart_putstring("fc 255,255,255\r");
                    uart_putstring("f Press PLAY Button,100,220\r");
                }
                uart_putstring("fc 255,0,0\r");       
                uart_putstring("f Remove USB and turn off power ,5,220\r");
                //HAL_Delay(2000);
                //GoToUserApp();
                while(1);
           }
            else //FATFS_Status != FR_OK
            {
                //FatFS mount error
                UI_DisplayErrorCode(EC_USB_DISK_FAIL);
                uart_putstring(UI_GetSndInfo(14));
            }
        }
        else {
            GoToUserApp();
       }
	}
	else {
		//usb check error
		GoToUserApp();
	}
	//while(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    //MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_PWR_Pin */
  GPIO_InitStruct.Pin = LCD_PWR_Pin;
#ifdef SUPPORT_BD_V21
    GPIO_InitStruct.Pin |= GPIO_PIN_7;
#endif
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STOP_SW_Pin */
  GPIO_InitStruct.Pin = STOP_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(STOP_SW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
