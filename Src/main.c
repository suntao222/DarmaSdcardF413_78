/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define __DEBUG 1
#define SAVE_FILE_TIME 300 //seconds
#define PKT_LEN_CACHE 1024
#define PKT_LEN_DATA 60
#define PKT_LEN_PARA 14
#define MAX_PKT_SIZE 60
#define MAX_BUF_SIZE 1024
//rtc
#define BufferSize 30
#define ADDR_1338RTC_Write 0xD0
#define ADDR_1338RTC_Read 0xD1
uint8_t ReadTime[BufferSize];

FATFS fs;
FIL file;
FRESULT f_res;
UINT fnum;

HAL_SD_CardStateTypedef state;
HAL_SD_CardCIDTypeDef SD_CardCID;

uint8_t g_cache_buf[60];
uint16_t g_cache_len;
uint16_t g_pkt_len;
uint16_t g_ms_cnt;
uint8_t g_fopen_flag;
uint8_t g_data_ready;
uint8_t g_sync_ready;
uint8_t g_sync_buf[30];
uint8_t g_stop_sdcard;
uint32_t g_err_cnt;
uint8_t g_curr_year, g_curr_month, g_curr_date, g_curr_week, g_curr_hour, g_curr_minute, g_curr_second;
uint8_t g_prev_year, g_prev_month, g_prev_date, g_prev_week, g_prev_hour, g_prev_minute, g_prev_second;
uint8_t g_dev_id;
uint32_t g_cnt;
uint8_t g_reset_dma;
uint8_t g_have_data;
uint32_t g_wait_cnt;
uint32_t g_reset_cnt;
uint32_t g_sec_cnt;
uint8_t g_test_flag;
uint32_t g_time_seq;
uint16_t g_seq;
uint8_t g_sdcard_ok;
uint8_t g_parse_flag;
uint8_t g_umount_flag;
uint32_t gg_cnt;
uint16_t g_file_cnt;
uint8_t g_need_reboot;
uint8_t g_need_synctime;
uint8_t g_pin12_flag;
uint16_t g_pin12_cnt;
uint8_t g_system_reboot;
uint8_t g_prev_sdcard;
uint8_t g_curr_sdcard;
uint8_t g_start_mount;
uint8_t g_mount_cnt;
uint8_t g_sdcard_full;
uint8_t g_start_cache;
uint8_t g_on_bed;
uint8_t g_cache_overflow;
uint8_t g_need_close_file;
uint8_t g_dma_full;

RTC_DateTypeDef sdatestructure;
RTC_TimeTypeDef stimestructure;


//struct darmaCache
//{
//	unsigned char buf[MAX_BUF_SIZE+256];
//	unsigned int  data_len;
//};
//struct darmaCache g_cache;
#define FAT_LEN 1024
uint32_t FAT_BUF[FAT_LEN];

//stonemm
#define PKT_LENGTH	512
uint8_t g_pkt_buf[PKT_LENGTH];
uint8_t g_pkt_buf2[PKT_LENGTH];
#define CACHE_SIZE	600
typedef struct
{
    unsigned char buf[PKT_LENGTH];
    unsigned int	len;
    unsigned int	status; //0-no data, 1-have data
}tDATA_CACHE;
tDATA_CACHE s_cache[CACHE_SIZE];// l_cache[CACHE_SIZE];

unsigned int w_index=0;
unsigned int r_index=0;
unsigned int lw_index=0; //lost write
unsigned int lr_index=0; //lost read
unsigned int g_lost_pkt=0;
unsigned int g_lost_flag=0;
unsigned int crc_fail=0, crc_pass=0, cnt_error=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void printf_fatfs_error(FRESULT fresult);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if 1
void sendDataTo_device(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
	for(uint16_t i=0;i<len;i++)
	{
		while((huart->Instance->SR & UART_FLAG_TC)==0);//循环发送,直到发送完毕 
		huart->Instance->DR = buf[i];
	}
}

//LED灯控制代码
#define LED_NUM 3
#define RED_LED	0
#define GREEN_LED 1
#define BLUE_LED 2
#define NO_FLASH 99
#define _FLASH 1
#define _BRIGHT 2
#define LED_FLASH_FREQ 500 //闪烁的频率,500ms
extern GPIO_TypeDef* LED_PORT[];
extern uint16_t LED_PIN[];
uint8_t g_led_flash;
uint16_t g_led_freq;
void LedControl(int n, int status)
{
//	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	
	//先关掉所有LED灯,关掉所有闪烁
	for(uint8_t i=0; i<LED_NUM; i++)
	{
		HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], GPIO_PIN_SET);
	}
	g_led_flash = NO_FLASH;
	
	//如果是常亮
	if(status == _BRIGHT)
	{
		HAL_GPIO_WritePin(LED_PORT[n], LED_PIN[n], GPIO_PIN_RESET);
	}
	//如果是闪烁
	else if(status == _FLASH)
	{
		g_led_flash = n;
	}
}
void LedFlash(int n)
{
	HAL_GPIO_TogglePin(LED_PORT[n], LED_PIN[n]);
}

void SendStopPkt()
{
	uint8_t buf[1]={0xfe};
	sendDataTo_device(&huart1, buf, 1);
}

void SendStartPkt()
{
	//printf("send 0xff\r\n");
	uint8_t buf[1]={0xff};
	sendDataTo_device(&huart1, buf, 1);
}

void SystemReboot()
{
	HAL_NVIC_SystemReset();
}

static void printf_fatfs_error(FRESULT fresult)
{
	LedControl(RED_LED, _FLASH);
  switch(fresult)
  {
    case FR_OK:                   //(0)
			printf("FR_OK\r\n");
    break;
    case FR_DISK_ERR:             //(1)
			printf("FR_DISK_ERR\r\n");
    break;
    case FR_INT_ERR:              //(2)
			printf("FR_INT_ERR\r\n");
    break;
    case FR_NOT_READY:            //(3)
			printf("FR_NOT_READY\r\n");
    break;
    case FR_NO_FILE:              //(4)
			printf("FR_NO_FILE\r\n");
    break;
    case FR_NO_PATH:              //(5)
			printf("FR_NO_PATH\r\n");
    break;
    case FR_INVALID_NAME:         //(6)
			printf("FR_INVALID_NAME\r\n");
    break;
    case FR_DENIED:               //(7)
    case FR_EXIST:                //(8)
			printf("FR_EXIST\r\n");
    break;
    case FR_INVALID_OBJECT:       //(9)
			printf("FR_INVALID_OBJECT\r\n");
    break;
    case FR_WRITE_PROTECTED:      //(10)
			printf("FR_WRITE_PROTECTED\r\n");
    break;
    case FR_INVALID_DRIVE:        //(11)
			printf("FR_INVALID_DRIVE\r\n");
    break;
    case FR_NOT_ENABLED:          //(12)
			printf("FR_NOT_ENABLED\r\n");
    break;
    case FR_NO_FILESYSTEM:        //(13)
			printf("FR_NO_FILESYSTEM\r\n");
    break;
    case FR_MKFS_ABORTED:         //(14)
			printf("FR_MKFS_ABORTED\r\n");
    break;
    case FR_TIMEOUT:              //(15)
			printf("FR_TIMEOUT\r\n");
    break;
    case FR_LOCKED:               //(16)
			printf("FR_LOCKED\r\n");
    break;
    case FR_NOT_ENOUGH_CORE:      //(17)
			printf("FR_NOT_ENOUGH_CORE\r\n");
    break;
    case FR_TOO_MANY_OPEN_FILES:  //(18)
			printf("FR_TOO_MANY_OPEN_FILES\r\n");
    break;
    case FR_INVALID_PARAMETER:    // (19)
			printf("FR_INVALID_PARAMETER\r\n");
    break;
  }
}

int CheckFreeSpace()
{
	//printf("get free size\r\n");
	FATFS *pfs;
	DWORD fre_clust;
	uint64_t TOT_SIZE, FRE_SIZE;
	f_res = f_getfree("0:", &fre_clust, &pfs);
	if(f_res == FR_OK)
	{
		TOT_SIZE = (pfs->n_fatent - 2) * pfs->csize/2; //KB
		FRE_SIZE = fre_clust * pfs->csize/2; //KB
		printf("TOT_SIZE: %llu\r\n", TOT_SIZE);
		printf("FRE_SIZE: %llu\r\n", FRE_SIZE);
		if(FRE_SIZE < 100*1000)//100M
		//if(FRE_SIZE < 50732670)//100M
		{
			g_sdcard_full = 1;
			return 0;
		}
		else
			g_sdcard_full = 0;
	}
	
	return 1;
}

void ActiveLowGpio()
{
//	HAL_GPIO_WritePin(PA0_GPIO_Port, PA0_Pin, GPIO_PIN_RESET); //notify spo2 board send data
}

//获取系统RTC时间，不是硬件读，速度快
void GetCurrentTime()
{
	if(__DEBUG) printf("GetCurrentTime()\r\n");
	/* Get the RTC current Time ,must get time first*/
	HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
	
	g_curr_year = sdatestructure.Year;
	g_curr_month = sdatestructure.Month;
	g_curr_date = sdatestructure.Date;
	g_curr_week = sdatestructure.WeekDay;
	g_curr_hour = stimestructure.Hours;
	g_curr_minute = stimestructure.Minutes;
	g_curr_second = stimestructure.Seconds;

	if(__DEBUG) printf("HAL TIME:");
	/* Display date Format : yy/mm/dd */
	if(__DEBUG) printf("%02d/%02d/%02d ",2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date); 
	/* Display time Format : hh:mm:ss */
	if(__DEBUG) printf("%02d:%02d:%02d\r\n",stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
}

//获取硬件时间
void GetRtcTime()
{
	int rtc_retry;

	//HAL_I2C_Mem_Read(&hi2c1, ADDR_1338RTC_Read, 0, I2C_MEMADD_SIZE_8BIT, ReadTime, 8, 1000);
	rtc_retry = 0;
	while(HAL_I2C_Mem_Read(&hi2c1, ADDR_1338RTC_Read, 0, I2C_MEMADD_SIZE_8BIT, ReadTime, 8, 1000) != HAL_OK)
	{
		rtc_retry++;
		printf("read RTC error, retry %d\r\n", rtc_retry);
		if(rtc_retry > 4)
		{
			printf("read RTC retry > 4, use default time\r\n");
			return;
		}
		HAL_Delay(1000);
	}

	sdatestructure.Year = ReadTime[6];
	sdatestructure.Month = ReadTime[5];
	sdatestructure.Date = ReadTime[4];
	sdatestructure.WeekDay = ReadTime[3];
	stimestructure.Hours = ReadTime[2];
	stimestructure.Minutes = ReadTime[1];
	stimestructure.Seconds = ReadTime[0];

	//把时间设置到系统中
	HAL_RTC_SetDate(&hrtc, &sdatestructure, RTC_FORMAT_BCD);
	HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD);
	
	g_curr_year=(ReadTime[6]>>4)*10+(ReadTime[6]&0x0f);
	g_curr_month=(ReadTime[5]>>4)*10+(ReadTime[5]&0x0f);
	g_curr_date=(ReadTime[4]>>4)*10+(ReadTime[4]&0x0f);
	g_curr_week=(ReadTime[3]&0x07);
	g_curr_hour=(ReadTime[2]>>4)*10+(ReadTime[2]&0x0f);
	g_curr_minute=(ReadTime[1]>>4)*10+(ReadTime[1]&0x0f);
	g_curr_second=(ReadTime[0]>>4)*10+(ReadTime[0]&0x0f);
}

//直接获取硬件时间
void GetRtcTime2()
{
	uint8_t fail_cnt=0;
	
	if(__DEBUG) printf("GetRtcTime2()\r\n");
	
	while(1)
	{
		if(HAL_I2C_Mem_Read(&hi2c1, ADDR_1338RTC_Read, 0, I2C_MEMADD_SIZE_8BIT, ReadTime, 8, 1000) == HAL_OK)
		{
			if(__DEBUG) printf("read RTC time success!\r\n");
			g_curr_year=(ReadTime[6]>>4)*10+(ReadTime[6]&0x0f);
			g_curr_month=(ReadTime[5]>>4)*10+(ReadTime[5]&0x0f);
			g_curr_date=(ReadTime[4]>>4)*10+(ReadTime[4]&0x0f);
			g_curr_week=(ReadTime[3]&0x07);
			g_curr_hour=(ReadTime[2]>>4)*10+(ReadTime[2]&0x0f);
			g_curr_minute=(ReadTime[1]>>4)*10+(ReadTime[1]&0x0f);
			g_curr_second=(ReadTime[0]>>4)*10+(ReadTime[0]&0x0f);
			break;
		}
		else
		{
			fail_cnt++;
			if(__DEBUG) printf("read RTC time fail %d\r\n", fail_cnt);
			if(fail_cnt > 5)
			{
				if(__DEBUG) printf("RTC fail 5, set dft 2000\r\n");
				break;
			}
		}
	}
}


void DisplayTime()
{
	printf("RTC TIME: 20%02d/%02d/%02d %02d:%02d:%02d\r\n",g_curr_year,g_curr_month,g_curr_date,g_curr_hour,g_curr_minute,g_curr_second);
}

int CreateNewFile()
{
	int retry_cnt=0;
	char dir_id[10];
	char dir_date[20];
	char dir_hour[20];
	char file_path[100];
	uint8_t l_curr_year, l_curr_month, l_curr_date, l_curr_week, l_curr_hour, l_curr_minute, l_curr_second;
	DIR dp;

	l_curr_year = g_curr_year;
	l_curr_month = g_curr_month;
	l_curr_date = g_curr_date;
	l_curr_week = g_curr_week;
	l_curr_hour = g_curr_hour;
	l_curr_minute = g_curr_minute;
	l_curr_second = g_curr_second;
	//if(__DEBUG) 
	printf("CreateNewFile() 20%02d/%02d/%02d %02d:%02d:%02d\r\n", l_curr_year, l_curr_month, l_curr_date, l_curr_hour, l_curr_minute, l_curr_second);
	memset(dir_id, 0, sizeof(dir_id));
	memset(dir_date, 0, sizeof(dir_date));
	memset(dir_hour, 0, sizeof(dir_hour));
	
	//用设备ID创建第一层目录
	sprintf(dir_id, "%d", g_dev_id);
	f_res = f_opendir(&dp, dir_id);
	if(f_res != FR_OK)
	{
		printf("create folder %s\r\n", dir_id);
		f_mkdir(dir_id);
	}
	else
	{
		printf("folder %s exist!\r\n", dir_id);
	}
	f_closedir(&dp);
	
	//用年月日创建第二层目录
	sprintf(dir_date, "%s/20%02d%02d%02d", dir_id, l_curr_year, l_curr_month, l_curr_date);
	f_res = f_opendir(&dp, dir_date);
	if(f_res != FR_OK)
	{
		printf("create folder %s\r\n", dir_date);
		f_mkdir(dir_date);
	}
	else
	{
		printf("folder %s exist!\r\n", dir_date);
	}
	f_closedir(&dp);

	//用小时创建第三层目录
	sprintf(dir_hour, "%s/%02d", dir_date, l_curr_hour);
	f_res = f_opendir(&dp, dir_hour);
	if(f_res != FR_OK)
	{
		printf("create folder %s\r\n", dir_hour);
		f_mkdir(dir_hour);
	}
	else
	{
		printf("folder %s exist!\r\n", dir_hour);
	}
	f_closedir(&dp);


	//建立文件,格式raw-data-dev_id-yyyymmddhhmm
	retry_cnt = 0;
	sprintf(file_path, "%s/raw-data-%d-20%02d-%02d%02d-%02d%02d%02d.dat", dir_hour, g_dev_id, l_curr_year, l_curr_month, l_curr_date, l_curr_hour, l_curr_minute, l_curr_second);
	do
	{
		f_res = f_open(&file, file_path, FA_OPEN_ALWAYS | FA_WRITE );   //在文件夹下创建文件
		retry_cnt++;
		if(retry_cnt > 10)
		{
			printf("create file %s fail > 5\r\n", file_path);
			printf_fatfs_error(f_res);
			SystemReboot();
		}
		if(f_res != FR_OK)
			HAL_Delay(1);
//		else
//		{
//			if(__DEBUG) printf("create file %s ok\r\n", file_path);
//		}
	}while(f_res != FR_OK);
	g_file_cnt++;
	if(__DEBUG) printf("create file %s ok, g_file_cnt:%d\r\n", file_path, g_file_cnt);	

	g_fopen_flag = 1;

	return 0;
}

void SaveDataToFile()
{
	
	f_res = f_write(&file, s_cache[r_index].buf, PKT_LENGTH, &fnum);
	if(f_res != FR_OK)
	{
		g_err_cnt++;
		printf("write file error, %d\r\n", g_err_cnt);
		printf_fatfs_error(f_res);
		if(g_err_cnt > 1)
		SystemReboot();
	}
	else
	{
		//if(__DEBUG) printf("write %d bytes ok, %d\r\n\r\n", fnum, gg_cnt++);
		if(fnum != PKT_LENGTH)
		{
			printf("write %d bytes\r\n", fnum);
			g_err_cnt++;
			printf_fatfs_error(f_res);
			if(g_err_cnt > 1)
				SystemReboot();
		}
	}

}

//可以从DEBUG口从上位机发送命令下来设置时间,BCD格式
//命令格式： FD A6 LEN CMD DEV_ID PKT_SIZE YEAR MONTH DATE WEEK HOUR MINUTE SECOND 0XFF, 都是一个字节
//           FD A6 0E 03 63 3C 18 06 27 03 11 47 00 FF -> 2018-06-27 week 3 11:47:00
// command 03:  校准时间
// command 04:  卸载SD卡
// command 05:  格式化SD卡
void ParsePara()
{
	//参数包，包括dev_id, pkt_len, date/time
	
	//CMD 03, 设置时间
	if(g_sync_buf[0]==0xFD &&
		 g_sync_buf[1]==0xA6 &&
		 g_sync_buf[2]==PKT_LEN_PARA && //LENGTH
		 g_sync_buf[3]==0x03 && //CMD 03
		 g_sync_buf[13]==0xFF)
	{
		uint8_t sync_time[10];
		//g_dev_id = g_pkt_buf[4];
		//g_pkt_len = g_pkt_buf[5];
		sync_time[6] = sdatestructure.Year = g_sync_buf[6];
		sync_time[5] = sdatestructure.Month = g_sync_buf[7];
		sync_time[4] = sdatestructure.Date = g_sync_buf[8];
		sync_time[3] = sdatestructure.WeekDay = g_sync_buf[9];
		sync_time[2] = stimestructure.Hours = g_sync_buf[10];
		sync_time[1] = stimestructure.Minutes = g_sync_buf[11];
		sync_time[0] = stimestructure.Seconds = g_sync_buf[12];
		sync_time[7] = 0xb3;

		//把时间设置到外部RTC中保存
		if(HAL_I2C_Mem_Write(&hi2c1, ADDR_1338RTC_Write, 0, I2C_MEMADD_SIZE_8BIT, sync_time, 8, 0x10) != HAL_OK)
		{
			printf("save RTC time fail\r\n");
		}
		else
		{
			printf("save RTC time ok\r\n");
		}

		//把时间设置到系统中
		HAL_RTC_SetDate(&hrtc, &sdatestructure, RTC_FORMAT_BCD);
		HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD);
		memset(g_sync_buf, 0, sizeof(g_sync_buf));
		
		//检验RTC时间是否写入正确
		printf("check the date/time\r\n");
		GetRtcTime2();
		DisplayTime();
	}
	//else
	//	printf("时间格式不正确\r\n");

	// CMD : 04, 卸载SD卡
	if(g_sync_buf[0]==0xFD &&
		 g_sync_buf[1]==0xA6 &&
		 g_sync_buf[2]==PKT_LEN_PARA && //LENGTH
		 g_sync_buf[3]==0x04 && //CMD 04
		 g_sync_buf[13]==0xFF)
	{
		printf("Unmount sdcard\r\n");
		//SendStopPkt(); //给1299发停止报文
		LedControl(BLUE_LED, _BRIGHT); //点亮蓝灯
		g_umount_flag = 1;
		g_sdcard_ok = 0;
	}
	
	// CMD : 05, 格式化SD卡
	if(g_sync_buf[0]==0xFD &&
		 g_sync_buf[1]==0xA6 &&
		 g_sync_buf[2]==PKT_LEN_PARA && //LENGTH
		 g_sync_buf[3]==0x05 && //CMD 05
		 g_sync_buf[13]==0xFF)
	{
		printf("》即将进行格式化...\r\n");
		//SendStopPkt(); //给1299发停止报文
		//LedControl(RED_LED, _BRIGHT); //点亮红灯
		//g_umount_flag = 1;
		//g_stop_sdcard = 1;
		f_res=f_mkfs("0:",0,0,FAT_BUF, FAT_LEN);	  //格式化 
		if(f_res == FR_OK)
		{
			printf("》串行FLASH已成功格式化文件系统。\r\n");		
			f_res = f_mount(NULL,"0:",1);    // 格式化后，先取消挂载 
			HAL_Delay(1000);
			SystemReboot();
		}
		else
		{
			printf("《《格式化失败。》》\r\n");
		}
	}
}

void UnMountSDcard()
{
	if(g_fopen_flag)
	{
		g_fopen_flag = 0;
		f_res = f_close(&file);
		if(f_res == FR_OK)
		{
			printf("f_close file success!\r\n");
		}
		else
		{
			printf("f_close file fail!\r\n");
		}
	}
	
	g_sdcard_ok = 0;
	f_res = f_mount(NULL,"0:",1);
	if(f_res == FR_OK)
	{
		printf("UnMount sdcard success!\r\n");
	}
	else
	{
		printf("UnMount sdcard fail!\r\n");
	}
	
	memset(s_cache, 0, sizeof(s_cache));
	r_index = 0;
	w_index = 0;
	
	LedControl(RED_LED, _BRIGHT); //点亮红灯
}

//挂载SD卡
uint8_t MountSDcard()
{
	uint8_t retry_mnt=0;

	state = HAL_SD_GetCardState(&hsd);
	//printf("state=%d\r\n", state);
	if(state == HAL_SD_CARD_TRANSFER)
	{
		HAL_SD_GetCardCID(&hsd, &SD_CardCID);
		printf("Initialize SD Card successfully!\r\n");
		printf("SD card infomation\r\n");
		printf("CardCapacity     : %llu\r\n", (QWORD)hsd.SdCard.BlockSize*hsd.SdCard.BlockNbr);
		printf("CardBlockSize    : %d\r\n", hsd.SdCard.BlockSize);
		printf("RCA              : %d\r\n", hsd.SdCard.RelCardAdd);
		printf("CardType         : %d\r\n", hsd.SdCard.CardType);
		printf("ManufacturerID   : %d\r\n", SD_CardCID.ManufacturerID);
	}
	else
	{
		printf("No SDCard !!!\r\n");
		//HAL_Delay(1000);
		//SystemReboot();
		return 0;
	}

	printf("》检测到SDCARD,尝试挂载\r\n");

	retry_mnt = 0;
	while(1)
	{
		//HAL_Delay(2000);
		g_start_mount = 1;
		f_res = f_mount(&fs,"0:",1); 					//挂载SD卡  
		//printf("f_res=%d\r\n", f_res);
		if(f_res == FR_NO_FILESYSTEM)
		{
			retry_mnt++;
			printf("》串行FLASH还没有文件系统，尝试 %d 次\r\n", retry_mnt);
			if(retry_mnt > 0)
			{
				retry_mnt = 0;
				printf("》即将进行格式化...\r\n");
				g_start_mount = 0;
			
				f_res=f_mkfs("0:",0,0, FAT_BUF, FAT_LEN);	  //格式化 
			
				if(f_res == FR_OK)
				{
					printf("》串行FLASH已成功格式化文件系统。\r\n");		
					f_res = f_mount(NULL,"0:",1);    // 格式化后，先取消挂载 
				}
				else
				{
					printf("《《格式化失败。》》\r\n");
				}
				SystemReboot();
			}
		}
		else if(f_res != FR_OK)
		{
			printf("！！串行FLASH挂载文件系统失败。(%d)\r\n",f_res);
			//HAL_Delay(3000);
			SystemReboot();
			//while(1);
		}
		else
		{
			printf("》文件系统挂载成功，可以进行读写测试  \r\n");
			g_start_mount = 0;
			g_mount_cnt = 0;
			//g_sdcard_ok = 1;
			return 1;
		}
	}

	return 0;
}


//判断UART1缓存中的数据是不是一帧合法数据
uint8_t ParseData(uint8_t *buf, uint16_t len)
{
	uint8_t *p;
	uint16_t lenth;
	
	p = buf;
	for(uint16_t i=0; i<len-60; i++)
	{
		//stonemm
#if 1
		if(*p==0xfd && *(p+1)==0xa6 && *(p+2)!=0)
		{
			lenth = *(p+2);
			g_dev_id = *(p+lenth-7);
			break;
		}
		else
			p++;
#else
		//is git test
		if(*p==0xfd && *(p+1)==0xa6 && *(p+PKT_LENGTH-1)==*(p+PKT_LENGTH-3) && *(p+PKT_LENGTH-2)==*(p+PKT_LENGTH-4))
		{
			g_dev_id = *(p+PKT_LENGTH-7);
			break;
		}
		else
			p++;
#endif
	}
	
	if(g_dev_id!=0 && g_dev_id!=255)
	{
		//if(__DEBUG) 
		printf("g_dev_id=%d\r\n", g_dev_id);
		return 0;
	}
	else
	{
		printf("parse dev_id fail\r\n");
	}
	
	return 1;
}



//保存数据
void SaveData()
{
	if(g_fopen_flag)
	{
		if(g_sec_cnt > SAVE_FILE_TIME)
		{
			//HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			//ParseData(s_cache[r_index].buf, 600);
			g_sec_cnt = 0;
			uint8_t retry_cnt=0;
			do
			{
				f_res = f_close(&file);
				if(f_res != FR_OK)
				{
					printf_fatfs_error(f_res);
					retry_cnt++;
					if(retry_cnt > 10)
					{
						printf("f_close file error > 10\r\n");
						break;
					}
					HAL_Delay(1);
				}
				else
				{
					printf("f_close ok!\r\n");
				}
			}while(f_res != FR_OK);
			g_fopen_flag = 0;
		}
		else
			goto __save;
	}
	GetRtcTime2();
	if(CheckFreeSpace() == 1)
	{
		CreateNewFile();
	}
	else
	{
		//SendStopPkt();
		printf("sdcard full\r\n");
		LedControl(BLUE_LED, _FLASH);
		g_umount_flag = 1;
		return;
	}
__save:
	SaveDataToFile();
			//HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}

#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t ret;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

	//初始化定时器
	HAL_TIM_Base_Start_IT(&htim2); //1ms timer
	HAL_TIM_Base_Start_IT(&htim3); //1s timer
	
	//初始化串口接收
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)g_cache_buf, 1);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)g_sync_buf, PKT_LEN_PARA);

	printf("STM32F413RGT6 sdcard version v1.0.2 start\r\n");

	//读取RTC时间并设置到系统中
	GetRtcTime();
	DisplayTime();

	//LOW PB10
	HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, GPIO_PIN_RESET);

#if 0
	/*## Initilise SD card hardware #################################*/
	uint8_t mr = BSP_SD_Init();
	if(mr != 0)
	{
		printf("BSP_SD_Init fail!\r\n");
		while(1);
	}
	else
		printf("mr=%d\r\n", mr);

	/*## Mount file system of the Initilised SD card #################################*/
	FRESULT fr = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	if(fr != FR_OK){
			//turn off SD
	printf("...3\r\n");
			while(1);
	}
	printf("f_mount=%d\r\n", fr);
	/*## Open and create a text file #################################*/
	printf("...4\r\n");
	fr = f_open(&SDFile, "BoomLvL.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK){
	printf("...5\r\n");
			while(1);
	}
	printf("...6\r\n");
	char data[] = "Verion2_dma\n\0";
	UINT written;
	UINT len = (UINT)strlen(data);
	/*## Write data to the text file ################################*/
	fr = f_write(&SDFile, &data[0],len, &written);
	if (fr != FR_OK || written != len) {
	printf("...7\r\n");
			while(1);
	}
	printf("...8\r\n");
	/*## Close the open text file #################################*/
	fr = f_close(&SDFile);
	printf("f_close=%d\r\n", fr);
	while(1){};
#else
	uint8_t mr = BSP_SD_Init();
	if(mr == 0)
	{
		printf("SDCARD init ok\r\n");
	}
//	HAL_SD_Init(&hsd);
//	if(HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
//	{
//		printf("config 4bus fail\r\n");
//	}

//	//test
//	state = HAL_SD_GetCardState(&hsd);
//	printf("state=%d\r\n", state);
//	if(state == HAL_SD_CARD_TRANSFER)
//	{
//		HAL_SD_GetCardCID(&hsd, &SD_CardCID);
//		printf("Initialize SD Card successfully!\r\n");
//		printf("SD card infomation\r\n");
//		printf("CardCapacity     : %llu\r\n", (QWORD)hsd.SdCard.BlockSize*hsd.SdCard.BlockNbr);
//		printf("CardBlockSize    : %d\r\n", hsd.SdCard.BlockSize);
//		printf("RCA              : %d\r\n", hsd.SdCard.RelCardAdd);
//		printf("CardType         : %d\r\n", hsd.SdCard.CardType);
//		printf("ManufacturerID   : %d\r\n", SD_CardCID.ManufacturerID);
//	}
//	else
//	{
//		printf("no SDCard !!!\r\n");
//		//HAL_Delay(1000);
//		//SystemRebooet();
//	}
	
//	f_res = f_mount(&fs,"0:",1); 					//挂载SD卡  
//	printf("f_mount=%d\r\n", f_res);
//	f_res = f_open(&file, "1.txt", FA_OPEN_ALWAYS | FA_WRITE );
//	printf("f_open=%d\r\n", f_res);
//	f_close(&file);
//	f_res = f_mkdir("163");
//	printf("f_mkdir=%d\r\n", f_res);

//	while(1){};
#endif
		
//  if(FATFS_LinkDriver(&SD_Driver, "0:") == 0)
//  {
//		printf("start mount\r\n");
//    /*##-2- Register the file system object to the FatFs module ##############*/
//    if(f_mount(&fs, "0:", 1) != FR_OK)
//    {
//      /* FatFs Initialization Error */
//      //Error_Handler();
//			printf("error\r\n");
//    }
//    else
//    {
//			printf("ok\r\n");
//		}
//	}
//	else
//	{
//		printf("link error\r\n");
//	}



	//变量初始化
	g_pkt_len = 0;
	g_fopen_flag = 0;
	g_data_ready = 0;
	g_err_cnt = 0;
	g_cnt = 0;
	g_reset_dma = 0;
	r_index = 0;
	w_index = 0;
	lr_index = 0;
	lw_index = 0;
	g_lost_pkt = 0;
	g_lost_flag = 0;
	g_have_data = 0;
	g_wait_cnt = 0;
	g_sync_ready = 0;
	g_reset_cnt = 0;
	g_dev_id = 0;
	g_sec_cnt = 0;
	g_test_flag = 0;
	g_time_seq = 0;
	g_seq = 0;
	g_cache_len = 0;
	g_led_flash = NO_FLASH;
	g_led_freq = 0;
	g_parse_flag = 0;
	g_umount_flag = 0;
	g_stop_sdcard = 0;
	g_ms_cnt = 0;
	gg_cnt = 0;
	g_file_cnt = 0;
	g_need_reboot = 0;
	g_need_synctime = 0;
	g_pin12_flag = 0;
	g_pin12_cnt = 0;
	g_system_reboot = 0;
	g_start_mount = 0;
	g_mount_cnt = 0;
	g_sdcard_full = 0;
	g_start_cache = 0;
	g_on_bed = 0;
	g_cache_overflow = 0;
	g_need_close_file = 0;
	g_dma_full = 0;
	
	memset(s_cache, 0, sizeof(s_cache));
	memset(g_pkt_buf, 0, sizeof(g_pkt_buf));
	memset(g_sync_buf, 0, sizeof(g_sync_buf));

	//挂载SD卡
	g_sdcard_ok = 0;
	g_sdcard_ok = MountSDcard();
	if(g_sdcard_ok != 1)
	{
		LedControl(RED_LED, _BRIGHT);
	}
	else
	{
		LedControl(GREEN_LED, _BRIGHT);
		
		//检查SD卡剩余容量是否足够100M
		CheckFreeSpace();
		if(g_sdcard_full)
		{
			//SendStopPkt();
			printf("sdcard full\r\n");
			LedControl(BLUE_LED, _FLASH);
		}
	}


	//告诉1299可以发数据过来了
	//ActiveLowGpio();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(g_system_reboot)
		{
			//HAL_Delay(1000);
			SystemReboot();
		}
		
		//处理UART1 DMA中的数据
		if(0)//g_dma_full)
		{
			if(g_reset_dma==1 && g_sdcard_ok==1)
			{
				//if(__DEBUG) for(uint8_t i=0; i<60; i++){printf("%02x ", g_pkt_buf[i]);};printf("\r\n");
				g_data_ready = 1;
				g_reset_cnt = 0;

				if(s_cache[w_index].status == 0)
				{
					if(g_cache_overflow == 1)
					{
						g_cache_overflow = 0;
						printf("cache recovery, index=%d\r\n", r_index);
					}
					memcpy(s_cache[w_index].buf, g_pkt_buf, PKT_LENGTH);
					s_cache[w_index].status = 1;
					if(w_index < CACHE_SIZE-1)
						w_index++;
					else
						w_index = 0;
					
					//写已经追上读了, 告诉1299开始缓存, stonemm
					if(w_index == r_index)
					{
						printf("w=%d r=%d\r\n", w_index, r_index);
						g_start_cache = 1;
						HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, GPIO_PIN_RESET); //NOTIFY 1299 STOP SEND
					}
				}
				else //s_cache中的数据还没有处理完，新来的数据要丢掉了
				{
					if(g_cache_overflow == 0)
					{
						g_cache_overflow = 1;
						printf("cache overflow, index=%d\r\n", r_index);
					}
				}
			}
			
			g_dma_full = 0;
		}
		
		//假如需要关闭文件
		if(g_need_close_file)
		{
			g_need_close_file = 0;
			if(g_fopen_flag)
			{
				g_fopen_flag = 0;
				printf("f_close file\r\n");
				f_res = f_close(&file);
				if(f_res != FR_OK)
				{
					printf("f_close fail\r\n");
				}
				else
				{
					printf("f_close success\r\n");
				}
			}
		}
	
		//卸载SD卡
		if(g_umount_flag)// && r_index==w_index)
		{
			g_umount_flag = 0;
			if(g_fopen_flag)
			{
				g_fopen_flag = 0;
				printf("f_close file\r\n");
				f_res = f_close(&file);
				if(f_res != FR_OK)
				{
					printf("f_close fail\r\n");
				}
				else
				{
					printf("f_close success\r\n");
				}
			}
			f_res = f_mount(NULL,"0:",1);
			if(f_res != FR_OK)
			{
				printf("卸载失败!\r\n");
			}
			else
			{
				printf("卸载成功!\r\n");
				g_sdcard_ok = 0;
				HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, GPIO_PIN_RESET);
			}

			memset(s_cache, 0, sizeof(s_cache));
			r_index = 0;
			w_index = 0;
			
			//LedControl(BLUE_LED, _BRIGHT); //点亮红灯
			if(g_need_reboot)
			{
				printf("reboot!\r\n");
				SystemReboot();
			}
		}
		
		//UART2命令，设置时间等
		if(g_sync_ready)
		{
			g_sync_ready = 0;
			ParsePara();
		}

		if(s_cache[r_index].status==1 && g_sdcard_ok==1 && g_sdcard_full==0)
		{
			//printf("r_index=%d\r\n", r_index);
			if(g_led_flash == NO_FLASH)
			{
				g_led_flash = GREEN_LED;
				LedControl(g_led_flash, _FLASH);
			}
			//解析dev_id
			if(g_parse_flag == 0)
			{
//				g_parse_flag = 1;
//				g_dev_id = 168;
				ret = ParseData(s_cache[r_index].buf, PKT_LENGTH);
				if(ret == 0)
					g_parse_flag = 1;
			}

			SaveData();
			s_cache[r_index].status = 0;
			if(r_index < CACHE_SIZE-1)
				r_index++;
			else
				r_index = 0;
			//stonemm
			if(g_start_cache == 1)
			{
					g_start_cache = 0;
					HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, GPIO_PIN_SET); //NOTIFY 1299 START SEND
			}
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SDIO
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//1ms timer
	if (htim->Instance == htim2.Instance)
	{
		//LED灯控制
		if(g_led_flash != NO_FLASH)
		{
			g_led_freq++;
			if(g_led_freq > LED_FLASH_FREQ)
			{
				g_led_freq = 0;
				LedFlash(g_led_flash);
			}
		}
	}

	//1s timer
	if (htim->Instance == htim3.Instance)
	{
		if(g_start_mount)
		{
			printf("g_mount_cnt=%d\r\n", g_mount_cnt);
			g_mount_cnt++;
			if(g_mount_cnt > 2)
			{
				printf("挂载失败，系统重启\r\n");
				//SendStopPkt();
				SystemReboot();
			}
		}

		if(g_sdcard_ok && !g_sdcard_full)
		{
			//假如持续2秒都没有数据过来则卸载SDCARD
			//防止串口死掉重启
			if(g_data_ready)
			{
				g_sec_cnt++;
				g_reset_cnt++;
				if(g_reset_cnt>5)
				{
					g_cnt = 0;
					g_reset_cnt = 0;
					g_data_ready = 0;
					printf("6s no data received\r\n");
					g_need_close_file = 1;
					LedControl(GREEN_LED, _BRIGHT);
//					if(g_fopen_flag)
//					{
//						printf("fclose file...1\r\n");
//						g_sec_cnt = 0;
//						g_fopen_flag = 0;
//						f_res = f_close(&file);
//						printf("fclose file...2\r\n");
//						if(f_res == FR_OK)
//						{
//							printf("f_close file success!\r\n");
//						}
//						else
//						{
//							printf("f_close file fail!\r\n");
//						}
//						memset(s_cache, 0, sizeof(s_cache));
//						r_index = 0;
//						w_index = 0;
//					}
//					else
//					{
//						printf("g_fopen_flag=0\r\n");
//					}
				}
			}
			
			//RESET DMA
			g_cnt++;
			if(g_cnt>3 && g_reset_dma==0)
			{
				//stone++ g_test_flag = 1;
				g_reset_dma = 1;
				printf("RESET UART1 DMA, HIGH PB10, IO-0\r\n");
				g_pkt_len = PKT_LENGTH;
				//HAL_UART_DMAStop(&huart1);
#if 1 //stonemm
				//HAL_UART_Receive_DMA(&huart1, (uint8_t *)g_pkt_buf, PKT_LENGTH);
				HAL_UART_Multi_Receive_DMA(&huart1, (uint8_t *)g_pkt_buf, (uint8_t *)g_pkt_buf2, PKT_LENGTH);
#else
				//HAL_UART_Receive_DMA(&huart1, (uint8_t *)g_pkt_buf, 3);
				HAL_UART_Multi_Receive_DMA(&huart1, (uint8_t *)g_pkt_buf, (uint8_t *)g_pkt_buf2, 6);
#endif
				HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, GPIO_PIN_SET); //NOTIFY 1299 START SEND
			}
			
			//发送模拟心跳
			if(g_cnt > 10)
			{
				g_cnt = 0;
				//SendStartPkt();
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_15)
	{
		if(g_sdcard_ok == 1)
		{
			g_sdcard_ok = 0;
			printf("PA15 button, 卸载SD卡\r\n");
			LedControl(BLUE_LED, _BRIGHT); //卸载SD卡，点亮蓝灯
			g_umount_flag = 1;
		}
		else
		{
			//printf("PA15 button, SD卡已卸载\r\n");
		}
	}
	
	//sdcard detect pin
	if(GPIO_Pin == GPIO_PIN_12)
	{
		g_sdcard_ok = 1 - g_sdcard_ok;
		if(g_sdcard_ok == 0)
		{
			//卡被拔出
			printf("SDCard plug-out, sdcard=%d\r\n", g_sdcard_ok);
		}
		else
		{
			//检测到卡插入
			printf("SDCard insert, sdcard=%d, system reboot\r\n", g_sdcard_ok);
		}
		SystemReboot();
	}
}

/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void SaveToCache(int i)
{
	if(g_reset_dma==1 && g_sdcard_ok==1)
	{
		//if(__DEBUG) for(uint8_t i=0; i<60; i++){printf("%02x ", g_pkt_buf[i]);};printf("\r\n");
		g_data_ready = 1;
		g_reset_cnt = 0;

		if(s_cache[w_index].status == 0)
		{
			if(g_cache_overflow == 1)
			{
				g_cache_overflow = 0;
				printf("cache recovery, index=%d\r\n", r_index);
			}
			
			if(i == 0)
				memcpy(s_cache[w_index].buf, g_pkt_buf, PKT_LENGTH);
			else
				memcpy(s_cache[w_index].buf, g_pkt_buf2, PKT_LENGTH);
			
			s_cache[w_index].status = 1;
			if(w_index < CACHE_SIZE-1)
				w_index++;
			else
				w_index = 0;
			
			//写已经追上读了, 告诉1299开始缓存, stonemm
			if(w_index == r_index)
			{
				printf("w=%d r=%d\r\n", w_index, r_index);
				g_start_cache = 1;
				HAL_GPIO_WritePin(PB10_GPIO_Port, PB10_Pin, GPIO_PIN_RESET); //NOTIFY 1299 STOP SEND
			}
		}
		else //s_cache中的数据还没有处理完，新来的数据要丢掉了
		{
			if(g_cache_overflow == 0)
			{
				g_cache_overflow = 1;
				printf("cache overflow, index=%d\r\n", r_index);
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	int ret;
//	int port;
	//uint8_t data;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */

	if(huart == &huart1)
	{
#if 0 //stonemm
		//printf("cplt1 %02x %02x\r\n", g_pkt_buf[0], g_pkt_buf[1]);
		printf("cplt1 -> \r\n");
		//sendDataTo_device(&huart2, g_pkt_buf, 13);
#else
//		if(g_dma_full == 0)
//			g_dma_full = 1;
//		else
//			printf("warning: uart1 dma full, lose data\r\n");

		SaveToCache(0);

#endif
	}
	
	if(huart == &huart2)
	{
		//if(__DEBUG) printf("uart2\r\n");
		g_sync_ready = 1;
	}
}

void HAL_UART_MultiRxCpltCallback(UART_HandleTypeDef *huart)
{
//	int ret;
//	int port;
	//uint8_t data;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */

	if(huart == &huart1)
	{
#if 0 //stonemm
		//printf("cplt1 %02x %02x\r\n", g_pkt_buf[0], g_pkt_buf[1]);
		printf("cplt2 -> \r\n");
		//sendDataTo_device(&huart2, g_pkt_buf, 13);
//		if(g_dma_full == 0)
//			g_dma_full = 1;
//		else
//			printf("warning: uart1 dma full, lose data\r\n");
#else
		SaveToCache(1);
		
#endif
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
