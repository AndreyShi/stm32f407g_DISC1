/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
#include <stdio.h>
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  //retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);
  //FRESULT d = f_mount(&USERFatFS,(TCHAR const*)USERPath,1);
  //printf(" FRESULT: %d\n",d);
  uint32_t wbytes; /* File write counts */
  uint8_t wtext[] = "text to write logical disk"; /* File write buffer */
  uint8_t rtext[_MAX_SS];/* File read buffer */
        if(FATFS_LinkDriver(&USER_Driver, USERPath) == 0)
        {
            if(f_mount(&USERFatFS, (TCHAR const*)USERPath, 0) == FR_OK)
            {
              FRESULT d = f_mkfs((TCHAR const*)USERPath, FM_ANY, 0, rtext, sizeof(rtext));
              printf(" f_mkfs FRESULT: %d\n",d);
              d = f_open(&USERFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE);
              printf(" f_open FRESULT: %d\n",d);
                if(d == FR_OK)
                {
                    if(f_write(&USERFile, wtext, sizeof(wtext), (void *)&wbytes) == FR_OK)
                    {
                      f_close(&USERFile);
                    }
                }
            } 
        }

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
