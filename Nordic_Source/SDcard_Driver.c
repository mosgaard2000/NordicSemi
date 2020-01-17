#include "SDcard_Driver.h"
#include "RTC_Driver.h"
#include "UICR.h"
#include "Data_Handling.h"
#include "version.h"
#include "registers.h"
#include "boards.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"

//Includes for FatFS SPIM3
   #include "bsp.h"
   #include "ff.h"
   #include "diskio_blkdev.h"
   #include "nrf_block_dev_sdc.h"
   #include "nrf_log.h"
   #include "nrf_log_ctrl.h"
   #include "nrf_log_default_backends.h"


uint8_t Error_Code = 0;

static FIL file;

char FILE_NAME_3[37];

char BLE_NAME[12];
char DevID[12];

char Rec_Len[6];

uint8_t Upgrade_EOF = 0x00;

FATFS fs;
DIR dir;


/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Test", "C12", "1.00")
);


//--------------------------------------------------------
//Generates filename based on date/time from RTC
//--------------------------------------------------------
void Create_FileName(void)
{
  FRESULT fr;
  FILINFO fno;


   do
   {
      //99
//      char *str_date;
//      str_date = Get_String_Date();
      char str_date[7];
      Get_String_Date(str_date);

//      char *str_time;
//      str_time = Get_String_Time();
      char str_time[7];
      Get_String_Time(str_time);

      sprintf(FILE_NAME_3,"%s%c%s%c%s%s",DevID,'-',str_date,'-',str_time,".c3s");


      fr = f_stat(FILE_NAME_3, &fno);

      if(fr != FR_NO_FILE)
      {
         nrf_delay_ms(1000);
         UpdateFileRegisters();
      }
   }
   while(fr != FR_NO_FILE);
}
//--------------------------------------------------------

void Create_Linetest_FileName(void)
{
  FRESULT fr;
  FILINFO fno;

  sprintf(FILE_NAME_3,"%s","c3diag.c3s");
}
//--------------------------------------------------------

//--------------------------------------------------------
//Write data to the SD-card
//--------------------------------------------------------
void Write_Data_To_SDcard3(uint8_t *ptr, uint32_t Length)
{
   FRESULT ff_result;
   uint32_t bytes_written;

   ff_result = f_write(&file, ptr, Length, (UINT *) &bytes_written);

   if (ff_result != FR_OK)
   {
      //NRF_LOG_INFO("Write failed\r\n.");
   }
   else
   {
      //NRF_LOG_INFO("%d bytes written.", bytes_written);
   }

   return;
}
//--------------------------------------------------------





//--------------------------------------------------------
//Creates a connection to the SD-card
//--------------------------------------------------------
uint8_t Open_SDcard(void)
{
   FRESULT ff_result;

   f_chdir("/");

nrf_delay_ms(20);

   //ff_result = f_open(&file, FILE_NAME_3, FA_READ | FA_WRITE | FA_OPEN_APPEND);
   ff_result = f_open(&file, FILE_NAME_3, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);

   return ff_result;
}
//--------------------------------------------------------



//--------------------------------------------------------
//Closes the file on the SD-card
//--------------------------------------------------------
void Close_SDcard(void)
{
//   (void) f_close(&file);
   uint8_t res = f_close(&file);

   if(res != FR_OK)     //Try to close again if something goes wrong
   {
      res = f_close(&file);
   }
}
//--------------------------------------------------------



//--------------------------------------------------------
//Closes the file on the SD-card
//--------------------------------------------------------
void Sync_SDcard(void)
{
   f_sync(&file);
}
//--------------------------------------------------------


//--------------------------------------------------------
//Delete the file on the SD-card
//--------------------------------------------------------
void Delete_Recording_File(void)
{
   f_unlink(FILE_NAME_3);
}
//--------------------------------------------------------



//--------------------------------------------------------
//Initializes and connects the SD-card
//--------------------------------------------------------

//--------------------------------------------------------

/* Initializes SPIM3 to be used together with SDcard
** Should only be called once
*/
void Init_SDcard_HW(void)
{
   // Initialize FATFS disk I/O interface by providing the block device.
   static diskio_blkdev_t drives[] =
   {
      DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
   };

   diskio_blockdev_register(drives, ARRAY_SIZE(drives));
}
//--------------------------------------------------------

/* Initializes the FatFS filesystem
** Should be called everytime the USB has been activated
*/
uint8_t Init_SDcard_FatFS(void)
{
//   static FATFS fs;
//   static DIR dir;
   /*static*/ FILINFO fno;

   FRESULT ff_result;
   DSTATUS disk_state = STA_NOINIT;

   //If already initialized we need to uninitialize it again.
   disk_state = disk_uninitialize(0);

   //NRF_LOG_INFO("Initializing disk 0 (SDC)...");
   for (uint32_t retries = 3; retries && disk_state; --retries)
   {
      disk_state = disk_initialize(0);
   }
   if (disk_state)
   {
      Error_Code = 1;
      //NRF_LOG_INFO("Disk initialization failed.");
      return 1;
   }

   uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
   uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;


//   NRF_LOG_INFO("Mounting volume...");
   ff_result = f_mount(&fs, "", 1);
   if (ff_result)
   {
      Error_Code = 2;
      //NRF_LOG_INFO("Mount failed.");
      return 2;
   }

   //NRF_LOG_INFO("\r\n Listing directory: /");
   ff_result = f_opendir(&dir, "/");
   if (ff_result)
   {
      Error_Code = 3;
      //NRF_LOG_INFO("Directory listing failed!");
      return 3;
   }

   do
   {
      ff_result = f_readdir(&dir, &fno);
      if (ff_result != FR_OK)
      {
	 Error_Code = 4;
	 //NRF_LOG_INFO("Directory read failed.");
	 return 4;
      }

      if (fno.fname[0])
      {
	 if (fno.fattrib & AM_DIR)
         {
	    //NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
         }
         else
         {
	    //NRF_LOG_RAW_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
         }
      }
   }
   while (fno.fname[0]);


   return 0;
}
//--------------------------------------------------------



//--------------------------------------------------------
//Change the state of the control pins for the SD-card
//--------------------------------------------------------
void Config_SD_HW(uint8_t enable, uint8_t dir, uint8_t VDDM_enable, uint8_t v_select)
{


   if(v_select == 0)			  //1.2V
   {
      nrfx_gpiote_out_clear(SDC_VSEL1);
      nrfx_gpiote_out_clear(SDC_VSEL2);
      nrfx_gpiote_out_clear(SDC_VSEL3);
   }
   else if(v_select == 1)			  //1.5V
   {
      nrfx_gpiote_out_set(SDC_VSEL1);
      nrfx_gpiote_out_clear(SDC_VSEL2);
      nrfx_gpiote_out_clear(SDC_VSEL3);
   }
   else if(v_select == 2)			  //1.8V
   {
      nrfx_gpiote_out_clear(SDC_VSEL1);
      nrfx_gpiote_out_set(SDC_VSEL2);
      nrfx_gpiote_out_clear(SDC_VSEL3);
   }
   else if(v_select == 3)			  //2.1V
   {
      nrfx_gpiote_out_set(SDC_VSEL1);
      nrfx_gpiote_out_set(SDC_VSEL2);
      nrfx_gpiote_out_clear(SDC_VSEL3);
   }
   else if(v_select == 4)			  //2.5V
   {
      nrfx_gpiote_out_clear(SDC_VSEL1);
      nrfx_gpiote_out_clear(SDC_VSEL2);
      nrfx_gpiote_out_set(SDC_VSEL3);
   }
   else if(v_select == 5)			  //2.8V
   {
      nrfx_gpiote_out_set(SDC_VSEL1);
      nrfx_gpiote_out_clear(SDC_VSEL2);
      nrfx_gpiote_out_set(SDC_VSEL3);
   }
   else if(v_select == 6)			  //3.0V
   {
      nrfx_gpiote_out_clear(SDC_VSEL1);
      nrfx_gpiote_out_set(SDC_VSEL2);
      nrfx_gpiote_out_set(SDC_VSEL3);
   }
   else//else if(v_select == 7)			  //3.3V
   {
      nrfx_gpiote_out_set(SDC_VSEL1);
      nrfx_gpiote_out_set(SDC_VSEL2);
      nrfx_gpiote_out_set(SDC_VSEL3);
   }
nrf_delay_ms(20);

   if(VDDM_enable == 1)			  //< Enables VDDM to SD-card (1-Enabled).
      nrfx_gpiote_out_set(SDC_VDDM_ENABLE);
   else
      nrfx_gpiote_out_clear(SDC_VDDM_ENABLE);

nrf_delay_ms(20);

   if(enable == 1)			  //< Enable level converter from nRF to SD-card (1-Enable).
      nrfx_gpiote_out_set(SDC_ENABLE);
   else
      nrfx_gpiote_out_clear(SDC_ENABLE);
nrf_delay_ms(20);
   if(dir == 1)				  //< Controls if it is USB or nRF that talks with SD-card (1-USB, 0-nRF).
      nrfx_gpiote_out_set(SDC_DIR);
   else
      nrfx_gpiote_out_clear(SDC_DIR);
}
//--------------------------------------------------------


//--------------------------------------------------------
//Setup the control pins for the SD-card (disable all)
//--------------------------------------------------------
void Setup_SD_HW(void)
{
   ret_code_t err_code;

   nrfx_gpiote_out_config_t output = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
   output.init_state = 0;

   err_code = nrfx_gpiote_out_init(SDC_ENABLE, &output);
   APP_ERROR_CHECK(err_code);
   err_code = nrfx_gpiote_out_init(SDC_DIR, &output);
   APP_ERROR_CHECK(err_code);
   err_code = nrfx_gpiote_out_init(SDC_VDDM_ENABLE, &output);
   APP_ERROR_CHECK(err_code);
   err_code = nrfx_gpiote_out_init(SDC_VSEL1, &output);
   APP_ERROR_CHECK(err_code);
   err_code = nrfx_gpiote_out_init(SDC_VSEL2, &output);
   APP_ERROR_CHECK(err_code);
   err_code = nrfx_gpiote_out_init(SDC_VSEL3, &output);
   APP_ERROR_CHECK(err_code);

   nrfx_gpiote_out_clear(SDC_ENABLE);	     //Disable level-converter
   nrfx_gpiote_out_clear(SDC_DIR);	     //Set direction to SD-card to nRF
   nrfx_gpiote_out_clear(SDC_VDDM_ENABLE);   //Disable VDD_M
   nrfx_gpiote_out_clear(SDC_VSEL1);	     //
   nrfx_gpiote_out_clear(SDC_VSEL2);	     //
   nrfx_gpiote_out_clear(SDC_VSEL3);	     //
}
//--------------------------------------------------------

void Shutdown_SDcard(void)
{
   FRESULT ff_result;
   //Close_SDcard();

   ff_result = f_mount(0, "", 0);

   ff_result = disk_uninitialize(0);
}
//--------------------------------------------------------

uint8_t Update_RTC_From_File(void)
{
   FRESULT fr;
   FILINFO fno;
   uint8_t SD_Err;
   uint8_t RTC_Updated = 0x00;

   SD_Err = Init_SDcard_FatFS();     //Initializes the SD-card

   fr = f_stat("Time.set", &fno);

   switch (fr)
   {
      case FR_OK:
      {
	 uint8_t YEAR = ((fno.fdate >> 9) & 0x7F) - 20;
         uint8_t MONTH = ((fno.fdate >> 5) & 0x0F);
         uint8_t DAY = ((fno.fdate) & 0x1F);

         uint8_t HOUR = ((fno.ftime >> 11) & 0x1F);
         uint8_t MIN = ((fno.ftime >> 5) & 0x3F);
         uint8_t SEC = ((fno.ftime) & 0x1F)*2;

	 uint8_t HN = YEAR / 10;
	 uint8_t LN = YEAR % 10;
	 YEAR = ((HN<<4) & 0xF0) | (LN & 0x0F);

	 HN = MONTH / 10;
	 LN = MONTH % 10;
	 MONTH = ((HN<<4) & 0x10) | (LN & 0x0F);

	 HN = DAY / 10;
	 LN = DAY % 10;
	 DAY = ((HN<<4) & 0x30) | (LN & 0x0F);

	 HN = HOUR / 10;
	 LN = HOUR % 10;
	 HOUR = ((HN<<4) & 0x30) | (LN & 0x0F);

	 HN = MIN / 10;
	 LN = MIN % 10;
	 MIN = ((HN<<4) & 0x70) | (LN & 0x0F);

	 HN = SEC / 10;
	 LN = SEC % 10;
	 SEC = ((HN<<4) & 0x70) | (LN & 0x0F);

         SDcard_Update_RTC_time(YEAR, MONTH, DAY, HOUR, MIN, SEC);

	 fr = f_unlink("Time.set");    //Delete file

         if(fr == FR_OK)
	 {
	    fr = f_mount(0, "", 0);
	 }

         RTC_Updated = 0x01;

	 break;
      }

      case FR_NO_FILE:
	 //printf("It is not exist.\n");
      break;

      default:
	 //printf("An error occured. (%d)\n", fr);
      break;
   }

   return RTC_Updated;
}
//--------------------------------------------------------

uint8_t Check_Config_File(void)
{
   FRESULT fr;
   FILINFO fno;
   uint8_t SD_Err;
   uint8_t Config_Updated = 0x00;
   uint8_t Linetest = 0x00;
   uint8_t Bootloader = 0x00;

   //We should already be initialized
   //SD_Err = Init_SDcard_FatFS();     //Initializes the SD-card

/*   uint8_t openfile = Check_File_Open();

   if(openfile == 0x01)
      f_close(&file);
*/
   char FILE_NAME[] = "c3cfg.dat";
   //char *Buffer;
   char Buffer[256];

   SD_Err = f_chdir("/.info");

   if(SD_Err == FR_OK)
   {
      fr = f_stat("c3cfg.dat", &fno);

      if(fr == FR_OK)
      {
         uint8_t YEAR = ((fno.fdate >> 9) & 0x7F) - 20;
         uint8_t MONTH = ((fno.fdate >> 5) & 0x0F);
         uint8_t DAY = ((fno.fdate) & 0x1F);

         uint8_t HOUR = ((fno.ftime >> 11) & 0x1F);
         uint8_t MIN = ((fno.ftime >> 5) & 0x3F);
         uint8_t SEC = ((fno.ftime) & 0x1F)*2;

	 uint8_t HN = YEAR / 10;
	 uint8_t LN = YEAR % 10;
	 YEAR = ((HN<<4) & 0xF0) | (LN & 0x0F);

	 HN = MONTH / 10;
	 LN = MONTH % 10;
	 MONTH = ((HN<<4) & 0x10) | (LN & 0x0F);

	 HN = DAY / 10;
	 LN = DAY % 10;
	 DAY = ((HN<<4) & 0x30) | (LN & 0x0F);

	 HN = HOUR / 10;
	 LN = HOUR % 10;
	 HOUR = ((HN<<4) & 0x30) | (LN & 0x0F);

	 HN = MIN / 10;
	 LN = MIN % 10;
	 MIN = ((HN<<4) & 0x70) | (LN & 0x0F);

	 HN = SEC / 10;
	 LN = SEC % 10;
	 SEC = ((HN<<4) & 0x70) | (LN & 0x0F);

         SDcard_Update_RTC_time(YEAR, MONTH, DAY, HOUR, MIN, SEC);


         fr = f_open(&file, FILE_NAME, FA_READ | FA_OPEN_EXISTING);

         int8_t res;

         do
         {
            //Read values from config file
            f_gets(Buffer,100,&file);

            res = strncmp(Buffer, "DevID=", 6);

            if(res == 0)
            {
               int8_t len = strlen(Buffer);

               char tmp_str[12] = {0x00};

               for(uint8_t i=6; i<len-1; i++)
                  tmp_str[i-6] = Buffer[i];

               SDcard_Update_UICR_DevID(tmp_str);
            }
            else
            {
               res = strncmp(Buffer, "SetDate=", 8);

               if(res == 0)
               {
               }
               else
               {
                  res = strncmp(Buffer, "SetTime=", 8);

                  if(res == 0)
                  {
                  }
                  else
                  {
                     res = strncmp(Buffer, "Linetest=1", 10);

                     if(res == 0)
                     {
                        Linetest = 0x10;
                     }
                     else
                     {
                        res = strncmp(Buffer, "Bootloader=1", 12);

                        if(res == 0)
                        {
                           Bootloader = 0x20;
                        }
                     }
                  }
               }
            }
         }
         while(strlen(Buffer) > 1);


         Close_SDcard();
         nrf_delay_ms(20);

	 fr = f_unlink("c3cfg.dat");    //Delete file

         if(fr == FR_OK)
	 {
	    fr = f_mount(0, "", 0);
	 }

         Config_Updated = (Bootloader | Linetest | 0x01);
      }
   }

   return Config_Updated;
}
//--------------------------------------------------------

uint8_t Check_Bootloader_File(void)
{
   FRESULT fr;
   FILINFO fno;
   uint8_t SD_Err;
   uint8_t Enable_Bootloader = 0x00;

   char FILE_NAME[] = "bootloader.dat";

   SD_Err = f_chdir("/.info");

   if(SD_Err == FR_OK)
   {
      fr = f_stat("bootloader.dat", &fno);

      if(fr == FR_OK)
      {
	 fr = f_unlink("bootloader.dat");    //Delete file

         if(fr == FR_OK)
	 {
	    fr = f_mount(0, "", 0);
	 }

         Enable_Bootloader = 0x01;
      }
   }

   return Enable_Bootloader;
}
//--------------------------------------------------------

void Get_Drive_Label(void)
{
   //char str[12];

   /* Get volume label of the default drive */
   f_getlabel("", BLE_NAME, 0);
}
//--------------------------------------------------------

uint8_t Check_File_Open(void)
{
   uint8_t result;

   if(file.obj.fs != 0x00)
      result = 0x01;
   else
      result = 0x00;

   return result;
}
//--------------------------------------------------------

uint32_t Get_Free_Space(void)
{
   uint32_t result = 0;

   FATFS *fs2;
   DWORD fre_clust, fre_sect, tot_sect;


    /* Get volume information and free clusters of drive 1 */
    uint32_t res = f_getfree("/", &fre_clust, &fs2);
    //if (res) die(res);

    /* Get total sectors and free sectors */
    //tot_sect = (fs2->n_fatent - 2) * fs2->csize;
    fre_sect = fre_clust * fs2->csize;

   result = fre_sect / 2;

   return result;
}
//--------------------------------------------------------

uint8_t Open_Upgrade_Image(uint8_t initialized)
{
   FRESULT fr;
   FILINFO fno;
   uint8_t SD_Err;

   uint8_t File_Opened = false;

   if(initialized == false)
      SD_Err = Init_SDcard_FatFS();     //Initializes the SD-card

   char FILE_NAME[] = "Update.fil";
   char Buffer[256];


   fr = f_stat("Update.fil", &fno);

   if(fr == FR_OK)
   {
      fr = f_open(&file, FILE_NAME, FA_READ | FA_OPEN_EXISTING);

      if(fr == FR_OK)
      {
         Upgrade_EOF = 0x00;
         File_Opened = true;
      }
   }

   return File_Opened;
}
//--------------------------------------------------------

uint8_t Read_Upgrade_Image(void)
{
   int8_t res;

   uint8_t Buffer[1];
   UINT Bytes_read;

   //Read values from upgrade file
   res = f_read(&file,Buffer,1, &Bytes_read);

   if(Bytes_read == 0)
   {
      Upgrade_EOF = 0x01;
      Close_SDcard();
      nrf_delay_ms(20);
   }

   return Buffer[0];
}
//--------------------------------------------------------

uint8_t Check_Upgrade_EOF(void)
{
   return Upgrade_EOF;
}
//--------------------------------------------------------
