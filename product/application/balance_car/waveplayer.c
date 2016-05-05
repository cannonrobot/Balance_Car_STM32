/**
  ******************************************************************************
  * @file    Audio_playback_and_record/Src/waveplayer.c 
  * @author  MCD Application Team
  * @version V1.3.3
  * @date    29-January-2016
  * @brief   I2S Audio player program. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define AUDIO_BUFFER_SIZE             16384

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int first_flag_my=0;
WAVE_FormatTypeDef WaveFormat;

FILELIST_FileTypeDef FileList;

Music_List MusicList={0,0,0};//固定播放列表

/* LED State (Toggle or OFF)*/
__IO uint32_t LEDsState;

 __IO uint32_t RepeatState=REPEAT_OFF, PauseResumeStatus=IDLE_STATUS, PressCount=1;
 
 static int16_t FilePos = 0;

/* Audio Play Start variable. 
   Defined as external in main.c*/
__IO uint32_t AudioPlayStart = 0;

/* Audio wave data length to be played */
static uint32_t WaveDataLength = 0;

/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0,AudioElapseSize=0;

/* Ping-Pong buffer used for audio play */
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef BufferOffset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
 uint8_t Volume = 50;

/* Variable used to indicate audio mode (play, record or stop). */
/* Defined in main.c */
 __IO uint32_t CmdIndex=CMD_PLAY;

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

/* Variable used to switch play from audio sample available on USB to recorded file. */
/* Defined in waverecorder.c */
 uint32_t WaveRecStatus=0;

AUDIO_PLAYBACK_StateTypeDef AudioState;



/* Variable to indicate USB state (start/idle) */
/* Defined in main.c */
 MSC_ApplicationTypeDef AppliState=APPLICATION_IDLE;

static uint8_t  USBH_USR_ApplicationState = USBH_USR_FS_INIT;

FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
char USBDISKPath[4];          /* USB Host logical drive path */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Starts Audio streaming.    
  * @param  idx: File index
  * @retval Audio error
  */ 
AUDIO_ErrorTypeDef AUDIO_PLAYER_Start(uint8_t idx)
{
   char path[] = "0:/";
  	 UINT bytesread = 0;
	FileList.file_P[0]=(uint8_t *)"ok.wav";
	FileList.file_P[1]=(uint8_t *)"id.wav";
	FileList.file_P[2]=(uint8_t *)"id2.wav";
	FileList.file_P[3]=(uint8_t *)"temp.wav";
	FileList.file_P[4]=(uint8_t *)"xpg.wav";//Song1:
	FileList.file_P[5]=(uint8_t *)"dream.wav";//Song2:
	FileList.file_P[6]=(uint8_t *)"爱的华.wav";//Song3:
	FileList.file_P[7]=(uint8_t *)"gcw.wav";//Song4
	FileList.file_P[8]=(uint8_t *)"qmgw.wav";//Song4
	
	
	
	
  f_close(&FileRead);
  //if(AUDIO_GetWavObjectNumber() > idx)
  //{ 
  if(f_opendir(&Directory, path) == FR_OK)
  {
    /* Open the Wave file to be played */
		
  //  if(f_open(&FileRead, (char *)FileList.file[FileList.ptr].name , FA_READ) == FR_OK)
			  if(f_open(&FileRead, (char *)FileList.file_P[idx] , FA_READ) == FR_OK)
    {  
      /* Read sizeof(WaveFormat) from the selected file */
      f_read (&FileRead, &WaveFormat, sizeof(WaveFormat), &bytesread);
    }    
  }
    /*Adjust the Audio frequency */
    WavePlayerInit(WaveFormat.SampleRate);

   BufferOffset= BUFFER_OFFSET_NONE;
    
    /* Get Data from USB Flash Disk */
    f_lseek(&FileRead, 0);
    
    /* Fill whole buffer at first time */
    f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
    {
      AudioState = AUDIO_STATE_PLAY;
      { 
        if(bytesread != 0)
        {
            AudioElapseSize = bytesread;
					 /* Start playing Wave */
						BSP_AUDIO_OUT_Play(SOUNDTERMINAL_DEV1,(uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
          return AUDIO_ERROR_NONE;
        }
      }
    }
 // }
  return AUDIO_ERROR_IO;
}

/**
  * @brief  Manages Audio process. 
  * @param  None
  * @retval Audio error
  */
AUDIO_ErrorTypeDef AUDIO_PLAYER_Process(void)
{
  uint32_t bytesread, elapsed_time;
  AUDIO_ErrorTypeDef audio_error = AUDIO_ERROR_NONE;
  static uint32_t prev_elapsed_time = 0xFFFFFFFF;
  uint8_t str[10];  
  
  switch(AudioState)
  {
  case AUDIO_STATE_PLAY:
    if(AudioElapseSize >= WaveFormat.FileSize)
    {
     WavePlayerStop();
		
		//	if(MusicList.Music_List_Ptr++>=MusicList.Music_List_All){
				AudioState = AUDIO_STATE_IDLE;
		//	}
		//	else{
		//		 AudioState =AUDIO_STATE_PLAY_P;
		//	}
    }
      if(BufferOffset == BUFFER_OFFSET_HALF)
      {
        f_read(&FileRead, 
               &Audio_Buffer[0], 
               AUDIO_BUFFER_SIZE/2, 
               (void *)&bytesread); 
        
        BufferOffset = BUFFER_OFFSET_NONE;
				AudioElapseSize+=bytesread;
      }
      
      if(BufferOffset == BUFFER_OFFSET_FULL)
      {
        f_read(&FileRead, 
               &Audio_Buffer[AUDIO_BUFFER_SIZE/2], 
               AUDIO_BUFFER_SIZE/2, 
               (void *)&bytesread); 
        
        BufferOffset = BUFFER_OFFSET_NONE;
				AudioElapseSize+=bytesread;
      } 
   
    break;
    
  case AUDIO_STATE_STOP:
    WavePlayerStop();
    AudioState = AUDIO_STATE_IDLE; 
    audio_error = AUDIO_ERROR_IO;
    break;
	case AUDIO_STATE_PLAY_P:
		   WavePlayerStop();
    AUDIO_PLAYER_Start(MusicList.list[MusicList.list[MusicList.Music_List_Ptr]]);
		break;
  case AUDIO_STATE_NEXT:
  //  if(++FilePos >= AUDIO_GetWavObjectNumber())
    {
  //    FilePos = 0; 
    }
  //  WavePlayerStop();
  //  AUDIO_PLAYER_Start(FilePos);
    break;    
    
  case AUDIO_STATE_PREVIOUS:
  //  if(--FilePos < 0)
    {
  //    FilePos = AUDIO_GetWavObjectNumber() - 1; 
    }
  //  WavePlayerStop();
  //  AUDIO_PLAYER_Start(FilePos);
    break;   
    
  case AUDIO_STATE_PAUSE:
    BSP_AUDIO_OUT_Pause(SOUNDTERMINAL_DEV1);
    AudioState = AUDIO_STATE_WAIT;
    break;
    
  case AUDIO_STATE_RESUME:
   
    BSP_AUDIO_OUT_Resume(SOUNDTERMINAL_DEV1);
    AudioState = AUDIO_STATE_PLAY;
    break;
    
  case AUDIO_STATE_VOLUME_UP: 
    if( Volume <= 90)
    {
      Volume += 10;
    }
    BSP_AUDIO_OUT_SetVolume(SOUNDTERMINAL_DEV1,STA350BW_CHANNEL_MASTER,Volume);
    AudioState = AUDIO_STATE_PLAY;
    break;
    
  case AUDIO_STATE_VOLUME_DOWN:    
    if( Volume >= 10)
    {
      Volume -= 10;
    }
    BSP_AUDIO_OUT_SetVolume(SOUNDTERMINAL_DEV1,STA350BW_CHANNEL_MASTER,Volume);
    AudioState = AUDIO_STATE_PLAY;
    break;
    
  case AUDIO_STATE_WAIT:
  case AUDIO_STATE_IDLE:
  case AUDIO_STATE_INIT:    
  default:
    /* Do Nothing */
    break;
  }
  return audio_error;
}








/**
  * @brief  Plays Wave from a mass storage.
  * @param  AudioFreq: Audio Sampling Frequency
  * @retval None
*/
void WavePlayBack(uint32_t AudioFreq)
{ 
  UINT bytesread = 0;
  
  /* Start playing */
  AudioPlayStart = 1;
  RepeatState = REPEAT_ON;
  
  /* Initialize Wave player (Codec, DMA, I2C) */

	if(first_flag_my==0){
		first_flag_my=1;
	//	if(WavePlayerInit(32000) != 0)
		if(WavePlayerInit(AudioFreq) != 0)
		{
			Error_Handler();
		}
	}
  /* Get Data from USB Flash Disk */
  f_lseek(&FileRead, 0);
  f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
  AudioRemSize = WaveDataLength - bytesread;
  
  /* Start playing Wave */
  BSP_AUDIO_OUT_Play(SOUNDTERMINAL_DEV1,(uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
  //LEDsState = LED6_TOGGLE;
  PauseResumeStatus = RESUME_STATUS;

  
  /* Check if the device is connected.*/
  while((AudioRemSize != 0) && (AppliState != APPLICATION_IDLE))
  { 
    /* Test on the command: Playing */
    if(CmdIndex == CMD_PLAY)
    { 
      if(PauseResumeStatus == PAUSE_STATUS)
      {
        /* Stop Toggling LED2 to signal Pause */
//        LEDsState = STOP_TOGGLE;
        /* Pause playing Wave */
        WavePlayerPauseResume(PauseResumeStatus);
        PauseResumeStatus = IDLE_STATUS;
      }
      else if(PauseResumeStatus == RESUME_STATUS)
      {
        /* Toggling LED6 to signal Play */
     //   LEDsState = LED6_TOGGLE;
        /* Resume playing Wave */
        WavePlayerPauseResume(PauseResumeStatus);
        PauseResumeStatus = IDLE_STATUS;
      }  
      
      bytesread = 0;
      
      if(BufferOffset == BUFFER_OFFSET_HALF)
      {
        f_read(&FileRead, 
               &Audio_Buffer[0], 
               AUDIO_BUFFER_SIZE/2, 
               (void *)&bytesread); 
        
        BufferOffset = BUFFER_OFFSET_NONE;
      }
      
      if(BufferOffset == BUFFER_OFFSET_FULL)
      {
        f_read(&FileRead, 
               &Audio_Buffer[AUDIO_BUFFER_SIZE/2], 
               AUDIO_BUFFER_SIZE/2, 
               (void *)&bytesread); 
        
        BufferOffset = BUFFER_OFFSET_NONE;
      } 
      if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2))
      {
        AudioRemSize -= bytesread;
      }
      else
      {
        AudioRemSize = 0;
      }
    }
    else 
    {
      /* Stop playing Wave */
      WavePlayerStop();
      f_close(&FileRead);
      AudioRemSize = 0;
      RepeatState = REPEAT_ON;
      break;
    }
  }

  RepeatState = REPEAT_ON;
  AudioPlayStart = 0;
  /* Stop playing Wave */
  WavePlayerStop();
  f_close(&FileRead);
}

/**
  * @brief  Pauses or Resumes a played Wave.
  * @param  state: Player state: Pause, Resume or Idle
  * @retval None
  */
void WavePlayerPauseResume(uint32_t wState)
{ 
  if(wState == PAUSE_STATUS)
  {
    BSP_AUDIO_OUT_Pause(SOUNDTERMINAL_DEV1);   
  }
  else
  {
    BSP_AUDIO_OUT_Resume(SOUNDTERMINAL_DEV1);   
  }
}

/**
  * @brief  Stops playing Wave.
  * @param  None
  * @retval None
  */
void WavePlayerStop(void)
{ 
  BSP_AUDIO_OUT_Stop(SOUNDTERMINAL_DEV1);
	f_close(&FileRead);
}
 
/**
  * @brief  Initializes the Wave player.
  * @param  AudioFreq: Audio sampling frequency
  * @retval None
  */
int WavePlayerInit(uint32_t AudioFreq)
{ 
  /* MEMS Accelerometer configure to manage PAUSE, RESUME operations */
  //BSP_ACCELERO_Click_ITConfig();

  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  return(BSP_AUDIO_OUT_Init(SOUNDTERMINAL_DEV1, Volume, AudioFreq));  
	 
}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio_codec.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(uint16_t OutputDevice)
{ 
  BufferOffset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(uint16_t OutputDevice)
{
  BufferOffset = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_ChangeBuffer(SOUNDTERMINAL_DEV1,(uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE /2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}
  
  /* Could also generate a system reset to recover from the error */
  /* .... */
}

/**
  * @brief  Starts Wave player.
  * @param  None
  * @retval None
  */
void WavePlayerStart(void)
{
  UINT bytesread = 0;
  char path[] = "0:/";
  char* wavefilename = NULL;
  WAVE_FormatTypeDef waveformat;
  
  FileList.ptr_P=0;
	FileList.file_P[0]=(uint8_t *)"bzdb.wav";
	FileList.file_P[1]=(uint8_t *)"audio2.wav";
	
	//	FileList.ptr=0;
	// strncpy((char *)FileList.file[FileList.ptr].name, (char *)"bzdb.wav", FILEMGR_FILE_NAME_SIZE);
	
	
  /* Get the read out protection status */
  if(f_opendir(&Directory, path) == FR_OK)
  {
    /* Open the Wave file to be played */
		
  //  if(f_open(&FileRead, (char *)FileList.file[FileList.ptr].name , FA_READ) == FR_OK)
			  if(f_open(&FileRead, (char *)FileList.file_P[FileList.ptr_P] , FA_READ) == FR_OK)
  
    {  
      /* Read sizeof(WaveFormat) from the selected file */
      f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
      
      /* Set WaveDataLenght to the Speech Wave length */
      WaveDataLength = waveformat.FileSize;
    
      /* Play the Wave */
      WavePlayBack(waveformat.SampleRate);
    }    
  }
}

/**
  * @brief  Resets the Wave player.
  * @param  None
  * @retval None
  */
void WavePlayer_CallBack(void)
{
  if(AppliState != APPLICATION_IDLE)
  {
    /* Reset the Wave player variables */
    RepeatState = REPEAT_ON;
    AudioPlayStart = 0;
//    LEDsState = LEDS_OFF;
    PauseResumeStatus = RESUME_STATUS;
    WaveDataLength =0;
    PressCount = 0;
    
    /* Stop the Codec */
    if(BSP_AUDIO_OUT_Stop(SOUNDTERMINAL_DEV1) != AUDIO_OK)
    {
      while(1){};
    }
    
    /* Turn OFF LED3, LED4 and LED6 */
   
  }
} 
/**
  * @brief  Main routine for Mass storage application
  * @param  None
  * @retval None
  */
 void MSC_Application(void)
{
    /* Go to Audio menu */
     switch(USBH_USR_ApplicationState)
  {
  case USBH_USR_AUDIO:    
    /* Go to Audio menu */
     if(RepeatState == REPEAT_ON)
      WavePlayerStart();
    /* Set user initialization flag */
    USBH_USR_ApplicationState = USBH_USR_FS_INIT;
    break;
    
  case USBH_USR_FS_INIT:    
    /* Initializes the File System */
    if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) 
    {
      /* FatFs initialization fails */
      Error_Handler();
    }
    
    /* Go to menu */
    USBH_USR_ApplicationState = USBH_USR_AUDIO;
    break;
    
  default:
    break;
  }
}

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
