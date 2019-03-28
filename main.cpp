//PROCEDURE FOR LINUX DIRECT UPLOAD when using PLATFORMIO
//CALL TEENSY_LOADER and SELECT HEX (located in subdir .pioenvs/teensy36)
//CALL TEENSY_REBOOT ... this will then directly upload the HEX
// ...change the source ... and recompile
//CALL TEENSY_REBOOT ... this will directly upload the changed HEX


/***********************************************************************
 *  TEENSY 3.6 BAT DETECTOR V0.8 20190324
 *
 *  Copyright (c) 2019, Cor Berrevoets, registax@gmail.com
 *
 *  TODO: use selectable presets
 *
 *
 *  HARDWARE USED:
 *     TEENSY 3.6
 *     TEENSY audio board
 *     Ultrasonic microphone with seperate preamplifier connected to mic/gnd on audioboard
 *       eg. Knowles MEMS SPU0410LR5H-QB
 *     TFT based on ILI9341
 *     2 rotary encoders with pushbutton
 *     2 pushbuttons
 *     SDCard
 *
*   IMPORTANT: uses the SD card slot of the Teensy, NOT the SD card slot of the audio board
 *
 *  4 operational modes: Heterodyne.
 *                       Frequency divider
 *                       Automatic heterodyne (1/10 implemented)
 *                       Automatic TimeExpansion (live)
 *
 *  Sample rates up to 352k
 *
 *  User controlled parameters:
 *     Volume
 *     Gain
 *     Frequency
 *     Display (none, spectrum, waterfall)
 *     Samplerate
 *
 *  Record raw data
 *  Play raw data (user selectable) on the SDcard using time_expansion (8, 11, 16,22,32,44k samplerate )
 *
 *
 *  Fixes compared to original base:
 *    - issue during recording due to not refilling part of the buffer (was repeating the original first 256 samples )
 *    - filenames have samplerate stored
 *    - RTC added (based on hardware)
 *
 * **********************************************************************
 *   Based on code by DD4WH
 *
 *   https://forum.pjrc.com/threads/38988-Bat-detector
 *
 *   https://github.com/DD4WH/Teensy-Bat-Detector
 *
 *   made possible by the samplerate code by Frank Boesing, thanks Frank!
 *   Audio sample rate code - function setI2SFreq
 *   Copyright (c) 2016, Frank Bösing, f.boesing@gmx.de
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **********************************************************************/

/* CORBEE */
/* TEENSY 3.6 PINSETUP (20180814)

                  GND                  Vin  - PREAMP V+
                   0                   Analog GND
                   1                   3.3V - MEMS MIC
                   2                   23 AUDIO -LRCLK
                   3                   22 AUDIO -TX
                   4                   21 TFT CS
                   5                   20 TFT DC
       AUDIO MEMCS 6                   19 AUDIO - SCL
       AUDIO MOSI  7                   18 AUDIO - SDA
                   8                   17
       AUDIO BCLK  9                   16
       AUDIO SDCS 10                  15 AUDIO -VOL
       AUDIO MCLK 11                  14 AUDIO -SCLK
       AUDIO MISO 12                  13 AUDIO -RX
                  3.3V                GND
                  24                  A22
                  25                  A21
                  26                  39  TFT MISO
        TFT SCLK  27                  38  MICROPUSH_L
        TFT MOSI  28                  37  MICROPUSH_R
     ENC_L-BUTTON 29                  36  ENC_R-BUTTON
     ENC_L A      30                  35  ENC_R A
     ENC_L B      31                  34  ENC_R B
                  32                  33

*/

// ***************************** GLOBAL DEFINES
//#define DEBUG

//USE a TFT (code will not function properly without !!! )
#define USETFT
//SD1 uses default SDcard Fat, TODO !! SD2 uses faster SDIO library
#define USESD1

// ************************************  SD *****************************
#ifdef USESD1
  #define USESD
  #include <SD.h>
  #include "ff.h"       // uSDFS lib
  #include "ff_utils.h" // uSDFS lib
  File root;
  FRESULT rc;        /* Result code */
  FATFS fatfs;      /* File system object */
  FIL fil;        /* File object */
#endif

// TODO: try and see if using the SdFs library is able to write faster
// started setup and included several #ifdefs inside the audio-library SDrelated files (play_raw play_wav)
#ifdef USESD2
  #define USESD
  //#include "SdFs.h"
  #include "logger_setup.h"
#endif

//default SD related
#ifdef USESD
    #define MAX_FILES    50
    #define MAX_FILE_LENGTH  13   // 8 chars plus 4 for.RAW plus NULL
    char filelist[ MAX_FILES ][ MAX_FILE_LENGTH ];
    int filecounter=0;
    int fileselect=0;
    int referencefile=0;
    int file_number = 0;
#endif

// *************************** LIBRARIES **************************

#include <TimeLib.h>

#include "Audio.h"
#include <SPI.h>
#include <Bounce.h>

// *************************** VARS  **************************

boolean SD_ACTIVE=false;
boolean continousPlay=false;
boolean Ultrasound_detected=false;//triggers when an ultrasonic signalpeak is found during FFT
boolean TE_ready=true; //when a TEcall is played this signals the end of the call

// ************************************  TIME *****************************
time_t getTeensy3Time()
{  return Teensy3Clock.get();
}

int helpmin; // definitions for time and date adjust - Menu
int helphour;
int helpday;
int helpmonth;
int helpyear;
int helpsec;
uint8_t hour10_old;
uint8_t hour1_old;
uint8_t minute10_old;
uint8_t minute1_old;
uint8_t second10_old;
uint8_t second1_old;
bool timeflag = 0;
uint32_t lastmillis;

// ************************************  TFT *****************************
// TFT definitions currently only setup for ILI9341
#ifdef USETFT
 #define ILI9341
 #ifdef ILI9341
  #include "ILI9341_t3.h"
  #include "font_Arial.h"

  #define BACKLIGHT_PIN 255
  #define TOP_OFFSET 90
  #define POWERGRAPH 45
  #define SPECTRUMSCALE 5

  #define BOTTOM_OFFSET 40
  #define BOTTOM_OFFSET_PART 20

  #define TFT_DC      20
  #define TFT_CS      21
  #define TFT_RST     255  // 255 = unused. connect to 3.3V
  #define TFT_MOSI    28
  #define TFT_SCLK    27
  #define TFT_MISO    39

  ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
  //#define Touch_CS    8
  //XPT2046_Touchscreen ts = XPT2046_Touchscreen(Touch_CS);
  //predefine menu background etc colors
  #define ENC_MENU_COLOR COLOR_YELLOW
  #define ENC_VALUE_COLOR COLOR_LIGHTGREY
  #define MENU_BCK_COLOR COLOR_DARKRED
  #define MENU_BCK_COLOR_PART COLOR_DARKGREEN
 #endif
#endif

// ******** LEFT AND RIGHT ENCODER CONNECTIONS/BUTTONS

/************** LButton Mode SETTINGS *****************/
const int8_t    MODE_DISPLAY =0;  //default left
const int8_t    MODE_REC = 1; //
const int8_t    MODE_PLAY = 2; //
const int8_t    MODE_MAX=3; // limiter for leftbutton

const int8_t    MODE_DETECT = 5; // default right

#include <Encoder.h>
//try to avoid interrupts as they can (possibly ?) interfere during recording
#define ENCODER_DO_NOT_USE_INTERRUPTS

#define MICROPUSH_RIGHT  37
Bounce micropushButton_R = Bounce(MICROPUSH_RIGHT, 50);
#define encoderButton_RIGHT      36
Bounce encoderButton_R = Bounce(encoderButton_RIGHT, 50);
Encoder EncRight(34,35);
int EncRightPos=0;
int EncRightchange=0;

#define MICROPUSH_LEFT  38
Bounce micropushButton_L = Bounce(MICROPUSH_LEFT, 50);
#define encoderButton_LEFT       29
Bounce encoderButton_L = Bounce(encoderButton_LEFT, 50);
Encoder EncLeft(30,31);
int EncLeftPos=0;
int EncLeftchange=0;

//************************* ENCODER variables/constants
const int8_t enc_menu=0; //changing encoder sets menuchoice
const int8_t enc_value=1; //changing encoder sets value for a menuchoice

const int8_t enc_leftside=0; //encoder
const int8_t enc_rightside=1; //encoder

const int8_t enc_up=1; //encoder goes up
const int8_t enc_nc=0;
const int8_t enc_dn=-1; //encoder goes down

int EncLeft_menu_idx=0;
int EncRight_menu_idx=0;

int EncLeft_function=0;
int EncRight_function=0;

uint8_t LeftButton_Mode=MODE_DISPLAY;
uint8_t LeftButton_Next=MODE_DISPLAY;
uint8_t RightButton_Mode=MODE_DETECT;

// **END************ LEFT AND RIGHT ENCODER DEFINITIONS


// *********************************** AUDIO SGTL5000 SETUP *******************************
// this audio comes from the codec by I2S2
AudioInputI2S                    i2s_in; // MIC input
AudioRecordQueue                 recorder;
AudioSynthWaveformSineHires      sine1; // local oscillator
AudioEffectMultiply              heterodyne_multiplier; // multiply = mix
AudioAnalyzeFFT256               myFFT; // for spectrum display
AudioPlaySdRaw                   player;
AudioEffectGranular              granular1;
AudioMixer4                      mixFFT;
AudioMixer4                      outputMixer; //selective output
AudioMixer4                      inputMixer; //selective input
AudioOutputI2S                   i2s_out; // headphone output
AudioConnection mic_toinput         (i2s_in, 0, inputMixer, 0); //microphone signal
AudioConnection mic_torecorder      (i2s_in, 0, recorder, 0); //microphone signal
AudioConnection player_toinput      (player, 0, inputMixer, 1); //player signal
AudioConnection input_toswitch      (inputMixer,0,  mixFFT,0);
AudioConnection input_todelay       (inputMixer,0, granular1, 0);
AudioConnection switch_toFFT        (mixFFT,0, myFFT,0 ); //raw recording channel
AudioConnection input_toheterodyne1 (inputMixer, 0, heterodyne_multiplier, 0); //heterodyne 1 signal
AudioConnection sineheterodyne1    (sine1, 0, heterodyne_multiplier, 1);//heterodyne 1 mixerfreq
AudioConnection granular_toout (granular1,0, outputMixer,1);
AudioConnection heterodyne1_toout      (heterodyne_multiplier, 0, outputMixer, 0);  //heterodyne 1 output to outputmixer
AudioConnection player_toout           (inputMixer,0, outputMixer, 2);    //direct signal (use with player) to outputmixer
AudioConnection output_toheadphoneleft      (outputMixer, 0, i2s_out, 0); // output to headphone
AudioConnection output_toheadphoneright     (outputMixer, 0, i2s_out, 1);

AudioControlSGTL5000        sgtl5000;

//const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;

//****************************************************************************************

// forward declaration Stop recording with message
#ifdef DEBUGSERIAL
   void die(char *str, FRESULT rc);
#endif

extern "C" uint32_t usd_getError(void);

// **************************** TIME VARS ********************************************
struct tm seconds2tm(uint32_t tt);
//continous timers
elapsedMillis started_detection; //start timing directly after FFT detects an ultrasound
elapsedMillis end_detection; //start timing directly after FFT detects the end of the ultrasound
elapsedMillis since_heterodyne=1000; //timing interval for auto_heterodyne frequency adjustments
uint16_t callLength=0;
//uint16_t clicker=0;


// ****************** TEENSY MODEL BASED BUFFERS ***********************************
#if defined(__MK20DX256__)
  #define BUFFSIZE (8*1024) // size of buffer  to be written
#elif defined(__MK66FX1M0__) // Teensy 3.6
  #define BUFF 96
  #define BUFFSIZE (BUFF*1024) // size of buffer to be written
#endif

// buffer to store audiosamples during recording
uint8_t sample_buffer[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
//uint8_t sample_buffer2[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
uint wr;
uint32_t nj = 0;

// ******************** BUFFER FOR TIMEEXPANSION ROUTINE ********************
#define GRANULAR_MEMORY_SIZE 16*1024  // enough for 50 ms at 281kHz sampling
int16_t granularMemory[GRANULAR_MEMORY_SIZE];
// ******************** SPECTRUM MODES ********************

uint8_t spectrum_mode=0;  //0=full, 1=selection1 2=xxx 

//set spectrum at 281khz
//full 2Khz-130khz (bin 2-120) pixelwidth 2 (240/120)
//25-105 khz              pixelwidth 3 (240/80)
//25-85 khz               pixelwidth 4 (240/60)
//25-73 khz               pixelwidth 5 (240/48)

#define waterfallgraph 1
#define spectrumgraph 2

int idx_t = 0;
int idx = 0;
int64_t sum;
float32_t mean;
int16_t FFT_bin [128];
//TEST
int16_t FFT_max1 = 0;
uint32_t FFT_max_bin1 = 0;
int16_t FFT_mean1 = 0;
int16_t FFT_max2 = 0;
uint32_t FFT_max_bin2 = 0;
int16_t FFT_mean2 = 0;
//int16_t FFT_threshold = 0;
int16_t FFT_bat [3]; // max of 3 frequencies are being displayed
int16_t index_FFT;
int l_limit;
int u_limit;
int index_l_limit;
int index_u_limit;
//const uint16_t FFT_points = 1024;
const uint16_t FFT_points = 256;
int barm [512];

// *********************************SAMPLING ****************************

#define SAMPLE_RATE_MIN               0
#define SAMPLE_RATE_8K                0
#define SAMPLE_RATE_11K               1
#define SAMPLE_RATE_16K               2
#define SAMPLE_RATE_22K               3
#define SAMPLE_RATE_32K               4
#define SAMPLE_RATE_44K               5
#define SAMPLE_RATE_48K               6
#define SAMPLE_RATE_88K               7
#define SAMPLE_RATE_96K               8
#define SAMPLE_RATE_176K              9
#define SAMPLE_RATE_192K              10
#define SAMPLE_RATE_234K              11
#define SAMPLE_RATE_281K              12
#define SAMPLE_RATE_352K              13
#define SAMPLE_RATE_MAX               13
// ***************** SAMPLE RATE DESCRIPTION
typedef struct SR_Descriptor
{   const int SR_n;
    const char* txt; //text for the display
    const u_int freq_real;
} SR_Desc;

// SRtext and position for the FFT spectrum display scale
const SR_Descriptor SR [SAMPLE_RATE_MAX + 1] =
{
    {  SAMPLE_RATE_8K,  "8" ,     8000},
    {  SAMPLE_RATE_11K,  "11",   11025},
    {  SAMPLE_RATE_16K,  "16",   16000},
    {  SAMPLE_RATE_22K,  "22",   22050},
    {  SAMPLE_RATE_32K,  "32",   32000},
    {  SAMPLE_RATE_44K,  "44",   44100},
    {  SAMPLE_RATE_48K,  "48",   48000},
    {  SAMPLE_RATE_88K,  "88",   88200},
    {  SAMPLE_RATE_96K,  "96",   96000},
    {  SAMPLE_RATE_176K,  "176", 176400},
    {  SAMPLE_RATE_192K,  "192", 192000},
    {  SAMPLE_RATE_234K,  "234", 234000},
    {  SAMPLE_RATE_281K,  "281", 281000},
    {  SAMPLE_RATE_352K,  "352", 352800}
};

// initial sampling setup
int sample_rate = SAMPLE_RATE_281K;
int sample_rate_real = SR[sample_rate].freq_real;
int last_sample_rate=sample_rate;
const char * SRtext=SR[sample_rate].txt;

// ***************************** FFT SETUP *******************************

// setup for FFTgraph denoising
uint32_t FFTcount=0; //count the # of FFTs done
uint16_t powerspectrumCounter=0;

float FFTavg[128];

float FFTpowerspectrum[128];
float powerspectrum_Max=0;

// defaults at startup functions
int displaychoice=waterfallgraph; //default display
int8_t mic_gain = 50; // start detecting with this MIC_GAIN in dB
int8_t volume=50;

int freq_real = 45000; // start heterodyne detecting at this frequency
int freq_real_backup=freq_real; //used to return to proper listening setting after using the play_function

float freq_Oscillator =50000;

/******************* MENU ********************************/
/*********************************************************/

//TODO constants should be part of the menuentry, a single structure to hold the info#
#define  MENU_VOL   0 //volume
#define  MENU_MIC   1 //mic_gain
#define  MENU_FRQ   2 //frequency
#define  MENU_BUTTONL   3 //set function for buttonL
#define  MENU_DNS   4 //denoise
#define  MENU_SPECTRUMMODE 5
#define  MENU_TIME  6
#define  MENU_SR    7 //sample rate
#define  MENU_REC   8 //record
#define  MENU_PLY   9 //play
#define  MENU_PLD   10 //play at original rate

#define  MENU_MAX 10

const int Leftchoices=MENU_MAX+1; //can have any value
const int Rightchoices=MENU_FRQ+1; //allow up to FRQ
const char* MenuEntry [Leftchoices] =
  {
    "Volume",
    "Gain",
    "Frequency",
    "ButtonL",
    "Denoise",
    "Spectrum",
    "SetTime",
    "SampleR",
    "Record",
    "Play",
    "PlayR",
    

  };

// typedef struct Menu_Descriptor
// {
//     const char* name;
//     // ********preset variables below NOT USED YET
//     const int len; // length of string to allow right-alignment
//     const int def; //default settings
//     const int low; // low threshold
//     const int high; //high threshold

// } Menu_Desc;

// const Menu_Descriptor MenuEntry1 [Leftchoices] =
// {  {"Volume",6,60,0,100}, //divide by 100
//    {"Gain",4,30,0,63},
//    {"Frequency",9,45,20,90}, //multiply 1000
//    {"Display",7,0,0,0},
//    {"Denoise",7,0,0,0},
//    {"SampleR",6,0,0,0},
//    {"Record",6,0,0,0}, //functions where the LeftEncoder
//    {"Play",4,0,0,0},
//    {"PlayD",5,0,0,0},
// } ;

//available modes
#define detector_heterodyne 0
#define detector_Auto_heterodyne 1
#define detector_Auto_TE 2 // time expansion
#define detector_divider 3
#define detector_passive 4 // no processing at all

//default
int detector_mode=detector_heterodyne;

// *********************************** CODE defs


#ifdef USESD1
void die(char *str, FRESULT rc)
{
   #ifdef DEBUGSERIAL
   Serial.printf("%s: Failed with rc=%u.\n", str, rc); for (;;) delay(100);
   #endif
   }

//=========================================================================
#endif



#ifdef USESD1
//uint32_t count=0;
uint32_t ifn=0;
uint32_t isFileOpen=0;

TCHAR wfilename[80];
uint32_t t0=0;
uint32_t t1=0;
#endif

char filename[80];


// ******************************************** DISPLAY

int calc_menu_dxoffset(const char* str)  // to position the menu on the right screenedge
{
#ifdef USETFT
  String s=String(str); 
  char charBuf[50];
  s.toCharArray(charBuf, 50) ;
  uint16_t dx=tft.strPixelLen(charBuf);
  return dx; 
#endif
  
}



void display_settings() {
  #ifdef USETFT

    tft.setTextColor(ENC_MENU_COLOR);

    tft.setFont(Arial_16);
    tft.fillRect(0,0,240,TOP_OFFSET-50,MENU_BCK_COLOR);
    tft.fillRect(0,TOP_OFFSET-10,240,10,COLOR_BLACK);
    tft.fillRect(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET,240,BOTTOM_OFFSET_PART,MENU_BCK_COLOR);
    tft.fillRect(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET_PART,240,BOTTOM_OFFSET,MENU_BCK_COLOR_PART);

    tft.setCursor(0,0);
    tft.print("g:"); tft.print(mic_gain);
    tft.print(" f:"); tft.print(freq_real);
    tft.print(" v:"); tft.print(volume);
    tft.print(" SR"); tft.print(SRtext);
    //tft.print(" M"); tft.print(LeftButton_Mode); tft.print(LeftButton_Next);
    tft.setCursor(0,20);

    switch (detector_mode) {
       case detector_heterodyne:
         tft.print("HT"); //
       break;
       case detector_divider:
         tft.print("FD");
       break;
       case detector_Auto_heterodyne:
         tft.print("Auto_HT");
       break;
       case detector_Auto_TE:
        tft.print("Auto_TE");
       break;
       case detector_passive:
        tft.print("PASS");
       break;
       default:
        tft.print("error");

     }
     // push the cursor to the lower part of the screen 
     tft.setCursor(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET); //position of encoder functions

     /****************** SHOW ENCODER SETTING ***********************/

     // set the colors according to the function of the encoders
     if (RightButton_Mode==MODE_DETECT )
     // show menu selection as menu-active of value-active
      {
       if (EncLeft_function==enc_value)
        { tft.setTextColor(ENC_MENU_COLOR);
         }
        else
        { tft.setTextColor(ENC_VALUE_COLOR);
         }
      
       tft.print(MenuEntry[EncLeft_menu_idx]);
      //  if (EncLeft_menu_idx==MENU_SPECTRUMMODE)
      //    tft.print(spectrum_mode);
       
       tft.print(" ");

       if (EncRight_function==enc_value) 
         { tft.setTextColor(ENC_MENU_COLOR);} //value is active
       else
         { tft.setTextColor(ENC_VALUE_COLOR);} //menu is active

       //if MENU on the left-side is PLAY and selected than show the filename
        if ((EncLeft_menu_idx==MENU_PLY) and (EncLeft_function==enc_value))
           { //tft.print(fileselect);
             tft.print(filelist[fileselect]);
           }
        else
         if (EncLeft_menu_idx==MENU_REC)
          // show the filename that will be used for the next recording
           {  sprintf(filename, "B%u_%s.raw", file_number+1,SRtext);
              tft.print(filename );
            }
         else
         if (EncLeft_menu_idx==MENU_SR)
          { tft.print(SR[sample_rate].txt);

          }
          else
          { //tft.print(EncRightchange);
            uint16_t dx=calc_menu_dxoffset((MenuEntry[EncRight_menu_idx]));
            //uint16_t dx=tft.strPixelLen(MenuEntry[EncRight_menu_idx]);
            tft.setCursor(ILI9341_TFTWIDTH-dx,ILI9341_TFTHEIGHT-BOTTOM_OFFSET);
            tft.print(MenuEntry[EncRight_menu_idx]);
          }
    }
    else
      {
        if (LeftButton_Mode==MODE_REC)
          { tft.setTextColor(ENC_VALUE_COLOR);
            tft.print("REC:");
            tft.print(filename);
         }
        if (LeftButton_Mode==MODE_PLAY)
         {if (EncLeft_menu_idx==MENU_PLY)
          { tft.setTextColor(ENC_VALUE_COLOR);
            tft.print("PLAY:");
            tft.print(filename);
          }
          else
           {tft.setTextColor(ENC_VALUE_COLOR);
            tft.print(MenuEntry[EncLeft_menu_idx]);
            
            uint16_t dx=calc_menu_dxoffset((MenuEntry[EncRight_menu_idx]));
            tft.setCursor(ILI9341_TFTWIDTH-dx,ILI9341_TFTHEIGHT-BOTTOM_OFFSET);
            tft.print(MenuEntry[EncRight_menu_idx]);

           }

        }
      }

    // push the cursor to the lower part of the screen 
    tft.setCursor(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET_PART); //position of button functions
    
    tft.setTextColor(COLOR_YELLOW);

    if ((EncLeft_menu_idx==MENU_BUTTONL ) and (EncLeft_function==enc_value)) //show the setting is actively getting changed
       tft.setTextColor(COLOR_WHITE);

    if (LeftButton_Next==MODE_DISPLAY)
      tft.print("DISPLAY");
    if (LeftButton_Next==MODE_REC)
      tft.print("RECORD");
    if (LeftButton_Next==MODE_PLAY)
      tft.print("PLAY");
    
    tft.setTextColor(COLOR_YELLOW);

    char* s="MODE";
    if (RightButton_Mode==MODE_DISPLAY)
      s=("DISPLAY");
    if (RightButton_Mode==MODE_REC)
      s=("RECORD");
    if (RightButton_Mode==MODE_PLAY)
      s=("PLAY");
    if (RightButton_Mode==MODE_DETECT)
      s=("DETECT");

    uint16_t sx=tft.strPixelLen(s);
    tft.setCursor(ILI9341_TFTWIDTH-sx-5,ILI9341_TFTHEIGHT-BOTTOM_OFFSET_PART);
    tft.print(s);
        
    //scale every 10kHz
    if (displaychoice>0)
      { float x_factor=10000/(0.5*(sample_rate_real / FFT_points));
        int curF=2*int(freq_real/(sample_rate_real / FFT_points));
        int maxScale=int(sample_rate_real/20000);
        for (int i=1; i<maxScale; i++)
        { tft.drawFastVLine(i*x_factor, TOP_OFFSET-SPECTRUMSCALE, SPECTRUMSCALE, ENC_MENU_COLOR);
        }
        tft.fillCircle(curF,TOP_OFFSET-4,3,ENC_MENU_COLOR);
      }
      else 
        { tft.setCursor(0,TOP_OFFSET-SPECTRUMSCALE);
         


          tft.print("DISPLAY DISABLED");
        }
   #endif
}


// ************************* FUNCTIONS
void       set_mic_gain(int8_t gain) {

    AudioNoInterrupts();
    //sgtl5000.micGainNew (24);
    sgtl5000.micGain (gain);
    //sgtl5000.lineInLevel(gain/4);
    AudioInterrupts();

    display_settings();
    powerspectrum_Max=0; // change the powerspectrum_Max for the FFTpowerspectrum
} // end function set_mic_gain

void       set_freq_Oscillator(int freq) {
    // audio lib thinks we are still in 44118sps sample rate
    // therefore we have to scale the frequency of the local oscillator
    // in accordance with the REAL sample rate

    freq_Oscillator = (freq) * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real);
    //float F_LO2= (freq+5000) * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real);
    // if we switch to LOWER samples rates, make sure the running LO
    // frequency is allowed ( < 22k) ! If not, adjust consequently, so that
    // LO freq never goes up 22k, also adjust the variable freq_real
    if(freq_Oscillator > 22000) {
      freq_Oscillator = 22000;
      freq_real = freq_Oscillator * (sample_rate_real / AUDIO_SAMPLE_RATE_EXACT) + 9;
    }
    AudioNoInterrupts();
    //setup bin2frequency SINE
    sine1.frequency(freq_Oscillator);
    //sine2.frequency(freq_Oscillator);

    AudioInterrupts();
    display_settings();
} // END of function set_freq_Oscillator

// set samplerate code by Frank Boesing
void setI2SFreq(int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } tmclk;
//MCLD Divide sets the MCLK divide ratio such that: MCLK output = MCLK input * ( (FRACT + 1) / (DIVIDE + 1) ).
// FRACT must be set equal or less than the value in the DIVIDE field.
//(double)F_PLL * (double)clkArr[iFreq].mult / (256.0 * (double)clkArr[iFreq].div);
//ex 180000000* 1 /(256* 3 )=234375Hz  setting   {1,3} at 180Mhz


#if (F_PLL==16000000)
  const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
#elif (F_PLL==72000000)
  const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
#elif (F_PLL==96000000)
  const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
#elif (F_PLL==120000000)
  const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
#elif (F_PLL==144000000)
  const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
#elif (F_PLL==168000000)
  const tmclk clkArr[numfreqs] = {{32, 2625}, {21, 1250}, {64, 2625}, {21, 625}, {128, 2625}, {42, 625}, {8, 119}, {64, 875}, {84, 625}, {16, 119}, {128, 875}, {168, 625}, {32, 119}, {189, 646} };
#elif (F_PLL==180000000)
  const int numfreqs = 17;
  const int samplefreqs[numfreqs] = {  8000,      11025,      16000,      22050,       32000,       44100, (int)44117.64706 , 48000,      88200, (int)44117.64706 * 2,   96000, 176400, (int)44117.64706 * 4, 192000,  234000, 281000, 352800};
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255},   {128, 1875}, {107, 853},     {32, 255},   {219, 1604}, {1, 4},      {64, 255},     {219,802}, { 1,3 },  {2,5} , {1,2} };  //last value 219 802

#elif (F_PLL==192000000)
  const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
#elif (F_PLL==216000000)
  const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
#elif (F_PLL==240000000)
  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
#endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF) ;
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}

void  set_sample_rate (int sr) {
  sample_rate_real=SR[sr].freq_real;
  SRtext=SR[sr].txt;
  AudioNoInterrupts();
  setI2SFreq (sample_rate_real);
  delay(200); // this delay seems to be very essential !
  set_freq_Oscillator (freq_real);
  AudioInterrupts();
  delay(20);
  display_settings();

}

// **************** NORMAL SPECTRUM GRAPH

void spectrum() { // spectrum analyser code by rheslip - modified
     #ifdef USETFT
     {
//   
  uint16_t OFFSET =ILI9341_TFTHEIGHT-BOTTOM_OFFSET-TOP_OFFSET;
 //tft.fillRect(0,TOP_OFFSET,ILI9341_TFTWIDTH,OFFSET, COLOR_BLACK);
  int16_t FFT_sbin [128];
  for (int16_t x = 2; x < 128; x++) {
     FFT_sbin[x] = (myFFT.output[x]);//-FFTavg[x]*0.9;
     int barnew = (FFT_sbin[x])*0.5 ;
     // this is a very simple first order IIR filter to smooth the reaction of the bars
     int bar = 0.05 * barnew + 0.95 * barm[x];
     if (bar > OFFSET) //clip
        { bar= OFFSET;
        }
     if (bar <0) 
        {bar=0;
        }
     if (barnew >OFFSET) //clip
        { barnew=OFFSET;
        }
     int g_x=x*2;
     int spectrumline=barm[x];
     int spectrumline_new=barnew;

     tft.drawFastVLine(g_x,TOP_OFFSET,spectrumline_new, COLOR_GREEN);
     tft.drawFastVLine(g_x,TOP_OFFSET+spectrumline_new,OFFSET-spectrumline_new, COLOR_BLACK);
     tft.drawFastVLine(g_x+1,TOP_OFFSET,spectrumline, COLOR_DARKGREEN);
     tft.drawFastVLine(g_x+1,TOP_OFFSET+spectrumline,ILI9341_TFTHEIGHT-BOTTOM_OFFSET-TOP_OFFSET-spectrumline, COLOR_BLACK);
     barm[x] = bar;
  }

    // if (mode == MODE_DETECT)  search_bats();
  } //end if
  
  #endif
}


// **************** General graph *************************************************************************************
void updatedisplay(void) 
{

#ifdef USETFT 

// code for 256 point FFT
 if (myFFT.available()) {

  const uint16_t Y_OFFSET = TOP_OFFSET;
  static int count = TOP_OFFSET;
  //int curF=int(freq_real/(sample_rate_real / FFT_points));

  // lowest frequencybin to detect as a batcall
  int batCall_LoF_bin= int(25000/(sample_rate_real / FFT_points));
  int batCall_HiF_bin= int(80000/(sample_rate_real / FFT_points));

  uint8_t spec_hi=120; //default 120
  uint8_t spec_lo=2; //default 2
  uint8_t spec_width=2;

  uint16_t FFT_pixels[240]; // maximum of 240 pixels, each one is the result of one FFT
  memset(FFT_pixels,0,sizeof(FFT_pixels));
  //FFT_pixels[0]=0; FFT_pixels[1]=0;  FFT_pixels[2]=0; FFT_pixels[3]=0;

    FFTcount++;

    //requested to start with a clean FFTavg array to denoise
    if (FFTcount==1)
       {for (int16_t i = 0; i < 128; i++) {
          FFTavg[i]=0;
       }
     }

    // collect 100 FFT samples for the denoise array
    if (FFTcount<100)
     { for (int i = 2; i < 128; i++) {
         //FFTavg[i]=FFTavg[i]+myFFT.read(i)*65536.0*5*0.001; //0.1% of total values
         FFTavg[i]=FFTavg[i]+myFFT.output[i]*10*0.001; //0.1% of total values
         }
     }

    int FFT_peakF_bin=0;
    int peak=512;
    int avgFFTbin=0;
    // there are 128 FFT different bins only 120 are shown on the graphs
   
    for (int i = spec_lo; i < spec_hi; i++) {
      int val = myFFT.output[i]*10 -FFTavg[i]*0.9 + 10; //v1
      avgFFTbin+=val;
      //detect the peakfrequency
      if (val>peak)
       { peak=val;
         FFT_peakF_bin=i;
        }
       if (val<5)
           {val=5;}

       uint8_t pixpos=(i-spec_lo)*spec_width;
       FFT_pixels[pixpos] = tft.color565(
              min(255, val/2),
              (val/6>255)? 255 : val/6,
              //(val/4>255)? 255 : val/4
                            0
              //((255-val)>>1) <0? 0: (255-val)>>1
             );
      for (int j=1; j<spec_width; j++)
         { FFT_pixels[pixpos+j]=FFT_pixels[pixpos];
         }
    }

    avgFFTbin=avgFFTbin/(spec_hi-spec_lo);
    if ((peak/avgFFTbin)<1.1) //very low peakvalue so probably noise
     { FFT_peakF_bin=0;
     }

  int powerSpectrum_Maxbin=0;
  // detected a peak in the bat frequencies
    if ((FFT_peakF_bin>batCall_LoF_bin) and (FFT_peakF_bin<batCall_HiF_bin))
    {
        //collect data for the powerspectrum
      for (int i = spec_lo; i < spec_hi; i++)
      {    //add new samples
          FFTpowerspectrum[i]+=myFFT.output[i];
          //keep track of the maximum
          if (FFTpowerspectrum[i]>powerspectrum_Max)
            { powerspectrum_Max=FFTpowerspectrum[i];
              powerSpectrum_Maxbin=i;
            }
      }
      //keep track of the no of samples with bat-activity
      powerspectrumCounter++;
    }
 
    if (displaychoice==spectrumgraph)
         if (detector_mode==detector_Auto_TE)
            {  if(powerspectrumCounter%5==0) 
                    { spectrum();
                    }
            }
          else
          {
            if(FFTcount%2==0) 
                spectrum();

          }
            

    // update spectrumdisplay after every 50th FFT sample with bat-activity
    if (displaychoice==waterfallgraph)
      if ((powerspectrumCounter>50)  )
       { powerspectrumCounter=0;
                  
         //clear powerspectrumbox
         tft.fillRect(0,TOP_OFFSET-POWERGRAPH-SPECTRUMSCALE,ILI9341_TFTWIDTH,POWERGRAPH, COLOR_BLACK);
           
         // keep a minimum maximumvalue to the powerspectrum
         int binLo=spec_lo; int binHi=0;
         //find the nearest frequencies below 10% of the maximum
         if (powerSpectrum_Maxbin!=0)
           {
              boolean searchedge=true;
              int i=powerSpectrum_Maxbin;
              while(searchedge) {
                   if ((FFTpowerspectrum[i]/(powerspectrum_Max+1))>0.1)
                     { i--;
                       if (i==spec_lo)
                        {searchedge=false;
                         binLo=spec_lo; }
                     }
                    else
                     { searchedge=false;
                       binLo=i;
                     }

                  }
              searchedge=true;
              i=powerSpectrum_Maxbin;
              while(searchedge) {
                   if ((FFTpowerspectrum[i]/(powerspectrum_Max+1))>0.1)
                     { i++;
                       if (i==spec_hi)
                        {searchedge=false;
                         binHi=0; }
                     }
                    else
                     { searchedge=false;
                       binHi=i;
                     }
                  }
           }
          //draw spectrumgraph
          for (int i=spec_lo; i<spec_hi; i++)
            { int ypos=FFTpowerspectrum[i]/powerspectrum_Max*POWERGRAPH;
              tft.drawFastVLine((i-spec_lo)*spec_width,TOP_OFFSET-ypos-SPECTRUMSCALE,ypos,COLOR_RED);
              if (i==powerSpectrum_Maxbin)
                { tft.drawFastVLine((i-spec_lo)*spec_width,TOP_OFFSET-ypos-SPECTRUMSCALE,ypos,COLOR_YELLOW);
                }
              //reset powerspectrum for next samples
              FFTpowerspectrum[i]=0;
            }
          
          float bin2frequency=(sample_rate_real / FFT_points)*0.001;
          powerspectrum_Max=powerspectrum_Max*0.5; //lower the max after a graphupdate

          tft.setCursor(130,TOP_OFFSET-POWERGRAPH);
          tft.setTextColor(ENC_VALUE_COLOR);
          tft.print(int(binLo*bin2frequency) );
          tft.print(" ");
          tft.setTextColor(ENC_MENU_COLOR);
          tft.print(int(powerSpectrum_Maxbin*bin2frequency) );
          tft.print(" ");
          tft.setTextColor(ENC_VALUE_COLOR);
          tft.print(int(binHi* bin2frequency) );
          tft.print('*');
          tft.print(spectrum_mode);
        
       }


    if ((FFT_peakF_bin>batCall_LoF_bin) and (FFT_peakF_bin<batCall_HiF_bin)) // we got a high-frequent signal peak
      {
        // when a batcall is first discovered
        if (not Ultrasound_detected)
          { started_detection=0; //start of the call mark
            //clicker=0;
            FFT_pixels[5]=ENC_VALUE_COLOR; // mark the start on the screen
            FFT_pixels[6]=ENC_VALUE_COLOR;
            FFT_pixels[7]=ENC_VALUE_COLOR;

            if (detector_mode==detector_Auto_heterodyne)
               if (since_heterodyne>1000) //update the most every second
                {freq_real=int((FFT_peakF_bin*(sample_rate_real / FFT_points)/500))*500; //round to nearest 500hz
                 set_freq_Oscillator(freq_real);
                 since_heterodyne=0;
                 //granular1.stop();
                }

            //restart the TimeExpansion only if the previous call was played
            if ((detector_mode==detector_Auto_TE) and (TE_ready) )
             { granular1.stop();
               granular1.beginTimeExpansion(GRANULAR_MEMORY_SIZE);
               granular1.setSpeed(0.05);
               TE_ready=false;

             }

          }
         //clicker++;
         Ultrasound_detected=true;

     }
   else // FFT_peakF_bin does not show a battcall
        {
          if (Ultrasound_detected) //previous sample was still a call
           { callLength=started_detection; // got a pause so store the time since the start of the call

             /*if (callLength>40) //call is too long
              { TE_ready=true; // break the TE replay
                }
             */
             end_detection=0; //start timing the length of the replay
             }
          Ultrasound_detected=false;
        }
    // restart TimeExpansion recording a bit after the call has finished completely
    if ((!TE_ready) and (started_detection>(80)))
      { //stop the time expansion
        TE_ready=true;
        granular1.stopTimeExpansion();
      }

   if (displaychoice==waterfallgraph)
   {
     if (end_detection<50) //keep scrolling 50ms after the last bat-call
      { //if (TE_ready) //not playing TE
        { tft.writeRect( 0,count, ILI9341_TFTWIDTH,1, (uint16_t*) &FFT_pixels); //show a line with spectrumdata
          tft.setScroll(count);
        count++;
        }

      }

     if (count >= ILI9341_TFTHEIGHT-BOTTOM_OFFSET) count = Y_OFFSET;
   }

   
  }
#endif

}





// ******************************************  RECORDING


void startRecording() {
  LeftButton_Mode = MODE_REC;
  #ifdef USESD1

    #ifdef DEBUGSERIAL
      Serial.print("startRecording");
    #endif

    // close file
    if(isFileOpen)
    {
      //close file
      rc = f_close(&fil);
      #ifdef DEBUGSERIAL
      if (rc) die("close", rc);
      #endif
      isFileOpen=0;
    }

  if(!isFileOpen)
  {
  file_number++;
  //automated filename BA_S.raw where A=file_number and S shows samplerate. Has to fit 8 chars
  // so max is B999_192.raw
  sprintf(filename, "B%u_%s.raw", file_number, SRtext);
    #ifdef DEBUGSERIAL
    Serial.println(filename);
    #endif
  char2tchar(filename, 13, wfilename);
  filecounter++;
  strcpy(filelist[filecounter],filename );

  rc = f_stat (wfilename, 0);
  #ifdef DEBUGSERIAL
    Serial.printf("stat %d %x\n",rc,fil.obj.sclust);
 #endif
  rc = f_open (&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);
#ifdef DEBUGSERIAL
    Serial.printf(" opened %d %x\n\r",rc,fil.obj.sclust);
#endif
    // check if file has errors
    if(rc == FR_INT_ERR)
    { // only option then is to close file
        rc = f_close(&fil);
        if(rc == FR_INVALID_OBJECT)
        {
          #ifdef DEBUGSERIAL
          Serial.println("unlinking file");
          #endif
          rc = f_unlink(wfilename);
          #ifdef DEBUGSERIAL
          if (rc) {
            die("unlink", rc);
          }
          #endif
        }
        else
        {
          #ifdef DEBUGSERIAL
          die("close", rc);
          #endif
        }
    }
    // retry open file
    rc = f_open(&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);

    if(rc) {
      #ifdef DEBUGSERIAL
      die("open", rc);
      #endif
    }
    isFileOpen=1;
  }

  #endif

  //clear the screen completely
  tft.setScroll(0);
  tft.fillRect(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT,COLOR_BLACK);
  tft.setTextColor(ENC_VALUE_COLOR);
  tft.setFont(Arial_28);
  tft.setCursor(0,100);
  tft.print("RECORDING");
  tft.setFont(Arial_16);

  display_settings();

  granular1.stop(); //stop granular

  //switch off several circuits
  mixFFT.gain(0,0);

  outputMixer.gain(1,0);  //shutdown granular output

  detector_mode=detector_heterodyne;

  outputMixer.gain(0,1);

  nj=0;
  recorder.begin();

}
// **************** continue

void continueRecording() {
  #ifdef USESD1
  const uint32_t N_BUFFER = 2;
  const uint32_t N_LOOPS = BUFF*N_BUFFER; // !!! NLOOPS and BUFFSIZE ARE DEPENDENT !!! NLOOPS = BUFFSIZE/N_BUFFER
  // buffer size total = 256 * n_buffer * n_loops
  // queue: write n_buffer blocks * 256 bytes to buffer at a time; free queue buffer;
  // repeat n_loops times ( * n_buffer * 256 = total amount to write at one time)
  // then write to SD card

  if (recorder.available() >= N_BUFFER  )
  {// one buffer = 256 (8bit)-bytes = block of 128 16-bit samples
    //read N_BUFFER sample-blocks into memory
    for (uint i = 0; i < N_BUFFER; i++) {
       //copy a new bufferblock from the audiorecorder into memory
       memcpy(sample_buffer + i*256 + nj * 256 * N_BUFFER, recorder.readBuffer(), 256);
       //free the last buffer that was read
       recorder.freeBuffer();
       }

    nj++;

    if (nj >  (N_LOOPS-1))
    {
      nj = 0;
      //old code used to copy into a 2nd buffer, not needed since the writing to SD of the buffer seems faster than the filling
      //this allows larger buffers to be used
      //memcpy(sample_buffer2,sample_buffer,BUFFSIZE);
      //push to SDcard
      rc =  f_write (&fil, sample_buffer, N_BUFFER * 256 * N_LOOPS, &wr);
      }
  }
  #endif
}
// **************** STOP
void stopRecording() {
#ifdef USESD1
  #ifdef DEBUGSERIAL
    Serial.print("stopRecording");
  #endif
    recorder.end();
    if (LeftButton_Mode == MODE_REC) {
      while (recorder.available() > 0) {
      rc = f_write (&fil, (byte*)recorder.readBuffer(), 256, &wr);
  //      frec.write((byte*)recorder.readBuffer(), 256);
        recorder.freeBuffer();
      }
        //close file
        rc = f_close(&fil);
        #ifdef DEBUGSERIAL
        if (rc) die("close", rc);
        #endif
        //
        isFileOpen=0;
  //    frec.close();
  //    playfile = recfile;
    }

    LeftButton_Mode = MODE_DISPLAY;
  //  clearname();
  #ifdef DEBUGSERIAL
    Serial.println (" Recording stopped!");
  #endif
#endif
  //switch on FFT
  tft.fillScreen(COLOR_BLACK);
  mixFFT.gain(0,1);

}
// ******************************************  END RECORDING *************************

// **************** PLAYING ************************************************
void startPlaying(int SR) {
//      String NAME = "Bat_"+String(file_number)+".raw";
//      char fi[15];
//      NAME.toCharArray(fi, sizeof(NAME));

inputMixer.gain(0,0); //switch off the mic-line as input
inputMixer.gain(1,1); //switch on the playerline as input

if (EncLeft_menu_idx==MENU_PLY)
  {
      outputMixer.gain(2,1);  //player to output
      outputMixer.gain(1,0);  //shutdown granular output
      outputMixer.gain(0,0);  //shutdown heterodyne output
      EncRight_menu_idx=MENU_SR;
      EncRight_function=enc_value;
      freq_real_backup=freq_real; //keep track of heterodyne setting
  }
  //direct play is used to test functionalty based on previous recorded data
  //this will play a previous recorded raw file through the system as if it were live data coming from the microphone
if (EncLeft_menu_idx==MENU_PLD)
  {
      outputMixer.gain(2,0);  //shutdown direct audio from player to output
      outputMixer.gain(0,1);  //default mode will be heterodyne based output
      if (detector_mode==detector_Auto_TE)
         { outputMixer.gain(1,1);  //start granular output processing
           outputMixer.gain(0,0);
         }
  }

//allow settling
  delay(100);

  //keep track of the sample_rate
  last_sample_rate=sample_rate;
  SR=constrain(SR,SAMPLE_RATE_MIN,SAMPLE_RATE_MAX);
  set_sample_rate(SR);

  fileselect=constrain(fileselect,0,filecounter);
  strncpy(filename, filelist[fileselect],  13);

  //default display is waterfall
  displaychoice=waterfallgraph;
  display_settings();

  player.play(filename);
  LeftButton_Mode = MODE_PLAY;

}



// **************** STOP PLAYING ***************************************
void stopPlaying() {

#ifdef DEBUGSERIAL
  Serial.print("stopPlaying");
#endif
  if (LeftButton_Mode == MODE_PLAY) player.stop();
  LeftButton_Mode = MODE_DISPLAY;
#ifdef DEBUGSERIAL
  Serial.println (" Playing stopped");
#endif

  //restore last sample_rate setting
  set_sample_rate(last_sample_rate);
if (EncLeft_menu_idx==MENU_PLY)
{
  freq_real=freq_real_backup;
  //restore heterodyne frequency
  set_freq_Oscillator (freq_real);
}
  outputMixer.gain(2,0); //stop the direct line output
  outputMixer.gain(1,1); // open granular output
  outputMixer.gain(0,1); // open heterodyne output

  inputMixer.gain(0,1); //switch on the mic-line
  inputMixer.gain(1,0); //switch off the playerline

}

// **************** CONTINUE PLAYING
void continuePlaying() {
  //the end of file was reached
  if (!player.isPlaying()) {
    stopPlaying();
    if (continousPlay) //keep playing until stopped by the user
      { startPlaying(SAMPLE_RATE_281K);
      }
  }
}

// *************************************** MODES *****************************
void defaultMenuPosition()
{EncLeft_menu_idx=MENU_VOL;
 EncLeft_function=enc_value;
 EncRight_menu_idx=MENU_MIC;
 EncRight_function=enc_value;
}

int heterodynemixer=0;
int granularmixer=1;

void setMixer(int mode)
 {if (mode==heterodynemixer)
  { outputMixer.gain(1,0);  //stop granular output
    outputMixer.gain(0,1);  //start heterodyne output
  }
 if (mode==granularmixer)
  { outputMixer.gain(1,1);  //start granular output
    outputMixer.gain(0,0);  //shutdown heterodyne output
  }
}
//  *********************** MAIN MODE CHANGE ROUTINE *************************
void changeDetector_mode()
{
  if (detector_mode==detector_heterodyne)
         { granular1.stop(); //stop other detecting routines
           setMixer(heterodynemixer);
           //switch menu to volume/frequency
           EncLeft_menu_idx=MENU_VOL;
           EncLeft_function=enc_value;
           EncRight_menu_idx=MENU_FRQ;
           EncRight_function=enc_value;
         }
  if (detector_mode==detector_divider)
         { granular1.beginDivider(GRANULAR_MEMORY_SIZE);
           setMixer(granularmixer);
           defaultMenuPosition();
         }

  if (detector_mode==detector_Auto_TE)
         { granular1.beginTimeExpansion(GRANULAR_MEMORY_SIZE);
           setMixer(granularmixer);
           granular1.setSpeed(0.05); //default TE is 1/0.06 ~ 1/16 :TODO, switch from 1/x floats to divider value x
           defaultMenuPosition();
       }
  if (detector_mode==detector_Auto_heterodyne)
         { granular1.stop();
           setMixer(heterodynemixer);
           defaultMenuPosition();
         }

  if (detector_mode==detector_passive)
         { granular1.stop(); //stop all other detecting routines
           outputMixer.gain(2,1);  //direct line to output
           outputMixer.gain(1,0);  //shutdown granular output
           outputMixer.gain(0,0);  //shutdown heterodyne output
           defaultMenuPosition();
         }
       else // always shut down the direct output line except for passive
        {  outputMixer.gain(2,0);  //shut down direct line to output
        }

}

//*****************************************************update encoders
void updateEncoder(uint8_t Encoderside )
 {
  /************************setup vars*************************/
   int encodermode=-1; // menu=0 value =1;
   int change=0;
   int menu_idx=0;
   int choices=0;

    //get encodermode
   if (Encoderside==enc_leftside)
    { encodermode=EncLeft_function;
      change=EncLeftchange;
      menu_idx=EncLeft_menu_idx;
      choices=Leftchoices; //available menu options
    }

   if (Encoderside==enc_rightside)
    { encodermode=EncRight_function;
      change=EncRightchange;
      menu_idx=EncRight_menu_idx;
      choices=Rightchoices; //available menu options
    }

  /************************proces selection changes*************************/
  //encoder is in menumode
  if (encodermode==enc_menu)
    { menu_idx=menu_idx+change;

      //allow revolving choices
      if (menu_idx<0)
        {menu_idx=choices-1;}
      if (menu_idx>=choices)
        {menu_idx=0;}

      //remove functionality when SD is not active, so no SDCARD mounted or SDCARD is unreadable
      if (!SD_ACTIVE)
        { if ((menu_idx==MENU_PLD) or (menu_idx==MENU_PLY) or (menu_idx==MENU_REC))
           { // move menu to volume
             menu_idx=MENU_VOL;
           }
        }

      if (Encoderside==enc_leftside)
          { EncLeft_menu_idx=menu_idx; //limit the menu
               }

     //limit the changes of the rightside encoder for specific functions
      if ((EncLeft_menu_idx!=MENU_SR) )
        if (Encoderside==enc_rightside)
          { EncRight_menu_idx=menu_idx; //limit the menu
               }
    }

  //encoder is in valuemode and has changed position
  if ((encodermode==enc_value) and (change!=0))
    {
      /******************************VOLUME  ***************/
      if (menu_idx==MENU_VOL)
        { volume+=change;
          volume=constrain(volume,0,90);
          float V=volume*0.01;
          AudioNoInterrupts();
          sgtl5000.volume(V);
          AudioInterrupts();
        }
      /******************************MIC_GAIN  ***************/
      if (menu_idx==MENU_MIC)
        {
         mic_gain+=change;
         mic_gain=constrain(mic_gain,0,63);
         set_mic_gain(mic_gain);
         FFTcount=0; //start denoise after changing gain
         //reset FFTdenoise array
         {for (int16_t i = 0; i < 128; i++) {
           FFTavg[i]=0;
         }}
        }
      /******************************FREQUENCY  ***************/
      if (menu_idx==MENU_FRQ)
         { int delta=500;
           uint32_t currentmillis=millis();
           //when turning the encoder faster make the changes larger
           if ((currentmillis-lastmillis)<500)
              { delta=1000;}
           if ((currentmillis-lastmillis)<250)
              { delta=2000;}
           if ((currentmillis-lastmillis)<100)
              { delta=5000;}

          freq_real=freq_real+delta*change;
          // limit the frequency to 500hz steps
          freq_real=constrain(freq_real,7000,int(sample_rate_real/2000)*1000-1000);
          set_freq_Oscillator (freq_real);
          lastmillis=millis();
         }
      /******************************DENOISE  ***************/
      if (menu_idx==MENU_DNS)
        { // setting FFTcount to 0 re-activates a 1000 sample denoise
          FFTcount=0;
        }

      if (menu_idx==MENU_SPECTRUMMODE)
        { 
           spectrum_mode+=change;
           spectrum_mode=spectrum_mode%3; // 0..2 0=full (2pix), 1=25-90 (4pix), 2=25-70 (6pix)
                   
        }


      /******************************LBUTTON SET MODE  ***************/
      if (menu_idx==MENU_BUTTONL)
      {
        uint8_t currentmode=LeftButton_Next;
        currentmode+=1;
        LeftButton_Next=currentmode%MODE_MAX; //truncate
        
      }
      /******************************TIME  ***************/
      if (EncLeft_menu_idx==MENU_TIME)
      {  struct tm tx = seconds2tm(RTC_TSR);
         
         if (Encoderside==enc_leftside)
            {
              int delta=change*1; //seconds
               uint32_t currentmillis=millis();
           if ((currentmillis-lastmillis)<500)
              { delta=change*5;} //5 seconds
           if ((currentmillis-lastmillis)<250)
              { delta=change*10;} //10 seconds
           if ((currentmillis-lastmillis)<100)
              { delta=change*60;} //1 minutes 

              time_t time_tst = now()+delta;
              
              tmElements_t tmtm;
              breakTime(time_tst, tmtm);
              Teensy3Clock.set(makeTime(tmtm));
              setTime(makeTime(tmtm));

              tft.setCursor(80,50);
              tft.fillRect(80,50,ILI9341_TFTWIDTH-80,20, COLOR_BLACK);
    
              tft.setFont(Arial_24);
              char tstr[9];
              struct tm tx = seconds2tm(RTC_TSR);
              snprintf(tstr,9, "%02d:%02d:%02d", tx.tm_hour, tx.tm_min, tx.tm_sec);
              tft.print(tstr);
              lastmillis=millis();
            }
        //  if (Encoderside==enc_rightside)
        //     {
        //       int minu=tx.tm_min;
        //       minu=minu+change;
        //       minu=constrain(minu,0,59);

        //     }


      }  
      /******************************DISPLAY  ***************/

      // if (menu_idx==MENU_DSP)
      //    {
      //      displaychoice+=change;
      //      displaychoice=displaychoice%3; //limit to 0(none),1(spectrum),2(waterfall)
      //      if (displaychoice==waterfallgraph)
      //         {  tft.setRotation( 0 );
      //         }
      //      if (displaychoice==spectrumgraph)
      //        {  tft.setScroll(0);
      //           tft.setRotation( 0 );
      //         }
      //       tft.fillScreen(COLOR_BLACK); //blank the screen
      //   }


/************** SPECIAL MODES WHERE THE LEFTENCODER SETS A FUNCTION AND THE RIGHT ENCODER SELECTS */

      /******************************SAMPLE_RATE  ***************/
      if (EncLeft_menu_idx==MENU_SR)  //only selects a possible sample_rate, user needs to press a button to SET sample_rate
        { sample_rate+=EncRightchange;
          sample_rate=constrain(sample_rate,SAMPLE_RATE_MIN,SAMPLE_RATE_MAX);
        }


      /******************************SELECT A FILE  ***************/
      if ((EncLeft_menu_idx==MENU_PLY) and (EncRight_menu_idx==MENU_PLY) and (EncRight_function==enc_value))//menu play selected on the left and right
         { if (LeftButton_Mode!=MODE_PLAY)
            { fileselect+=EncRightchange;
              fileselect=constrain(fileselect,0,filecounter-1);
            }
         }

      /******************************CHANGE SR during PLAY  ***************/
      if ((EncLeft_menu_idx==MENU_PLY) and (EncRight_menu_idx==MENU_SR) and (EncRight_function==enc_value))//menu play selected on the left and right
          {if (LeftButton_Mode==MODE_PLAY)
              {  sample_rate+=EncRightchange;
                 sample_rate=constrain(sample_rate,SAMPLE_RATE_8K,SAMPLE_RATE_44K);
                 set_sample_rate(sample_rate);
              }
        }
    }
 }
// **************************  ENCODERS
void updateEncoders()
{
//only react to changes large enough (depending on the steps of the encoder for one rotation)
 long EncRightnewPos = EncRight.read()/4;
 if (EncRightnewPos>EncRightPos)
   { EncRightchange=enc_up; } // up
   else
   if (EncRightnewPos<EncRightPos)
    { EncRightchange=enc_dn; } // down
   else
    { EncRightchange=enc_nc; } //no change =0

 if (EncRightchange!=0)
    {updateEncoder(enc_rightside);
     }

 EncRightPos=EncRightnewPos;

 long EncLeftnewPos = EncLeft.read()/4;
 if (EncLeftnewPos>EncLeftPos)
   { EncLeftchange=enc_up; }
   else
   if (EncLeftnewPos<EncLeftPos)
   { EncLeftchange=enc_dn; }
   else
   { EncLeftchange=enc_nc; }

 if (EncLeftchange!=0)
    {updateEncoder(enc_leftside);
    }

 EncLeftPos=EncLeftnewPos;

 //update display only if a change has happened to at least one encoder
 if ((EncRightchange!=0) or (EncLeftchange!=0))
      display_settings();

}
// **************************  BUTTONS
void updateButtons()
{// Respond to button presses
 // try to make the interrupts as short as possible when recording

 if (LeftButton_Mode==MODE_REC)
   {  encoderButton_L.update(); //check the left encoderbutton
      if ((encoderButton_L.risingEdge())  )
       { stopRecording();
          EncLeft_function=enc_menu; //force into active-menu
          display_settings();
       }
   }
 else // ************** NORMAL BUTTON PROCESSING

  {
    encoderButton_L.update();
    encoderButton_R.update();

    micropushButton_L.update();
    micropushButton_R.update();

  //rightbutton is dedicated to detectormode
   if (micropushButton_R.risingEdge()) {
        detector_mode++;
        if (detector_mode>detector_passive)
          {detector_mode=0;}
        changeDetector_mode();
        display_settings();
    }
   //leftbutton can change function based on leftbutton_mode)
    if (micropushButton_L.risingEdge()) {
        if (LeftButton_Mode==MODE_DISPLAY) 
          {
           displaychoice+=1;
           displaychoice=displaychoice%3; //limit to 0(none),1(spectrum),2(waterfall)
           if (displaychoice==waterfallgraph)
              {  tft.setRotation( 0 );
              }
           if (displaychoice==spectrumgraph)
             {  tft.setScroll(0);
                tft.setRotation( 0 );
              }
             tft.fillScreen(COLOR_BLACK); //blank the screen
            }
      //no function yet
      display_settings();
    }

    /************  LEFT ENCODER BUTTON *******************/
    if (encoderButton_L.risingEdge())
    {
      EncLeft_function=!EncLeft_function;
      if ((EncLeft_menu_idx==MENU_BUTTONL))
          LeftButton_Mode=LeftButton_Next;

      //*SPECIAL MODES that change rightside Encoder based on leftside Encoder menusetting
      //****************************************SAMPLERATE
      if ((EncLeft_menu_idx==MENU_SR) and (EncLeft_function==enc_value))
       { EncRight_menu_idx=MENU_SR;
         EncRight_function=enc_value; // set the rightcontroller to select
       }

     if (SD_ACTIVE)
     {
        if ((LeftButton_Mode==MODE_PLAY) and ((EncLeft_menu_idx==MENU_PLY) or (EncLeft_menu_idx==MENU_PLD)))
                 { stopPlaying();
                   EncLeft_menu_idx=MENU_PLY;
                   EncLeft_function=enc_menu; //force into active-menu
                   continousPlay=false;
                 }
        //Direct play menu got choosen, now change the rightencoder to choose a file
        if ((EncLeft_menu_idx==MENU_PLD) and (EncLeft_function==enc_value))
         {
           EncRight_menu_idx=MENU_FRQ ;
           EncRight_function=enc_value; // set the rightcontroller to select
         }
        //play menu got choosen, now change the rightencoder to choose a file
        if ((EncLeft_menu_idx==MENU_PLY) and (EncLeft_function==enc_value))
         {
           EncRight_menu_idx=MENU_PLY ;
           EncRight_function=enc_value; // set the rightcontroller to select
         }
     } //END SD_ACTIVE
     display_settings();
    }

    /************  RIGHT ENCODER BUTTON *******************/

    if (encoderButton_R.risingEdge())
    {
      EncRight_function=!EncRight_function; //switch between menu/value control

      //when EncLeftoder is SR menu than EncRightoder can directly be setting the samplerate at a press
      //the press should be a transfer from enc_value to enc_menu before it gets activated
      if ( (EncLeft_menu_idx==MENU_SR) and (EncRight_menu_idx==MENU_SR) and (EncRight_function==enc_menu))
      { set_sample_rate(sample_rate);
        last_sample_rate=sample_rate;
      }

      //recording and playing are only possible with an active SD card
      if (SD_ACTIVE)
       {
        if (EncLeft_menu_idx==MENU_REC)
              {
                if (LeftButton_Mode == MODE_DETECT)
                   startRecording();
                }
                else
                  if (LeftButton_Mode==MODE_PLAY)
                    {
                        if (not continousPlay)
                          { stopPlaying();
                            EncLeft_menu_idx=MENU_PLY;
                            EncLeft_function=enc_menu;
                          }
                    }
                  else
                  if (LeftButton_Mode==MODE_DETECT)
                    { if (EncLeft_menu_idx==MENU_PLY)
                        { last_sample_rate=sample_rate;
                          startPlaying(SAMPLE_RATE_8K);
                        }
                      if (EncLeft_menu_idx==MENU_PLD)
                        {  fileselect=referencefile;
                            continousPlay=true;
                            last_sample_rate=sample_rate;
                            startPlaying(SAMPLE_RATE_281K);
                        }
                    }
       }

      display_settings();
    }

  } // ************** END NORMAL BUTTON PROCESSING

}
// **************************  END BUTTONS



//###########################################################################
//###########################################################################
//##########################   MAIN ROUTINE   ###############################
//###########################################################################
//###########################################################################

void setup() {
 #ifdef DEBUGSERIAL
  Serial.begin(115200);
 #endif
  delay(200);

//setup Encoder Buttonpins with pullups
  pinMode(encoderButton_RIGHT,INPUT_PULLUP);
  pinMode(encoderButton_LEFT,INPUT_PULLUP);

  pinMode(MICROPUSH_RIGHT,INPUT_PULLUP);
  pinMode(MICROPUSH_LEFT,INPUT_PULLUP);

  // startup menu
  EncLeft_menu_idx=MENU_VOL;
  EncRight_menu_idx=MENU_FRQ;
  EncLeft_function=enc_menu;
  EncRight_function=enc_menu;

  // Audio connections require memory.
  AudioMemory(300);

  setSyncProvider(getTeensy3Time);

// Enable the audio shield. select input. and enable output
  sgtl5000.enable();
  sgtl5000.inputSelect(myInput);
  sgtl5000.volume(0.45);
  sgtl5000.micGain (mic_gain);
  //sgtl5000.adcHighPassFilterDisable(); // does not help too much!
  sgtl5000.lineInLevel(0);
  mixFFT.gain(0,1);

// Init TFT display
#ifdef USETFT
  tft.begin();
  tft.setRotation( 0 );
  tft.fillScreen(COLOR_BLACK);
  tft.setCursor(0, 0);
  tft.setScrollarea(TOP_OFFSET,BOTTOM_OFFSET);
  display_settings();
  tft.setCursor(80,50);
  tft.setFont(Arial_24);
  char tstr[9];
  struct tm tx = seconds2tm(RTC_TSR);
  snprintf(tstr,9, "%02d:%02d:%02d", tx.tm_hour, tx.tm_min, tx.tm_sec);
  tft.print(tstr);
  delay(2000); //wait a second to clearly show the time
#endif

//Init SD card use
// uses the SD card slot of the Teensy, NOT that of the audio board !!!!!
#ifdef USESD1
  if(!(SD.begin(BUILTIN_SDCARD)))
  {
      #ifdef DEBUGSERIAL
          Serial.println("Unable to access the SD card");
          delay(500);
      #endif
    SD_ACTIVE=false;
    tft.fillCircle(70,50,5,COLOR_RED);
  }
  else  {
    SD_ACTIVE=true;
    tft.fillCircle(70,50,5,COLOR_GREEN);
    filecounter=0;
    root = SD.open("/");

    //TODO: check if file is a RAW file and also read the SAMPLERATE
    while (true) {
        File entry =  root.openNextFile();
        if (! entry) {
          // no more files
          tft.setCursor(0,50);
          tft.print(filecounter);
          break;
        }

        if (entry.isDirectory()) {
          // do nothing, only look for raw files in the root
        }
        else   {
          strcpy(filelist[filecounter],entry.name() );
          if (String(entry.name())=="TEST_281.RAW")
             {referencefile=filecounter;
              }
          filecounter++;
        }
        entry.close();
       }
    }

if (SD_ACTIVE)
// Recording on SD card by uSDFS library
  {f_mount (&fatfs, (TCHAR *)_T("0:/"), 0);      /* Mount/Unmount a logical drive */
   for (int i=0; i<filecounter; i++)
     {
       #ifdef DEBUGSERIAL
          #ifdef USETFT
            tft.setCursor(0,50+i*20);
            tft.print(filelist[i]);
          #endif
      #endif
     }
     file_number=filecounter+1;
  }
#endif

#ifdef USESD2
 uSD.init();
#endif

// ***************** SETUP AUDIO *******************************
set_sample_rate (sample_rate);
set_freq_Oscillator (freq_real);
inputMixer.gain(0,1); //microphone active
inputMixer.gain(1,0); //player off

outputMixer.gain(0,1); // heterodyne1 to output
outputMixer.gain(1,0); // granular to output off
outputMixer.gain(2,0); // player to output off

// the Granular effect requires memory to operate
granular1.begin(granularMemory, GRANULAR_MEMORY_SIZE);

// reset the FFT denoising array at startup
for (int16_t i = 0; i < 128; i++) {
  FFTavg[i]=0;
    }
} // END SETUP

//start the processing loop !
void loop()
{
// If we're playing or recording, carry on...
  if (LeftButton_Mode == MODE_REC) {
    continueRecording();
  }

  if (LeftButton_Mode == MODE_PLAY) {
    continuePlaying();
  }

  updateButtons();

  // during recording screens are not updated to reduce interference !
  if (LeftButton_Mode!=MODE_REC)
  { updateEncoders();
    #ifdef USETFT
    updatedisplay();
    // if (displaychoice==waterfallgraph)
    //   { waterfall();
    //    }
    // else
    //   if (displaychoice==spectrumgraph)
    //     {  spectrum();
    //     }
    #endif
   }
}
