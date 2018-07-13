
/***********************************************************************
 *  (c) 2016  
 *      
 *  https://forum.pjrc.com/threads/38988-Bat-detector
 *         
 *        made possible by the samplerate code by Frank Boesing, thanks Frank!
 * 
 *  tested on Teensy 3.5 + Teensy Audio board 
 *  + standard tiny electret MIC soldered to the MIC Input of the audio board 
 *  
 *  needs the new version of the SD library by Paul Stoffregen (newer than 2016_11_01)
 *  
 *  uses the SD card slot of the Teensy, NOT the SD card slot of the audio board  
 *  use an old SD card, preferably 2GB, no HC or class 10 
 *        en
 * Audio sample rate code - function setI2SFreq  
 * Copyright (c) 2016, Frank Bösing, f.boesing@gmx.de
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
 * NEW:
 * - uses the uSDFS lib for recording and SD lib for playback
 * - time expansion playback enabled (new sample rates 8k to 32k implemented): 
 *     Try recording a shaken keyring at 96k and playback at 16k, sounds like tons of heavy steel ;-)
 * - waterfall display by Frank B
 * 
 * TODO: 
 * - fix ticking noise issue with "newer" SD cards
 * - implement proper menu for time expansion playback (just choose the time expansion factor)
 * - file names with sample rate indicator
 * - display file names for playback
 * - menu to choose which file to playback
 * - realtime frequency translation of the whole spectrum (no idea how to do this efficiently: maybe just by downsampling???)
 * - activate realtime clock for accurate time/date stamps on recordings
 * - proper documentation/manual ?
 */

/* additions CORBEE */
/* TEENSY 3.6 PINSETUP

                  GND                 Vin  - PREAMP MIC
                   0                   Analog GND
                   1                   3.3V (250mA0
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
       TFTTouchCS 26                  39  TFT MISO
        TFT SCLK  27                  38
        TFT MOSI  28                  37
     ENC_R-BUTTON 29                  36  ENC_L-BUTTON
     ENC_R A      30                  35  ENC_L A
     ENC_R B      31                  34  ENC_L B
                  32                  33

*/

//#define DEBUG
#define USESD
#define USETFT

//#include <Time.h>
#include <TimeLib.h>

#include "Audio.h"
//#include <Wire.h>
#include <SPI.h>
#include <Bounce.h>
#include <Metro.h>

boolean SD_ACTIVE=false;
boolean continousPlay=false;
boolean batTrigger=false;
boolean TE_ready=true; //when a TEcall is played this signals the end of the call

#ifdef USESD
 #include <SD.h>
 #include "ff.h"       // uSDFS lib
 #include "ff_utils.h" // uSDFS lib
 File root;
 SdFile filer;
 #define MAX_FILES    50
 #define MAX_FILE_LENGTH  13   // 8 chars plus 4 for.RAW plus NULL
 char filelist[ MAX_FILES ][ MAX_FILE_LENGTH ];
 int filecounter=0;
 int fileselect=0;
 int referencefile=0;

#endif

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
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


uint32_t lastmicros;

#ifdef USETFT

 #define ILI9341
 #ifdef ILI9341
  #include "ILI9341_t3.h"
  #include "font_Arial.h"
  //#include "XPT2046_Touchscreen.h"

  #define BACKLIGHT_PIN 255
  #define TOP_OFFSET 90
  #define BOTTOM_OFFSET 20

  #define TFT_DC      20
  #define TFT_CS      21
  #define TFT_RST     255  // 255 = unused. connect to 3.3V

  #define TFT_MOSI    28
  #define TFT_SCLK    27 
  #define TFT_MISO    39
  //#define Touch_CS    8

  ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
  //XPT2046_Touchscreen ts = XPT2046_Touchscreen(Touch_CS);
 #endif

#endif



// this audio comes from the codec by I2S2
AudioInputI2S                    i2s_in; // MIC input
AudioRecordQueue                 recorder; 
AudioSynthWaveformSineHires      sine1; // local oscillator
//AudioSynthWaveformSineHires      sine2; // local oscillator
//
AudioEffectMultiply              heterodyne_multiplier; // multiply = mix
//AudioEffectMultiply              mult2; // multiply = mix

//AudioAnalyzeFFT1024         fft1024_1; // for waterfall display
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
//AudioConnection input_toheterodyne2 (granular1, 0, mult2, 0); //heterodyne 2
//AudioConnection sineheterodyne2     (sine2, 0, mult2, 1);//heterodyne 2 mixerfreq

AudioConnection heterodyne1_toout      (heterodyne_multiplier, 0, outputMixer, 0);  //heterodyne 1 output to outputmixer
//AudioConnection heterodyne2_toout      (mult2, 0, outputMixer, 1);  //heterodyne 2 output to outputmixer
AudioConnection player_toout           (inputMixer,0, outputMixer, 2);    //direct signal (use with player) to outputmixer

AudioConnection output_toheadphoneleft      (outputMixer, 0, i2s_out, 0); // output to headphone
AudioConnection output_toheadphoneright     (outputMixer, 0, i2s_out, 1);
//AudioConnection granular_toheadphone        (granular1,0,i2s_out,1);

AudioControlSGTL5000        sgtl5000_1;  

//const int myInput = AUDIO_INPUT_LINEIN;
const int myInput = AUDIO_INPUT_MIC;

#define GRANULAR_MEMORY_SIZE 30000  // enough for 100 ms at 281kHz
int16_t granularMemory[GRANULAR_MEMORY_SIZE];

// forward declaration Stop recording with dying message 
#ifdef DEBUG
void die(char *str, FRESULT rc);
#endif

extern "C" uint32_t usd_getError(void);

struct tm seconds2tm(uint32_t tt);

// Metro 1 second

Metro secondo = Metro(1000);
elapsedMillis since_bat_detection1;
elapsedMillis since_bat_detection2;
elapsedMillis since_heterodyne=1000;
uint16_t callLength=0;

/************** RECORDING PLAYING SETTINGS *****************/

const int8_t    MODE_DETECT = 0;
const int8_t    MODE_REC = 1;
const int8_t    MODE_PLAY = 2;

int mode = MODE_DETECT; 

//File frec; // audio is recorded to this file first
int file_number = 0;
FRESULT rc;        /* Result code */
FATFS fatfs;      /* File system object */
FIL fil;        /* File object */

//#define MXFN 100 // maximal number of files 
#if defined(__MK20DX256__)
  #define BUFFSIZE (8*1024) // size of buffer to be written
#elif defined(__MK66FX1M0__)
  #define BUFFSIZE (32*1024) // size of buffer to be written
#endif

uint8_t buffern[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
uint8_t buffern2[BUFFSIZE] __attribute__( ( aligned ( 16 ) ) );
UINT wr;
uint32_t nj = 0;

//int count=0;
int count_help = 0;
int8_t waterfall_flag = 1;
int8_t spectrum_flag = 0;
int idx_t = 0;
int idx = 0;
int64_t sum;
float32_t mean;
int16_t FFT_bin [128]; 
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

typedef struct SR_Descriptor
{
    const int SR_n;
    const char* const f1;
    const char* const f2;
    const char* const f3;
    const char* const f4;
    const float32_t x_factor;
} SR_Desc;

// SRtext and position for the FFT spectrum display scale
const SR_Descriptor SR [SAMPLE_RATE_MAX + 1] =
{
    //   SR_n ,  f1, f2, f3, f4, x_factor = pixels per f1 kHz in spectrum display
    {  SAMPLE_RATE_8K,  "8", " 2", " 3", " 4", 64.0}, // which means 64 pixels per 1 kHz
    {  SAMPLE_RATE_11K,  "11", " 2", " 3", " 4", 43.1}, 
    {  SAMPLE_RATE_16K,  "16", " 4", " 6", " 8", 64.0}, 
    {  SAMPLE_RATE_22K,  "22", " 4", " 6", " 8", 43.1}, 
    {  SAMPLE_RATE_32K,  "32", "10", "15", "20", 80.0}, 
    {  SAMPLE_RATE_44K,  "44", "10", "15", "20", 58.05}, 
    {  SAMPLE_RATE_48K,  "48", "10", "15", "20", 53.33},
    {  SAMPLE_RATE_88K,  "88", "20", "30", "40", 58.05},
    {  SAMPLE_RATE_96K,  "96", "20", "30", "40", 53.33},
    {  SAMPLE_RATE_176K,  "176", "40", "60", "80", 58.05},
    {  SAMPLE_RATE_192K,  "192", "40", "60", "80", 53.33}, // which means 53.33 pixels per 20kHz
    {  SAMPLE_RATE_234K,  "234", "40", "60", "80", 53.33}, // which means 53.33 pixels per 20kHz
    {  SAMPLE_RATE_281K,  "281", "40", "60", "80", 53.33}, // which means 53.33 pixels per 20kHz
    {  SAMPLE_RATE_352K,  "352", "40", "60", "80", 53.33}
};    

// setup for FFTgraph denoising 
uint32_t FFTcount=0; //count the # of FFTs done 
uint16_t powerspectrumCounter=0;

float FFTavg[128];

float FFTpowerspectrum[128];
float powerspectrum_Max=0;

// defaults at startup functions
int displaychoice=0; //default display
int8_t mic_gain = 35; // start detecting with this MIC_GAIN in dB
int8_t volume=50;

int freq_real = 45000; // start detecting at this frequency
int freq_real_backup=freq_real; //used to return to proper settingafter using the play_function

// initial sampling setup
int sample_rate = SAMPLE_RATE_281K;
int sample_rate_real = 281000;
char * SRtext="281k";

int last_sample_rate=sample_rate;

float freq_Oscillator =50000;

/************************************************* MENU ********************************/
/***************************************************************************************/

typedef struct Menu_Descriptor
{
    const char* name;
    // ********preset variables below NOT USED YET
    const int len; // length of string to allow right-alignment
    const int def; //default settings
    const int low; // low threshold
    const int high; //high threshold
    
} Menu_Desc;


const int Leftchoices=10; //can have any value
const int Rightchoices=10;
const Menu_Descriptor MenuEntry [Leftchoices] =
{  {"Volume",6,60,0,100}, //divide by 100
   {"Gain",4,30,0,63},
   {"Frequency",9,45,20,90}, //multiply 1000
   {"Display",7,0,0,0},
   {"Denoise",7,0,0,0},
   {"Detectmode",1,0,0},
   {"Record",6,0,0,0}, //functions where the LeftEncoder 
   {"Play",4,0,0,0},
   {"SampleR",6,0,0,0},
   {"PlayD",5,0,0,0},

} ;

const int8_t    MENU_VOL = 0; //volume
const int8_t    MENU_MIC = 1; //mic_gain
const int8_t    MENU_FRQ = 2; //frequency
const int8_t    MENU_DSP = 3; //display
const int8_t    MENU_DNS = 4; //denoise
const int8_t    MENU_DTM = 5; //detectormode
const int8_t    MENU_REC = 6; //record
const int8_t    MENU_PLY = 7; //play 
const int8_t    MENU_SR  = 8; //sample rate
const int8_t    MENU_PLD = 9; //play at original rate 


const int detector_heterodyne=0;
const int detector_divider=1;
const int detector_Auto_heterodyne=2;
const int detector_Auto_TE=3;
const int detector_passive=4;

int detector_mode=detector_heterodyne;  


//************************* ENCODER variables/constants
const int8_t enc_menu=0; //encoder sets menu
const int8_t enc_value=1; //encoder sets value

const int8_t enc_leftside=0; //encoder 
const int8_t enc_rightside=1; //encoder

const int8_t enc_up=1;
const int8_t enc_nc=0;
const int8_t enc_dn=-1;

int EncLeft_menu_idx=0;
int EncRight_menu_idx=0;

int EncLeft_function=0;
int EncRight_function=0;

/************************** */


// ******** LEFT AND RIGHT ENCODER CONNECTIONS
#include <Encoder.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS

#define BUTTON_RIGHT      29    
Bounce button_R = Bounce(BUTTON_RIGHT, 50); 
Encoder EncRight(30,31);
int EncRightPos=0;
int EncRightchange=0;

#define BUTTON_LEFT       36
Bounce button_L = Bounce(BUTTON_LEFT, 50); 
Encoder EncLeft(34,35);
//start encoders far away from the 0 position to avoid negative numbers
int EncLeftPos=0;
int EncLeftchange=0;

// **END************ LEFT AND RIGHT ENCODER DEFINITIONS

void die(char *str, FRESULT rc) 
{
  #ifdef DEBUG 
   Serial.printf("%s: Failed with rc=%u.\n", str, rc); for (;;) delay(100); 
  #endif 
   }

//=========================================================================
//uint32_t count=0;
uint32_t ifn=0;
uint32_t isFileOpen=0;
char filename[80];
TCHAR wfilename[80];
uint32_t t0=0;
uint32_t t1=0;


void display_settings() {
  #ifdef USETFT
    
    tft.setTextColor(COLOR_WHITE);
    
    tft.setFont(Arial_16);
    tft.fillRect(0,0,240,TOP_OFFSET,COLOR_BLACK);
    tft.fillRect(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET,240,BOTTOM_OFFSET,COLOR_DARKBLUE);

    tft.setCursor(0,0);
    tft.print("g:"); tft.print(mic_gain);
    tft.print(" f:"); tft.print(freq_real);
    tft.print(" v:"); tft.print(volume);
    tft.print(" s"); tft.print(SRtext);
    tft.setCursor(0,20);
    
    switch (detector_mode) {
       case detector_heterodyne:
         tft.print("HTD"); // 
       break;
       case detector_divider:
         tft.print("FD");
       break;
       case detector_Auto_heterodyne:
         tft.print("Auto_HTD");
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
     tft.setCursor(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET);

     // set the colors 
     if (mode==MODE_DETECT ) 
     // show menu selection as menu-active (green) or value-active (yellow)
      {if (EncLeft_function==enc_value) 
       { tft.setTextColor(COLOR_YELLOW);
        }
        else
       { tft.setTextColor(COLOR_GREEN);}

       //tft.print(EncLeftchange); // Left alignment is OK
       tft.print(MenuEntry[EncLeft_menu_idx].name);
       tft.print(" "); 

       if (EncRight_function==enc_value) 
         { tft.setTextColor(COLOR_YELLOW);} //value is active yellow
       else
         { tft.setTextColor(COLOR_GREEN);} //menu is active green
       
       //if MENU on the left-side is PLAY and selected than show the filename
        if ((EncLeft_menu_idx==MENU_PLY) and (EncLeft_function==enc_value))     
           { //tft.print(fileselect); 
             tft.print(filelist[fileselect]);
            
           }
        else
         if (EncLeft_menu_idx==MENU_REC)      
          // show the filename that will be used forthe next recording
           {  sprintf(filename, "B%u_%s.raw", file_number+1,SRtext);
              tft.print(filename );
            }
         else
         if (EncLeft_menu_idx==MENU_SR)
          { tft.print(SR[sample_rate].f1);

          }
          else
          { //tft.print(EncRightchange); 
            tft.setCursor(ILI9341_TFTWIDTH/2  ,ILI9341_TFTHEIGHT-BOTTOM_OFFSET);
            tft.print(MenuEntry[EncRight_menu_idx].name);
          }
    }
    else
      { 
        if (mode==MODE_REC)
          { tft.setTextColor(COLOR_RED);
            tft.print("REC:"); 
            tft.setTextColor(COLOR_WHITE);
            tft.print(filename);
         }
        if (mode==MODE_PLAY) 
         {if (EncLeft_menu_idx==MENU_PLY)
          { tft.setTextColor(COLOR_GREEN);
            tft.print("PLAY:"); 
            tft.setTextColor(COLOR_WHITE);
            tft.print(filename);
          }
          else
           {tft.print(MenuEntry[EncLeft_menu_idx].name);
            tft.print(" "); 
            tft.print(MenuEntry[EncRight_menu_idx].name);
            

           }

        }
      }

    //scale every 10kHz  
    float x_factor=10000/(0.5*(sample_rate_real / FFT_points)); 
    int curF=2*int(freq_real/(sample_rate_real / FFT_points));

    int maxScale=int(sample_rate_real/20000);
    for (int i=1; i<maxScale; i++) 
     { tft.drawFastVLine(i*x_factor, TOP_OFFSET-10, 9, COLOR_YELLOW);  
     }    
    tft.fillCircle(curF,TOP_OFFSET-4,3,COLOR_YELLOW);
    
   #endif
}


void       set_mic_gain(int8_t gain) {
    
    AudioNoInterrupts();
    sgtl5000_1.micGain (gain);
    AudioInterrupts();
    display_settings();
    powerspectrum_Max=0; // change the powerspectrum_Max for the FFTpowerspectrum
} // end function set_mic_gain

void       set_freq_Oscillator(int freq) {
    // audio lib thinks we are still in 44118sps sample rate
    // therefore we have to scale the frequency of the local oscillator
    // in accordance with the REAL sample rate
      
    freq_Oscillator = (freq+0) * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real); 
    //float F_LO2= (freq+5000) * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real); 
    // if we switch to LOWER samples rates, make sure the running LO 
    // frequency is allowed ( < 22k) ! If not, adjust consequently, so that
    // LO freq never goes up 22k, also adjust the variable freq_real  
    if(freq_Oscillator > 22000) {
      freq_Oscillator = 22000;
      freq_real = freq_Oscillator * (sample_rate_real / AUDIO_SAMPLE_RATE_EXACT) + 9;
    }
    AudioNoInterrupts();
    //setup multiplier SINE
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

void prepare_spectrum_display() {
  #ifdef USETFT
     
    float x_factor=10000/(0.5*(sample_rate_real / FFT_points)); 
    
    int maxScale=int(sample_rate_real/20000);
    for (int i=1; i<maxScale; i++) 
     { tft.drawFastVLine(i*x_factor, 40, 6, COLOR_YELLOW);  
     }    
    
  #endif
  //  tft.setFont(Arial_14);
} // END prepare_spectrum_display

void      set_sample_rate (int sr) {
  switch (sr) {
    case SAMPLE_RATE_8K:
    sample_rate_real = 8000;
    SRtext = " 8k";
    break;
    case SAMPLE_RATE_11K:
    sample_rate_real = 11025;
    SRtext = "11k";
    break;
    case SAMPLE_RATE_16K:
    sample_rate_real = 16000;
    SRtext = "16k";
    break;
    case SAMPLE_RATE_22K:
    sample_rate_real = 22050;
    SRtext = "22k";
    break;
    case SAMPLE_RATE_32K:
    sample_rate_real = 32000;
    SRtext = "32k";
    break;
    case SAMPLE_RATE_44K:
    sample_rate_real = 44100;
    SRtext = "44.1k";
    break;
    case SAMPLE_RATE_48K:
    sample_rate_real = 48000;
    SRtext = "48k";
    break;
    case SAMPLE_RATE_88K:
    sample_rate_real = 88200;
    SRtext = "88.2k";
    break;
    case SAMPLE_RATE_96K:
    sample_rate_real = 96000;
    SRtext = "96k";
    break;
    case SAMPLE_RATE_176K:
    sample_rate_real = 176400;
    SRtext = "176k";
    break;
    case SAMPLE_RATE_192K:
    sample_rate_real = 192000;
    SRtext = "192k";
    break;
    case SAMPLE_RATE_234K:
    sample_rate_real = 234000;
    SRtext = "234k";
    break;
    case SAMPLE_RATE_281K:
    sample_rate_real = 281000;
    SRtext = "281k";
    break;
    case SAMPLE_RATE_352K:
    sample_rate_real = 352800;
    SRtext = "352k";
    break;
  }
    
    AudioNoInterrupts();
    setI2SFreq (sample_rate_real); 
    delay(200); // this delay seems to be very essential !
    set_freq_Oscillator (freq_real);
    AudioInterrupts();
    delay(20);
    display_settings();
   
} // END function set_sample_rate



void spectrum() { // spectrum analyser code by rheslip - modified
     #ifdef USETFT
     if (myFFT.available()) {
//     if (fft1024_1.available()) {
    int16_t peak=0; uint16_t avgF=0;

    // find the BIN corresponding to the current frequency-setting 
    int curF=int(freq_real/(sample_rate_real / FFT_points));

//    for (int i = 0; i < 240; i++) {
    //startup sequence to denoise the FFT
    FFTcount++;
    if (FFTcount==1)
     {for (int16_t i = 0; i < 128; i++) {
         FFTavg[i]=0; 
     }
     }

    if (FFTcount<1000)
     { 
       for (int i = 0; i < 128; i++) {
         FFTavg[i]=FFTavg[i]+abs(myFFT.output[i])*0.001; //0.1% of total values
         }
     }

for (int16_t x = 2; x < 128; x++) {
   avgF=avgF+FFT_bin[x];
   if (FFT_bin[x]>peak)
      {
        peak=FFT_bin[x];   
      }
}

/*
avgF=avgF/128;
//check if the peak is at least 2x higher than the average otherwise set the indicator low
if ((peak-avgF)<(avgF/3))
  { maxF=2;}
  */  
  for (int16_t x = 2; x < 128; x++) {
//  for (uint16_t x = 8; x < 512; x+=4) {
     FFT_bin[x] = (myFFT.output[x])-FFTavg[x]*0.9; 
     int colF=COLOR_WHITE;
     
//     FFT_bin[x/4] = abs(fft1024_1.output[x]); 
     int bar = (FFT_bin[x]) ;
     
     // this is a very simple first order IIR filter to smooth the reaction of the bars
     bar = 0.05 * bar + 0.95 * barm[x]; 
     if (bar >240) bar=240;
     if (bar <0) bar=0;
     
     int g_x=x*2;
     tft.drawFastVLine(g_x,TOP_OFFSET,240-barm[x], COLOR_BLACK);
    /* if (x==maxF)
       { colF=COLOR_ORANGE;
         tft.drawFastVLine(g_x,TOP_OFFSET,240-bar, colF);
         }
      */   
     if (x==curF)
     { colF=COLOR_ORANGE;
         tft.drawFastVLine(g_x,TOP_OFFSET,240-bar, colF);
         }
     tft.drawPixel(g_x,240- bar,colF);

     barm[x] = bar;
  }
  
    // if (mode == MODE_DETECT)  search_bats();     
  } //end if
  if (mode==MODE_PLAY)
    {//float ww=( player.positionMillis()/player.lengthMillis()*240.0);
     tft.drawFastHLine(0,320-BOTTOM_OFFSET-5,240*player.positionMillis()/player.lengthMillis()-10,COLOR_BLACK);
     tft.drawFastHLine(240*player.positionMillis()/player.lengthMillis()-9,320-BOTTOM_OFFSET-5,5,COLOR_YELLOW);
    }
  #endif
} // end void spectrum

#if DEBUG
void check_processor() {
      if (second.check() == 1) {
      Serial.print("Proc = ");
      Serial.print(AudioProcessorUsage());
      Serial.print(" (");    
      Serial.print(AudioProcessorUsageMax());
      Serial.print("),  Mem = ");
      Serial.print(AudioMemoryUsage());
      Serial.print(" (");    
      Serial.print(AudioMemoryUsageMax());
      Serial.println(")");
 
      AudioProcessorUsageMaxReset();
      AudioMemoryUsageMaxReset();
    }
} // END function check_processor
#endif



void waterfall(void) // thanks to Frank B !
{ 
  
#ifdef USETFT

// code for 256 point FFT 
     
  if (myFFT.available()) {
  const uint16_t Y_OFFSET = TOP_OFFSET;
  static int count = TOP_OFFSET;
  //int curF=int(freq_real/(sample_rate_real / FFT_points));

  // lowest frequencybin to detect as a batcall
  int batCall_LoF_bin= int(30000/(sample_rate_real / FFT_points));
  int batCall_HiF_bin= int(100000/(sample_rate_real / FFT_points));

  uint16_t lbuf[240]; // maximum of 240 vertical stripes, each one is the result of one FFT 
  lbuf[0]=0; lbuf[1]=0;  lbuf[2]=0; lbuf[3]=0;
  
  if (millis()%250<20) // show every 250ms a small visible timeblock
    {lbuf[1]=2000;
     lbuf[2]=2000;
     lbuf[3]=2000;
     }

     FFTcount++;

    //start with a clean FFTavg array to denoise
    if (FFTcount==1)
     {for (int16_t i = 0; i < 128; i++) {
        FFTavg[i]=0; 
     }
     }

    // collect 1000 FFT samples for the denoise array
    if (FFTcount<1000)
     { for (int i = 2; i < 128; i++) {
         //FFTavg[i]=FFTavg[i]+myFFT.read(i)*65536.0*5*0.001; //0.1% of total values
         FFTavg[i]=FFTavg[i]+myFFT.output[i]*10*0.001; //0.1% of total values
         }
     }
    

    int FFT_peakF_bin=0; 
    int peak=512;
    
    // there are 128 FFT different bins only 120 are shown on the graphs  
    for (int i = 2; i < 120; i++) { 
      int val = myFFT.output[i]*10 -FFTavg[i]*0.9 + 10; //v1
      //detect the peakfrequency
      if (val>peak)
       { peak=val; 
         FFT_peakF_bin=i;
        }
       if (val<5) 
           {val=5;}
       lbuf[i*2] = tft.color565(
              (val/8>255)? 255 : val/8,
              min(255, val),
              ((255-val)>>1) <0? 0: (255-val)>>1 
             ); 
            
      lbuf[i*2+1]=lbuf[i*2];       
    }

  int powerSpectrum_Maxbin=0;
  if ((FFT_peakF_bin>batCall_LoF_bin) and (FFT_peakF_bin>batCall_LoF_bin))
  {
    //collect data for the powerspectrum 
    for (int i = 2; i < 120; i++)
     { 
       FFTpowerspectrum[i]+=myFFT.output[i];
        if (FFTpowerspectrum[i]>powerspectrum_Max) 
           { powerspectrum_Max=FFTpowerspectrum[i];
             powerSpectrum_Maxbin=i;
           }

     }
     powerspectrumCounter++;
  }
    // update display after every 100th FFT sample with a batcall
    if ((powerspectrumCounter>100)  )
       { powerspectrumCounter=0;
         //clear powerspectrumbox
         tft.fillRect(0,TOP_OFFSET-45,240,40, COLOR_BLACK);
         // keep a minimum maximumvalue to the powerspectrum, square the values
         //powerspectrum_Max=powerspectrum_Max*powerspectrum_Max;
         if (powerspectrum_Max<20000)
            {powerspectrum_Max=20000;}

         int binLo=2; int binHi=0;

         for (int i=2; i<120; i++)
          { //square the signal
            //FFTpowerspectrum[i]=FFTpowerspectrum[i]*FFTpowerspectrum[i];
            int ypos=FFTpowerspectrum[i]/powerspectrum_Max*40; 
            // first encounter of 1/20 of maximum
            if ((FFTpowerspectrum[i]>powerspectrum_Max/10) and (binLo==2))
                    {binLo=i;}
            
            // last encounterof 1/20 maximum
            if ((FFTpowerspectrum[i]>powerspectrum_Max/10))
                    {binHi=i;}
            
                                    
            tft.drawFastVLine(i*2,TOP_OFFSET-ypos-6,ypos,COLOR_GREEN);
            tft.drawFastVLine(i*2+1,TOP_OFFSET-ypos-6,ypos,COLOR_GREEN);
            FFTpowerspectrum[i]=0;
          }
         
         //tft.setCursor(0,TOP_OFFSET-45);
         //tft.print(powerspectrum_Max);
         if (powerspectrum_Max==20000)
          {binLo=0; binHi=0;

          }
         powerspectrum_Max=powerspectrum_Max*0.5; //lower the max after a graphupdate
         tft.setCursor(150,TOP_OFFSET-45);
         tft.setTextColor(COLOR_WHITE);
         tft.print(int(binLo*(sample_rate_real / FFT_points)/1000) );
         tft.print(" ");
         tft.setTextColor(COLOR_YELLOW);
         tft.print(int(powerSpectrum_Maxbin*(sample_rate_real / FFT_points)/1000) );
         tft.print(" ");
         tft.setTextColor(COLOR_WHITE);
         tft.print(int(binHi*(sample_rate_real / FFT_points)/1000) );

       }
      
    
    if ((FFT_peakF_bin>batCall_LoF_bin) and (FFT_peakF_bin<batCall_HiF_bin)) // we got a high-frequent signal peak
      { 
        // when a batcall is first discovered 
        if (not batTrigger) 
          { since_bat_detection1=0; //start of the call mark
            lbuf[5]=COLOR_WHITE; // mark the start on the screen
            lbuf[6]=COLOR_WHITE;
            lbuf[7]=COLOR_WHITE;
            
            if (detector_mode==detector_Auto_heterodyne)
               if (since_heterodyne>1000)
                {freq_real=int((FFT_peakF_bin*(sample_rate_real / FFT_points)/500))*500; 
                 set_freq_Oscillator(freq_real); 
                 since_heterodyne=0;
                 granular1.stop();
                }
            
            //restart the TimeExpansion only if the previous call was played
            if ((detector_mode==detector_Auto_TE) and (TE_ready))
             { granular1.stop();
               granular1.beginTimeExpansion(GRANULAR_MEMORY_SIZE);
               granular1.setSpeed(0.05);
               TE_ready=false;
             }
                      
          }
            
         batTrigger=true;
         
     }
   else // FFT_peakF_bin does not show a battcall anymore 
        { 
          if (batTrigger) //last sample was still triggering 
          { callLength=since_bat_detection1; // got a pause so store the time since the start of the call
            since_bat_detection2=0;
            //constrain(callLength,1,230);
            //for (int i=0; i<callLength; i++)
            //   {lbuf[i+10]=COLOR_YELLOW; }
            
          }
          batTrigger=false;
        }    
    // restart TimeExpansion recording a bit after the call has finished completely
    if (since_bat_detection2>(callLength*3))
      { //stop the time expansion
        //ready for a new call
        TE_ready=true;
        since_bat_detection2=0;
        
      }

    //if (since_bat_detection1<100) // keep running the screen for a maximum of 100ms after the recording of a call
    //  { batTrigger=true;
    //  }

    if (since_bat_detection1<200)
      { tft.writeRect( 0,count, 239,1, (uint16_t*) &lbuf); //show a line with spectrumdata
        tft.setScroll(320-count);
        count++;
      } 


    if (count >= 320-BOTTOM_OFFSET) count = Y_OFFSET;
    count_help = count;

  }
#endif

}

void startRecording() {
  mode = MODE_REC;
#ifdef DEBUG    
  Serial.print("startRecording");
#endif  
    // close file
    if(isFileOpen)
    {
      //close file
      rc = f_close(&fil);
      if (rc) die("close", rc);
      //
      isFileOpen=0;
    }
  
  if(!isFileOpen)
  {
  file_number++;
  //automated filename BA_S.raw where A=file_number and S shows samplerate. Has to fit 8 chars
  // so max is B999_192.raw
  sprintf(filename, "B%u_%s.raw", file_number, SRtext);
#ifdef DEBUG    
  Serial.println(filename);
#endif  
  char2tchar(filename, 80, wfilename);
  filecounter++;
  strcpy(filelist[filecounter],filename );
  rc = f_stat (wfilename, 0);
#ifdef DEBUG    
    Serial.printf("stat %d %x\n",rc,fil.obj.sclust);
 #endif   
  rc = f_open (&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);
#ifdef DEBUG    
    Serial.printf(" opened %d %x\n\r",rc,fil.obj.sclust);
#endif 
    // check if file is Good
    if(rc == FR_INT_ERR)
    { // only option is to close file
        rc = f_close(&fil);
        if(rc == FR_INVALID_OBJECT)
        { 
          #ifdef DEBUG  
          Serial.println("unlinking file");
          #endif
          rc = f_unlink(wfilename);
          if (rc) {
            die("unlink", rc);
          }
        }
        else
        {
          die("close", rc);
        }
    }
    // retry open file
    rc = f_open(&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);
    if(rc) { 
      die("open", rc);
    }
    isFileOpen=1;
  }
  //clear the screen completely
  tft.fillRect(0,0,240,320,COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setFont(Arial_28);
  tft.setCursor(0,100);
  tft.print("RECORDING");
  tft.setFont(Arial_16);
  
  display_settings();

  //switch off several circuits
  mixFFT.gain(0,0);
  outputMixer.gain(1,0);  //shutdown granular output      
  granular1.stop(); //stop granular
  recorder.begin();
}

void continueRecording() {
  const uint32_t N_BUFFER = 2;
  const uint32_t N_LOOPS = 64;
  // buffer size total = 256 * n_buffer * n_loops
  // queue: write n_buffer blocks * 256 bytes to buffer at a time; free queue buffer;
  // repeat n_loops times ( * n_buffer * 256 = total amount to write at one time)
  // then write to SD card

  if (recorder.available() >= N_BUFFER  ) {// one buffer = 256 (8bit)-bytes = block of 128 16-bit samples
    for (int i = 0; i < N_BUFFER; i++) {
    memcpy(buffern + i*256 + nj * 256 * N_BUFFER, recorder.readBuffer(), 256);
    recorder.freeBuffer();
//    Serial.println(en);
//    en++;
    } 
    if (nj >=  (N_LOOPS - 1)) {
      nj = 0;
    // copy to 2nd buffer
    // use arm_copy function??
//    arm_copy_q7(buffern, buffern2, N_BUFFER * 256 * N_LOOPS);
    for (int ii = 0; ii < BUFFSIZE; ii++) {
      buffern2[ii] = buffern [ii];
    }
    rc = f_write (&fil, buffern2, N_BUFFER * 256 * N_LOOPS, &wr);

//    rc = f_write (&fil, buffern, N_BUFFER * 256 * N_LOOPS, &wr);


//    Serial.println ("Writing");
       if (rc== FR_DISK_ERR) // IO error
     {  uint32_t usd_error = usd_getError();
     #ifdef DEBUG
        Serial.printf(" write FR_DISK_ERR : %x\n\r",usd_error);
     #endif   
        // only option is to close file
        // force closing file
     }
     else if(rc) die("write",rc);
    }
  nj ++;
  }

}

void stopRecording() {
#ifdef DEBUG  
  Serial.print("stopRecording");
#endif  
  recorder.end();
  if (mode == MODE_REC) {
    while (recorder.available() > 0) {
    rc = f_write (&fil, (byte*)recorder.readBuffer(), 256, &wr);
//      frec.write((byte*)recorder.readBuffer(), 256);
      recorder.freeBuffer();
    }
      //close file
      rc = f_close(&fil);
      if (rc) die("close", rc);
      //
      isFileOpen=0;
//    frec.close();
//    playfile = recfile;
  }
  
  mode = MODE_DETECT;
//  clearname();
#ifdef DEBUG  
  Serial.println (" Recording stopped!");
#endif 
  //switch on FFT
  tft.fillScreen(COLOR_BLACK);
  
  mixFFT.gain(0,1); // allow input signal to 
      
}

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
      freq_real_backup=freq_real;
  }
  //direct play is used to test functionalty based on previous recorded data
if (EncLeft_menu_idx==MENU_PLD) 
  {
      outputMixer.gain(2,0);  //dont send audio from player to output 
      outputMixer.gain(1,1);  //start granular output processing      
      outputMixer.gain(0,1);  //start heterodyne output processing
  }
  
  delay(100);
  
  SR=constrain(SR,SAMPLE_RATE_MIN,SAMPLE_RATE_MAX);
  set_sample_rate(SR);
  
  fileselect=constrain(fileselect,0,filecounter-1);
  strncpy(filename, filelist[fileselect],  13);
  
  waterfall_flag=1;
  spectrum_flag=0; 
  display_settings();

  player.play(filename);
  
  mode = MODE_PLAY;

}
  

void stopPlaying() {
  
#ifdef DEBUG      
  Serial.print("stopPlaying");
#endif  
  if (mode == MODE_PLAY) player.stop();
  mode = MODE_DETECT;
#ifdef DEBUG      
  Serial.println (" Playing stopped");
#endif  
  
  //restore last settings
  set_sample_rate(last_sample_rate);
if (EncLeft_menu_idx==MENU_PLY)
{
  freq_real=freq_real_backup;
  set_freq_Oscillator (freq_real);
}
  outputMixer.gain(2,0); //stop the direct line 
  outputMixer.gain(1,1); // granular output
  outputMixer.gain(0,1); // heterodyne output  

  inputMixer.gain(0,1); //switch on the mic-line
  inputMixer.gain(1,0); //switch off the playerline

}


void continuePlaying() {
  //the end of file was reached
  if (!player.isPlaying()) {
    stopPlaying();
    if (continousPlay) //keep playing until stopped by the user
      {
        
       startPlaying(SAMPLE_RATE_176K);
      }

    /*
    
    player.stop();
    mode = MODE_DETECT;
        EncRight_menu_idx=  EncRight_menu_idx_old;
        EncRight_function= EncRight_function_old;

      outputMixer.gain(2,0);
      outputMixer.gain(1,1);       
      outputMixer.gain(0,1);       
    Serial.println("End of recording");
    set_sample_rate(SAMPLE_RATE_281K);
    freq_real=freq_real_backup;
    set_freq_Oscillator (freq_real);

      inputMixer.gain(0,1); //switch off the mic-line as input
      inputMixer.gain(1,0); //switch on the playerline as input
    */
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


  /************************proces changes*************************/
  //encoder is in menumode
  if (encodermode==enc_menu)
    { menu_idx=menu_idx+change;
      if (menu_idx<0)
        {menu_idx=choices-1;}
      if (menu_idx>=choices)
        {menu_idx=0;}
        
      if (Encoderside==enc_leftside)
          { EncLeft_menu_idx=menu_idx; //limit the menu 
               }
     
     //limit the changes of the rightside encoder for specific functions
      if ((EncLeft_menu_idx!=MENU_SR) and (EncLeft_menu_idx!=MENU_DTM))      
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
          //if (volume>100) {volume=100;}
          //if (volume<0) {volume=0;}
          volume=constrain(volume,0,100);
          float V=volume*0.01;
          AudioNoInterrupts();
          sgtl5000_1.volume(V);
          AudioInterrupts();
        }
      /******************************MIC_GAIN  ***************/
      if (menu_idx==MENU_MIC)
        {
         mic_gain+=change;
         //if (mic_gain>63) {mic_gain=63;}
         //if (mic_gain<0) {mic_gain=0;}
         mic_gain=constrain(mic_gain,0,63);
         set_mic_gain(mic_gain);
         //AudioNoInterrupts();
         //sgtl5000_1.micGain (mic_gain);
         //AudioInterrupts();

         //reset FFTdenoise array
         {for (int16_t i = 0; i < 128; i++) {
           FFTavg[i]=0; 
         }}  
        }
      /******************************FREQUENCY  ***************/
      if (menu_idx==MENU_FRQ)
         { int delta=500;
           uint32_t curmicro=millis();
           //when turning the encoder fast make the change larger
           if ((curmicro-lastmicros)<500)
              { delta=1000;}
           if ((curmicro-lastmicros)<250)
              { delta=2000;}

          freq_real=freq_real+delta*change;
          // limit 
          /*if (freq_real>int(sample_rate_real/2000)*1000-1000)
             { freq_real=int(sample_rate_real/2000)*1000-1000;
               }
          if (freq_real<5000) { freq_real=5000;}     
          */
          freq_real=constrain(freq_real,5000,int(sample_rate_real/2000)*1000-1000);

           set_freq_Oscillator (freq_real);
           lastmicros=millis();
         }
      /******************************DENOISE  ***************/
      if (menu_idx==MENU_DNS)
        { // setting FFTcount to 0 activates a 1000 sample denoise
          FFTcount=0;
        }

      
      if (menu_idx==MENU_DSP)
         { 
           displaychoice+=change;
           displaychoice=displaychoice%3; //limit to 0(none),1(spectrum),2(waterfall)
           spectrum_flag=0; waterfall_flag=0;
           if (displaychoice==2) 
              {waterfall_flag=1; 
               tft.setRotation( 2 );
            }
           if (displaychoice==1) 
            {spectrum_flag=1; 
            tft.setScroll(0);
            tft.setRotation( 2 );
              }
            tft.fillScreen(COLOR_BLACK);
          }

      

/************** SPECIAL MODES WHERE THE LEFTENCODER SETS A FUNCTION AND THE RIGHT ENCODER SELECTS */
      
      /******************************SAMPLE_RATE  ***************/
      if (EncLeft_menu_idx==MENU_SR)  //only selects a possible sample_rate, user needs to press a button to SET sample_rate
        { sample_rate+=EncRightchange;
          sample_rate=constrain(sample_rate,SAMPLE_RATE_MIN,SAMPLE_RATE_MAX);

          /*if (sample_rate>SAMPLE_RATE_MAX)
                    {sample_rate=SAMPLE_RATE_MAX;}
          if (sample_rate<SAMPLE_RATE_MIN)
                    {sample_rate=SAMPLE_RATE_MIN;}
                    */
        }   

        /******************************SAMPLE_RATE  ***************/
      if (EncLeft_menu_idx==MENU_DTM)  //only selects a possible sample_rate, user needs to press a button to SET sample_rate
        { detector_mode+=EncRightchange;
          detector_mode=constrain(detector_mode,0,4);
          /*if (sample_rate>SAMPLE_RATE_MAX)
                    {sample_rate=SAMPLE_RATE_MAX;}
          if (sample_rate<SAMPLE_RATE_MIN)
                    {sample_rate=SAMPLE_RATE_MIN;}
                    */
        }  

      /******************************SELECT A FILE  ***************/
      if ((EncLeft_menu_idx==MENU_PLY) and (EncRight_menu_idx==MENU_PLY) and (EncRight_function==enc_value))//menu play selected on the left and right 
         {  
           if (mode!=MODE_PLAY)
            {
             fileselect+=EncRightchange; 
             fileselect=constrain(fileselect,0,filecounter-1);
            /*if (fileselect<0)
               { fileselect=filecounter-1;}
            if (fileselect>=filecounter)
               { fileselect=0;}*/
            }   
         }  

      /******************************CHANGE SR during PLAY  ***************/
      if ((EncLeft_menu_idx==MENU_PLY) and (EncRight_menu_idx==MENU_SR) and (EncRight_function==enc_value))//menu play selected on the left and right    
          {
           if (mode==MODE_PLAY)
              {
                 sample_rate+=EncRightchange;
                 sample_rate=constrain(sample_rate,SAMPLE_RATE_8K,SAMPLE_RATE_44K);
                 /*
                 if (sample_rate>SAMPLE_RATE_44K)
                    {sample_rate=SAMPLE_RATE_8K;}
                 if (sample_rate<SAMPLE_RATE_8K)
                    {sample_rate=SAMPLE_RATE_8K;}
                 */                 
                 set_sample_rate(sample_rate);
                 last_sample_rate=sample_rate;
              }
            

        }
          
     

    }

 }

void updateEncoders()
{

 long EncRightnewPos = EncRight.read()/4;
 if (EncRightnewPos>EncRightPos)
   { EncRightchange=enc_dn; }
   else
   if (EncRightnewPos<EncRightPos)
   { EncRightchange=enc_up; }
   else
   { EncRightchange=enc_nc; }

 if (EncRightchange!=0)
    {updateEncoder(enc_rightside);
     } 

 EncRightPos=EncRightnewPos;
 
 long EncLeftnewPos = EncLeft.read()/4;
 if (EncLeftnewPos>EncLeftPos)
   { EncLeftchange=enc_dn; }
   else
   if (EncLeftnewPos<EncLeftPos)
   { EncLeftchange=enc_up; }
   else
   { EncLeftchange=enc_nc; }

 if (EncLeftchange!=0)
    {updateEncoder(enc_leftside);
    } 
 
 EncLeftPos=EncLeftnewPos;
 if ((EncRightchange!=0) or (EncLeftchange!=0))
      display_settings();

}

void updateButtons()
{
 // Respond to button presses
  button_L.update();
  button_R.update();
 
 // try to make the interrupt as short as possible when recording
 if ( (mode==MODE_REC) and ((button_L.risingEdge()) or (button_R.risingEdge()) ))
    { stopRecording();
      EncLeft_function=enc_menu; //force into active-menu
      display_settings();      
    }
 else
  {
 
  /************  LEFT ENCODER BUTTON *******************/
  if (button_L.risingEdge()) {

      EncLeft_function=!EncLeft_function;
        
      //*SPECIAL MODES that change rightside Encoder based on leftside Encoder menusetting
      //****************************************SAMPLERATE
      if ((EncLeft_menu_idx==MENU_SR) and (EncLeft_function==enc_value))
       { EncRight_menu_idx=MENU_SR;
         EncRight_function=enc_value; // set the rightcontroller to select
       }
      if ((EncLeft_menu_idx==MENU_DTM) and (EncLeft_function==enc_value))
       { EncRight_menu_idx=MENU_DTM;
         EncRight_function=enc_value; // set the rightcontroller to select
       }

     if (SD_ACTIVE)
     {
        /*if (mode==MODE_REC)
                 { stopRecording();
                   EncLeft_function=enc_menu; //force into active-menu
                 }*/
        if ((mode==MODE_PLAY) and ((EncLeft_menu_idx==MENU_PLY) or (EncLeft_menu_idx==MENU_PLD)))
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

 if (button_R.risingEdge()) {
    
    EncRight_function=!EncRight_function; //switch between menu/value control
        
    //when EncLeftoder is SR menu than EncRightoder can directly be setting the samplerate at a press 
    //the press should be a transfer from enc_value to enc_menu before it gets activated
    if ( (EncLeft_menu_idx==MENU_SR) and (EncRight_menu_idx==MENU_SR) and (EncRight_function==enc_menu))
    { set_sample_rate(sample_rate);
      last_sample_rate=sample_rate;

    }
    if ( (EncLeft_menu_idx==MENU_DTM) and (EncRight_menu_idx==MENU_DTM) and (EncRight_function==enc_menu))
    { 
      if (detector_mode==detector_heterodyne)
         { granular1.stop(); //stop all other detecting routines
           
         } 
      if (detector_mode==detector_divider)
         { granular1.beginDivider(GRANULAR_MEMORY_SIZE);
           granular1.setSpeed(0.1);
         }  

      if (detector_mode==detector_Auto_TE)
         { granular1.beginTimeExpansion(GRANULAR_MEMORY_SIZE);
           granular1.setSpeed(0.1);
         }  
      if (detector_mode==detector_Auto_heterodyne)
         { granular1.stop(); 
           
         }  
      if (detector_mode==detector_passive)
         { granular1.stop(); //stop all other detecting routines
           outputMixer.gain(2,1);  //direct line to output 
           outputMixer.gain(1,0);  //shutdown granular output      
           outputMixer.gain(0,0);  //shutdown heterodyne output
         }
       else //all other options use the heterodyne and granular output line
        {  outputMixer.gain(2,0);  //shut down direct line to output 
           outputMixer.gain(1,1);  //granular   output      
           outputMixer.gain(0,1);  //heterodyne output
 
        }  
    }

    //recording and playing are only possible with an active SD card
    if (SD_ACTIVE)
     {
      /*if (mode==MODE_REC)
             { stopRecording();
               EncRight_function=enc_menu; //force into active-menu
              }
          else*/
           if (EncLeft_menu_idx==MENU_REC)
            {
              if (mode == MODE_DETECT) startRecording();
              }  
      else
      if (mode==MODE_PLAY)
             {
              if (not continousPlay) 
                { stopPlaying();
                  EncLeft_menu_idx=MENU_PLY;
                  EncLeft_function=enc_menu;
                }
             }     
      else       
      if (mode==MODE_DETECT)   
       { if (EncLeft_menu_idx==MENU_PLY)
            { last_sample_rate=sample_rate; 
              startPlaying(SAMPLE_RATE_8K);
             }
          
          if (EncLeft_menu_idx==MENU_PLD)
              {  fileselect=referencefile;
                 continousPlay=true;
                 last_sample_rate=sample_rate;
                 startPlaying(SAMPLE_RATE_176K);
              }  
       }
     }
      
      display_settings();
  }

  }
}

/********************************************************* */

void setup() {
 #if DEBUG
  Serial.begin(115200);
 #endif  
  delay(200);
 
//setup Encoder Buttonpins with pullups
  pinMode(BUTTON_RIGHT,INPUT_PULLUP);
  pinMode(BUTTON_LEFT,INPUT_PULLUP);
 // EncLeft.write(10000); //set default far away from 0 to avoid negative transitions
  //EncRight.write(10000); //set default far away from 0 to avoid negative transitions

//startup menu
  EncLeft_menu_idx=MENU_VOL;
  EncRight_menu_idx=MENU_FRQ;
  EncLeft_function=enc_menu;
  EncRight_function=enc_menu;

  // Audio connections require memory. 
  AudioMemory(300);

  setSyncProvider(getTeensy3Time);

// Enable the audio shield. select input. and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  
  sgtl5000_1.volume(0.45);
  sgtl5000_1.micGain (mic_gain);
  //sgtl5000_1.adcHighPassFilterDisable(); // does not help too much!
  sgtl5000_1.lineInLevel(0);
  mixFFT.gain(0,1);

// Init TFT display  
#ifdef USETFT
  tft.begin();
  //ts.begin();
  tft.setRotation( 2 );
  tft.fillScreen(COLOR_BLACK);

  tft.setCursor(0, 0);
  tft.setScrollarea(BOTTOM_OFFSET,TOP_OFFSET);
  display_settings();
  tft.setCursor(80,50);
  tft.setFont(Arial_24);
  char tstr[9];
  snprintf(tstr,9, "%02d:%02d:%02d",  hour(), minute(), second() );
  tft.print(tstr);
  delay(2000); //wait a second to clearly show the time 
#endif

   //Init SD card use
// uses the SD card slot of the Teensy, NOT that of the audio boards
// this init only for playback
#ifdef USESD
  if(!(SD.begin(BUILTIN_SDCARD))) 
  {
      #if DEBUG
          Serial.println("Unable to access the SD card");
          delay(500);  
     #endif     
      
    SD_ACTIVE=false;
     
  }
  else
  { SD_ACTIVE=true;
    root = SD.open("/");

    while (true) {
        File entry =  root.openNextFile();
        
        if (! entry) {
          // no more files
          break;
        }
        
        if (entry.isDirectory()) {
          //Serial.println("/");
          //printDirectory(entry, numTabs + 1);
        } else {
          #ifdef USETFT
            //tft.setCursor(0,40+filecounter*20);
           // tft.print(entry.name());
          #endif
          
        strcpy(filelist[filecounter],entry.name() );
        if (String(entry.name())=="B04_176K.RAW")
           {referencefile=filecounter;
            }
        filecounter++;  
          // files have sizes, directories do not
          //Serial.print("\t\t");
          //Serial.println(entry.size(), DEC);
        }
        entry.close();
       }
    }


if (SD_ACTIVE)
// Recording on SD card by uSDFS library
  {f_mount (&fatfs, (TCHAR *)_T("0:/"), 0);      /* Mount/Unmount a logical drive */
   for (int i=0; i<filecounter; i++)
     {   
       #ifdef DEBUG
          #ifdef USETFT
            tft.setCursor(0,50+i*20);
            tft.print(filelist[i]);
          #endif
      #endif    

     }
     file_number=filecounter+1;

  }

#endif

  set_sample_rate (sample_rate);
  set_freq_Oscillator (freq_real);
  inputMixer.gain(0,1); //microphone       
  inputMixer.gain(1,0); //player

  outputMixer.gain(0,1); // heterodyne1 to output 
  outputMixer.gain(1,1); // heterodyne2 to output
  outputMixer.gain(2,0); // player to output shutdown
  
  // the Granular effect requires memory to operate
  granular1.begin(granularMemory, GRANULAR_MEMORY_SIZE);
  
  // reset the FFT denoising array at startup
  for (int16_t i = 0; i < 128; i++) {
   FFTavg[i]=0; 
     }
} // END SETUP


void loop() {

// If we're playing or recording, carry on...
  if (mode == MODE_REC) {
    continueRecording();
  }
  if (mode == MODE_PLAY) {
    continuePlaying();
  }
  
updateButtons();   

// during recording the encoders are not used and screens are not updated
if (mode!=MODE_REC)
{
 updateEncoders();
 #ifdef USETFT
   if (waterfall_flag==1) 
    { waterfall();
      
     }
   else
    if (spectrum_flag==1)
    {  spectrum();
     }
 #endif
 }   

 
}

