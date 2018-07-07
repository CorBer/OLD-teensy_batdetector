
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
//#include <TimeLib.h>

#include "Audio.h"
//#include <Wire.h>
#include <SPI.h>
#include <Bounce.h>
#include <Metro.h>

boolean SD_ACTIVE=false;

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

#endif

/*time_t getTeensy3Time()
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
*/

uint32_t lastmicros;

#ifdef USETFT

 #define ILI9341
 #ifdef ILI9341
  #include "ILI9341_t3.h"
  #include "font_Arial.h"
  //#include "XPT2046_Touchscreen.h"

  #define BACKLIGHT_PIN 255
  #define TOP_OFFSET 30
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
AudioEffectMultiply              mult1; // multiply = mix
//AudioEffectMultiply              mult2; // multiply = mix

//AudioAnalyzeFFT1024         fft1024_1; // for waterfall display
AudioAnalyzeFFT256               myFFT; // for spectrum display

AudioPlaySdRaw                   player; 

AudioEffectGranular              granular1;

AudioMixer4                      mixFFT;
AudioMixer4                      mix1; //selective output
AudioMixer4                      mix0; //selective input
AudioOutputI2S                   i2s_out; // headphone output          

AudioConnection mic_toinput         (i2s_in, 0, mix0, 0); //microphone signal
AudioConnection mic_torecorder      (i2s_in, 0, recorder, 0); //microphone signal
AudioConnection player_toinput      (player, 0, mix0, 1); //player signal

AudioConnection input_toswitch      (mix0,0,  mixFFT,0);

AudioConnection input_todelay       (mix0,0, granular1, 0);

AudioConnection switch_toFFT        (mixFFT,0, myFFT,0 ); //raw recording channel 

AudioConnection input_toheterodyne1 (mix0, 0, mult1, 0); //heterodyne 1 signal
AudioConnection sineheterodyne1    (sine1, 0, mult1, 1);//heterodyne 1 mixerfreq

AudioConnection granular_toout (granular1,0, mix1,1);
//AudioConnection input_toheterodyne2 (granular1, 0, mult2, 0); //heterodyne 2
//AudioConnection sineheterodyne2     (sine2, 0, mult2, 1);//heterodyne 2 mixerfreq

AudioConnection heterodyne1_toout      (mult1, 0, mix1, 0);  //heterodyne 1 output to outputmixer
//AudioConnection heterodyne2_toout      (mult2, 0, mix1, 1);  //heterodyne 2 output to outputmixer
AudioConnection player_toout           (mix0,0, mix1, 2);    //direct signal (use with player) to outputmixer

AudioConnection output_toheadphoneleft      (mix1, 0, i2s_out, 0); // output to headphone
AudioConnection output_toheadphoneright     (mix1, 0, i2s_out, 1);
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

Metro second = Metro(1000);
elapsedMillis since_bat_detection1;
elapsedMillis since_bat_detection2;

/************** RECORDING PLAYING SETTINGS *****************/

const int8_t    MODE_STOP = 0;
const int8_t    MODE_REC = 1;
const int8_t    MODE_PLAY = 2;

int mode = MODE_STOP; 

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
int8_t waterfall_flag = 0;
int8_t spectrum_flag = 1;
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
#define SAMPLE_RATE_MAX               12

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
    {  SAMPLE_RATE_281K,  "281", "40", "60", "80", 53.33} // which means 53.33 pixels per 20kHz
};    

// setup for FFTgraph denoising 
uint32_t FFTcount=1000; //count the # of FFTs done 
float FFTavg[128];

// initial values for main functions
int displaychoice=0; //default display
int8_t mic_gain = 35; // start detecting with this MIC_GAIN in dB
int8_t volume=50;

int freq_real = 45000; // start detecting at this frequency
int freq_real_backup=freq_real; //used to return to proper settingafter using the play_function

// initial sampling setup
int sample_rate = SAMPLE_RATE_281K;
int sample_rate_real = 281250;

char * SRtext="281";
float freq_LO =50000;

typedef struct Menu_Descriptor
{
    const char* name;
    // ********preset variables below NOT USED YET
    const int len; // length of string to allow right-alignment
    const int def; //default settings
    const int low; // low threshold
    const int high; //high threshold
    
} Menu_Desc;

const int Leftchoices=9; //can have any value
const int Rightchoices=4;
const Menu_Descriptor MenuEntry [Leftchoices] =
{ { "Volume",6,60,0,100}, //divide by 100
   {"mic_gain",8,30,0,63},
   {"Frequency",9,45,20,90}, //multiply 1000
   {"Display",7,0,0,0},
   {"Denoise",7,0,0,0},
   {"Record",6,0,0,0}, //functions where the LeftEncoder 
   {"Play",4,0,0,0},
   {"SampleR",6,0,0,0},
   {"Gran",4,0,0,0}
} ;

const int8_t    MENU_VOL = 0;
const int8_t    MENU_MIC = 1;
const int8_t    MENU_FRQ = 2;
const int8_t    MENU_DSP = 3;
const int8_t    MENU_DNS = 4;
const int8_t    MENU_REC = 5;
const int8_t    MENU_PLY = 6;
const int8_t    MENU_SR  = 7;
const int8_t    MENU_GRN = 8;


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
    tft.setCursor(0,0);
    tft.setFont(Arial_16);
    int fontw=16;;
    tft.fillRect(0,0,240,TOP_OFFSET,COLOR_BLACK);
    tft.fillRect(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET,240,BOTTOM_OFFSET,COLOR_BLACK);
    tft.print("g:"); tft.print(mic_gain);
    tft.print(" f:"); tft.print(freq_real);
    tft.print(" v:"); tft.print(volume);
    tft.print(" SR"); tft.print(SRtext);

    tft.setCursor(0,ILI9341_TFTHEIGHT-BOTTOM_OFFSET);

    //in the default MODE, so not recording/playing
    /*
    tft.print(EncLeftPos);     tft.print(" ");
    tft.print(EncLeft_function);tft.print(" ");
    tft.print(EncLeftchange);tft.print(" ");

    tft.print(EncRightPos);tft.print(" ");
    tft.print(EncRight_function);tft.print(" ");
    tft.print(EncRightchange);tft.print(" ");
    */

    //if (mode!=MODE_STOP)
     if ((mode!=MODE_REC) and (mode!=MODE_PLAY)) 
     // show menu selection as active (green) or selected (yellow)
      {if (EncLeft_function==enc_value) 
       { tft.setTextColor(COLOR_YELLOW);}
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
          { tft.setTextColor(COLOR_GREEN);
            tft.print("PLAY:"); 
            tft.setTextColor(COLOR_WHITE);
            tft.print(filename);
         }
      }

    //scale every 10kHz  
    float x_factor=10000/(0.5*(sample_rate_real / FFT_points)); 
    int curF=2*int(freq_real/(sample_rate_real / FFT_points));

    int maxScale=int(sample_rate_real/20000);
    for (int i=1; i<maxScale; i++) 
     { tft.drawFastVLine(i*x_factor, TOP_OFFSET-6, 6, COLOR_YELLOW);  
     }    
    tft.fillCircle(curF,TOP_OFFSET-4,3,COLOR_YELLOW);
    
   #endif
}


void       set_mic_gain(int8_t gain) {
    
    AudioNoInterrupts();
    sgtl5000_1.micGain (gain);
    AudioInterrupts();
    display_settings();    
} // end function set_mic_gain

void       set_freq_LO(int freq) {
    // audio lib thinks we are still in 44118sps sample rate
    // therefore we have to scale the frequency of the local oscillator
    // in accordance with the REAL sample rate
      
    freq_LO = (freq+250) * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real); 
    float F_LO2= (freq+5000) * (AUDIO_SAMPLE_RATE_EXACT / sample_rate_real); 
    // if we switch to LOWER samples rates, make sure the running LO 
    // frequency is allowed ( < 22k) ! If not, adjust consequently, so that
    // LO freq never goes up 22k, also adjust the variable freq_real  
    if(freq_LO > 22000) {
      freq_LO = 22000;
      freq_real = freq_LO * (sample_rate_real / AUDIO_SAMPLE_RATE_EXACT) + 9;
    }
    AudioNoInterrupts();
    //setup multiplier SINE
    sine1.frequency(freq_LO);
    //sine2.frequency(freq_LO);
        
    AudioInterrupts();
    display_settings();
} // END of function set_freq_LO

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
  const int numfreqs = 16;
  const int samplefreqs[numfreqs] = {  8000,      11025,      16000,      22050,       32000,       44100, (int)44117.64706 , 48000,      88200, (int)44117.64706 * 2,   96000, 176400, (int)44117.64706 * 4, 192000,  234000, 281000};
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255},   {128, 1875}, {107, 853},     {32, 255},   {219, 1604}, {1, 4},      {64, 255},     {219,802}, { 1,3 },  {2,5} };  //last value 219 802

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
  }
    
    AudioNoInterrupts();
    setI2SFreq (sample_rate_real); 
    delay(200); // this delay seems to be very essential !
    set_freq_LO (freq_real);
    AudioInterrupts();
    delay(20);
    display_settings();
   
} // END function set_sample_rate


void display_found_bats() {
    #ifdef USETFT
    int sign_x = 148; 
    int sign_y = 57; 
    if (since_bat_detection1 > 6000 && since_bat_detection2 > 6000) {
// DELETE red BAT sign
//      Serial.println("RED SIGN DELETED");      
      tft.fillRect(sign_x, sign_y, 64, 22, COLOR_BLACK);
      tft.fillRect(sign_x + 86, sign_y + 4, 320 - 86 - sign_x, 40, COLOR_BLACK);
//      since_bat_detection = 0;
    }

// DELETE FREQUENCY 1
    if (since_bat_detection1 > 6000) {
      tft.fillRect(sign_x + 86, sign_y + 4, 320 - 86 - sign_x, 14, COLOR_BLACK);
//      since_bat_detection1 = 0;
    }

// DELETE FREQUENCY 2
    if (since_bat_detection2 > 6000) {
      tft.fillRect(sign_x + 86, sign_y + 30, 320 - 86 - sign_x, 14, COLOR_BLACK);
//      since_bat_detection2 = 0;
    }
// PRINT RED BAT SIGN    
    if(FFT_max1 > (FFT_mean1 + 5) || FFT_max2 > (FFT_mean2 + 5)) {
//      Serial.println("BAT");
      tft.fillRect(sign_x, sign_y, 64, 22, COLOR_RED);
      tft.setTextColor(COLOR_WHITE);
  //    tft.setFont(Arial_14);
      tft.setCursor(sign_x + 2, sign_y + 4);
      tft.print("B A T !");      
//      since_bat_detection = 0;
    }
// PRINT frequency 1
    if(FFT_max1 > FFT_mean1 + 5) {
      tft.fillRect(sign_x + 86, sign_y + 4, 320 - 86 - sign_x, 14, COLOR_BLACK);
      tft.setTextColor(COLOR_ORANGE);
   //   tft.setFont(Arial_12);
      tft.setCursor(sign_x + 86, sign_y + 4);
      tft.print(FFT_max_bin1 * sample_rate_real / FFT_points);
      tft.print(" Hz");      
      since_bat_detection1 = 0;
    
//      Serial.print ("max Freq 1: ");
//      Serial.println(FFT_max_bin1 * sample_rate_real / 256);

    }
    
// PRINT frequency 2    
    if(FFT_max2 > FFT_mean2 + 5) { 
      tft.fillRect(sign_x + 86, sign_y + 30, 320 - 86 - sign_x, 14, COLOR_BLACK);
      tft.setTextColor(COLOR_ORANGE);
 //     tft.setFont(Arial_12);
      tft.setCursor(sign_x + 86, sign_y + 30);
      tft.print(FFT_max_bin2 * sample_rate_real / FFT_points);
      tft.print(" Hz");      
//      Serial.print ("max Freq 2: ");
//      Serial.println(FFT_max_bin2 * sample_rate_real / 256);
      since_bat_detection2 = 0;
    }
      tft.setTextColor(COLOR_WHITE);
      #endif
} // END function display_found_bats

 

void  search_bats() {
    // the array FFT_bin contains the results of the 256 point FFT --> 127 magnitude values
    // we look for bins that have a high amplitude compared to the mean noise, indicating the presence of ultrasound
    // 1. only search in those parts of the array > 14kHz and not around +-10kHz of the LO freq -->
    //    thus it is best, if I search in two parts --> 14kHz to freq_real-10k AND freq_real+10k to sample_rate/2
    // 2. determine mean and max in both parts of the array
    // 3. if we find a bin that is much larger than the mean (take care of situations where mean is zero!) --> identify the no. of the bin
    // 4. determine frequency of that bin (depends on sample_rate_real)
    //    a.) by simply multiplying bin# with bin width
    //    b.) by using an interpolator (not (yet) implemented)
    // 5. display frequency in bold and RED for 1-2 sec. (TODO: also display possible bat species ;-)) and then delete
    // goto 1.    

    // search array in two parts: 
    //  1.)  14k to (freq_real - 10k)
    // upper and lower limits for maximum search
     l_limit = 14000;
     u_limit = freq_real - 10000;
     index_l_limit =  (l_limit * FFT_points / sample_rate_real);  // 1024 !
     index_u_limit =  (u_limit * FFT_points / sample_rate_real);  // 1024 !
//     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);

     if (index_u_limit > index_l_limit) { 
        arm_max_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_max1, &FFT_max_bin1);
            // this is the efficient CMSIS function to calculate the mean
        arm_mean_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_mean1);
            // shift bin_max because we have not searched through ALL the 256 FFT bins
//     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
        FFT_max_bin1 = FFT_max_bin1 + index_l_limit;
     }
//     Serial.print(FFT_max1); Serial.print ("  "); Serial.println(FFT_mean1);

    //  2.)  (freq_real + 10k) to 256 
    // upper and lower limits for maximum search
     l_limit = freq_real + 10000;
     if (l_limit < 14000) {
      l_limit = 14000;
     }
     index_l_limit = (l_limit * FFT_points / sample_rate_real); 
     index_u_limit = (FFT_points / 2) - 1; 
//     Serial.print(index_l_limit); Serial.print ("  "); Serial.println(index_u_limit);
     if (index_u_limit > index_l_limit) { 
        arm_max_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_max2, &FFT_max_bin2);
            // this is the efficient CMSIS function to calculate the mean
        arm_mean_q15(&FFT_bin[index_l_limit], index_u_limit - index_l_limit, &FFT_mean2);
            // shift bin_max because we have not searched through ALL the 128 FFT bins
        FFT_max_bin2 = FFT_max_bin2 + index_l_limit;
     }
//         Serial.print(FFT_max2); Serial.print ("  "); Serial.println(FFT_mean2);

      display_found_bats();
    FFT_max1 = 0;
    FFT_mean1 = 0;
    FFT_max_bin1 = 0;
    FFT_max2 = 0;
    FFT_mean2 = 0;
    FFT_max_bin2 = 0;
}  // END function search_bats()


void spectrum() { // spectrum analyser code by rheslip - modified
     #ifdef USETFT
     if (myFFT.available()) {
//     if (fft1024_1.available()) {
    int scale;
    scale = 1;
    int maxF=2; int16_t peak=0; uint16_t avgF=0;

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
        maxF=x;
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
  
    // if (mode == MODE_STOP)  search_bats();     
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

  #ifdef USETFT1
  int curF=int(freq_real/(sample_rate_real / FFT_points));
  const uint16_t Y_OFFSET = 0;
// code for 256 point FFT 
  static int count = 0;
  uint16_t lbuf[220]; // 320 vertical stripes, each one is the result of one FFT 
  if (myFFT.available()) {
    FFTcount++;
//    for (int i = 0; i < 240; i++) {
    //startup sequence to denoise the FFT
    if (FFTcount==1)
     {for (int16_t i = 0; i < 128; i++) {
     FFTavg[i]=0; 
     }
     }

    if (FFTcount<1000)
     { 
       for (int i = 2; i < 128; i++) {
         FFTavg[i]=FFTavg[i]+myFFT.read(i)*65536.0*5*0.001; //0.1% of total values
         }
     }

    for (int i = 2; i <128; i++) { // we have 128 FFT bins, we take 120 of them
//      int val = fft1024_1.read(i) * 65536.0; //v1
      int val = myFFT.read(i) * 65536.0 * 5 -FFTavg[i]*0.9 + 10; //v1
      //int va l= myFFT.output[i]*5;
      //int val = fft1024_1.read(i*2, i*2+1 ) * 65536.0; //v2 
      if (val<5) {val=5;}
      
      lbuf[i] = tft.color565(
              min(255, val), //r
              (val/8>255)? 255 : val/8, //g
              ((255-val)>>1) <0? 0: (255-val)>>1 //b
             ); 
    }
    tft.writeRect(0,count+40, 128,1, (uint16_t*) &lbuf);
    //show the current heterodyne listening frequency
    tft.fillCircle(curF,40,3,COLOR_YELLOW);
   // tft.setScroll(220-count);
   
    // print one FFT result (120 bins) with 240 pixels
    //tft.writeRect(count, Y_OFFSET, 1, 239 - Y_OFFSET, (uint16_t*) &lbuf);
    //tft.setScroll(319 - count);
    count++;
    if (count >= 180) count = 0;
    count_help = count;
  }
#endif
#ifdef USETFT

// code for 256 point FFT 
  
   
  if (myFFT.available()) {
  const uint16_t Y_OFFSET = TOP_OFFSET;
  static int count = TOP_OFFSET;
  int curF=int(freq_real/(sample_rate_real / FFT_points));
  uint16_t lbuf[320]; // 320 vertical stripes, each one is the result of one FFT 
  lbuf[0]=0; lbuf[1]=0;  lbuf[2]=0; lbuf[3]=0;
  if (millis()%250<50) // show every 250ms a small block
    {lbuf[1]=2000;
     lbuf[2]=2000;
     lbuf[3]=2000;
     }

     FFTcount++;
//    for (int i = 0; i < 240; i++) {
    //startup sequence to denoise the FFT
    if (FFTcount==1)
     {for (int16_t i = 0; i < 128; i++) {
        FFTavg[i]=0; 
     }
     }

    if (FFTcount<1000)
     { for (int i = 2; i < 128; i++) {
         //FFTavg[i]=FFTavg[i]+myFFT.read(i)*65536.0*5*0.001; //0.1% of total values
         FFTavg[i]=FFTavg[i]+myFFT.output[i]*10*0.001; //0.1% of total values
         }
     }

//    for (int i = 0; i < 240; i++) {
    for (int i = 2; i < 120; i++) { // we have 128 FFT bins, we take 120 of them
//      int val = fft1024_1.read(i) * 65536.0; //v1
      //int val = myFFT.read(i) * 65536.0 * 5; //v1
      //int val = fft1024_1.read(i*2, i*2+1 ) * 65536.0; //v2      
      
      //int val = myFFT.read(i) * 65536.0 * 5 -FFTavg[i]*0.9 + 10; //v1
      int val = myFFT.output[i]*10 -FFTavg[i]*0.9 + 10; //v1
      //int va l= myFFT.output[i]*5;
      //int val = fft1024_1.read(i*2, i*2+1 ) * 65536.0; //v2 
      if (val<5) {val=5;}
      lbuf[i*2] = tft.color565(
              min(255, val), //rcalc
              (val/8>255)? 255 : val/8, //g
              ((255-val)>>1) <0? 0: (255-val)>>1 //b
             ); 
       
            
      lbuf[i*2+1]=lbuf[i*2];       
    }
//    tft.writeRect(count, 16, 1, 239-16, (uint16_t*) &lbuf);
    // print one FFT result (120 bins) with 240 pixels
    //tft.writeRect(count, Y_OFFSET, 1, 239 - Y_OFFSET, (uint16_t*) &lbuf);
    tft.writeRect( 0,count, 239,1, (uint16_t*) &lbuf);
    tft.setScroll(320-count);
    count++;
    if (count >= 320-BOTTOM_OFFSET) count = Y_OFFSET;
    count_help = count;

  }
#endif

}

void startRecording() {
  mode = MODE_REC;
  Serial.print("startRecording");
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
  
  Serial.println(filename);
  char2tchar(filename, 80, wfilename);
  filecounter++;
  strcpy(filelist[filecounter],filename );
  rc = f_stat (wfilename, 0);
    Serial.printf("stat %d %x\n",rc,fil.obj.sclust);
  rc = f_open (&fil, wfilename, FA_WRITE | FA_CREATE_ALWAYS);
    Serial.printf(" opened %d %x\n\r",rc,fil.obj.sclust);
 
    // check if file is Good
    if(rc == FR_INT_ERR)
    { // only option is to close file
        rc = f_close(&fil);
        if(rc == FR_INVALID_OBJECT)
        { Serial.println("unlinking file");
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

  display_settings();
  //switch off FFT by closing the inputchannel
  mixFFT.gain(0,0);
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
        Serial.printf(" write FR_DISK_ERR : %x\n\r",usd_error);
        // only option is to close file
        // force closing file
     }
     else if(rc) die("write",rc);
    }
  nj ++;
  }

}

void stopRecording() {
  Serial.print("stopRecording");
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
  mode = MODE_STOP;
//  clearname();
  Serial.println (" Recording stopped!");
  //switch on FFT
  mixFFT.gain(0,1);
    
  
}

void startPlaying() {
//      String NAME = "Bat_"+String(file_number)+".raw";
//      char fi[15];
//      NAME.toCharArray(fi, sizeof(NAME));

      mix0.gain(0,0); //switch off the mic-line as input
      mix0.gain(1,1); //switch on the playerline as input
 
      mix1.gain(2,1);  //player to output 
      mix1.gain(1,0);  //shutdown heterodyne output      
      mix1.gain(0,0);  //shutdown heterodyne output

  delay(100);
  
  freq_real_backup=freq_real;
  set_sample_rate(SAMPLE_RATE_8K);
  
  strncpy(filename, filelist[fileselect],  13);

  EncRight_menu_idx=MENU_SR;
  EncRight_function=enc_value;

  display_settings();

  player.play(filename);
  
  mode = MODE_PLAY;

}
  

void stopPlaying() {
  
      
  Serial.print("stopPlaying");
  if (mode == MODE_PLAY) player.stop();
  mode = MODE_STOP;
  
  Serial.println (" Playing stopped");
  //restore last settings
  set_sample_rate(SAMPLE_RATE_281K);
  freq_real=freq_real_backup;
  set_freq_LO (freq_real);

  mix1.gain(2,0); 
  mix1.gain(1,1);       
  mix1.gain(0,1);   

  mix0.gain(0,1); //switch on the mic-line
  mix0.gain(1,0); //switch off the playerline

}


void continuePlaying() {
  if (!player.isPlaying()) {
    stopPlaying();
    /*
    
    player.stop();
    mode = MODE_STOP;
        EncRight_menu_idx=  EncRight_menu_idx_old;
        EncRight_function= EncRight_function_old;

      mix1.gain(2,0);
      mix1.gain(1,1);       
      mix1.gain(0,1);       
    Serial.println("End of recording");
    set_sample_rate(SAMPLE_RATE_281K);
    freq_real=freq_real_backup;
    set_freq_LO (freq_real);

      mix0.gain(0,1); //switch off the mic-line as input
      mix0.gain(1,0); //switch on the playerline as input
    */
  }
}


//update encoders
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
      choices=Leftchoices;
    }

   if (Encoderside==enc_rightside)
    { encodermode=EncRight_function;
      change=EncRightchange;
      menu_idx=EncRight_menu_idx;
      choices=Rightchoices;
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
      if (Encoderside==enc_rightside)
          { EncRight_menu_idx=menu_idx; //limit the menu 
               }

      //*SPECIAL MODES that change rightside Encoder based on leftside Encoder menusetting
      //****************************************SAMPLERATE
      if ((EncLeft_menu_idx==MENU_SR) and (EncLeft_function==enc_value))
       { EncRight_menu_idx=MENU_SR;
         EncRight_function=enc_value; // set the rightcontroller to select
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

           set_freq_LO (freq_real);
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

/************  LEFT ENCODER BUTTON *******************/
  if (button_L.risingEdge()) {
    EncLeft_function=!EncLeft_function; 
    //recording and playing are only possible with an active SD card
    
    //play menu got choosen, now change the rightencoder to choose a file
    if ((EncLeft_menu_idx==MENU_PLY) and (EncLeft_function==enc_value))
     { 
        EncRight_menu_idx=MENU_PLY ;
       EncRight_function=enc_value; // set the rightcontroller to select
       
     }

    if ((EncLeft_menu_idx==MENU_GRN) and (EncLeft_function==enc_menu)) 
      { //granular1.beginFreeze(1000);
        granular1.beginTimeExpansion(GRANULAR_MEMORY_SIZE);
        granular1.setSpeed(0.1);
        //set_sample_rate(SAMPLE_RATE_44K);
         }
   if ((EncLeft_menu_idx==MENU_GRN) and (EncLeft_function==enc_value)) 
      { granular1.stop();
        //set_sample_rate(SAMPLE_RATE_192K);
         }


    if (SD_ACTIVE)
     {
        if (mode==MODE_REC)
                 { stopRecording();
                   EncLeft_function=enc_menu; //force into active-menu
                 }
        if (mode==MODE_PLAY)
                 { stopPlaying();
                   EncLeft_menu_idx=MENU_PLY;
                   EncLeft_function=enc_menu; //force into active-menu
                  
       

      }
        
     display_settings(); 
     }     
  }

/************  RIGHT ENCODER BUTTON *******************/

 if (button_R.risingEdge()) {
    EncRight_function=!EncRight_function; //switch between menu/value control
    
    //when EncLeftoder is SR menu than EncRightoder can directly be setting the samplerate at a press 
    if ( (EncLeft_menu_idx==MENU_SR) and (EncRight_menu_idx==MENU_SR) and (EncRight_function==enc_value))
    { set_sample_rate(sample_rate);

    }
    //recording and playing are only possible with an active SD card
    if (SD_ACTIVE)
     {
      if (mode==MODE_REC)
             {  stopRecording();
              }
          else
           if (EncLeft_menu_idx==MENU_REC)
            {
              if (mode == MODE_STOP) startRecording();
              }  
    
      if (mode==MODE_PLAY)
             {
              stopPlaying();
              EncLeft_menu_idx=MENU_PLY;
              EncLeft_function=enc_menu;

              
       
             }     
          else   
          if (EncLeft_menu_idx==MENU_PLY)
          { if (mode == MODE_STOP) startPlaying();
            }  
     
     }
      
      display_settings();
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
/*
  setSyncProvider(getTeensy3Time);
*/
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
  set_freq_LO (freq_real);
  mix0.gain(0,1); //microphone       
  mix0.gain(1,0); //player

  mix1.gain(0,1); // heterodyne1 to output 
  mix1.gain(1,1); // heterodyne2 to output
  mix1.gain(2,0); // player to output shutdown
  
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

// during recording the encoders are notused and screens are not updated
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

