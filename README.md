# teensy_batdetector
Batdetector on Teensy3.6 (Based on original-code from Frank (DD4WH)
https://github.com/DD4WH/Teensy-Bat-Detector )

latest changes<br>
<b>0.86</b><br>
-changes by WMXZ to use latest uSDFS library and not be depending on ff_utils and SD.h<br>


<b>v0.85</b> (23 06 2019) <br>
  -changed the usage of seconds2tm from the ff_utils library,<br> 
   created ff_utils_copy library that is independent of other uSDFS libraries.


This is a work in progress project, the code and setup is still changing. If you have a specific request or otherwise questions please share them. 

Several specific libraries have been added to this repository, in de directory <b>lib</b> you can find them.

I am using a non-default programming setup for Teensy, that means that the code might not work in other environments.

Setup @2019-06-22

Operating system Linux Mint 19.1

<b>Visual Code 1.35.1</b><br>
-PlatformIO Home 2.2.0·Core 3.6.7<br>
-platformio.ini<br>
  [env:teensy36]<br>
  platform = teensy<br>
  board = teensy36<br>
  framework = arduino<br>

