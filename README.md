# ESP Line Following Robot

**GROUP 48:** 2nd Year Embedded Systems Project (ESP) 2023/24.

Project is in collaboration with 5 teammates: 
- [@Amrlxyz (me)](https://github.com/Amrlxyz) : Main Programmer + 3D CAD Designer 
- [@Hubr1z](https://github.com/Hubr1z) : 2nd Programmer
- [@Sarahelma](https://github.com/Sarahelma) : PCB Designer
- [@nishoujiwojiubuhuine](https://github.com/nishoujiwojiubuhuine) : 3rd Programmer + General Tech 
- [@Giselle-zheng](https://github.com/Giselle-zheng) : Wiring and Electrical

## Gallery

![Buggy Render](https://github.com/Amrlxyz/esp-lfr-buggy/blob/master/misc/Race%20Day%20Picture.jpg?raw=true)

*Buggy Picture on the final race day*

![Buggy Render](https://github.com/Amrlxyz/esp-lfr-buggy/blob/master/misc/Render%20Final.JPG?raw=true)

*Final Buggy Render Using SolidWorks*

## Achievements

- **ESP 2023/24 Final Race Winner !!!**
- 100% for TDA (Techincal Demonstration A)
- 100% for TDB (Techincal Demonstration B)
- 100% for TDC (Techincal Demonstration C)

## Features

- Mbed v5.15
- Nucleo STM32F401RE 
- Custom Sensor Array PCB
- Array of 6x TCRT5000 IR sensor for line following
- Custom CAD Model Designed in Solidworks

## API Documentation

Link to documentation: [Github Pages](https://amrlxyz.github.io/esp-lfr-buggy/)

## How to Use

### Flashing the MCU

1. Use the wiring diagram to connect the components.
2. Clone the project directly to Keil Studio Cloud.
3. Compile and flash on to the microcontroller.
4. Control the buggy using Bluetooth commands.

### How to run

1. Connect BLE to a phone
2. Place the whole sensor array in the black area of the track
3. Send "calibrate" Command
4. Wait until message is recieved on the phone
5. Place the buggy in the middle of the line
6. Send "line_follow" Command

## Buggy Commands

### Setup

1. Use "Serial Bluetooth Terminal" app on Android
2. Settings -> Send -> Newline -> None
3. Send Commands

### Command Format 

[2/3 Capital Letters] [optional value for certain commands] ['/' -> end character]

Full command list is in main.cpp

__Examples:__

EC/ -> "Execute" "Calibrate"

EF/ -> "Execute" "line_follow"

GSB/ -> Get the speed of both motors


## Linkedin Article

[Article Link](https://www.linkedin.com/pulse/2nd-year-embedded-systems-project-final-race-winner-hakeem-jfuzf)

I might have mentioned a follow-up technical write-up at the end... but life (and distractions) keep happening. We'll see.


## Hello Future UoM 2nd Yr EEE Students

Feel free to use this code as inspiration (at your own risk). Looking back, definitely there are ways to improve it.

If you find the code helpful, feel free to drop a star ⭐ on the repo _(I crave external validation)_

Goodluck!


## Dependencies

Imported 3rd Party Mbed Libraries

- [Driver Board Onboard Battery Monitor](https://os.mbed.com/users/EmbeddedSam/code/Nucleo_F401RE_DS271_Battery_Monitor/) by Sam Walsh
- [QEI Library](https://os.mbed.com/cookbook/QEI) by Aron Berk