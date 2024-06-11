# ESP Line Following Robot

**GROUP 48:** 2nd Year Embedded Systems Project (ESP) 2023/24.

Project is in collaboration with 5 teammates: 
- [@Amrlxyz](https://github.com/Amrlxyz)
- [@Hubr1z](https://github.com/Hubr1z)
- [@nishoujiwojiubuhuine](https://github.com/nishoujiwojiubuhuine)
- [@Sarahelma](https://github.com/Sarahelma)
- [@Giselle-zheng](https://github.com/Giselle-zheng)

## Gallery

![Buggy Render](https://github.com/Amrlxyz/esp-lfr-buggy/blob/master/misc/Race%20Day%20Picture.jpg?raw=true)

*Buggy Picture on the final race day*

![Buggy Render](https://github.com/Amrlxyz/esp-lfr-buggy/blob/master/misc/Render%20Final.JPG?raw=true)

*Final Buggy Render Using SolidWorks*

## Achievements

- ESP Final Race Winner !!!
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

1. Use the wiring diagram to connect the components.
2. Clone the project directly to Keil Studio Cloud.
3. Compile and flash on to the microcontroller.
4. Control the buggy using Bluetooth commands.

**WARNING:** Due to significant differences in mechanical setup and gearbox, it is not recommended to use this code for any other buggy. The code is specifically tailored for our buggy, and compatibility with other mechanical designs and sensor array configuration is not the main consideration.

## Dependencies

Imported 3rd Party Mbed Libraries

- [Driver Board Onboard Battery Monitor](https://os.mbed.com/users/EmbeddedSam/code/Nucleo_F401RE_DS271_Battery_Monitor/) by Sam Walsh
- [QEI Library](https://os.mbed.com/cookbook/QEI) by Aron Berk