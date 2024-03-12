#include "OneWire_Methods.h"
#include "mbed.h"

//Peripherals and typedefs
extern DigitalInOut   one_wire_pin;

unsigned char OneWire_ReadByte(void)
{
    unsigned char result=0;

    for (int i = 0; i < 8; i++){    
        result >>= 1;           // shift the result to get it ready for the next bit
        // if result is one, then set MS bit
        if (OneWire_ReadBit()){result |= 0x80;}    
    }
    return result;
}

void OneWire_WriteByte(unsigned char byte_to_write)
{  
    for (int i = 0; i<8; i++)     // Loop to write each bit in the byte, LS-bit first
    {
        OneWire_WriteBit(byte_to_write & 0x01);        
        byte_to_write >>= 1;      // shift the data byte for the next bit
    }
}

bool OneWire_ReadBit(void)
{  
    bool result;
    one_wire_pin.output();
    one_wire_pin = 0;
    OneWire_Delay('A');
    one_wire_pin.input();
    one_wire_pin.mode(PullUp);
    OneWire_Delay('E');
    result = one_wire_pin.read();
    OneWire_Delay('F');
    return result;
}

void OneWire_WriteBit (bool bit_to_write){
    
    if (bit_to_write == 1)
    {
        // Write '1' bit
        one_wire_pin.output();
        one_wire_pin = 0;
        OneWire_Delay('A');
        one_wire_pin.input();
        one_wire_pin.mode(PullUp);
        OneWire_Delay('B');
    }
    else
    {
        one_wire_pin.output();
        one_wire_pin = 0;
        OneWire_Delay('C');
        one_wire_pin.input();
        OneWire_Delay('D');
    }
}
bool OneWire_Reset (void)
{
    //Checks whether there is a device connected, returns true or false
    //Debugging console output can be used if it is defined
    bool result = false;
    one_wire_pin.output();
    one_wire_pin.write(0); //Pull the line high
    OneWire_Delay('H');
    one_wire_pin.input();
    one_wire_pin.mode(PullUp);
    OneWire_Delay('I');
    if(one_wire_pin.read() == 0){ result = true;}
    OneWire_Delay('J'); 
    #ifdef Debugging
        pc.printf("\n\rResult from reset: %d",result);
    #endif
    return result;
}

void OneWire_TestDelays(void)
{
    //Cycles the output pin through all the delays so you
    //can see whether they are all the right timings.
    one_wire_pin.output();
    for(int i=0;i<6;i++){
        one_wire_pin = !one_wire_pin; //toggle the pin 6 times to signal start of test at 100us
        wait_us(100);   
    }
    OneWire_Delay('A');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('A');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('B');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('B');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('C');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('C');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('D');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('D');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('E');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('E');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('F');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('F');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('G');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('G');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('H');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('H');
    one_wire_pin = !one_wire_pin;
    
        
    OneWire_Delay('I');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('I');
    one_wire_pin = !one_wire_pin;
    
    OneWire_Delay('J');
    one_wire_pin = !one_wire_pin;
    OneWire_Delay('J');
    one_wire_pin = !one_wire_pin;
    
}

//Functions
void OneWire_Delay(char letter)
{
    //Recommended delays from Maxim (standard speed not overdrive)
    //https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
    
    //Letter  Speed       Recommended (µs)
    //A       Standard    6
    //B       Standard    64
    //C       Standard    60
    //D       Standard    10
    //E       Standard    9
    //F       Standard    55
    //G       Standard    0
    //H       Standard    480
    //I       Standard    70
    //J       Standard    410
    switch(letter)
    {
        case 'A': wait_us(1); break;
        case 'B': wait_us(64); break;
        case 'C': wait_us(60); break;
        case 'D': wait_us(10); break;
        case 'E': wait_us(5); break;
        case 'F': wait_us(55); break;
        case 'G': wait_us(0); break;
        case 'H': wait_us(480); break;
        case 'I': wait_us(70); break;
        case 'J': wait_us(410); break;
        default: break;
    }
}
