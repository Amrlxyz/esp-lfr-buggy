#include "ds2781.h"
#include "OneWire_Methods.h"

/* --------------------------------------------------------------------------
   This file includes the functions needed to access and modify the registers
   in a DS2781 using the 1-Wire protocol. The DS2781 is an IC that measures
   voltage, current, accumulated current and temperature. It implements
   capacity estimation algorithms for rechargeable batteries. However, this
   file only includes routines to access the electrical parameters and not
   the age-estimation registers.
   --------------------------------------------------------------------------
----------------------
   NOTE_1: The functions that return parameters, do so in the units reported
   in the description of each function. The user should implement the scaling
   on his/her own.  
   -------------------------------------------------------------------------- */

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadVoltage                                                 *
 * Overview : Returns the voltage measured at the VIN input of the DS2781      *
 *            in units of 9.76mV                                               *
 * Return type : 16-bit unsigned int                                           *
 * Parameters : None                                                           *
 * Time : < 4.3ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

int ReadVoltage (void)
{
    uint16_t result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( 0x0C );                      //Register Address
        result  = OneWire_ReadByte()  << 8;     //MSB   
        result |= OneWire_ReadByte() ;          //LSB
    }
    return (result >> 5);
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadCurrent                                                 *
 * Overview : Returns the current measured through Rsns external to DS2781 in  *
 *            units of 1.5625uV/Rsns. Positive current indicates discharge     *
 * Return type : 16-bit unsigned int                                           *
 * Parameters : None                                                           *
 * Time : < 4.3ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

uint16_t ReadCurrent (void)
{
    uint16_t result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( 0x0E );                      //Register Address
        result  = ((uint16_t)OneWire_ReadByte() ) << 8;     //MSB   
        result |= ((uint16_t)OneWire_ReadByte() );          //LSB
    }
    return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadAccumulatedCurrent                                      *
 * Overview : Returns the accumulated current at the DS2781 in units of        *
 *            1.526nVhr/Rsns                                                   *
 * Return type : 32-bit unsigned long                                          *
 * Parameters : None                                                           *
 * Time : < 5.8ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

uint32_t ReadAccumulatedCurrent (void)
{
    unsigned long result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( 0x10 );                      //Register Address
        result  = ((unsigned long)OneWire_ReadByte() ) << 24;       //MSB
        result |= ((unsigned long)OneWire_ReadByte() ) << 16;
        result |= ((unsigned long)OneWire_ReadByte() ) << 8;    
        result |= ((unsigned long)OneWire_ReadByte() );         //LSB
    }
    return (result >>  4);
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ResetAccumulatedCurrent                                     *
 * Overview : Resets the accumulated current register at the DS2781            *
 * Return type : Void                                                          *
 * Parameters : None                                                           *
 * Time : < 4.2ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void ResetAccumulatedCurrent (void)
{
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( WRITE_DATA );
        OneWire_WriteByte( 0x10 );                      //Register Address
        OneWire_WriteByte( 0x00 );                      //MSB
        OneWire_WriteByte( 0x00 );                      //LSB
    }
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadNetAddress                                              *
 * Overview : Returns the net address of the DS2781                            *
 * Return type : 64-bit unsigned long long                                     *
 * Parameters : None                                                           *
 * Time : < 7.3ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

uint32_t ReadNetAddress (void)
{
    uint16_t result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( READ_NETADDRESS );
        //result  = ((Quint16_t)OneWire_ReadByte() );       //MSB
        //result |= ((Quint16_t)OneWire_ReadByte() ) << 8;
        //result |= ((Quint16_t)OneWire_ReadByte() ) << 16;
        //result |= ((Quint16_t)OneWire_ReadByte() ) << 24;
        //result |= ((Quint16_t)OneWire_ReadByte() ) << 32;
        //result |= ((Quint16_t)OneWire_ReadByte() ) << 40;
        //result |= ((Quint16_t)OneWire_ReadByte() ) << 48; 
        //result |= ((Quint16_t)OneWire_ReadByte() ) <<56;          //LSB
    }
    return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadTemperature                                             *
 * Overview : Returns the temperature measured by the DS2781 in units of       *
 *            0.125°C                                                          *
 * Return type : 16-bit unsigned int                                           *
 * Parameters : None                                                           *
 * Time : < 4.3ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

uint16_t ReadTemperature (void)
{
    uint16_t result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( 0x0A );                      //Register Address
        result  = ((uint16_t)OneWire_ReadByte() ) << 8;     //MSB   
        result |= ((uint16_t)OneWire_ReadByte() );          //LSB
    }
    return (result >> 5);
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadCurrentOffset                                           *
 * Overview : Returns the value of the current offset register of the DS2781   *
 *            in units of 1.56uV/Rsns                                          *
 * Return type : 8-bit uint8_t                                         *
 * Parameters : None                                                           *
 * Time : < 3.6ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

uint8_t ReadCurrentOffset (void)
{
    uint8_t result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( 0x7B );                      //Register Address
        result  = OneWire_ReadByte();
    }
    return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : WriteCurrentOffset                                          *
 * Overview : Writes to the current offset register of the DS2781 in units of  *
 *            1.56uV/Rsns                                                      *
 * Return type : Void                                                          *
 * Parameters : Byte to be written to the register in 2's complement           *
 * Time : < 3.6ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void  WriteCurrentOffset (uint8_t offset)
{
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( WRITE_DATA );
        OneWire_WriteByte( 0x7B );                      //Register Address
        OneWire_WriteByte( offset );
    }
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : AdjustCurrentOffset                                         *
 * Overview : Adjusts the value of the current offset register of the DS2781   *
 *            by taking into account the offset at no current. Should only     *
 *            be called when the battery is supplying no current               *
 * Return type : Void                                                          *
 * Parameters : None                                                           *
 * Time : < 3.62s                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void AdjustCurrentOffset (void)
{
    char offset = 0;
    
    WriteCurrentOffset ( 0x0 );                     //Reset Current Offset Register

    //Delay100MSx(36);                                //Wait 3.6s for current register to update

    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( 0x0F );                      //Current Register LSB
        offset  = OneWire_ReadByte();   
    }

    offset = 256 - offset;                          //2's complement Negating

    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( WRITE_DATA );
        OneWire_WriteByte( 0x7B );                      //Current Offset Register
        OneWire_WriteByte( offset );    
    }
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : UpdateControlRegister                                       *
 * Overview : Writes to the Control register of the DS2781 using the values    *
 *            supplied as a byte parameter. Writes to EEPROM addresses are     *
 *            ignored for up to 15ms after this function is called.            *
 * Return type : Void                                                          *
 * Parameters : None                                                           *
 * Time : < 6.4ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void UpdateControlRegister (uint8_t control)
{
    if( OneWire_Reset() == true )
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( WRITE_DATA );
        OneWire_WriteByte( 0x60 );                      //Register Address
        OneWire_WriteByte( control );
    }

    if( OneWire_Reset() == true )
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( COPY_DATA );
        OneWire_WriteByte( 0x60 );                      //Register Address
    }
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadRAM                                                     *
 * Overview : Reads a byte from the shadow RAM of the DS2781 at the given      *
 *            memory address                                                   *
 * Return type : 8-bit uint8_t                                         *
 * Parameters : Address of register to be read                                 *
 * Time : < 3.6ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

uint8_t ReadRAM (uint8_t addr)
{
    uint8_t result = 0;
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( READ_DATA );
        OneWire_WriteByte( addr );                      //Register Address
        result  = OneWire_ReadByte();
    }
    return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : WriteRAM                                                    *
 * Overview : Writes the given byte to the shadow RAM of the DS2781 at the     *
 *            given memory address                                             *
 * Return type : Void                                                          *
 * Parameters : Byte to be written, address of register                        *
 * Time : < 3.6ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void WriteRAM (uint8_t byte, uint8_t addr)
{
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( WRITE_DATA );
        OneWire_WriteByte( addr );                      //Register Address
        OneWire_WriteByte( byte );
    }
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : CopyEEPROM                                                  *
 * Overview : This function copies the contents of the EEPROM shadow RAM to    *
 *            EEPROM cells for the EEPROM block containing thr given address.  *
 *            Writes to EEPROM addresses are ignored for up to 15ms after this *
 *            function is called.                                              *
 * Return type : Void                                                          *
 * Parameters : Memory address of shadow RAM to be copied                      *
 * Time : < 2.9ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void CopyEEPROM (uint8_t addr)
{
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( COPY_DATA );
        OneWire_WriteByte( addr );
    }
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : RecallEEPROM                                                *
 * Overview : This function copies the contents of the EEPROM cells to the     *
 *            shadow RAM for the EEPROM block containing the given address.    *                                           *
 * Return type : Void                                                          *
 * Parameters : Memory address of EEPROM to be copied                          *
 * Time : < 2.9ms                                                              *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void RecallEEPROM (uint8_t addr)
{
    if( OneWire_Reset() == true)
    {       
        OneWire_WriteByte( SKIP_NETADDRESS );
        OneWire_WriteByte( RECALL_DATA );
        OneWire_WriteByte( addr );
    }
}
/* EOF */
