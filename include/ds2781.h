#include "stdint.h"

/* ***** net address commands ***** */
#define READ_NETADDRESS 0x33
#define SKIP_NETADDRESS 0xCC


/* ***** function commands ***** */
#define READ_DATA 0x69
#define WRITE_DATA 0x6C
#define COPY_DATA 0x48
#define RECALL_DATA 0xB8
#define LOCK_EEPROM 0x6A    //DO NOT USE


/* ***** function prototypes ***** */
/* Function details can be found in the .c file */
int ReadVoltage (void);
uint16_t ReadCurrent (void);
uint32_t ReadAccumulatedCurrent (void);
void ResetAccumulatedCurrent (void);
uint32_t ReadNetAddress (void);
uint16_t ReadTemperature (void);
uint8_t ReadCurrentOffset (void);
void  WriteCurrentOffset (uint8_t offset);
void AdjustCurrentOffset (void);
void UpdateControlRegister (uint8_t control);
uint8_t ReadRAM (uint8_t addr);
void WriteRAM (uint8_t byte, uint8_t addr);
void CopyEEPROM (uint8_t addr);
void RecallEEPROM (uint8_t addr);

