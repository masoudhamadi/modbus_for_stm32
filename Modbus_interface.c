/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Modbus.h"



#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)


/*--------------------------------------------------------------------*/
bool mb_read_coil(modbusHandler_t *modH, int16_t address){
	
	     uint16_t u16currentRegister =  (address / 16);
       uint8_t u8currentBit = (uint8_t) (address % 16);
       if(bitRead( modH->u16regsCoils[ u16currentRegister ], u8currentBit ))
			 {
				 return true;
			 }
			 else
			 {
				 return false;
			 }
}
void mb_write_coil(modbusHandler_t *modH, int16_t address, bool state){
				uint16_t u16currentRegister =  (address / 16);
				uint8_t u8currentBit = (uint8_t) (address % 16);
				bitWrite( modH->u16regsCoils[ u16currentRegister ], u8currentBit,state );

}
uint16_t mb_read_register(modbusHandler_t *modH, int16_t address){
	return modH->u16regsHR[address];
}
void mb_write_register(modbusHandler_t *modH, int16_t address,uint16_t *data,uint16_t len){

	for (int i=0;i>len;i++)
	{
		modH->u16regsHR[address+i]=data[i];
	}
	
}