/* Includes ------------------------------------------------------------------*/
#include "Modbus_interface.h"
#include "BitUtils.h"





/*--------------------------------------------------------------------*/
void mb_config(void){
  ModbusH.port = &huart3;
  ModbusH.u8id = 1;
  ModbusH.u16timeOut = 1000;
  ModbusH.EN_Port = ENB_GPIO_Port;
  ModbusH.EN_Pin = ENB_Pin;
  ModbusH.u16regsHR = Holding_Registers_Database;
  ModbusH.u16regsRO = Input_Register_Database;
  ModbusH.u16regsCoils = Holding_Coils_Database;
  ModbusH.u16regsCoilsRO = Input_Coils_Database;
  ModbusH.u16regHR_size = sizeof(Holding_Registers_Database)/sizeof(Holding_Registers_Database[0]);
  ModbusH.u16regRO_size = sizeof(Input_Register_Database)/sizeof(Input_Register_Database[0]);
  ModbusH.u16regCoils_size = sizeof(Holding_Coils_Database)/sizeof(Holding_Coils_Database[0]);
  ModbusH.u16regCoilsRO_size = sizeof(Input_Coils_Database)/sizeof(Input_Coils_Database[0]);
  ModbusH.xTypeHW = USART_HW_DMA;
}
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