/**
 ********************************************************************************
 * @file    $modbus_interface
 * @author  $masoud hamadi
 * @date    $11/12/2025
 * @brief   
 ********************************************************************************
 */

#ifndef $modbus_interface
#define $modbus_interface

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
bool mb_read_coil(modbusHandler_t *modH, int16_t address);
void mb_write_coil(modbusHandler_t *modH, int16_t address, bool state);
uint16_t mb_read_register(modbusHandler_t *modH, int16_t address);
void mb_write_register(modbusHandler_t *modH, int16_t address,uint16_t *data,uint16_t len);
/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/


#ifdef __cplusplus
}
#endif

#endif 