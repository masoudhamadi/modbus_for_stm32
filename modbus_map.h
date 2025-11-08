#ifndef __modbus_map_H__
#define __modbus_map_H__

#ifdef __cplusplus
extern "C" {
#endif
# include "main.h"




/*--------------------------------------------------------------------coils---------------------------------------------------------------*/
#define machin_is_active 454
#define validation_cmd 566
#define power_machin_coil  1
#define paper_absense_sensor 2
#define up_roll_proximity_sensor 3
#define down_roll_proximity_sensor 4

#define move_left_macro 7
#define move_left_micro 8
#define move_right_micro 9
#define move_right_macro 10
#define move_left_macro_react 11
#define move_left_micro_react 12
#define move_right_micro_react 13
#define move_right_macro_react 14
#define door_open_warning 15
#define auto_process_cmd 16
#define auto_process_flag 17
#define manual_process_flag 18
#define paper_error 19
#define jack_heater_v_is_open 20
#define move_foward_contrast 21
#define move_backward_contrast 22
#define move_forward_contrast_react 23
#define move_backward_contrast_react 24
#define manual_key_scan_cmd 25
#define manual_jack_heater_v_cmd 26
#define manual_blender_cmd 27
#define manual_jack_heater_h_cmd 28
#define manual_motor_m1_cmd 29
#define manual_motor_m2_cmd 30
#define manual_left_tower_motor_cmd 31
#define manual_right_tower_motor_cmd 32
#define manual_forward_contrast_motor_cmd 33
#define manual_backward_contrast_motor_cmd 34
#define manual_dancer_motor_cmd 35
#define manual_new_pack_cmd 37
#define neck_motor_is_on 1000
#define machin_commend_mode 10010
#define machin_triger_mode 5444
#define next_pack_triger 11025
#define wighter_release_triger 151
#define emergency 152

/*-*/
#define output_1_cmd 38
#define output_1_flag 39
#define output_1_mode 40
/*-*/
#define output_2_cmd 38
#define output_2_flag 39
#define output_2_mode 40
/*-*/
#define output_3_cmd 38
#define output_3_flag 39
#define output_3_mode 40
/*-*/
#define output_4_cmd 38
#define output_4_flag 39
#define output_4_mode 40
/*-*/
#define output_5_cmd 38
#define output_5_flag 39
#define output_5_mode 40
/*-*/
#define output_6_cmd 38
#define output_6_flag 39
#define output_6_mode 40
/*-*/
#define output_7_cmd 38
#define output_7_flag 39
#define output_7_mode 40
/*-*/
#define output_8_cmd 38
#define output_8_flag 39
#define output_8_mode 40
/*-*/
#define output_9_cmd 38
#define output_9_flag 39
#define output_9_mode 40
/*-*/
#define output_10_cmd 38
#define output_10_flag 39
#define output_10_mode 40
/*-*/
#define output_11_cmd 38
#define output_11_flag 39
#define output_11_mode 40
/*-*/
#define output_12_cmd 38
#define output_12_flag 39
#define output_12_mode 40
/*-*/
#define output_13_cmd 38
#define output_13_flag 39
#define output_13_mode 40
/*-*/
#define output_14_cmd 38
#define output_14_flag 39
#define output_14_mode 40
/*-*/
#define output_15_cmd 38
#define output_15_flag 39
#define output_15_mode 40
/*-*/
#define output_16_cmd 38
#define output_16_flag 39
#define output_16_mode 40
/*-*/
#define pmw_1_cmd 38
/*-*/
#define pmw_2_cmd 38
/*-*/
#define pmw_3_cmd 38
/*-*/
#define input_1_flag 39
#define input_1_mode 40
/*-*/
#define input_2_flag 39
#define input_2_mode 40
/*-*/
#define input_3_flag 39
#define input_3_mode 40
/*-*/
#define input_4_flag 39
#define input_4_mode 40
/*-*/
#define input_5_flag 39
#define input_5_mode 40
/*-*/
#define input_6_flag 39
#define input_6_mode 40
/*-*/
#define input_7_flag 39
#define input_7_mode 40
/*-*/
#define input_8_flag 39
#define input_8_mode 40
/*-*/
#define input_9_flag 39
#define input_9_mode 40
/*-*/
#define input_10_flag 39
#define input_10_mode 40
/*-*/
#define input_11_flag 39
#define input_11_mode 40
/*-*/
#define input_12_flag 39
#define input_12_mode 40
/*-*/
#define input_13_flag 39
#define input_13_mode 40
/*-------------------------------------------------------------------parameters-----------------------------------------------------------*/
#define output_1_config 1
#define output_2_config 2
#define output_3_config 3
#define output_4_config 4
#define output_5_config 5
#define output_6_config 6
#define output_7_config 7
#define output_8_config 8
#define output_9_config 9
#define output_10_config 10
#define output_11_config 11
#define output_12_config 12
#define output_13_config 13
#define nack_motor_speed 14
#define nack_motor_step 1002
#define pack_length 15
#define pack_before_blending_time 1003
#define pack_after_blending_time 1003
#define machin_id 16
#define licence  17
#define input_1_config 1
#define input_2_config 2
#define input_3_config 3
#define input_4_config 4
#define input_5_config 5
#define input_6_config 6
#define input_7_config 7
#define input_8_config 8
#define input_9_config 9
#define input_10_config 10
#define input_11_config 11
#define input_12_config 12
#define input_13_config 13
#define release_time 14
#define mb_licence_char_1 15
#define mb_licence_char_2 16
#define mb_licence_char_3 17
#define mb_licence_char_4 18
#define mb_licence_char_5 19
#define mb_licence_char_6 20
#define mb_licence_char_7 21
#define mb_licence_char_8 22
#define mb_licence_char_9 23
#define mb_licence_char_10 24
#define mb_licence_char_11 25
#define mb_licence_char_12 26
#define mb_licence_char_13 27
#define mb_licence_char_14 28
#define mb_licence_char_15 29
#define mb_licence_char_16 30
#define mb_machin_id_1 31
#define mb_machin_id_2 32
#define mb_machin_id_3 33
#define mb_machin_id_4 34
#define mb_machin_id_5 35
#define mb_machin_id_6 36
/*----------------------------------------------------------------------------------------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __modbus_map_H__ */

