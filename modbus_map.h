#ifndef __modbus_map_H__
#define __modbus_map_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*--------------------------------------------------------------------coils---------------------------------------------------------------*/
#define machin_is_active                   0
#define validation_cmd                     1
#define power_machin_coil                  2
#define paper_absense_sensor               3
#define up_roll_proximity_sensor           4
#define down_roll_proximity_sensor         5
#define move_left_macro                    6
#define move_left_micro                    7
#define move_right_micro                   8
#define move_right_macro                   9
#define move_left_macro_react              10
#define move_left_micro_react              11
#define move_right_micro_react             12
#define move_right_macro_react             13
#define door_open_warning                  14
#define auto_process_cmd                   15
#define auto_process_flag                  16
#define manual_process_flag                17
#define paper_error                        18
#define jack_heater_v_is_open              19
#define move_foward_contrast               20
#define move_backward_contrast             21
#define move_forward_contrast_react        22
#define move_backward_contrast_react       23
#define manual_key_scan_cmd                24
#define manual_jack_heater_v_cmd           25
#define manual_blender_cmd                 26
#define manual_jack_heater_h_cmd           27
#define manual_motor_m1_cmd                28
#define manual_motor_m2_cmd                29
#define manual_left_tower_motor_cmd        30
#define manual_right_tower_motor_cmd       31
#define manual_forward_contrast_motor_cmd  32
#define manual_backward_contrast_motor_cmd 33
#define manual_dancer_motor_cmd            34
#define manual_new_pack_cmd                35
#define neck_motor_is_on                   36
#define machin_commend_mode                37
#define machin_triger_mode                 38
#define next_pack_triger                   39
#define wighter_release_triger             40
#define emergency_flag                     41

#define output_1_cmd                       42
#define output_1_flag                      43
#define output_1_mode                      44
#define output_2_cmd                       45
#define output_2_flag                      46
#define output_2_mode                      47
#define output_3_cmd                       48
#define output_3_flag                      49
#define output_3_mode                      50
#define output_4_cmd                       51
#define output_4_flag                      52
#define output_4_mode                      53
#define output_5_cmd                       54
#define output_5_flag                      55
#define output_5_mode                      56
#define output_6_cmd                       57
#define output_6_flag                      58
#define output_6_mode                      59
#define output_7_cmd                       60
#define output_7_flag                      61
#define output_7_mode                      62
#define output_8_cmd                       63
#define output_8_flag                      64
#define output_8_mode                      65
#define output_9_cmd                       66
#define output_9_flag                      67
#define output_9_mode                      68
#define output_10_cmd                      69
#define output_10_flag                     70
#define output_10_mode                     71
#define output_11_cmd                      72
#define output_11_flag                     73
#define output_11_mode                     74
#define output_12_cmd                      75
#define output_12_flag                     76
#define output_12_mode                     77
#define output_13_cmd                      78
#define output_13_flag                     79
#define output_13_mode                     80
#define output_14_cmd                      81
#define output_14_flag                     82
#define output_14_mode                     83
#define output_15_cmd                      84
#define output_15_flag                     85
#define output_15_mode                     86
#define output_16_cmd                      87
#define output_16_flag                     88
#define output_16_mode                     89

#define pmw_1_cmd                          90
#define pmw_2_cmd                          91
#define pmw_3_cmd                          92

#define input_1_flag                       93
#define input_1_mode                       94
#define input_2_flag                       95
#define input_2_mode                       96
#define input_3_flag                       97
#define input_3_mode                       98
#define input_4_flag                       99
#define input_4_mode                       100
#define input_5_flag                       101
#define input_5_mode                       102
#define input_6_flag                       103
#define input_6_mode                       104
#define input_7_flag                       105
#define input_7_mode                       106
#define input_8_flag                       107
#define input_8_mode                       108
#define input_9_flag                       109
#define input_9_mode                       110
#define input_10_flag                      111
#define input_10_mode                      112
#define input_11_flag                      113
#define input_11_mode                      114
#define input_12_flag                      115
#define input_12_mode                      116
#define input_13_flag                      117
#define input_13_mode                      118

/*-------------------------------------------------------------------parameters-----------------------------------------------------------*/
#define output_1_config                     0
#define output_2_config                     1
#define output_3_config                     2
#define output_4_config                     3
#define output_5_config                     4
#define output_6_config                     5
#define output_7_config                     6
#define output_8_config                     7
#define output_9_config                     8
#define output_10_config                    9
#define output_11_config                    10
#define output_12_config                    11
#define output_13_config                    12
#define nack_motor_speed                    13
#define nack_motor_step                     14
#define pack_length                         15
#define pack_before_blending_time           16
#define pack_after_blending_time            17
#define machin_id                           18
#define licence                             19
#define input_1_config                      20
#define input_2_config                      21
#define input_3_config                      22
#define input_4_config                      23
#define input_5_config                      24
#define input_6_config                      25
#define input_7_config                      26
#define input_8_config                      27
#define input_9_config                      28
#define input_10_config                     29
#define input_11_config                     30
#define input_12_config                     31
#define input_13_config                     32
#define release_time                        33
#define mb_licence_char_1                   34
#define mb_licence_char_2                   35
#define mb_licence_char_3                   36
#define mb_licence_char_4                   37
#define mb_licence_char_5                   38
#define mb_licence_char_6                   39
#define mb_licence_char_7                   40
#define mb_licence_char_8                   41
#define mb_licence_char_9                   42
#define mb_licence_char_10                  43
#define mb_licence_char_11                  44
#define mb_licence_char_12                  45
#define mb_licence_char_13                  46
#define mb_licence_char_14                  47
#define mb_licence_char_15                  48
#define mb_licence_char_16                  49
#define mb_machin_id_1                      50
#define mb_machin_id_2                      51
#define mb_machin_id_3                      52
#define mb_machin_id_4                      53
#define mb_machin_id_5                      54
#define mb_machin_id_6                      55

/*----------------------------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* __modbus_map_H__ */
