/**
 * @file hts221_register_map.h
 * @author Eryk Możdżeń
 * @date 2023-05-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef HTS221_REGISTER_MAP_H
#define HTS221_REGISTER_MAP_H

#define HTS221_WHO_AM_I         0x0F
#define HTS221_AV_CONF          0x10
#define HTS221_CTRL_REG1        0x20
#define HTS221_CTRL_REG2        0x21
#define HTS221_CTRL_REG3        0x22
#define HTS221_STATUS_REG       0x27
#define HTS221_HUMIDITY_OUT_L   0x28
#define HTS221_HUMIDITY_OUT_H   0x29
#define HTS221_TEMP_OUT_L       0x2A
#define HTS221_TEMP_OUT_H       0x2B
#define HTS221_H0_rH_x2			0x30
#define HTS221_H1_rH_x2			0x31
#define HTS221_T0_degC_x8		0x32
#define HTS221_T1_degC_x8		0x33
#define HTS221_T1_T0_msb		0x35
#define HTS221_H0_T0_OUT_L		0x36
#define HTS221_H0_T0_OUT_H		0x37
#define HTS221_H1_T0_OUT_L		0x3A
#define HTS221_H1_T0_OUT_H		0x3B
#define HTS221_T0_OUT_L			0x3C
#define HTS221_T0_OUT_H			0x3D
#define HTS221_T1_OUT_L			0x3E
#define HTS221_T1_OUT_H			0x3F
#define HTS221_REBOOT			HTS221_CTRL_REG2, (1<<7)

#endif
