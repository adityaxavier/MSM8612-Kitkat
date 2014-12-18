/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *         Modify History For This Module
 * When           Who             What,Where,Why
 * --------------------------------------------------------------------------------------
 * 13/11/25              Add GC2155 camera driver code   
 * --------------------------------------------------------------------------------------
*/
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include <linux/leds.h>

#define GC2155_SENSOR_NAME "gc2155"
#define PLATFORM_DRIVER_NAME "msm_camera_gc2155"


#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) printk(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MSM_MUTEX(gc2155_mut);
static struct msm_sensor_ctrl_t gc2155_s_ctrl;
/*
*  flash mode: 0-off, 1-auto, 2-on, 3-torch
*/
static int flash_mode = 0;
static int is_flash_work = 0;
static struct msm_sensor_power_setting gc2155_power_setting[] = {
	{
			.seq_type = SENSOR_GPIO,
			.seq_val = SENSOR_GPIO_RESET,
			.config_val = GPIO_OUT_LOW,
			.delay = 10,
	},

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},

	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 20,
	},

	
	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};

//for snapshot
static struct msm_camera_i2c_reg_conf gc2155_uxga_settings[] = {
    //uxga = 1600*1200, 2M
    //  {0xf7 ,  0x1d},
    //  {0xf8 ,  0x84},
    {0xfa ,  0x10},
    {0xfd ,  0x00},
 	//// crop window
    {0xfe , 0x00},
    {0x90 , 0x01},
    {0x91 , 0x00},
    {0x92 , 0x00},
    {0x93 , 0x00},
    {0x94 , 0x00},
    {0x95 , 0x04},
    {0x96 , 0xb0},
    {0x97 , 0x06},
    {0x98 , 0x40},
    {0x99 , 0x11},
    {0x9a , 0x06},
    // AWB
    {0xfe , 0x00},
    {0xec , 0x02},
    {0xed , 0x04},
    {0xee , 0x60},
    {0xef , 0x90},
    {0xfe , 0x01},
    {0x74 , 0x01},
    // AEC
    {0xfe , 0x01},
    {0x01 , 0x08},
    {0x02 , 0xc0},
    {0x03 , 0x04},
    {0x04 , 0x90},
    {0x05 , 0x30},
    {0x06 , 0x98},
    {0x07 , 0x28},
    {0x08 , 0x6c},
    {0x0a , 0xc2},
    {0x21 , 0x15},
    {0xfe , 0x00},
    //MIPI
    {0xfe ,  0x03},  //page 3
    {0x04 ,  0x20}, 
    {0x12 ,  0x80}, //LWC[7:0]  
    {0x13 ,  0x0c}, //LWC[15:8]
    {0xfe ,  0x00},
};




static struct msm_camera_i2c_reg_conf gc2155_start_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x94},
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc2155_stop_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x84},
	{0xfe, 0x00},
};





//set sensor init setting
static struct msm_camera_i2c_reg_conf gc2155_recommend_settings[] = {
     //SVGA init
     {0xfe , 0xf0},
     {0xfe , 0xf0},
     {0xfe , 0xf0},
     {0xfc , 0x06},
     {0xf6 , 0x00},
     {0xf7 , 0x1d}, 
     {0xf8 , 0x84},
     {0xfa , 0x00},
     {0xf9 , 0xfe}, 
     {0xf2 , 0x00},
     {0xfe , 0x00},
     {0x03 , 0x04}, //exp time
     {0x04 , 0xf8}, //exp time
     {0x05 , 0x01}, //hb[11:8]
     {0x06 , 0x68}, //hb
     {0x09 , 0x00}, //row start
     {0x0a , 0x00}, //
     {0x0b , 0x00}, //col start
     {0x0c , 0x00},
     {0x0d , 0x04}, //height
     {0x0e , 0xc0},
     {0x0f , 0x06}, //width
     {0x10 , 0x52},//0x52
     {0x12 , 0x2e}, 
     {0x17 , 0x14}, //0x14  direction
     {0x18 , 0x02}, 
     {0x19 , 0x0f}, 
     {0x1a , 0x01}, 
     {0x1b , 0x4b}, 
     {0x1c , 0x07}, 
     {0x1d , 0x10}, 
     {0x1e , 0x98}, 
     {0x1f , 0x78}, 
     {0x20 , 0x05}, 
     {0x21 , 0x40}, 
     {0x22 , 0xf0}, 
     {0x24 , 0x16}, 
     {0x25 , 0x01}, 
     {0x26 , 0x10}, 
     {0x2d , 0x40}, 
     {0x30 , 0x01}, 
     {0x31 , 0x90},   
     {0x33 , 0x04}, 
     {0x34 , 0x01},
     {0x80 , 0xff}, 
     {0x81 , 0x2c}, 
     {0x82 , 0xfa}, 
     {0x83 , 0x00}, 
     {0x84 , 0x02}, //yuv  
     {0x85 , 0x08}, 
     {0x86 , 0x06}, 
     {0x88 , 0x03}, 
     {0x89 , 0x03}, 
     {0x8a , 0x00}, 
     {0x8b , 0x00}, 
     {0xb0 , 0x55}, 
     {0xc3 , 0x00}, 
     {0xc4 , 0x80}, 
     {0xc5 , 0x90}, 
     {0xc6 , 0x38}, 
     {0xc7 , 0x40}, 
     {0xec , 0x02}, 
     {0xed , 0x04},
     {0xee , 0x60}, 
     {0xef , 0x90}, 
     {0xb6 , 0x01}, 
     {0x90 , 0x01}, 
     {0x91 , 0x00},
     {0x92 , 0x00},
     {0x93 , 0x00},
     {0x94 , 0x00}, 
     {0x95 , 0x04},
     {0x96 , 0xb0},
     {0x97 , 0x06},
     {0x98 , 0x40},
     {0x18 , 0x02},
     {0x40 , 0x42},
     {0x41 , 0x00},
     {0x42 , 0x60},
     {0x43 , 0x54},
     {0x5e , 0x00},
     {0x5f , 0x00},
     {0x60 , 0x00},
     {0x61 , 0x00},
     {0x62 , 0x00},
     {0x63 , 0x00},
     {0x64 , 0x00},
     {0x65 , 0x00},
     {0x66 , 0x20},
     {0x67 , 0x20},
     {0x68 , 0x20},
     {0x69 , 0x20},
     {0x6a , 0x04},
     {0x6b , 0x04},
     {0x6c , 0x04},
     {0x6d , 0x04},
     {0x6e , 0x04},
     {0x6f , 0x04},
     {0x70 , 0x04},
     {0x71 , 0x04}, 
     {0x72 , 0xf0}, 
     {0x7e , 0x3c}, 
     {0x7f , 0x00}, 
     {0xfe , 0x01},
     {0x01 , 0x08}, 
     {0x02 , 0xc0}, 
     {0x03 , 0x04}, 
     {0x04 , 0x90}, 
     {0x05 , 0x30}, 
     {0x06 , 0x98}, 
     {0x07 , 0x28}, 
     {0x08 , 0x6c}, 
     {0x09 , 0x00}, 
     {0x0a , 0xc2}, 
     {0x0b , 0x11}, 
     {0x0c , 0x10}, 
     {0x13 , 0x2a}, 
     {0x17 , 0x00}, 
     {0x1b , 0x04},
     {0x1c , 0x11}, 
     {0x1e , 0x61}, 
     {0x1f , 0x30}, 
     {0x20 , 0x40}, 
     {0x22 , 0x80}, 
     {0x23 , 0x20}, 
     {0x12 , 0x35}, 
     {0x15 , 0x50}, 
     {0x10 , 0x31}, 
     {0x3e , 0x28}, 
     {0x3f , 0xe0}, 
     {0x40 , 0xe0}, 
     {0x41 , 0x08}, 
     {0xfe , 0x02}, 
     {0x0f , 0x05},
     {0x90 , 0x6c}, 
     {0x91 , 0x02}, 
     {0x92 , 0x44}, 
     {0x97 , 0x64}, 
     {0x98 , 0x88},
     {0x9d , 0x08}, 
     
     {0xa2 , 0x11}, 
     {0xfe , 0x00},
     {0xfe , 0x02},
     {0x80 , 0xc1}, 
     {0x81 , 0x08}, 
     {0x82 , 0x05}, 
     {0x83 , 0x04}, 
     {0x84 , 0x0a}, 
     {0x86 , 0x80}, 
     {0x87 , 0x30}, 
     {0x88 , 0x15}, 
     {0x89 , 0x80}, 
     {0x8a , 0x60}, 
     {0x8b , 0x30}, 
     {0xfe , 0x01}, 
     {0x21 , 0x14}, 
     {0xfe , 0x02}, 
     {0x3c , 0x06}, 
     {0x3d , 0x40},
     {0x48 , 0x30}, 
     {0x49 , 0x06}, 
     {0x4b , 0x08}, 
     {0x4c , 0x20}, 
     {0xa3 , 0x40},
     {0xa4 , 0x20},
     {0xa5 , 0x40},
     {0xa6 , 0x80}, 
     {0xab , 0x20}, 
     {0xae , 0x0c}, 
     {0xb3 , 0x42}, 
     {0xb4 , 0x24}, 
     {0xb6 , 0x50}, 
     {0xb7 , 0x01}, 
     {0xb9 , 0x25}, 
     {0xfe , 0x00},
     {0xfe , 0x02},
     {0x10 , 0x0a},
     {0x11 , 0x12},
     {0x12 , 0x19},
     {0x13 , 0x1f},
     {0x14 , 0x2c},
     {0x15 , 0x38},
     {0x16 , 0x42},
     {0x17 , 0x4e},
     {0x18 , 0x63},
     {0x19 , 0x76},
     {0x1a , 0x87},
     {0x1b , 0x96},
     {0x1c , 0xa2},
     {0x1d , 0xb8},
     {0x1e , 0xcb},
     {0x1f , 0xd8},
     {0x20 , 0xe2},
     {0x21 , 0xe9},
     {0x22 , 0xf0},
     {0x23 , 0xf8},
     {0x24 , 0xfd},
     {0x25 , 0xff},
     {0xfe , 0x02},
     {0x26 , 0x0d},
     {0x27 , 0x12},
     {0x28 , 0x17},
     {0x29 , 0x1c},
     {0x2a , 0x27},
     {0x2b , 0x34},
     {0x2c , 0x44},
     {0x2d , 0x55},
     {0x2e , 0x6e},
     {0x2f , 0x81},
     {0x30 , 0x91},
     {0x31 , 0x9c},
     {0x32 , 0xaa},
     {0x33 , 0xbb},
     {0x34 , 0xca},
     {0x35 , 0xd5},
     {0x36 , 0xe0},
     {0x37 , 0xe7},
     {0x38 , 0xed},
     {0x39 , 0xf6},
     {0x3a , 0xfb},
     {0x3b , 0xff},
     {0xfe , 0x02},
     {0xd1 , 0x32}, 
     {0xd2 , 0x32}, 
     {0xdd , 0x14}, 
     {0xde , 0x88}, 
     {0xed , 0x80},
     {0xfe , 0x01}, 
     {0xc2 , 0x15},
     {0xc3 , 0x0c},
     {0xc4 , 0x0b},
     {0xc8 , 0x12},
     {0xc9 , 0x0b},
     {0xca , 0x07},
     {0xbc , 0x3e},
     {0xbd , 0x2e},
     {0xbe , 0x2d},
     {0xb6 , 0x3e},
     {0xb7 , 0x2e},
     {0xb8 , 0x2d},
     {0xc5 , 0x00},
     {0xc6 , 0x00},
     {0xc7 , 0x00},
     {0xcb , 0x00},
     {0xcc , 0x00},
     {0xcd , 0x00},
     {0xbf , 0x09},
     {0xc0 , 0x00},
     {0xc1 , 0x00},
     {0xb9 , 0x09},
     {0xba , 0x00},
     {0xbb , 0x00},
     {0xaa , 0x01},
     {0xab , 0x0f},
     {0xac , 0x0d},
     {0xad , 0x00},
     {0xae , 0x06},
     {0xaf , 0x08},
     {0xb0 , 0x00},
     {0xb1 , 0x06},
     {0xb2 , 0x02},
     {0xb3 , 0x01},
     {0xb4 , 0x08},
     {0xb5 , 0x05},
     {0xd0 , 0x00},
     {0xd1 , 0x00},
     {0xd2 , 0x00},
     {0xd6 , 0x00},
     {0xd7 , 0x00},
     {0xd8 , 0x00},
     {0xd9 , 0x00},
     {0xda , 0x00},
     {0xdb , 0x00},
     {0xd3 , 0x00},
     {0xd4 , 0x00},
     {0xd5 , 0x00},
     {0xa4 , 0x00},
     {0xa5 , 0x00},
     {0xa6 , 0x77},
     {0xa7 , 0x77},
     {0xa8 , 0x77},
     {0xa9 , 0x77},
     {0xa1 , 0x80},
     {0xa2 , 0x80},
     {0xfe , 0x01},
     {0xdc , 0x35},
     {0xdd , 0x28},
     {0xdf , 0x0c}, //1d
     {0xe0 , 0x70},
     {0xe1 , 0x80},
     {0xe2 , 0x80},
     {0xe3 , 0x80},
     {0xe6 , 0x90},
     {0xe7 , 0x50},
     {0xe8 , 0x90},
     {0xe9 , 0x60},
     {0xfe , 0x00},
     {0xfe , 0x01},
     {0x4f , 0x00},
     {0x4f , 0x00},
     {0x4b , 0x01},
     {0x4f , 0x00},
     {0x4c , 0x01},
     {0x4d , 0x6f},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0x70},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0x8f},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0x90},
     {0x4e , 0x02}, //light
     {0x4c , 0x01},
     {0x4d , 0x91},
     {0x4c , 0x01},
     {0x4d , 0xaf},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xb0},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xb1},
     {0x4c , 0x01},
     {0x4d , 0xcf},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xd0},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xed},
     {0x4e , 0x33}, //light
     {0x4c , 0x01},
     {0x4d , 0xcd},
     {0x4e , 0x33}, //light
     {0x4c , 0x01},
     {0x4d , 0xec},
     {0x4e , 0x03}, //light
     {0x4c , 0x01},
     {0x4d , 0x6c},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0x6d},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0x6e},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0x8c},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0x8d},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0x8e},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xab},
     {0x4c , 0x01},
     {0x4d , 0xac},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0xad},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0xae},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xcb},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0xcc},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0xce},
     {0x4e , 0x02},
     {0x4c , 0x01},
     {0x4d , 0xea},
     {0x4c , 0x01},
     {0x4d , 0xec},
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0xee},
     {0x4e , 0x03},
     {0x4c , 0x02},
     {0x4d , 0x0c},
     {0x4e , 0x03},
     {0x4c , 0x02},
     {0x4d , 0x0d},
     {0x4e , 0x03},
     {0x4c , 0x02},
     {0x4d , 0x2c}, //OUT SUNSHINE
     {0x4e , 0x03},
     {0x4c , 0x01},
     {0x4d , 0xca},
     {0x4e , 0x34}, //light
     {0x4c , 0x01},
     {0x4d , 0xcb},
     {0x4c , 0x01},
     {0x4d , 0xeb},
     {0x4c , 0x01},
     {0x4d , 0xc9}, 
     {0x4c , 0x01},
     {0x4d , 0xa9}, 
     {0x4c , 0x01},
     {0x4d , 0xe9}, 
     {0x4e , 0x04},
     {0x4c , 0x01},
     {0x4d , 0xc9}, 
     {0x4c , 0x02},
     {0x4d , 0x0a}, 
     {0x4e , 0x05},
     {0x4c , 0x02},
     {0x4d , 0x0b},
     {0x4e , 0x35}, //light
     {0x4c , 0x02},
     {0x4d , 0x09},
     {0x4c , 0x01},
     {0x4d , 0xea},
     {0x4c , 0x02},
     {0x4d , 0x2a},
     {0x4c , 0x02},
     {0x4d , 0x49},
     {0x4c , 0x02},
     {0x4d , 0x29},
     {0x4c , 0x02},
     {0x4d , 0xc8},
     {0x4e , 0x36}, //light 100lux
     {0x4c , 0x02},
     {0x4d , 0xa8},
     {0x4e , 0x36}, //light
     {0x4c , 0x02},
     {0x4d , 0x88},
     {0x4e , 0x06},
     {0x4c , 0x02},
     {0x4d , 0xa9},
     {0x4e , 0x06}, //light
     {0x4c , 0x02},
     {0x4d , 0xc9},
     {0x4e , 0x06}, //light
     {0x4c , 0x02},
     {0x4d , 0x89},
     {0x4e , 0x06}, //400lux
     {0x4c , 0x02},
     {0x4d , 0x69},
     {0x4e , 0x06}, //f12
     {0x4c , 0x02},
     {0x4d , 0x6a},
     {0x4c , 0x02},
     {0x4d , 0xc7},
     {0x4e , 0x07},
     {0x4c , 0x02},
     {0x4d , 0xe7},
     {0x4e , 0x07}, //100lux
     {0x4c , 0x03},
     {0x4d , 0x06},
     {0x4c , 0x03},
     {0x4d , 0x07},
     {0x4e , 0x37}, //light
     {0x4c , 0x03},
     {0x4d , 0x08},
     {0x4e , 0x07}, //light
     {0x4c , 0x02},
     {0x4d , 0xe8},
     {0x4e , 0x07},
     {0x4c , 0x03},
     {0x4d , 0x28},
     {0x4e , 0x07},
     {0x4f , 0x01},
     {0xfe , 0x01},
     {0x50 , 0x80},
     {0x51 , 0xa8},
     {0x52 , 0x57},
     {0x53 , 0x38},
     {0x54 , 0xc7},
     {0x56 , 0x0e},
     {0x58 , 0x08},
     {0x5b , 0x00},
     {0x5c , 0x74},
     {0x5d , 0x8b},
     {0x61 , 0xd3},
     {0x62 , 0x90},
     {0x63 , 0x04},
     {0x65 , 0x04},
     {0x67 , 0xb2},
     {0x68 , 0xac},
     {0x69 , 0x00},
     {0x6a , 0xb2},
     {0x6b , 0xac},
     {0x6c , 0xdc},
     {0x6d , 0xb0},
     {0x6e , 0x30},
     {0x6f , 0xff},
     {0x73 , 0x00},
     {0x70 , 0x05},
     {0x71 , 0x80},
     {0x72 , 0xc1},
     {0x74 , 0x01},
     {0x75 , 0x01},
     {0x7f , 0x0c},
     {0x76 , 0x70},
     {0x77 , 0x48},
     {0x78 , 0x90},
     {0x79 , 0x55},
     {0x7a , 0x48},
     {0x7b , 0x60},
     {0xfe , 0x00},
     {0xfe , 0x02},
     {0xc0 , 0x01},
     {0xc1 , 0x50},
     {0xc2 , 0xf8},
     {0xc3 , 0x02},
     {0xc4 , 0xe0},
     {0xc5 , 0x45},
     {0xc6 , 0xe8},
     {0xc7 , 0x55},
     {0xc8 , 0xf5},
     {0xc9 , 0x00},
     {0xca , 0xea},
     {0xcb , 0x45},
     {0xcc , 0xf0},
     {0xCd , 0x45},
     {0xce , 0xf0},
     {0xcf , 0x00},
     {0xe3 , 0xf0},
     {0xe4 , 0x45},
     {0xe5 , 0xe8},
     {0xfe , 0x00},
     {0xf2 , 0x0f},
     {0xfe , 0x00},///banding
     {0x05 , 0x01},
     {0x06 , 0x56},
     {0x07 , 0x00},
     {0x08 , 0x32},//32
     {0xfe , 0x01},
     {0x25 , 0x00},
     {0x26 , 0xfa}, //step 250
     {0x27 , 0x04},
     {0x28 , 0xe2}, //20 fps 00
     {0x29 , 0x05},
     {0x2a , 0xdc}, //14 fps 01
     {0x2b , 0x06},
     {0x2c , 0xd6}, //10 fps 10
     {0x2d , 0x0b},
     {0x2e , 0xb8}, //8  fps 11
     {0x3c , 0x40},    

     {0xfe , 0x03},  //page 3
     {0x01 , 0x83},  
     {0x02 , 0x22},  //52-800mV
     {0x03 , 0x10},  
     {0x04 , 0x10},  //fifo_prog 
     {0x05 , 0x00},  //fifo_prog 
     {0x06 , 0x88},  //YUV ISP data  
     {0x10 , 0x84}, //94 // last bit  lane num 
     {0x11 , 0x1e},  //LDI set YUV422
     {0x12 , 0x40}, //LWC[7:0]  
     {0x13 , 0x06}, //LWC[15:8]
     {0x15 , 0x10},  //DPHYY_MODE read_ready 12  
     {0x17 , 0xf0},  //  01wdiv set 

     {0x21 , 0x03},//LPX         4 * 16.7 =67 ns
     {0x22 , 0x03},
     {0x23 , 0x03},
     {0x24 , 0x10},
     {0x29 , 0x03},//H-prepare   4 * 16.7  = 67    //T = 200
     {0x2a , 0x07},//H-zero      8 * 16.7  = 133   //settel = 100
     {0xfe , 0x00},
};

static struct v4l2_subdev_info gc2155_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};



//for preview setting
static struct msm_camera_i2c_reg_conf gc2155_svga_settings[] = {
    //SVGA  out size
    // 0xf7 , 0x1d
    // 0xf8 , 0x84
    {0xfa ,  0x00},  
    {0xfd ,  0x01}, //scaler

    // crop window
    {0xfe ,  0x00},
    {0xb6 ,  0x01},//aec on
    {0x90 ,  0x01}, //crop
    {0x91 ,  0x00},
    {0x92 ,  0x00},
    {0x93 ,  0x00},
    {0x94 ,  0x00},
    {0x95 ,  0x02},
    {0x96 ,  0x58},
    {0x97 ,  0x03},
    {0x98 ,  0x20},
    {0x99 ,  0x11}, //subsample
    {0x9a ,  0x06},
    //    AWB
    {0xfe ,  0x00},                     
    {0xec ,  0x01},  
    {0xed ,  0x02},  
    {0xee ,  0x30},  
    {0xef ,  0x48},  
    {0xfe ,  0x01},  
    {0x74 ,  0x00},  
    // AEC
    {0xfe ,  0x01},
    {0x01 ,  0x04},
    {0x02 ,  0x60},
    {0x03 ,  0x02},
    {0x04 ,  0x48},
    {0x05 ,  0x18},
    {0x06 ,  0x4c},
    {0x07 ,  0x14},
    {0x08 ,  0x36},
    {0x0a ,  0xc0},
    {0x21 ,  0x14},
    {0xfe ,  0x00},
    //MIPI
    {0xfe ,  0x03},  //page 3
    {0x04 ,  0x10},  // fifo_prog 
    {0x12 ,  0x40}, //LWC[7:0]  //
    {0x13 ,  0x06}, //LWC[15:8]
    {0xfe ,  0x00},
};

static struct msm_camera_i2c_reg_conf gc2155_sleep_settings[] = {

};


static struct msm_camera_i2c_reg_conf gc2155_reg_saturation[11][4] = {
	{
		{0xfe, 0x02},{0xd1, 0x1a},{0xd2, 0x1a},{0xfe, 0x00},//0
	},
	{
		{0xfe, 0x02},{0xd1, 0x1e},{0xd2, 0x1e},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x22},{0xd2, 0x22},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x26},{0xd2, 0x26},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x2a},{0xd2, 0x2a},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x32},{0xd2, 0x32},{0xfe, 0x00},//deault
	},
	{
		{0xfe, 0x02},{0xd1, 0x3a},{0xd2, 0x3a},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x40},{0xd2, 0x40},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x46},{0xd2, 0x46},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x4c},{0xd2, 0x4c},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd1, 0x54},{0xd2, 0x54},{0xfe, 0x00},//10
	},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_contrast[11][3] = {
	{
		{0xfe, 0x02},{0xd3, 0x18},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x20},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x28},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x30},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x38},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x40},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x48},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x50},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x58},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x60},{0xfe, 0x00},
	},
	{
		{0xfe, 0x02},{0xd3, 0x68},{0xfe, 0x00},
	},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_sharpness[6][3] = {
	{{0xfe, 0x02},{0x97, 0x26},{0xfe, 0x00}},//Sharpness -2
	{{0xfe, 0x02},{0x97, 0x37},{0xfe, 0x00}},//Sharpness -1
	{{0xfe, 0x02},{0x97, 0x45},{0xfe, 0x00}},//Sharpness
	{{0xfe, 0x02},{0x97, 0x59},{0xfe, 0x00}},//Sharpness +1
	{{0xfe, 0x02},{0x97, 0x6a},{0xfe, 0x00}},//Sharpness +2
	{{0xfe, 0x02},{0x97, 0x7b},{0xfe, 0x00}},//Sharpness +3
};
static struct msm_camera_i2c_reg_conf gc2155_reg_iso[7][2] = {
//not supported
	/* auto */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* auto hjt */  
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 100 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 200 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 400 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 800 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
	/* iso 1600 */
	{
		{0xfe, 0x00},
		{0xfe, 0x00},
	},
};
static struct msm_camera_i2c_reg_conf gc2155_reg_exposure_compensation[5][3] = {
	{{0xfe, 0x01},{0x13, 0x10},{0xfe, 0x00}},//Exposure -2
	{{0xfe, 0x01},{0x13, 0x20},{0xfe, 0x00}},//Exposure -1
	{{0xfe, 0x01},{0x13, 0x30},{0xfe, 0x00}},//Exposure     //70
	{{0xfe, 0x01},{0x13, 0x40},{0xfe, 0x00}},//Exposure +1
	{{0xfe, 0x01},{0x13, 0x50},{0xfe, 0x00}},//Exposure +2
};
static struct msm_camera_i2c_reg_conf gc2155_reg_antibanding[4][17] = {
	/* OFF */  //60-1  50-2   auto-off NC
	{
       {0xfe , 0x00},
       {0x05 , 0x01},
       {0x06 , 0x56},
       {0x07 , 0x00},
       {0x08 , 0x32},
       {0xfe , 0x01},
       {0x25 , 0x00},
       {0x26 , 0xfa}, //step    250
       {0x27 , 0x04},
       {0x28 , 0xe2}, //20  fps 00
       {0x29 , 0x06},
       {0x2a , 0xd6}, //14  fps 01
       {0x2b , 0x09},
       {0x2c , 0xc4}, //10  fps 10
       {0x2d , 0x0e},
       {0x2e , 0xa6}, //6.6 fps 11 bb8-8
       {0xfe , 0x00},
	}, /*ANTIBANDING 60HZ*/
	
	/* 60Hz */
	{
       {0xfe , 0x00},
       {0x05 , 0x01},
       {0x06 , 0x56},
       {0x07 , 0x00},
       {0x08 , 0x32},
       {0xfe , 0x01},
       {0x25 , 0x00},
       {0x26 , 0xd0}, //step     208  0x3c->0x40
       {0x27 , 0x04},
       {0x28 , 0xe0}, //19fps 00
       {0x29 , 0x07},
       {0x2a , 0x50}, //13.3fps 01
       {0x2b , 0x09},
       {0x2c , 0xc0}, //10fps 10
       {0x2d , 0x0d},
       {0x2e , 0xd0}, //7fps 11
       {0xfe , 0x00},
	}, /*ANTIBANDING 50HZ*/

	/* 50Hz */
	{
       {0xfe , 0x00},
       {0x05 , 0x01},
       {0x06 , 0x56},
       {0x07 , 0x00},
       {0x08 , 0x32},
       {0xfe , 0x01},
       {0x25 , 0x00},
       {0x26 , 0xfa}, //step    250 
       {0x27 , 0x04},
       {0x28 , 0xe2}, //20fps 00
       {0x29 , 0x06},
       {0x2a , 0xd6}, //14fps 01
       {0x2b , 0x09},
       {0x2c , 0xc4}, //10fps 10
       {0x2d , 0x0b},
       {0x2e , 0xb8}, //8fps 11
       {0xfe , 0x00},
	}, /*ANTIBANDING 60HZ*/
	/* AUTO */
	{
       {0xfe , 0x00},
       {0x05 , 0x01},
       {0x06 , 0x56},
       {0x07 , 0x00},
       {0x08 , 0x32},
       {0xfe , 0x01},
       {0x25 , 0x00},
       {0x26 , 0xfa}, //step
       {0x27 , 0x04},
       {0x28 , 0xe2}, //20fps 00
       {0x29 , 0x06},
       {0x2a , 0xd6}, //14fps 01
       {0x2b , 0x09},
       {0x2c , 0xc4}, //10fps 10
       {0x2d , 0x0d},
       {0x2e , 0xd0}, //8fps 11
       {0xfe , 0x00},
	},/*ANTIBANDING 50HZ*/
};

//begin effect
static struct msm_camera_i2c_reg_conf gc2155_reg_effect_normal[] = {
	/* normal: */
	{0x83, 0x00},//0xe0
};

static struct msm_camera_i2c_reg_conf gc2155_reg_effect_black_white[] = {
	/* B&W: */
	{0x83, 0x12},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_effect_negative[] = {
	/* Negative: */
	{0x83, 0x01},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_effect_old_movie[] = {
	/* Sepia(antique): */
	{0x83, 0x82},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_effect_solarize[] = {
	{0xfe, 0x00},
};
// end effect


//begin scene, not realised
static struct msm_camera_i2c_reg_conf gc2155_reg_scene_auto[] = {
	/* <SCENE_auto> */
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_scene_night[] = {
	/* <SCENE_NIGHT> */
	{0xfe, 0x00},
};
//end scene


//begin white balance
static struct msm_camera_i2c_reg_conf gc2155_reg_wb_auto[] = {
	/* Auto: */
	{0xb3, 0x61},{0xb4, 0x40},{0xb5, 0x61},{0x82, 0xfa},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_wb_sunny[] = {
	/* Sunny: */
	{0x82, 0xf8},{0xb3, 0x58},{0xb4, 0x40},{0xb5, 0x50},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_wb_cloudy[] = {
	/* Cloudy: */
	{0x82, 0xf8},{0xb3, 0x8c},{0xb4, 0x50},{0xb5, 0x40},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_wb_office[] = {
	/* Office: */
	{0x82, 0xf8},{0xb3, 0x72},{0xb4, 0x40},{0xb5, 0x5b},
};

static struct msm_camera_i2c_reg_conf gc2155_reg_wb_home[] = {
	/* Home: */
	{0x82, 0xf8},{0xb3, 0x50},{0xb4, 0x40},{0xb5, 0xa8},
};
//end white balance


static const struct i2c_device_id gc2155_i2c_id[] = {
	{GC2155_SENSOR_NAME, (kernel_ulong_t)&gc2155_s_ctrl},
	{ }
};


static struct msm_camera_i2c_client gc2155_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id gc2155_dt_match[] = {
	{.compatible = "qcom,gc2155", .data = &gc2155_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gc2155_dt_match);

static struct platform_driver gc2155_platform_driver = {
	.driver = {
		.name = "qcom,gc2155",
		.owner = THIS_MODULE,
		.of_match_table = gc2155_dt_match,
	},
};
static int32_t msm_gc2155_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gc2155_s_ctrl);
}

static struct i2c_driver gc2155_i2c_driver = {
	.id_table = gc2155_i2c_id,
	.probe  = msm_gc2155_i2c_probe,
	.driver = {
		.name = GC2155_SENSOR_NAME,
	},
};
static void gc2155_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}

static int32_t gc2155_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	gc2155_i2c_write_table(s_ctrl, &gc2155_sleep_settings[0],
		ARRAY_SIZE(gc2155_sleep_settings));
	return msm_sensor_power_down(s_ctrl);
}

static int32_t gc2155_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(gc2155_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gc2155_init_module(void)
{
	int32_t rc;
	CDBG("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&gc2155_platform_driver,
		gc2155_platform_probe);
	if (!rc)
		return rc;
	CDBG("%s:%d\n", __func__, __LINE__);
		
	return i2c_add_driver(&gc2155_i2c_driver);

}

static void __exit gc2155_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gc2155_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc2155_s_ctrl);
		platform_driver_unregister(&gc2155_platform_driver);
	} else
		i2c_del_driver(&gc2155_i2c_driver);
	return;
}

static int32_t gc2155_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid_H = 0;
	uint16_t chipid_L = 0;
	uint16_t chipid = 0;

	CDBG("%s, E. calling i2c_read:, i2c_addr:%d, id_reg_addr:%d\n",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0xf0,
			&chipid_H, MSM_CAMERA_I2C_BYTE_DATA);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0xf1,
			&chipid_L, MSM_CAMERA_I2C_BYTE_DATA);
			
	chipid = (chipid_H << 8) | (chipid_L & 0xff) ;
	
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	//CDBG("%s:  read id: %x expected id gc2155:\n", __func__, chipid);
	CDBG("gc2155_PETER-2155-read chipid = %x" , chipid);
	
		  
	if (chipid != 0x2155) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	return rc;
}

static void gc2155_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{

	gc2155_i2c_write_table(s_ctrl, &gc2155_reg_saturation[value][0],
		ARRAY_SIZE(gc2155_reg_saturation[value]));


		
}

static void gc2155_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	gc2155_i2c_write_table(s_ctrl, &gc2155_reg_contrast[value][0],
		ARRAY_SIZE(gc2155_reg_contrast[value]));
}

static void gc2155_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	CDBG("%s %d\n", __func__, value);
	gc2155_i2c_write_table(s_ctrl, &gc2155_reg_sharpness[val][0],
		ARRAY_SIZE(gc2155_reg_sharpness[val]));
}


static void gc2155_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	gc2155_i2c_write_table(s_ctrl, &gc2155_reg_iso[value][0],
		ARRAY_SIZE(gc2155_reg_iso[value]));
}

static void gc2155_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	CDBG("%s %d\n", __func__, value);
	gc2155_i2c_write_table(s_ctrl, &gc2155_reg_exposure_compensation[val][0],
		ARRAY_SIZE(gc2155_reg_exposure_compensation[val]));
}

static void gc2155_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_effect_normal[0],
			ARRAY_SIZE(gc2155_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_effect_black_white[0],
			ARRAY_SIZE(gc2155_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_effect_negative[0],
			ARRAY_SIZE(gc2155_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_effect_old_movie[0],
			ARRAY_SIZE(gc2155_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_effect_solarize[0],
			ARRAY_SIZE(gc2155_reg_effect_solarize));
		break;
	}
	default:
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_effect_normal[0],
			ARRAY_SIZE(gc2155_reg_effect_normal));
	}
}

static void gc2155_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	CDBG("gc2155_PETER£¬gc2155_set_antibanding = %x" , value);
	gc2155_i2c_write_table(s_ctrl, &gc2155_reg_antibanding[value][0],
		ARRAY_SIZE(gc2155_reg_antibanding[value]));
}

static void gc2155_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_scene_auto[0],
			ARRAY_SIZE(gc2155_reg_scene_auto));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_scene_night[0],
			ARRAY_SIZE(gc2155_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_scene_landscape[0],
			ARRAY_SIZE(gc2155_reg_scene_landscape));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_scene_portrait[0],
			ARRAY_SIZE(gc2155_reg_scene_portrait));
					break;
	}
	default:
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_scene_auto[0],
			ARRAY_SIZE(gc2155_reg_scene_auto));
	}
}

static void gc2155_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_wb_auto[0],
			ARRAY_SIZE(gc2155_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_wb_home[0],
			ARRAY_SIZE(gc2155_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_wb_sunny[0],
			ARRAY_SIZE(gc2155_reg_wb_sunny));
					break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_wb_office[0],
			ARRAY_SIZE(gc2155_reg_wb_office));
					break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_wb_cloudy[0],
			ARRAY_SIZE(gc2155_reg_wb_cloudy));
					break;
	}
	default:
		gc2155_i2c_write_table(s_ctrl, &gc2155_reg_wb_auto[0],
		ARRAY_SIZE(gc2155_reg_wb_auto));
	}
}




void gc2155_set_shutter(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t temp = 1;
	uint16_t shutter = 1;
	uint16_t shutter_h = 1,shutter_L = 1 ;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0xfe,
		0x00, MSM_CAMERA_I2C_BYTE_DATA); 
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0xb6,
		0x00, MSM_CAMERA_I2C_BYTE_DATA);  //AEC off
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x03,
			&shutter_h, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x04,
			&shutter_L, MSM_CAMERA_I2C_BYTE_DATA);

	shutter = (shutter_h << 8) | (shutter_L & 0xff) ;
    
	CDBG("gc2155_PETER£¬preview_shutter = %d" , shutter);
	shutter = shutter / 2;
	if (shutter < 1)
		shutter = 1;
	shutter = shutter & 0x1fff;
	temp = shutter & 0xff;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x04,
		temp, MSM_CAMERA_I2C_BYTE_DATA);

	temp = (shutter >> 8) & 0xff;		
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x03,
		temp, MSM_CAMERA_I2C_BYTE_DATA);		
}

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

static int gc2155_set_flash_light(enum led_brightness brightness)
{
	struct led_classdev *led_cdev;

	CDBG("--CAMERA-- gc2155_set_flash_light brightness = %d\n", brightness);

	down_read(&leds_list_lock);
	list_for_each_entry(led_cdev, &leds_list, node) {
		if (!strncmp(led_cdev->name, "flashlight", 10)) {
			break;
		}
	}
	up_read(&leds_list_lock);

	if (led_cdev) {
		led_brightness_set(led_cdev, brightness);
	} else {
		printk("--CAMERA-- get flashlight device failed\n");
		return -1;
	}

	return 0;
}

static int gc2155_get_current_lux(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t tmp = 0;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0xfe, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_read(s_ctrl->sensor_i2c_client,
		0x14, &tmp, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0xfe, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

	if(rc < 0){
		printk("%s: failed, rc = %d\n", __func__, rc);
		return rc;
	}
	return tmp;
}

int32_t gc2155_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
    
	mutex_lock(s_ctrl->msm_sensor_mutex);
	printk("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		CDBG("init setting\n");
		gc2155_i2c_write_table(s_ctrl,
				&gc2155_recommend_settings[0],
				ARRAY_SIZE(gc2155_recommend_settings));
		CDBG("init setting X\n");
		break;
	case CFG_SET_RESOLUTION:  //peter
	{
		int val = 0;
		int lux = 0;

		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		CDBG("gc2155_PETER-preview/capture-VAL = %d\n" , val);

		if (val == 0)
		{
			lux = gc2155_get_current_lux(s_ctrl);
			CDBG("current lux = %d" , lux);
			if(( (flash_mode == 1)&&(lux < 25)&&(lux >= 0) ) ||
			   (flash_mode == 2)){
				gc2155_set_flash_light(LED_FULL);
				is_flash_work = 1;
			}
			gc2155_set_shutter(s_ctrl);
		
			gc2155_i2c_write_table(s_ctrl, &gc2155_uxga_settings[0],
				ARRAY_SIZE(gc2155_uxga_settings));
			msleep(250);
		}
		else if (val == 1)
		{
	        if(is_flash_work == 1){
				gc2155_set_flash_light(LED_OFF);
				is_flash_work = 0;
	        }
			gc2155_i2c_write_table(s_ctrl, &gc2155_svga_settings[0],
				ARRAY_SIZE(gc2155_svga_settings));
		}
	}
		break;
	case CFG_SET_STOP_STREAM:
		gc2155_i2c_write_table(s_ctrl,
			&gc2155_stop_settings[0],
			ARRAY_SIZE(gc2155_stop_settings));
		break;
	case CFG_SET_START_STREAM:
		gc2155_i2c_write_table(s_ctrl,
			&gc2155_start_settings[0],
			ARRAY_SIZE(gc2155_start_settings));
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:

		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;

		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
		}
		case CFG_SET_SATURATION: {
			int32_t sat_lev;
			if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
	
		gc2155_set_stauration(s_ctrl, sat_lev);

		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
			if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);

		gc2155_set_contrast(s_ctrl, con_lev);

		break;
		}
		case CFG_SET_SHARPNESS: {
			int32_t shp_lev;
			if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);

		gc2155_set_sharpness(s_ctrl, shp_lev);

		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: ISO Value is %d\n", __func__, iso_lev);

		gc2155_set_iso(s_ctrl, iso_lev);
		break;
		}
	case CFG_SET_EXPOSURE_COMPENSATION: {

		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
				//if(0)
		gc2155_set_exposure_compensation(s_ctrl, ec_lev);

		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Effect mode is %d\n", __func__, effect_mode);
	
		gc2155_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: anti-banding mode is %d\n", __func__,
			antibanding_mode);
	
		gc2155_set_antibanding(s_ctrl, antibanding_mode);
		break;
		}
	case CFG_SET_BESTSHOT_MODE: {

		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: best shot mode is %d\n", __func__, bs_mode);
	
		gc2155_set_scene_mode(s_ctrl, bs_mode);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: white balance is %d\n", __func__, wb_mode);

		gc2155_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	case CFG_SET_FLASH_MODE: {
		if (copy_from_user(&flash_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		printk("%s: flash_mode is %d", __func__, flash_mode);

		break;
		}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t gc2155_sensor_func_tbl = {
	.sensor_config = gc2155_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = gc2155_sensor_power_down,
	.sensor_match_id = gc2155_sensor_match_id,
};

static struct msm_sensor_ctrl_t gc2155_s_ctrl = {
	.sensor_i2c_client = &gc2155_sensor_i2c_client,
	.power_setting_array.power_setting = gc2155_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gc2155_power_setting),
	.msm_sensor_mutex = &gc2155_mut,
	.sensor_v4l2_subdev_info = gc2155_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc2155_subdev_info),
	.func_tbl = &gc2155_sensor_func_tbl,
};

module_init(gc2155_init_module);
module_exit(gc2155_exit_module);
MODULE_DESCRIPTION("GC2155 2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");

