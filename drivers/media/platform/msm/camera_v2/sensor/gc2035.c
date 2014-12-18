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
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "gc2035.h"
#include <linux/leds.h>
//#define CONFIG_MSMB_CAMERA_DEBUG

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) printk(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
#define GC2035_SENSOR_NAME "gc2035"
DEFINE_MSM_MUTEX(gc2035_mut);

static struct msm_sensor_ctrl_t gc2035_s_ctrl;
/*
*  flash mode: 0-off, 1-auto, 2-on, 3-torch
*/
static int flash_mode = 0;
static int is_flash_work = 0;
static struct msm_sensor_power_setting gc2035_power_setting[] = {
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
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
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
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
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
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

static struct v4l2_subdev_info gc2035_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static struct msm_camera_i2c_reg_conf gc2035_reg_saturation[][4] = {
	{{0xfe, 0x02},{0xd1, 0x10},{0xd2, 0x10},{0xfe, 0x00}},/* SATURATION LEVEL0*/
	{{0xfe, 0x02},{0xd1, 0x18},{0xd2, 0x18},{0xfe, 0x00}},/* SATURATION LEVEL1*/
	{{0xfe, 0x02},{0xd1, 0x20},{0xd2, 0x20},{0xfe, 0x00}},/* SATURATION LEVEL2*/
	{{0xfe, 0x02},{0xd1, 0x28},{0xd2, 0x28},{0xfe, 0x00}},/* SATURATION LEVEL3*/
	{{0xfe, 0x02},{0xd1, 0x30},{0xd2, 0x30},{0xfe, 0x00}},/* SATURATION LEVEL4*/
	{{0xfe, 0x02},{0xd1, 0x34},{0xd2, 0x34},{0xfe, 0x00}},/* SATURATION LEVEL5*/
	{{0xfe, 0x02},{0xd1, 0x40},{0xd2, 0x40},{0xfe, 0x00}},/* SATURATION LEVEL6*/
	{{0xfe, 0x02},{0xd1, 0x48},{0xd2, 0x48},{0xfe, 0x00}},/* SATURATION LEVEL7*/
	{{0xfe, 0x02},{0xd1, 0x50},{0xd2, 0x50},{0xfe, 0x00}},/* SATURATION LEVEL8*/
	{{0xfe, 0x02},{0xd1, 0x58},{0xd2, 0x58},{0xfe, 0x00}},/* SATURATION LEVEL9*/
	{{0xfe, 0x02},{0xd1, 0x60},{0xd2, 0x60},{0xfe, 0x00}},/* SATURATION LEVEL10*/
};

static struct msm_camera_i2c_reg_conf gc2035_reg_contrast[][4] = {
	{{0xfe, 0x02},{0xd3, 0x18},{0xfe, 0x00}}, /* CONTRAST L0*/
	{{0xfe, 0x02},{0xd3, 0x20},{0xfe, 0x00}}, /* CONTRAST L1*/
	{{0xfe, 0x02},{0xd3, 0x28},{0xfe, 0x00}}, /* CONTRAST L2*/
	{{0xfe, 0x02},{0xd3, 0x30},{0xfe, 0x00}}, /* CONTRAST L3*/
	{{0xfe, 0x02},{0xd3, 0x38},{0xfe, 0x00}}, /* CONTRAST L4*/
	{{0xfe, 0x02},{0xd3, 0x3b},{0xfe, 0x00}}, /* CONTRAST L5*///ethan 0x40
	{{0xfe, 0x02},{0xd3, 0x48},{0xfe, 0x00}}, /* CONTRAST L6*/
	{{0xfe, 0x02},{0xd3, 0x50},{0xfe, 0x00}}, /* CONTRAST L7*/
	{{0xfe, 0x02},{0xd3, 0x58},{0xfe, 0x00}}, /* CONTRAST L8*/
	{{0xfe, 0x02},{0xd3, 0x60},{0xfe, 0x00}}, /* CONTRAST L9*/
	{{0xfe, 0x02},{0xd3, 0x68},{0xfe, 0x00}}, /* CONTRAST L10*/
};

static struct msm_camera_i2c_reg_conf gc2035_reg_sharpness[][5] = {
	{{0xfe, 0x02},{0x97, 0x00},{0xfe, 0x00}}, /* SHARPNESS LEVEL 0*/
	{{0xfe, 0x02},{0x97, 0x24},{0xfe, 0x00}}, /* SHARPNESS LEVEL 1*/
	{{0xfe, 0x02},{0x97, 0x48},{0xfe, 0x00}}, /* SHARPNESS LEVEL 2*/
	{{0xfe, 0x02},{0x97, 0x7c},{0xfe, 0x00}}, /* SHARPNESS LEVEL 3*/
	{{0xfe, 0x02},{0x97, 0x9d},{0xfe, 0x00}}, /* SHARPNESS LEVEL 4*/
	{{0xfe, 0x02},{0x97, 0xbf},{0xfe, 0x00}}, /* SHARPNESS LEVEL 5*/
	{{0xfe, 0x02},{0x97, 0xff},{0xfe, 0x00}}, /* SHARPNESS LEVEL 6*/
};

static struct msm_camera_i2c_reg_conf gc2035_reg_iso[][4] = {
	{{0xfe, 0x01},{0x20, 0x60},{0xfe, 0x00},{0xb0, 0x80}}, /*ISO_AUTO*/
	{{0xfe, 0x01},{0x20, 0x60},{0xfe, 0x00},{0xb0, 0x80}}, /*ISO_DEBLUR*/
	{{0xfe, 0x01},{0x20, 0x40},{0xfe, 0x00},{0xb0, 0x60}}, /*ISO_100*/
	{{0xfe, 0x01},{0x20, 0x60},{0xfe, 0x00},{0xb0, 0x80}}, /*ISO_200*/
	{{0xfe, 0x01},{0x20, 0x80},{0xfe, 0x00},{0xb0, 0xa0}}, /*ISO_400*/
	{{0xfe, 0x01},{0x20, 0xc0},{0xfe, 0x00},{0xb0, 0xc0}}, /*ISO_800*/
	{{0xfe, 0x01},{0x20, 0xf0},{0xfe, 0x00},{0xb0, 0xe0}}, /*ISO_1600*/
};

static struct msm_camera_i2c_reg_conf gc2035_reg_exposure_compensation[][3] = {
	{{0xfe, 0x02},{0xd5, 0xd0},{0xfe, 0x00}}, /*EXPOSURECOMPENSATIONN2*/
	{{0xfe, 0x02},{0xd5, 0xe8},{0xfe, 0x00}}, /*EXPOSURECOMPENSATIONN1*/
	{{0xfe, 0x02},{0xd5, 0x03},{0xfe, 0x00}}, /*EXPOSURECOMPENSATIOND*///ethan 0x00
	{{0xfe, 0x02},{0xd5, 0x18},{0xfe, 0x00}}, /*EXPOSURECOMPENSATIONp1*/
	{{0xfe, 0x02},{0xd5, 0x30},{0xfe, 0x00}}, /*EXPOSURECOMPENSATIONP2*/
};

static struct msm_camera_i2c_reg_conf gc2035_reg_antibanding[][17] = {
	{{0xfe, 0x00},{0x05, 0x01},{0x06, 0x0a},{0x07, 0x00},{0x08, 0x58},{0xfe, 0x01},{0x27, 0x00},{0x28, 0xa1},
	{0x29, 0x05},{0x2a, 0x08},{0x2b, 0x06},{0x2c, 0x4a},{0x2d, 0x06},{0x2e, 0x4a},{0x2f, 0x09},{0x30, 0x6f},{0xfe, 0x00}}, /*ANTIBANDING 60HZ*/
	{{0xfe, 0x00},{0x05, 0x01},{0x06, 0x0a},{0x07, 0x00},{0x08, 0x58},{0xfe, 0x01},{0x27, 0x00},{0x28, 0xa1},
	{0x29, 0x05},{0x2a, 0x08},{0x2b, 0x06},{0x2c, 0x4a},{0x2d, 0x06},{0x2e, 0x4a},{0x2f, 0x09},{0x30, 0x6f},{0xfe, 0x00}}, /*ANTIBANDING 50HZ*/
	{{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},
	{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1}}, /*ANTIBANDING AUTO*/
};

//xionggh effect need recheck
static struct msm_camera_i2c_reg_conf gc2035_reg_effect_normal[] = {
	{0x83, 0xe0},
};

static struct msm_camera_i2c_reg_conf gc2035_reg_effect_black_white[] = {
	{0x83, 0x12}
};

static struct msm_camera_i2c_reg_conf gc2035_reg_effect_negative[] = {
	{0x83, 0x01}
};

static struct msm_camera_i2c_reg_conf gc2035_reg_effect_old_movie[] = {
	{0x83, 0x82}
};//not surport

static struct msm_camera_i2c_reg_conf gc2035_reg_effect_solarize[] = {
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc2035_reg_wb_auto[] = {
	{0xb3, 0x61},{0xb4, 0x40},{0xb5, 0x61},{0x82, 0xfe}
};

static struct msm_camera_i2c_reg_conf gc2035_reg_wb_sunny[] = {
	{0x82, 0xfc},{0xb3, 0x58},{0xb4, 0x40},{0xb5, 0x50}
};

static struct msm_camera_i2c_reg_conf gc2035_reg_wb_cloudy[] = {
	{0x82, 0xfc},{0xb3, 0x8c},{0xb4, 0x50},{0xb5, 0x40}
};

static struct msm_camera_i2c_reg_conf gc2035_reg_wb_office[] = {
	{0x82, 0xfc},{0xb3, 0x72},{0xb4, 0x40},{0xb5, 0x5b}
};/*INCANDISCENT*/

static struct msm_camera_i2c_reg_conf gc2035_reg_wb_home[] = {
	{0x82, 0xfc},{0xb3, 0x50},{0xb4, 0x40},{0xb5, 0xa8}
};//????/*WHITEBALNACE CUSTOM*/

static const struct i2c_device_id gc2035_i2c_id[] = {
	{GC2035_SENSOR_NAME,(kernel_ulong_t)&gc2035_s_ctrl},
	{ }
};

static int32_t msm_gc2035_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gc2035_s_ctrl);
}

static struct i2c_driver gc2035_i2c_driver = {
	.id_table = gc2035_i2c_id,
	.probe  = msm_gc2035_i2c_probe,
	.driver = {
		.name = GC2035_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc2035_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};


static const struct of_device_id gc2035_dt_match[] = {
	{
		.compatible = "qcom,gc2035",
		.data = &gc2035_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, gc2035_dt_match);

static struct platform_driver gc2035_platform_driver = {
	.driver = {
		.name = "qcom,gc2035",
		.owner = THIS_MODULE,
		.of_match_table = gc2035_dt_match,
	},
};

static int32_t gc2035_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(gc2035_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gc2035_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&gc2035_platform_driver,
		gc2035_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&gc2035_i2c_driver);
}

static void __exit gc2035_exit_module(void)
{
	if (gc2035_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc2035_s_ctrl);
		platform_driver_unregister(&gc2035_platform_driver);
	} else
		i2c_del_driver(&gc2035_i2c_driver);
	return;
}

static void gc2035_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, &gc2035_reg_saturation[value][0],
		ARRAY_SIZE(gc2035_reg_saturation[value]), MSM_CAMERA_I2C_BYTE_DATA);
}

static void gc2035_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, &gc2035_reg_contrast[value][0],
		ARRAY_SIZE(gc2035_reg_contrast[value]), MSM_CAMERA_I2C_BYTE_DATA);
}

static void gc2035_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	CDBG("%s %d", __func__, value);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, &gc2035_reg_sharpness[val][0],
		ARRAY_SIZE(gc2035_reg_sharpness[val]), MSM_CAMERA_I2C_BYTE_DATA);
}


static void gc2035_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, &gc2035_reg_iso[value][0],
		ARRAY_SIZE(gc2035_reg_iso[value]), MSM_CAMERA_I2C_BYTE_DATA);
}

static void gc2035_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	CDBG("%s %d", __func__, val);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, &gc2035_reg_exposure_compensation[val][0],
		ARRAY_SIZE(gc2035_reg_exposure_compensation[val]), MSM_CAMERA_I2C_BYTE_DATA);
}

static void gc2035_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_effect_normal[0],
			ARRAY_SIZE(gc2035_reg_effect_normal), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_effect_black_white[0],
			ARRAY_SIZE(gc2035_reg_effect_black_white), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_effect_negative[0],
			ARRAY_SIZE(gc2035_reg_effect_negative), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_effect_old_movie[0],
			ARRAY_SIZE(gc2035_reg_effect_old_movie), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_effect_solarize[0],
			ARRAY_SIZE(gc2035_reg_effect_solarize), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	default:
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_effect_normal[0],
			ARRAY_SIZE(gc2035_reg_effect_normal), MSM_CAMERA_I2C_BYTE_DATA);
	}
}

static void gc2035_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, &gc2035_reg_antibanding[value][0],
		ARRAY_SIZE(gc2035_reg_antibanding[value]), MSM_CAMERA_I2C_BYTE_DATA);
}

static void gc2035_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	CDBG("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_wb_auto[0],
			ARRAY_SIZE(gc2035_reg_wb_auto), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_wb_home[0],
			ARRAY_SIZE(gc2035_reg_wb_home), MSM_CAMERA_I2C_BYTE_DATA);
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_wb_sunny[0],
			ARRAY_SIZE(gc2035_reg_wb_sunny), MSM_CAMERA_I2C_BYTE_DATA);
					break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_wb_office[0],
			ARRAY_SIZE(gc2035_reg_wb_office), MSM_CAMERA_I2C_BYTE_DATA);
					break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_wb_cloudy[0],
			ARRAY_SIZE(gc2035_reg_wb_cloudy), MSM_CAMERA_I2C_BYTE_DATA);
					break;
	}
	default:
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, &gc2035_reg_wb_auto[0],
		ARRAY_SIZE(gc2035_reg_wb_auto), MSM_CAMERA_I2C_BYTE_DATA);
	}
}

void gc2035_set_shutter(struct msm_sensor_ctrl_t *s_ctrl)
{
	// write shutter, in number of line period
	unsigned short temp = 0;
	unsigned int shutter = 0;
	unsigned short ret_l, ret_h;

	ret_l = ret_h = 0;

	// turn off AEC & AGC
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
		0xb6, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(10);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
		0x03, &ret_h, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
		0x04, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);

	shutter = (ret_h << 8) | (ret_l & 0xff) ;

	//CDBG("gc2035_set_shutter shutter = %d, step_val = %d\n",shutter, step_val);

	shutter = shutter / 2;

	if (shutter < 1)
		shutter = 1;
	shutter = shutter & 0x1fff;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
		0xfa, 0x11, MSM_CAMERA_I2C_BYTE_DATA);

	temp = shutter & 0xff;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
			0x04, temp, MSM_CAMERA_I2C_BYTE_DATA);
	temp = shutter >> 8;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
			0x03, temp, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(200);
	CDBG("gc2035_set_shutter done");
}

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

static int gc2035_set_flash_light(enum led_brightness brightness)
{
	struct led_classdev *led_cdev;

	CDBG("--CAMERA-- gc2035_set_flash_light brightness = %d\n", brightness);

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

static int gc2035_get_current_lux(struct msm_sensor_ctrl_t *s_ctrl)
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

int32_t gc2035_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
			s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		CDBG("%s: %s CFG_GET_SENSOR_INFO\n", __func__, s_ctrl->sensordata->sensor_name);
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
		/* Write Recommend settings */
		CDBG("%s: %s CFG_SET_INIT_SETTING\n", __func__, s_ctrl->sensordata->sensor_name);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			gc2035_init_settings,
			ARRAY_SIZE(gc2035_init_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		msleep(30);

		break;
	case CFG_SET_RESOLUTION:{
		int val = 0;
		int lux = 0;
		CDBG("%s: %s CFG_SET_RESOLUTION\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s : CFG_SET_RESOLUTION, val = %d\n", __func__, val);
		if (val == 0){
			lux = gc2035_get_current_lux(s_ctrl);
			CDBG("current lux = %d" , lux);
			if(( (flash_mode == 1)&&(lux < 50)&&(lux >= 0) ) ||
			   (flash_mode == 2)){
				gc2035_set_flash_light(LED_FULL);
				is_flash_work = 1;
			}
			gc2035_set_shutter(s_ctrl);
			msleep(30);
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client,
				gc2035_snap_settings, ARRAY_SIZE(gc2035_snap_settings),
				MSM_CAMERA_I2C_BYTE_DATA);
		}else if (val == 1){
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client,
				gc2035_prev_settings, ARRAY_SIZE(gc2035_prev_settings),
				MSM_CAMERA_I2C_BYTE_DATA);
				msleep(300);
			}
		}
		msleep(50);
		break;
	case CFG_SET_STOP_STREAM:
		CDBG("%s: %s CFG_SET_STOP_STREAM\n", __func__, s_ctrl->sensordata->sensor_name);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			gc2035_stop_settings,
			ARRAY_SIZE(gc2035_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		msleep(50);
		break;
	case CFG_SET_START_STREAM:
		CDBG("%s: %s CFG_SET_START_STREAM\n", __func__, s_ctrl->sensordata->sensor_name);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
			gc2035_start_settings,
			ARRAY_SIZE(gc2035_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		msleep(100);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		CDBG("%s: %s CFG_GET_SENSOR_INIT_PARAMS\n", __func__, s_ctrl->sensordata->sensor_name);
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

		CDBG("%s: %s CFG_SET_SLAVE_INFO\n", __func__, s_ctrl->sensordata->sensor_name);
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

		CDBG("%s: %s CFG_WRITE_I2C_ARRAY\n", __func__, s_ctrl->sensordata->sensor_name);
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

		CDBG("%s: %s CFG_WRITE_I2C_SEQ_ARRAY\n", __func__, s_ctrl->sensordata->sensor_name);
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
		CDBG("%s: %s CFG_POWER_UP\n", __func__, s_ctrl->sensordata->sensor_name);
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		CDBG("%s: %s CFG_POWER_DOWN\n", __func__, s_ctrl->sensordata->sensor_name);
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		CDBG("%s: %s CFG_SET_STOP_STREAM_SETTING\n", __func__, s_ctrl->sensordata->sensor_name);
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

		CDBG("%s: %s CFG_SET_SATURATION\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Saturation Value is %d", __func__, sat_lev);
		gc2035_set_stauration(s_ctrl, sat_lev);
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t con_lev;

		CDBG("%s: %s CFG_SET_CONTRAST\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Contrast Value is %d", __func__, con_lev);
		gc2035_set_contrast(s_ctrl, con_lev);
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;

		CDBG("%s: %s CFG_SET_SHARPNESS\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Sharpness Value is %d", __func__, shp_lev);
		gc2035_set_sharpness(s_ctrl, shp_lev);
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;

		CDBG("%s: %s CFG_SET_ISO\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: ISO Value is %d", __func__, iso_lev);
		gc2035_set_iso(s_ctrl, iso_lev);
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;

		CDBG("%s: %s CFG_SET_EXPOSURE_COMPENSATION\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		gc2035_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;

		CDBG("%s: %s CFG_SET_EFFECT\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: Effect mode is %d", __func__, effect_mode);
		gc2035_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;

		CDBG("%s: %s CFG_SET_ANTIBANDING\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		gc2035_set_antibanding(s_ctrl, antibanding_mode);
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;

		CDBG("%s: %s CFG_SET_BESTSHOT_MODE\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: best shot mode is %d", __func__, bs_mode);
		//gc2035_set_scene_mode(s_ctrl, bs_mode);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;

		CDBG("%s: %s CFG_SET_WHITE_BALANCE\n", __func__, s_ctrl->sensordata->sensor_name);
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: white balance is %d", __func__, wb_mode);
		gc2035_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
static struct msm_sensor_fn_t gc2035_sensor_func_tbl = {
	.sensor_config = gc2035_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
};
static struct msm_sensor_ctrl_t gc2035_s_ctrl = {
	.sensor_i2c_client = &gc2035_sensor_i2c_client,
	.power_setting_array.power_setting = gc2035_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gc2035_power_setting),
	.msm_sensor_mutex = &gc2035_mut,
	.sensor_v4l2_subdev_info = gc2035_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc2035_subdev_info),
	.func_tbl = &gc2035_sensor_func_tbl,
};
module_init(gc2035_init_module);
module_exit(gc2035_exit_module);
MODULE_DESCRIPTION("gc2035");
MODULE_LICENSE("GPL v2");
