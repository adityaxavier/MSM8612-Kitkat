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
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

#define OV8858_SENSOR_NAME "ov8858"
DEFINE_MSM_MUTEX(ov8858_mut);

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_sensor_ctrl_t ov8858_s_ctrl;

static struct msm_sensor_power_setting ov8858_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
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
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
};

static struct v4l2_subdev_info ov8858_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8858_i2c_id[] = {
	{OV8858_SENSOR_NAME,
		(kernel_ulong_t)&ov8858_s_ctrl},
	{ }
};

extern void camera_af_software_powerdown(struct i2c_client *client); ///@Hxin add
static int32_t msm_ov8858_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8858_s_ctrl);
}

static struct i2c_driver ov8858_i2c_driver = {
	.id_table = ov8858_i2c_id,
	.probe  = msm_ov8858_i2c_probe,
	.driver = {
		.name = OV8858_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8858_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

///@Hxin add for avoid current leak start
int32_t ov8858_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	rc = msm_sensor_power_up(s_ctrl);

	return rc;
}

int32_t ov8858_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	camera_af_software_powerdown(s_ctrl->sensor_i2c_client->client);
	rc = msm_sensor_power_down(s_ctrl);

	return rc;
}

int32_t ov8858_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	rc = msm_sensor_match_id(s_ctrl);

	return rc;
}

int32_t ov8858_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	int32_t rc = 0;

	rc = msm_sensor_config(s_ctrl, argp);

	return rc;
}

static struct msm_sensor_fn_t ov8858_sensor_fn_t = {
	.sensor_power_up = ov8858_power_up,
	.sensor_power_down = ov8858_power_down,
	.sensor_match_id = ov8858_match_id,
	.sensor_config = ov8858_config,
};
///@Hxin add for avoid current leak end

static struct msm_sensor_ctrl_t ov8858_s_ctrl = {
	.sensor_i2c_client = &ov8858_sensor_i2c_client,
	.power_setting_array.power_setting = ov8858_power_setting,
	.power_setting_array.size =
			ARRAY_SIZE(ov8858_power_setting),
	.msm_sensor_mutex = &ov8858_mut,
	.sensor_v4l2_subdev_info = ov8858_subdev_info,
	.sensor_v4l2_subdev_info_size =
			ARRAY_SIZE(ov8858_subdev_info),
	.func_tbl = &ov8858_sensor_fn_t, ///@Hxin add
};

static const struct of_device_id ov8858_dt_match[] = {
	{
		.compatible = "ovti,ov8858",
		.data = &ov8858_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, ov8858_dt_match);

static struct platform_driver ov8858_platform_driver = {
	.driver = {
		.name = "ovti,ov8858",
		.owner = THIS_MODULE,
		.of_match_table = ov8858_dt_match,
	},
};

static int32_t ov8858_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(ov8858_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov8858_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&ov8858_platform_driver,
		ov8858_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&ov8858_i2c_driver);
}

static void __exit ov8858_exit_module(void)
{
	if (ov8858_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8858_s_ctrl);
		platform_driver_unregister(&ov8858_platform_driver);
	} else
		i2c_del_driver(&ov8858_i2c_driver);
	return;
}

module_init(ov8858_init_module);
module_exit(ov8858_exit_module);
MODULE_DESCRIPTION("ov8858");
MODULE_LICENSE("GPL v2");
