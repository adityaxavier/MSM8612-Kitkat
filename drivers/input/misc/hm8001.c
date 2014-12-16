/*
  * hm8001.c - Linux kernel modules for ambient light sensor
  *
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with this program; if not, write to the Free Software
  *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
  */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/timer.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* POWER SUPPLY VOLTAGE RANGE */
#define STK3X1X_VDD_MIN_UV	2000000
#define STK3X1X_VDD_MAX_UV	3300000
#define STK3X1X_VIO_MIN_UV	1750000
#define STK3X1X_VIO_MAX_UV	1950000

#include "hm8001.h"

#define DRIVER_VERSION		   "2.1"

#define DEBUG_TAG              "[hm8001]: "

static int hm8001_debug_mask = 1;
module_param_named(debug_mask, hm8001_debug_mask,int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DEBUG_ERR(fmt, args...)   \
	do { \
		if(hm8001_debug_mask & 0x01) \
			printk(KERN_ERR  DEBUG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args); \
		}while (0)
#define DEBUG_WAR(fmt, args...)   \
	do { \
		if(hm8001_debug_mask & 0x01) \
			printk(KERN_WARNING fmt, ##args); \
		}while (0)
#define DEBUG_LOG(fmt, args...)   \
	do { \
		if(hm8001_debug_mask & 0x01) \
			printk(KERN_INFO DEBUG_TAG fmt, ##args); \
		}while (0)

#define DEBUG_DBG(fmt, args...)   \
	do { \
		if(hm8001_debug_mask & 0x01) \
			printk(KERN_DEBUG fmt, ##args); \
		}while (0)


#define PATH_LEN					64
#define LIGHT_SENSOR_NAME			"light"
#define PROXIMITY_SENSOR_NAME		"proximity"

struct hm8001als_data {
	struct i2c_client*  client;
	struct hm8001_platform_data *pdata;
	struct work_struct  alsps_work;
	struct wake_lock    alsps_wakelock;
	struct delayed_work light_work;
	
	u16 als_intensity;
	u16 ps_intensity;
	u16 als_register;
	u16 ps_register;
	u8  irq_enable;
	
	atomic_t light_delay;
	atomic_t light_enable;
	atomic_t proximity_enable;

	struct input_dev* light_input;
	struct input_dev* proximity_input;
	
#ifndef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
	bool use_fir;


};

struct hm8001als_data *hm8001_obj = NULL;
static struct i2c_client *hm8001_i2c_client = NULL;
//struct hm8001alsps_platform_data *hm8001_platform_data = NULL;
static u16 hm8001lux_table[] = {50, 200, 450, 650, 850}; // DLS Level convert to Lux

// device configuration
unsigned int ps_work_flag = 0;
static u16 previous_als_intensity = 0;


static void hm8001_proximity_report(void);
static void hm8001_light_report(void);
#if 0//ndef CONFIG_HAS_EARLYSUSPEND
static void hm8001als_early_suspend(struct early_suspend *h);
static void hm8001als_late_resume(struct early_suspend *h);
#endif


static int hm8001_power_ctl(struct hm8001als_data *hm8001ps, bool on)
{
	int ret = 0;
	printk("%s",__func__);

	if (!on && hm8001ps->power_enabled) {
		ret = regulator_disable(hm8001ps->vdd);
		if (ret) {
			dev_err(&hm8001ps->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(hm8001ps->vio);
		if (ret) {
			dev_err(&hm8001ps->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			regulator_enable(hm8001ps->vdd);
			return ret;
		}
		hm8001ps->power_enabled = on;
		dev_dbg(&hm8001ps->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else if (on && !hm8001ps->power_enabled) {

		ret = regulator_enable(hm8001ps->vdd);
		if (ret) {
			dev_err(&hm8001ps->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(hm8001ps->vio);
		if (ret) {
			dev_err(&hm8001ps->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(hm8001ps->vdd);
			return ret;
		}
		hm8001ps->power_enabled = on;
		dev_dbg(&hm8001ps->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&hm8001ps->client->dev,
				"Power on=%d. enabled=%d\n",
				on, hm8001ps->power_enabled);
	}

	return ret;
}

static int hm8001_power_init(struct hm8001als_data *hm8001ps, bool on)
{
	int ret;
	printk("%s on%d\n",__func__,on);
	if (!on) {
		if (regulator_count_voltages(hm8001ps->vdd) > 0)
			regulator_set_voltage(hm8001ps->vdd,
					0, STK3X1X_VDD_MAX_UV);

		regulator_put(hm8001ps->vdd);

		if (regulator_count_voltages(hm8001ps->vio) > 0)
			regulator_set_voltage(hm8001ps->vio,
					0, STK3X1X_VIO_MAX_UV);

		regulator_put(hm8001ps->vio);
	} else {
		printk("regulator_get\n");
		hm8001ps->vdd = regulator_get(&hm8001ps->client->dev, "vdd");
		if (IS_ERR(hm8001ps->vdd)) {
			ret = PTR_ERR(hm8001ps->vdd);
			dev_err(&hm8001ps->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		printk("regulator_count_voltages\n");

		if (regulator_count_voltages(hm8001ps->vdd) > 0) {
			ret = regulator_set_voltage(hm8001ps->vdd,
					STK3X1X_VDD_MIN_UV,
					STK3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&hm8001ps->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}
		printk("regulator_count vio\n");

		hm8001ps->vio = regulator_get(&hm8001ps->client->dev, "vio");
		if (IS_ERR(hm8001ps->vio)) {
			ret = PTR_ERR(hm8001ps->vio);
			dev_err(&hm8001ps->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(hm8001ps->vio) > 0) {
			ret = regulator_set_voltage(hm8001ps->vio,
					STK3X1X_VIO_MIN_UV,
					STK3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&hm8001ps->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(hm8001ps->vio);
reg_vdd_set:
	if (regulator_count_voltages(hm8001ps->vdd) > 0)
		regulator_set_voltage(hm8001ps->vdd, 0, STK3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(hm8001ps->vdd);
	return ret;
}


static int hm8001_irq_query(void)
{
	return gpio_get_value(80) ? 1 : 0;
}


static int hm8001_enable_func(int func, int enable)
{
	int ret = 0;
	static u8 hm8001_control_param = 0;
	static u8 hm8001_control_param2 = 0;
	static int ps_control_status = 0;
	static int als_control_status = 0;
	
	if (ALS_CONTROL_ENABLE == func) {
		als_control_status = enable;
	} else if (PS_CONTROL_ENABLE == func) {
		ps_control_status = enable;
	} else {
		return -EINVAL;
	}

    if (als_control_status && ps_control_status)
        hm8001_control_param = INT_OUTPUT_ENABLE | PS_INT_MODE | ALS_CONTROL_ENABLE | PS_CONTROL_ENABLE; //0x17;
    else if (ps_control_status)
        hm8001_control_param = INT_OUTPUT_ENABLE | ALS_INT_MODE | PS_INT_MODE | PS_CONTROL_ENABLE; //0x1D;
    else if (als_control_status)
        hm8001_control_param = INT_OUTPUT_ENABLE | ALS_CONTROL_ENABLE; //0x12;
    else
	    hm8001_control_param = POWER_DOWN_MODE;  //INT_PS out control = 1, active mode = 1 (enable int out, power down)

       hm8001_control_param2 = 0x10;     //INT_PS out control = 1, active mode = 0 (enable int out, power on)
	ret = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_OPCON, hm8001_control_param2);
	msleep(2); // add delay fuction  delay 2ms
	ret = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_OPCON, hm8001_control_param);
	msleep(2); // add delay fuction  delay 2ms
	if (ret < 0) {
		DEBUG_ERR("%s HM8001_OPCON register i2c write failed.\n", __FUNCTION__);
		return ret;
	}

	return ret;
}

static int hm8001_init_alsdevice(void)
{
	int err = 0;
	
	if (hm8001_obj == NULL) {
		DEBUG_ERR("hm8001_init_alsdevice(): hm8001_obj null.\n");
		return -EINVAL;
	}
	
	hm8001_obj->als_intensity = LUX_DEFAULT;
	previous_als_intensity = LUX_DEFAULT;
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_WAIT_TIME, ALS_WAIT_TIME_50MS);
	if (err < 0) {
		DEBUG_ERR("ALS_WAIT_TIME register i2c write failed.\n");
		return err;
	}

	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_ANA_WAKEUP_TIME, ALS_ANA_WAKEUP_TIME_20US);
	if (err < 0) {
		DEBUG_ERR("ALS_ANA_WAKEUP_TIME register i2c write failed.\n");
		return err;
	}

#if 0  //Do not use this arg in polling mode
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_CON, 0x00);
	if (err < 0) {
		DEBUG_ERR("ALS_CON register i2c write failed.\n");
		return err;
	}
#endif	
    err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_INTG, 0x64);   // 0x64
	if (err < 0) {
		DEBUG_ERR("ALS_INTG register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_STEPH, 0x0f); 
	if (err < 0) {
		DEBUG_ERR("ALS_STEPH register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_STEPL, 0xE0);
	if (err < 0) {
		DEBUG_ERR("ALS_STEPL register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_ANA_CON1, 0x26);// add gain  //0x26  //0x27
	if (err < 0) {
		DEBUG_ERR("ALS_ANA_CON1 register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_ALS_ANA_CON2, ALS_ANA_CAP0P5); // 1 * 160fF   //0x01  //  0x02
	if (err < 0) {
		DEBUG_ERR("ALS_ANA_CON2 register i2c write failed.\n");
		return err;
	}
	
	return err;
}

static int hm8001_init_psdevice(void)
{
	int err = 0;
	if (hm8001_obj == NULL) {
		DEBUG_ERR("hm8001_init_device(): hm8001_obj null\n");
		return -EINVAL;
	}
	hm8001_obj->ps_intensity = PROX_DETECTION_FAR;
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_CON, PS_POLLING_BIT_HIGH);
	if (err < 0) {
		DEBUG_ERR("PS_CON register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_BIT_CHECK, BIT_CHECK_OBJ);
	if (err < 0) {
		DEBUG_ERR("PS_BIT_CHECK register i2c write failed.\n");
		return err;
	}
	
	#if defined(CONFIG_CALA01) || defined(CONFIG_UALA01)
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_MTIME, 0x18); // [7:0] : Margin time (1 step = margin counter step * 1.333us)
	#else
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_MTIME, 0x08); // [7:0] : Margin time (1 step = margin counter step * 1.333us)
	#endif
	
	if (err < 0) {
		DEBUG_ERR("PS_MTIME register i2c write failed.\n");
		return err;
	} 
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_CYC_THR, CYC_THR_COUNT);
	if (err < 0) {
		DEBUG_ERR("PS_CYC_THR register i2c write failed.\n");
		return err;
	}
	
	#if defined(CONFIG_CALA01) || defined(CONFIG_UALA01)
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_LED_DET, PS_DET_ILED80MA | PS_DET_WIDTH22US661);  //0xEE
	#else
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_LED_DET, PS_DET_ILED140MA | PS_DET_WIDTH22US661);  //0xEE
	#endif
	if (err < 0) {
		DEBUG_ERR("PS_LED_DET register i2c write failed.\n");
		return err;
	}

	#if defined(CONFIG_CALA01) || defined(CONFIG_UALA01)
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_LED_REL, PS_DET_ILED125MA | PS_DET_WIDTH22US661);  //0x6E
	#else
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_LED_REL, PS_DET_ILED175MA | PS_DET_WIDTH22US661);  //0x6E
	#endif
	if (err < 0) {
		DEBUG_ERR("PS_LED_REL register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_LED_PAT, 0xFF); // PS LED pattern data
	if (err < 0){
		DEBUG_ERR("PS_LED_PAT register i2c write failed.\n");
		return err;
	}

	#if defined(CONFIG_CALA01) || defined(CONFIG_UALA01)
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_ANA_CON1, DET_ANA39MV | REL_ANA75MV); //0x87// ps thread contrl, bigger and closer, the distance  detected object closer than release
	#else
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_ANA_CON1, DET_ANA36MV | REL_ANA69MV); //0x87// ps thread contrl, bigger and closer, the distance  detected object closer than release
	#endif
	if (err < 0) {
		DEBUG_ERR("_PS_ANA_CON1 register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_ANA_CON2, PS_NOISE_CANCEL |
																		   PS_CONV_GAIN2M | 
																           PS_COMMON_VOL);  //0x45
	if(err < 0){
		DEBUG_ERR("PS_ANA_CON2 register i2c write failed.\n");
		return err;
	}
	
	err = i2c_smbus_write_byte_data(hm8001_i2c_client, HM8001_PS_ANA_CON3, PS_ANA_FUSE_ENABLE |
																		   PS_R3SEL_BANDWIDTH500K | 
																		   PS_ANA_CAP2_SEL |
																           PS_ANA_CAP3_SEL);  //0x8C
	if (err < 0) {
		DEBUG_ERR("PS_ANA_CON3 register i2c write failed.\n");
		return err;
	}

	return err;
}

static void hm8001_get_als_value(bool reset_thresh_enable) 
{
	u8 i = 0;
	static u32 raw_lux = 0;
	s32 als_cnt_high   = 0;
	s32 als_cnt_middle = 0;
	s32 als_cnt_low    = 0;

	als_cnt_high = i2c_smbus_read_byte_data(hm8001_i2c_client, HM8001_ALS_CNT_H);
	if (als_cnt_high < 0) {
		DEBUG_ERR("ALS_CNT_H register i2c read failed.\n");
	}
	
	als_cnt_middle = i2c_smbus_read_byte_data(hm8001_i2c_client, HM8001_ALS_CNT_M);
	if (als_cnt_middle < 0) {
		DEBUG_ERR("ALS_CNT_M register i2c read failed.\n");
	}
	
	als_cnt_low = i2c_smbus_read_byte_data(hm8001_i2c_client, HM8001_ALS_CNT_L);
	if (als_cnt_low < 0) {
		DEBUG_ERR("ALS_CNT_L register i2c read failed.\n");
	}
	
	raw_lux = als_cnt_high << 16 | als_cnt_middle << 8 | als_cnt_low;
	
	DEBUG_LOG("raw_lux = %u.\n", raw_lux);
	
    /* set als threshold */
	if (reset_thresh_enable) {
		if (raw_lux > 4000) {
			i = 4;
		} else if (raw_lux > 2000) {
			i = 3;
		} else if (raw_lux > 300) {
			i = 2;
		} else if (raw_lux > 50) {
			i = 1;
		} else {
			i = 0;
		}
	}
		
	hm8001_obj->als_intensity = hm8001lux_table[i];
	
	return ;
}

static int hm8001_get_ps_value(bool reset_thresh_enable) 
{
	int ret = 0;
	
	if ((ret = i2c_smbus_read_byte_data(hm8001_i2c_client, HM8001_PS_DET)) < 0) {
		DEBUG_LOG("%s read ps value error %d\n", __FUNCTION__, ret);
		return ret;
	}
	
	printk("ps detection data = %d.\n", ret);
	
	if (ret) {
		hm8001_obj->ps_intensity = PROX_DETECTION_CLOSE;
	} else {
		hm8001_obj->ps_intensity = PROX_DETECTION_FAR;
	}
	
	return ret;
}

static void hm8001_handle_interrupt(void)
{	
	int interrupt_status = 0;
	
	if ((interrupt_status = i2c_smbus_read_byte_data(hm8001_i2c_client, HM8001_INT_FLAG)) < 0) {
		DEBUG_LOG("%s read interrupt error %d.\n", __FUNCTION__, interrupt_status);
		return ;
	}

	DEBUG_LOG("interrupt status = %d.\n", interrupt_status);

	switch(interrupt_status) {
		case HM8001_PS_INT: 
		case HM8001_ALSPS_INT: 
			hm8001_get_ps_value(TRUE);
 			hm8001_proximity_report();
			break;
            
		default:
			DEBUG_ERR("%s unknow interrupt.\n", __FUNCTION__);
			break;
	}
}

static void alsps_work_handler(struct work_struct *work)
{
	if (hm8001_obj == NULL) {
		DEBUG_ERR("alsps_work_handler(): hm8001_obj null\n");
		return;
	}

	hm8001_handle_interrupt();
	
	return;
}

static irqreturn_t alsps_irq_handler(int irq, void *dev_id)
{
	
	if (hm8001_obj == NULL) {
		DEBUG_ERR("alsps_irq_handler(): hm8001_obj null.\n");
		return IRQ_NONE;
	}

	printk("hm8001_irq_query %d\n",hm8001_irq_query());
	if (!hm8001_irq_query())
	{
		DEBUG_LOG("alsps interrupter no generate.\n");
		return IRQ_NONE;						
	}

    wake_lock_timeout(&hm8001_obj->alsps_wakelock, 2 * HZ);
	schedule_work(&hm8001_obj->alsps_work);
	
	return IRQ_HANDLED;
}

static int hm8001_proximity_enable(void)
{
	int err = 0;
	if (hm8001_obj == NULL) {
		DEBUG_ERR("hm8001als_proximity_poweron(): hm8001_obj null\n");
		return -EINVAL;
	}

	/*if (!atomic_read(&hm8001_obj->light_enable) && !atomic_read(&hm8001_obj->proximity_enable)) {
		hm8001_platform_data->power_control(POWER_ON);
	}*/

	atomic_set(&hm8001_obj->proximity_enable, TRUE);
	ps_work_flag = 1;

	if (hm8001_irq_query()) 
	{
		hm8001_handle_interrupt();
	}
	
	if (!hm8001_obj->irq_enable) {
		enable_irq(hm8001_i2c_client->irq);
		hm8001_obj->irq_enable = TRUE;
	}

	err = hm8001_init_psdevice();
	if (err < 0) {
		DEBUG_ERR("%s device init failed.\n", __FUNCTION__);
		return err;
	}

	hm8001_enable_func(PS_CONTROL_ENABLE, TRUE);
	
	hm8001_proximity_report();
	
	printk("proximity sensor enable OK!\n");
	return err;
}


static int hm8001_proximity_disable(void)
{
	int err = 0;
	if (hm8001_obj == NULL) {
		DEBUG_ERR("als_proximity_poweroff(): hm8001_obj null\n");
		return -EINVAL;
	}

	atomic_set(&hm8001_obj->proximity_enable, FALSE);

    hm8001_enable_func(PS_CONTROL_ENABLE, FALSE);
	
	
	if (hm8001_obj->irq_enable) {
		disable_irq(hm8001_i2c_client->irq);
		hm8001_obj->irq_enable = FALSE;
	}	

	/*if(!atomic_read(&hm8001_obj->light_enable))
	{
		hm8001_platform_data->power_control(POWER_DOWN);
	}*/
	
	ps_work_flag = 0;
	
	printk("proximity sensor disable OK!\n");

	return err;
}

static void hm8001_proximity_report(void)
{
	if (hm8001_obj == NULL) {
		DEBUG_ERR("als_proximity_report(): hm8001_obj null\n");
		return;
	}

	if (atomic_read(&hm8001_obj->proximity_enable)) {
		DEBUG_LOG("ps value %d.\n", hm8001_obj->ps_intensity);
	}
	else {
		DEBUG_LOG("ps not ready, ps value %d.\n", hm8001_obj->ps_intensity);
	}
    if (NULL != hm8001_obj->proximity_input)
    {
        input_report_abs(hm8001_obj->proximity_input, ABS_DISTANCE, hm8001_obj->ps_intensity);
        input_sync(hm8001_obj->proximity_input);
    }
}

static int hm8001_light_enable(void)
{	
	int err = 0;
	if (hm8001_obj == NULL) {
		DEBUG_ERR("%s hm8001_obj null.\n", __FUNCTION__);
		return -EINVAL;
	}

	/*if (!atomic_read(&hm8001_obj->light_enable) && !atomic_read(&hm8001_obj->proximity_enable)) {
		hm8001_platform_data->power_control(POWER_ON);
	}*/

	atomic_set(&hm8001_obj->light_enable, TRUE);
	
	err = hm8001_init_alsdevice();
	if (err < 0) {
		DEBUG_ERR("%s device init failed.\n", __FUNCTION__);
		return err;
	}

	hm8001_enable_func(ALS_CONTROL_ENABLE, TRUE);
	
	hm8001_light_report();
	schedule_delayed_work(&hm8001_obj->light_work,\
				msecs_to_jiffies(atomic_read(&hm8001_obj->light_delay)));
	
	printk("light sensor enbale OK!\n");
	return err;
}


static int hm8001_light_disable(void)
{
	int err = 0;
	if (hm8001_obj == NULL) {
		DEBUG_ERR("%s hm8001_obj null.\n", __FUNCTION__);
		return -EINVAL;;;
	}

	atomic_set(&hm8001_obj->light_enable, FALSE);

    hm8001_enable_func(ALS_CONTROL_ENABLE, FALSE);
	
	/*if (!atomic_read(&hm8001_obj->proximity_enable)) {
		hm8001_platform_data->power_control(POWER_DOWN);
	}*/

	cancel_delayed_work_sync(&hm8001_obj->light_work);
	atomic_set(&hm8001_obj->light_delay, DEFAULT_DELAY);
	printk("light sensor disable OK!\n");

	return err;
}

static void hm8001_light_report(void)
{
	if (hm8001_obj == NULL) {
		DEBUG_ERR("als_light_report(): hm8001_obj null\n");
		return;
	}
	DEBUG_LOG("hm8001_light_report Lux value %d.\n", hm8001_obj->als_intensity);
    if (NULL != hm8001_obj->proximity_input)
    {
        input_report_abs(hm8001_obj->light_input, ABS_MISC, hm8001_obj->als_intensity);
        input_sync(hm8001_obj->light_input);
    }
}

static void light_work_func(struct work_struct *work)
{
	unsigned long delay = msecs_to_jiffies(atomic_read(&hm8001_obj->light_delay));
	
	if (atomic_read(&hm8001_obj->light_enable)) {
		hm8001_get_als_value(TRUE);
		DEBUG_LOG("Lux value %d.\n", hm8001_obj->als_intensity);
	}
	else {
		DEBUG_LOG("Lux not ready, Lux value %d.\n", hm8001_obj->als_intensity);
	}
	
	if(previous_als_intensity != hm8001_obj->als_intensity)
	{
		hm8001_light_report();
		previous_als_intensity = hm8001_obj->als_intensity;
	}
	else
	{
		DEBUG_LOG("ALS data not change.\n");
	}
	
	schedule_delayed_work(&hm8001_obj->light_work, delay);
}

/******************************************************************************
 								Proximity sysfs attributes
*******************************************************************************/
static ssize_t hm8001_proximity_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", atomic_read(&hm8001_obj->proximity_enable) ? "enable" : "disable");
}

static ssize_t hm8001_proximity_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;
    int	ret;
    
	ret = sscanf(buf, "%d", &enable);
	
	if(ret == 1)
	{
		if(enable)
		{
			hm8001_proximity_enable();
		}
		else
		{
			hm8001_proximity_disable();
		}
	}
	
	return count;
}

static int hm8001_sysfs_create_proximity_group(void)
{
	int err;
	static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			hm8001_proximity_enable_show, hm8001_proximity_enable_store);
	
	static struct attribute *hm8001_proximity_attributes[] =
	{
		&dev_attr_enable.attr,
		NULL
	};
	
	static struct attribute_group hm8001_proximity_attribute_group =
	{
		.attrs = hm8001_proximity_attributes
	};
	
	err = sysfs_create_group(&hm8001_obj->proximity_input->dev.kobj,
				&hm8001_proximity_attribute_group);
	if(err < 0)
	{
		DEBUG_ERR("hm8001 sysfs create proximity sys group fail.\n");
		sysfs_remove_group(&hm8001_obj->proximity_input->dev.kobj, 
				&hm8001_proximity_attribute_group);
	}

	return err;
}

/******************************************************************************
 								Light sysfs attributes
*******************************************************************************/
static ssize_t hm8001_light_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", atomic_read(&hm8001_obj->light_enable) ? "enable" : "disable");
}

static ssize_t hm8001_light_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;
    int	ret;
    
	ret = sscanf(buf, "%d", &enable);
	
	if(ret == 1)
	{
		if(enable)
		{
			hm8001_light_enable();
		}
		else
		{
			hm8001_light_disable();
		}
	}

	return count;
}


static ssize_t hm8001_light_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&hm8001_obj->light_delay));
}

static ssize_t hm8001_light_poll_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ns;
	int	delay;
	int ret;

    ret = sscanf(buf, "%d", &ns);
	delay = ns ; 

    if(ret == 1)
    {
		if(FTM_LIGHT_SENSOR_DELAY == delay)
		{
            if(atomic_read(&hm8001_obj->light_enable))
			{
                hm8001_light_disable();
            }
				
                atomic_set(&hm8001_obj->light_delay, 200);
                hm8001_light_enable();
		    }
		    else
		    {   
			    atomic_set(&hm8001_obj->light_delay, delay);
			}
		}

	return count;
}

static int hm8001_sysfs_create_light_group(void)
{
	int err;
	static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			hm8001_light_enable_show, hm8001_light_enable_store);
	static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			hm8001_light_poll_delay_show, hm8001_light_poll_delay_store);
	
	static struct attribute *hm8001_light_attributes[] =
	{
		&dev_attr_enable.attr,
		&dev_attr_poll_delay.attr,
		NULL
	};
	
	static struct attribute_group hm8001_light_attribute_group =
	{
		.attrs = hm8001_light_attributes
	};
	
	err = sysfs_create_group(&hm8001_obj->light_input->dev.kobj,
				&hm8001_light_attribute_group);
	if(err < 0)
	{
		DEBUG_ERR("hm8001 sysfs create light sys group fail.\n");
		sysfs_remove_group(&hm8001_obj->light_input->dev.kobj, 
				&hm8001_light_attribute_group);
	}

	return err;

}

static int hm8001_proximity_input_init(struct hm8001als_data *hm8001ps)
{
	struct input_dev *dev;
	int err;

	/* create proximity input devices */
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = PROXIMITY_SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(dev, ABS_DISTANCE, 0, 255, 0, 0);
	input_set_drvdata(dev, hm8001ps);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		hm8001ps->proximity_input = NULL;
		printk(KERN_ERR "register hm8001 input device fali\n");
		return err;
	}
	hm8001ps->proximity_input = dev;

	return 0;
}

static int hm8001_light_input_init(struct hm8001als_data *hm8001als)
{
	struct input_dev *dev;
	int err;

	/* create light input devices */
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = LIGHT_SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_MISC, 0, 1000000, 0, 0);
	input_set_drvdata(dev, hm8001als);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		hm8001als->light_input = NULL;
		return err;
	}
	hm8001als->light_input = dev;

	return 0;
}

static void hm8001_input_delete(struct hm8001als_data *hm8001alsps)
{
	if (hm8001alsps->proximity_input != NULL) {
		input_unregister_device(hm8001alsps->proximity_input);
		input_free_device(hm8001alsps->proximity_input);
	}

	if (hm8001alsps->light_input != NULL) {
		input_unregister_device(hm8001alsps->light_input);
		input_free_device(hm8001alsps->light_input);
	}
}

#ifdef CONFIG_OF
 static int hm8001_parse_dt(struct device *dev,
			 struct hm8001_platform_data *pdata)
 {
	 //int rc;
	 struct device_node *np = dev->of_node;
	 pdata->int_pin = of_get_named_gpio_flags(np, "hm,irq-gpio",
				 0, &pdata->int_flags);
	 if (pdata->int_pin < 0) {
		 dev_err(dev, "Unable to read irq-gpio\n");
		 return pdata->int_pin;
	 }
 

	 pdata->use_fir = of_property_read_bool(np, "hm ,use-fir");
 
	 return 0;
 }
#else
 static int hm8001_parse_dt(struct device *dev,
			 struct hm8001_platform_data *pdata)
 {
	 return -ENODEV;
 }
#endif /* !CONFIG_OF */



 /*
  * I2C probing/exit functions
  */
static int __devinit hm8001_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct hm8001_platform_data *pdata;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA | I2C_SMBUS_I2C_BLOCK_DATA)) {
		DEBUG_ERR("%s - i2c smbus functions unsupported.\n", __FUNCTION__);
		err = -EIO;
		goto exit;
	}

	hm8001_obj = kzalloc(sizeof(struct hm8001als_data), GFP_KERNEL);
	if (hm8001_obj == NULL) {
		err = -ENOMEM;
	 	goto exit;
	}

		hm8001_obj->client = client;
		hm8001_i2c_client = client;
	if (client->dev.of_node) {
			pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
			if (!pdata) {
				dev_err(&client->dev, "Failed to allocate memory for pdata\n");
				err = -ENOMEM;
				goto exit;
			}
	
			err = hm8001_parse_dt(&client->dev, pdata);
			if (err) {
				dev_err(&client->dev, "Failed to get pdata from device tree\n");
				goto exit;
			}
		} else {
			pdata = client->dev.platform_data;
			if (!pdata) {
				dev_err(&client->dev, "%s: Assign platform_data error!!\n",
						__func__);
				err = -EBUSY;
				goto exit;
			}
		}

		if (gpio_is_valid(pdata->int_pin )) {
		err = gpio_request(pdata->int_pin , "hm8001_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto exit;
		}
		err = gpio_direction_input(pdata->int_pin);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto exit;
		}
	}

	wake_lock_init(&hm8001_obj->alsps_wakelock, WAKE_LOCK_SUSPEND, "hm8001alsps");
	
	i2c_set_clientdata(client, hm8001_obj);
	
	INIT_WORK(&hm8001_obj->alsps_work, alsps_work_handler);

	
	err = hm8001_power_init(hm8001_obj, true);
			if (err)
				goto exit;
		
			err = hm8001_power_ctl(hm8001_obj, true);
			if (err)
				goto exit;
	//hm8001_platform_data = client->dev.platform_data;
	//hm8001_platform_data->init_irq();

        //hm8001_default_init();
		
	//hm8001_platform_data->power_control(POWER_ON);
	
	err = hm8001_enable_func(ALS_CONTROL_ENABLE, FALSE);
	if (err < 0) {
		DEBUG_ERR("ambient light device init fail.\n");
		goto kfree_exit;
	} 

	err = hm8001_enable_func(PS_CONTROL_ENABLE, FALSE);
	if (err < 0) {
		DEBUG_ERR("proximity device init fail.\n");
		goto kfree_exit;
	} 
		
	err = request_irq(client->irq, alsps_irq_handler, IRQF_TRIGGER_RISING, "hm8001alsps", client);
	if (err) {
		DEBUG_ERR("Request IRQ for alsps failed, return:%d\n", err);
		goto kfree_exit;
	}
	
	disable_irq(client->irq);
	hm8001_obj->irq_enable = FALSE;
	
	err = hm8001_light_input_init(hm8001_obj);
	if (err < 0)
		goto error_exit;
	
	err = hm8001_proximity_input_init(hm8001_obj);
	if (err < 0)
		goto error_exit;

	err = hm8001_sysfs_create_proximity_group( );
	if (err < 0)
		goto error_exit;

	err = hm8001_sysfs_create_light_group( );
	if (err < 0)
		goto error_exit;
	
	atomic_set(&hm8001_obj->proximity_enable, FALSE);
	INIT_DELAYED_WORK(&hm8001_obj->light_work, light_work_func);
	atomic_set(&hm8001_obj->light_delay, DEFAULT_DELAY);
	atomic_set(&hm8001_obj->light_enable, FALSE);
	
#if 0//ndef CONFIG_HAS_EARLYSUSPEND
	hm8001_obj->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	hm8001_obj->early_suspend.suspend = hm8001als_early_suspend;
	hm8001_obj->early_suspend.resume = hm8001als_late_resume;
	register_early_suspend(&hm8001_obj->early_suspend);
#endif
	
	err = 0;
	//hm8001_platform_data->power_control(POWER_DOWN);
	DEBUG_LOG("support ver. %s enabled!\n", DRIVER_VERSION);
	return err;
		
error_exit:
	hm8001_input_delete(hm8001_obj);
kfree_exit:
    //hm8001_platform_data->power_control(POWER_DOWN);

    wake_lock_destroy(&hm8001_obj->alsps_wakelock);
	kfree(hm8001_obj);

    hm8001_i2c_client = NULL;
   // hm8001_platform_data = NULL;
	DEBUG_ERR("support ver. %s failed!\n", DRIVER_VERSION);
exit:
	return err;
}
	
static int __devexit hm8001_remove(struct i2c_client *client)
{
	wake_lock_destroy(&hm8001_obj->alsps_wakelock);
	free_irq(client->irq, NULL);
	kfree(hm8001_obj);
	
	return 0;
}

#ifdef CONFIG_PM
static int hm8001als_suspend(struct i2c_client *client, pm_message_t state)
{
	if (atomic_read(&hm8001_obj->proximity_enable)) {
		if (hm8001_obj->irq_enable) {
			disable_irq(client->irq);
			hm8001_obj->irq_enable = FALSE;
		}

		enable_irq_wake(client->irq);
	}

	return 0;
}

static int hm8001als_resume(struct i2c_client *client)
{
	if (atomic_read(&hm8001_obj->proximity_enable)) {

		disable_irq_wake(client->irq);

		if (!hm8001_obj->irq_enable) {
			enable_irq(client->irq);
			hm8001_obj->irq_enable = TRUE;
		}
	}

	return 0;
}

#else 
#define hm8001als_suspend  NULL
#define hm8001als_resume   NULL
#endif

#if 0//ndef CONFIG_HAS_EARLYSUSPEND
static void hm8001als_early_suspend(struct early_suspend *h)
{
	DEBUG_LOG("hm8001-alsps enter %s\n", __FUNCTION__);
	hm8001als_suspend(hm8001_obj->client, PMSG_SUSPEND);
}

static void hm8001als_late_resume(struct early_suspend *h)
{
	hm8001als_resume(hm8001_obj->client);
	DEBUG_LOG("hm8001-alsps exit %s\n", __FUNCTION__);
}
#endif


static const struct i2c_device_id hm8001_id[] = {
	{ "hm8001alsps", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, hm8001_id);


static struct of_device_id hm8001_match_table[] = {
	{ .compatible = "hm,hm8001",},
	{ },
};

static struct i2c_driver hm8001_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "hm8001alsps",	
		.of_match_table = hm8001_match_table,
	},
	.probe = hm8001_probe,
	.remove = __devexit_p(hm8001_remove),
	.suspend = hm8001als_suspend,
	.resume = hm8001als_resume,
	.id_table = hm8001_id,
};

static int __init hm8001_init(void)
{
	return i2c_add_driver(&hm8001_driver);
}
 
static void __exit hm8001_exit(void)
{
	i2c_del_driver(&hm8001_driver);
	DEBUG_LOG("hm8001 exit\n");
}

MODULE_AUTHOR("wangchengqiang <wangchengqiang@gosomo.cn>");
MODULE_DESCRIPTION("hm8001 ambient&light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

late_initcall(hm8001_init);
module_exit(hm8001_exit);

