/*
  *  tmd2771x.c - Linux kernel modules for ambient light sensor
  *
  *  Copyright (C) 2011 Fan Wang <wangfan@gosomo.cn>
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

#include <linux/kdev_t.h>
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


#include "tmd2771x.h"
 
#define DRIVER_VERSION		"2.1"

#define DEBUG_TAG                  "[TMD2771X]: "

static int tmd2771x_debug_mask = 0;
module_param_named(debug_mask, tmd2771x_debug_mask,int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DEBUG_ERR(fmt, args...)   \
			printk(KERN_ERR  DEBUG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args);

#define DEBUG_WAR(fmt, args...)   \
	do { \
		if(tmd2771x_debug_mask & 0x01) \
			printk(KERN_WARNING fmt, ##args); \
		}while (0)
#define DEBUG_LOG(fmt, args...)   \
	do { \
		if(tmd2771x_debug_mask & 0x01) \
			printk(KERN_INFO DEBUG_TAG fmt, ##args); \
		}while (0)

#define DEBUG_DBG(fmt, args...)   \
	do { \
		if(tmd2771x_debug_mask & 0x01) \
			printk(KERN_DEBUG fmt, ##args); \
		}while (0)


#define PROX_DETECTION_CLOSE 3
#define PROX_DETECTION_FAR   8
#define LUX_DEFAULT   40   //modify by yeruiyong@20140114.
#define LUX_DEFAULT_OLD   0   //add by yeruiyong@20140114.

#define ALS_THRESHOLD_HI_DEFAULT 60   //modify by yeruiyong@20140114.
#define ALS_THRESHOLD_LO_DEFAULT 0   //modify by yeruiyong@20140114.

#define STK3X1X_VDD_MIN_UV	2000000
#define STK3X1X_VDD_MAX_UV	3300000
#define STK3X1X_VIO_MIN_UV	1750000
#define STK3X1X_VIO_MAX_UV	1950000



static struct i2c_client *tmd2771x_i2c_client = NULL;
struct tmd2771x_platform_data *g_platform_data = NULL;
static int tmd277x_ftm_working = FALSE;

struct tmd2771x_priv {
	struct i2c_client *client;
	struct tmd2771x_platform_data *pdata;
	struct work_struct  alsps_work;
	struct tmd2771x_reg_cfg_t tmd2771x_reg_cfg;
	struct wake_lock alsps_wakelock;
	
	u8 	status;
	u8	irq_enable;	
	int	als_intensity_old;
	int	als_intensity;			 /*als data*/
	int ps_intensity;			 /*ps data*/
	atomic_t 	 light_delay;
	atomic_t	 als_debounce;	 /*debounce time after enabling als*/
	atomic_t	 als_deb_on;	 /*indicates if the debounce is on*/
	atomic_t	 als_deb_end;	 /*the jiffies representing the end of debounce*/
	atomic_t	 ps_debounce;	 /*debounce time after enabling ps*/
	atomic_t	 ps_deb_on; 	 /*indicates if the debounce is on*/
	atomic_t	 ps_deb_end;	 /*the jiffies representing the end of debounce*/
	struct input_dev *light_input;
	struct input_dev *proximity_input;
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
	bool use_fir;
};

struct tmd2771x_priv *g_obj = NULL;

// device configuration
static u16 als_threshold_hi_param = 0x00;
static u16 als_threshold_lo_param = 0x00;
static u16 prox_threshold_value = 100;

static u16 prox_threshold_hi_param = 100;
static u16 prox_threshold_lo_param = 0;
static u8 taos_als_adc_time_param = 0xDB;    //modify by yeruiyong@20140114, set 100ms atime.
static u8 taos_prox_adc_time_param = 0xFF;
static u8 taos_prox_wait_time_param = 0xEE;
static u8 prox_intr_filter_param = 0x37;   //modify by yeruiyong@20140114, for als:20 consecutive values out of range to generate a interrupt.
static u8 taos_configration_param = 0x00;
static u8 taos_prox_pulse_cnt_param = 0x08;
static u8 taos_gain_control_param = 0x61;    //modify by yeruiyong@20140114, 50% led current strength, 8Xgain for less power consume.
static u8 taos_enable_param = 0x00;

struct taos_prox_info prox_cur_info;
struct taos_als_info als_cur_info;
unsigned int ps_work_flag = 0;

static int tmd2771lux_table[] = {50, 100, 200, 450, 650, 850, 1000, 1200};   //modify by yeruiyong@20140114.

static int tmd2771lux_ftm_table[] = {149, 299, 599, 749, 649, 849, 999, 1199};   //add by yeruiyong@20140120,for factory test mode

static int tmd2771x_init_device(struct i2c_client *client);
static void tmd2771x_proximity_report(void);
static void tmd2771x_ambient_report(void);



static int tmd277x_power_ctl(struct tmd2771x_priv *tmd2771alsps, bool on)
{
	int ret = 0;
	printk("%s",__func__);

	if (!on && tmd2771alsps->power_enabled) {
		ret = regulator_disable(tmd2771alsps->vdd);
		if (ret) {
			dev_err(&tmd2771alsps->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(tmd2771alsps->vio);
		if (ret) {
			dev_err(&tmd2771alsps->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			regulator_enable(tmd2771alsps->vdd);
			return ret;
		}
		tmd2771alsps->power_enabled = on;
		dev_dbg(&tmd2771alsps->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else if (on && !tmd2771alsps->power_enabled) {

		ret = regulator_enable(tmd2771alsps->vdd);
		if (ret) {
			dev_err(&tmd2771alsps->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(tmd2771alsps->vio);
		if (ret) {
			dev_err(&tmd2771alsps->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(tmd2771alsps->vdd);
			return ret;
		}
		tmd2771alsps->power_enabled = on;
		dev_dbg(&tmd2771alsps->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&tmd2771alsps->client->dev,
				"Power on=%d. enabled=%d\n",
				on, tmd2771alsps->power_enabled);
	}

	return ret;
}

static int tmd277x_power_init(struct tmd2771x_priv *tmd2771alsps, bool on)
{
	int ret;
	printk("%s on%d\n",__func__,on);
	if (!on) {
		if (regulator_count_voltages(tmd2771alsps->vdd) > 0)
			regulator_set_voltage(tmd2771alsps->vdd,
					0, STK3X1X_VDD_MAX_UV);

		regulator_put(tmd2771alsps->vdd);

		if (regulator_count_voltages(tmd2771alsps->vio) > 0)
			regulator_set_voltage(tmd2771alsps->vio,
					0, STK3X1X_VIO_MAX_UV);

		regulator_put(tmd2771alsps->vio);
	} else {
		printk("regulator_get \n");
		
		tmd2771alsps->vdd = regulator_get(&tmd2771alsps->client->dev, "vdd");
		if (IS_ERR(tmd2771alsps->vdd)) {
			ret = PTR_ERR(tmd2771alsps->vdd);
			dev_err(&tmd2771alsps->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		printk("regulator_count_voltages\n");

		if (regulator_count_voltages(tmd2771alsps->vdd) > 0) {
			ret = regulator_set_voltage(tmd2771alsps->vdd,
					STK3X1X_VDD_MIN_UV,
					STK3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&tmd2771alsps->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}
		printk("regulator_count vio\n");

		tmd2771alsps->vio = regulator_get(&tmd2771alsps->client->dev, "vio");
		if (IS_ERR(tmd2771alsps->vio)) {
			ret = PTR_ERR(tmd2771alsps->vio);
			dev_err(&tmd2771alsps->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(tmd2771alsps->vio) > 0) {
			ret = regulator_set_voltage(tmd2771alsps->vio,
					STK3X1X_VIO_MIN_UV,
					STK3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&tmd2771alsps->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(tmd2771alsps->vio);
reg_vdd_set:
	if (regulator_count_voltages(tmd2771alsps->vdd) > 0)
		regulator_set_voltage(tmd2771alsps->vdd, 0, STK3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(tmd2771alsps->vdd);
	return ret;
}


static int tmd277x_irq_query(void)
{
	return gpio_get_value(80) ? 1 : 0;
}


static int tmd2771x_enable_func(int func, int enable)
{
	int ret = 0;
	
	if (TAOS_TMD2771X_CNTL_PROX_ENBL == func){
		if (enable){
			taos_enable_param |= (TAOS_TMD2771X_CNTL_PROX_ENBL | TAOS_TMD2771X_CNTL_PROX_INT_ENBL | TAOS_TMD2771X_CNTL_WAIT_ENBL | TAOS_TMD2771X_CNTL_PWRON);
			atomic_set(&g_obj->ps_deb_on, 1);
			atomic_set(&g_obj->ps_deb_end, jiffies + atomic_read(&g_obj->ps_debounce)/(1000/HZ));
		}else{
			taos_enable_param &= ((~TAOS_TMD2771X_CNTL_PROX_ENBL) & (~TAOS_TMD2771X_CNTL_PROX_INT_ENBL));
		}
	}
	else if (TAOS_TMD2771X_CNTL_ALS_ENBL == func){
		if (enable){
			taos_enable_param |= (TAOS_TMD2771X_CNTL_ALS_ENBL | TAOS_TMD2771X_CNTL_ALS_INT_ENBL | TAOS_TMD2771X_CNTL_WAIT_ENBL | TAOS_TMD2771X_CNTL_PWRON);
			atomic_set(&g_obj->als_deb_on, 1);
			atomic_set(&g_obj->als_deb_end, jiffies + atomic_read(&g_obj->als_debounce)/(1000/HZ));
		}else{
			taos_enable_param &= ((~TAOS_TMD2771X_CNTL_ALS_ENBL) & (~TAOS_TMD2771X_CNTL_ALS_INT_ENBL));
		}
	}
	else{
		return -EINVAL;
	}

	ret = tmd2771x_init_device(tmd2771x_i2c_client);

	return ret;
}

//add begin by yeruiyong@20140114
/*
 * get interrupt type, but not clear the irq status
 */
static int tmd2771x_query_intr_id(void) {
   int ret = 0;
   int interrupt_id = 0;
   if ((ret = i2c_smbus_read_byte_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_STATUS))) < 0) {
       printk(KERN_ERR "TAOS: i2c_smbus_read_byte_data failed in tmd2771x_check_intr() err %d\n", ret);

       ret = 0x30;
   }
   interrupt_id = ret & 0x30;
   return interrupt_id;
}


static int tmd2771x_clear_intr(int interrupt_id) {
    int ret = -1;
    switch(interrupt_id)
    {
        case TAOS_TMD2771X_STATUS_PINT:

        case TAOS_TMD2771X_STATUS_AINT:

        case (TAOS_TMD2771X_STATUS_PINT | TAOS_TMD2771X_STATUS_AINT):
            if ((ret = (i2c_smbus_write_byte(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_SPL_FN | TAOS_TMD2771X_CMD_ALS_AND_PS_INTCLR)))) < 0) {
                printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in tmd2771x_clear_intr() err %d\n", ret);
                return ret;
            }
            break;

        default:
            DEBUG_LOG("TAOS: No interrupt bit need to be cleared in tmd2771x_clear_intr()\n");
            break;
    }
    return ret;
}
//add end.


static int tmd2771x_check_and_clear_intr(void)
{	
	int interrupt_id = 0;
    int ret = 0;
    
	if ((ret = i2c_smbus_read_byte_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_STATUS))) < 0){
		printk(KERN_ERR "TAOS: i2c_smbus_read_byte_data failed in tmd2771x_check_and_clear_intr() err %d\n", ret);

		ret = 0x30; 
	}

	interrupt_id = ret & 0x30;

	switch(interrupt_id)
	{
	case TAOS_TMD2771X_STATUS_PINT:
		
	case TAOS_TMD2771X_STATUS_AINT:
	
	case (TAOS_TMD2771X_STATUS_PINT | TAOS_TMD2771X_STATUS_AINT):
		if ((ret = (i2c_smbus_write_byte(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_SPL_FN | TAOS_TMD2771X_CMD_ALS_AND_PS_INTCLR)))) < 0) {
			printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in tmd2771x_check_and_clear_intr() err %d\n", ret);
			return (ret);
		}
		break;

    default:
		DEBUG_LOG("TAOS: No interrupt bit need to be cleared in tmd2771x_check_and_clear_intr()\n");
		break;
	}
	
	return (interrupt_id);
}

static int tmd2771x_get_lux_value(struct taos_als_info *alsp, bool reset_thresh_enable) {

	u16 raw_clear = 0, raw_ir = 0;
	unsigned long endt;
	u8 i = 0;
	u8 rawdata[4];
	u8 chdata[4];
 
	if (i2c_smbus_read_i2c_block_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_AUTO_INCREMENT | TAOS_TMD2771X_ALS_CHAN0LO ), 4, chdata) < 4){
		printk(KERN_ERR "TAOS: i2c_smbus_read_i2c_block_data() to als/prox data reg failed in tmd2771x_get_lux_value()\n");
		return -EBUSY;
	}
	
	raw_clear = chdata[1] << 8 | chdata[0];
	raw_ir = chdata[3] << 8 | chdata[2];
	
	DEBUG_LOG("\ntmd2771x_get_lux() ch0=%d, ch1=%d.\n", raw_clear, raw_ir);

        //modify beging by yeruiyong@2014014
	if (reset_thresh_enable){
	if (raw_clear>40000) {
			als_threshold_hi_param = 65535;
			als_threshold_lo_param = 32000; //37000
			 i = 7;
		} else if (raw_clear >25000){
			als_threshold_hi_param = 42000;
			als_threshold_lo_param = 20000; //23500
			i = 6;
		}else if (raw_clear >10000){
			als_threshold_hi_param = 27000;
			als_threshold_lo_param = 8000; //10000
			i = 5;
		}else if (raw_clear > 3500){
			als_threshold_hi_param = 12000;
			als_threshold_lo_param = 3000; //3500
			i = 4;
		}else if (raw_clear > 1500){
			als_threshold_hi_param = 4200; //3500
			als_threshold_lo_param = 1200; //1500
			i = 3;
		}else if (raw_clear > 500){
			als_threshold_hi_param = 1800; //1500
			als_threshold_lo_param = 400; //500
			i = 2;
		}else if (raw_clear  > 50){
			als_threshold_hi_param = 600; //500
			als_threshold_lo_param = 40; //50
			i = 1;
		}else {
			als_threshold_hi_param = 60;  //50
			als_threshold_lo_param = 0;
			i = 0;
		}
        //modify end


		rawdata[0] = als_threshold_lo_param & 0x00ff;
		rawdata[1] = als_threshold_lo_param >> 8;
		rawdata[2] = als_threshold_hi_param & 0x00ff;
		rawdata[3] = als_threshold_hi_param >> 8;

		if (i2c_smbus_write_i2c_block_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_AUTO_INCREMENT | TAOS_TMD2771X_ALS_MINTHRESHLO), 4, rawdata) < 0){
			printk(KERN_ERR "TAOS: reset als threshold: i2c_smbus_write_i2c_block_data failed\n");
			return -EAGAIN;	
		}
	}
	if (1 == atomic_read(&g_obj->als_deb_on)) {
		endt = atomic_read(&g_obj->als_deb_end);
		if (time_after(jiffies, endt)){
			atomic_set(&g_obj->als_deb_on, 0);
		}else{
		
			rawdata[0] = ALS_THRESHOLD_LO_DEFAULT & 0x00ff;
			rawdata[1] = ALS_THRESHOLD_LO_DEFAULT >> 8;
			rawdata[2] = ALS_THRESHOLD_HI_DEFAULT & 0x00ff;
			rawdata[3] = ALS_THRESHOLD_HI_DEFAULT >> 8;
			
			if (i2c_smbus_write_i2c_block_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_AUTO_INCREMENT | TAOS_TMD2771X_ALS_MINTHRESHLO), 4, rawdata) < 0){
				printk(KERN_ERR "TAOS: reset als threshold: i2c_smbus_write_i2c_block_data failed\n");
				return -EAGAIN; 
			}
			
			g_obj->als_intensity = LUX_DEFAULT;
			return LUX_DEFAULT;
		}
	}

        if (tmd277x_ftm_working) {
            g_obj->als_intensity = tmd2771lux_ftm_table[i];
            DEBUG_LOG("tmd2771x work in FTM,tmd2771x_get_lux()=%d\n", g_obj->als_intensity);
        } else {
            g_obj->als_intensity = tmd2771lux_table[i];
        }
        return g_obj->als_intensity;
}

static int tmd2771x_get_ps_value(struct taos_prox_info *prxp, bool reset_thresh_enable) 
{
	unsigned long endt;
	u8 chdata[6];
	u8 rawdata[4];
	
	if (1 == atomic_read(&g_obj->ps_deb_on)) {
		endt = atomic_read(&g_obj->ps_deb_end);
		if (time_after(jiffies, endt)){
			atomic_set(&g_obj->ps_deb_on, 0);
		}else{
			return g_obj->ps_intensity;
		}
	}

	if (i2c_smbus_read_i2c_block_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_AUTO_INCREMENT | TAOS_TMD2771X_ALS_CHAN0LO ), 6, chdata) < 6){
		printk(KERN_ERR "TAOS: i2c_smbus_read_i2c_block_data() to als/prox data reg failed in tmd2771x_get_ps_value()\n");
		return -EBUSY;
	}

	prxp->prox_data = chdata[5] << 8 | chdata[4];
	DEBUG_LOG("TAOS: ####### prox_data = %d\n", prxp->prox_data);

	g_obj->ps_intensity = prxp->prox_data > prox_threshold_value ? PROX_DETECTION_CLOSE : PROX_DETECTION_FAR;

	if (reset_thresh_enable){
		if (prxp->prox_data > prox_threshold_value){
			prox_threshold_lo_param = prox_threshold_value;
			prox_threshold_hi_param = 0xffff;
		}
		else if (prxp->prox_data < prox_threshold_value){
			prox_threshold_lo_param = 0;
			prox_threshold_hi_param = prox_threshold_value;
		}

		rawdata[0] = prox_threshold_lo_param & 0x00ff;
		rawdata[1] = prox_threshold_lo_param >> 8;
		rawdata[2] = prox_threshold_hi_param & 0x00ff;
		rawdata[3] = prox_threshold_hi_param >> 8;

		if (i2c_smbus_write_i2c_block_data(tmd2771x_i2c_client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_AUTO_INCREMENT | TAOS_TMD2771X_PRX_MINTHRESHLO), 4, rawdata) < 0){
			printk(KERN_ERR "TAOS: reset prox threshold: i2c_smbus_write_i2c_block_data failed\n");
			return -EAGAIN;	
		}
	}

	DEBUG_LOG("TAOS:$$$$$$$$ ps intensity = %d\n", g_obj->ps_intensity);

	return g_obj->ps_intensity;
}

static void alsps_work_handler(struct work_struct *work)
{
	int ret = 0;
	int interrupt_status = 0;

    //modify begin yeruiyong@20140114
    if (!g_obj){
        DEBUG_ERR("alsps_work_handler(): g_obj null\n");
        enable_irq(tmd2771x_i2c_client->irq);
        if (!tmd277x_irq_query()) {
            tmd2771x_check_and_clear_intr();
        }
        return;
    }
    //modify end.

	interrupt_status = tmd2771x_query_intr_id();   //modify by yeruiyong@20140114.
	printk(KERN_ERR "TAOS PS status = %d\n", interrupt_status);
	switch (interrupt_status){
		case TAOS_TMD2771X_STATUS_PINT:
			if ((ret = tmd2771x_get_ps_value(&prox_cur_info, TRUE)) > 0){
				tmd2771x_proximity_report();
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS PS Interrupt get_ps_value err in alsps_work_handler() err %d\n", ret);
			}
			break;
		
		case TAOS_TMD2771X_STATUS_AINT:
			
			if ((ret = tmd2771x_get_lux_value(&als_cur_info, TRUE)) > 0){
                                tmd2771x_ambient_report();    //add by yeruiyong@20141014.
				DEBUG_LOG(KERN_INFO "TAOS LAS Interrupt get_als_value = %d\n", g_obj->als_intensity);
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS ALS Interrupt get_als_value err in alsps_work_handler() err %d\n",ret);
			}
			
			break;

		case (TAOS_TMD2771X_STATUS_AINT | TAOS_TMD2771X_STATUS_PINT):

			if ((ret = tmd2771x_get_ps_value(&prox_cur_info, TRUE)) >= 0){
				tmd2771x_proximity_report();
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS ALS & PS Interrupt get_ps_value err in alsps_work_handler() err %d\n", ret);
			}
			
			if ((ret = tmd2771x_get_lux_value(&als_cur_info, TRUE)) > 0){
                                tmd2771x_ambient_report();    //add by yeruiyong@20141014.
				DEBUG_LOG(KERN_INFO "TAOS ALS Interrupt get_als_value = %d\n",g_obj->als_intensity);
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS ALS Interrupt get_als_value err in alsps_work_handler() err %d\n", ret);
			}
			break;
			
		default:
			break;
	}
    //add begin by yeruiyong@20141014
    enable_irq(tmd2771x_i2c_client->irq);
    if ( (ret =  tmd2771x_clear_intr(interrupt_status)) < 0) {
         DEBUG_ERR("clear irq status failed, try again");
         tmd2771x_clear_intr(interrupt_status);
    }
    //add end
	return;
}

static irqreturn_t alsps_irq_handler(int irq, void *dev_id)
{
    disable_irq_nosync(irq);          //add by yeruiyong@20141014.
    if (!g_obj){
        DEBUG_ERR("alsps_irq_handler(): g_obj null\n");
        // add begin by yeruiyong@20140114
        enable_irq(irq);
        if (!tmd277x_irq_query()) {
            tmd2771x_check_and_clear_intr();
        }
        // add end.
    	return IRQ_NONE;
    }
	
	wake_lock_timeout(&g_obj->alsps_wakelock, HZ);   //modify begin by yeruiyong@20140114

    //modify begin by yeruiyong@20140114
    if (tmd277x_irq_query()) {
        enable_irq(irq);
        return IRQ_NONE;
    }
    //modify end.

    schedule_work(&g_obj->alsps_work);
	
	return IRQ_HANDLED;
}

static void tmd2771x_proximity_poweron(void)
{	
	//int rc = 0;
	if (!g_obj){
		DEBUG_ERR("tmd2771x_proximity_poweron(): g_obj null\n");
		return;
	}

	/*if (!(g_obj->status & TAOS_TMD2771X_STATUS_MASK)){
		rc = g_platform_data->set_power_control(POWER_ON);
		if(rc < 0){
			printk("tmd2771 proximitysensor set poweron failed\n");
		}
	}*/
	tmd2771x_enable_func(TAOS_TMD2771X_CNTL_PROX_ENBL, TRUE);
	ps_work_flag = 1;

	if (!g_obj->irq_enable){
		enable_irq(tmd2771x_i2c_client->irq);
		g_obj->irq_enable = TRUE;
	}

	if (!tmd277x_irq_query()){
		tmd2771x_check_and_clear_intr();
	}

	g_obj->ps_intensity = PROX_DETECTION_FAR;
	g_obj->status |=  TAOS_TMD2771X_STATUS_PROX_WORKING;
	tmd2771x_proximity_report();
	DEBUG_LOG("TAOS: tmd2771x proximity poweron OK!\n");
	return;
}

static void tmd2771x_proximity_poweroff(void)
{
	//int rc = 0;
	if (!g_obj){
		DEBUG_ERR("tmd2771x_proximity_poweroff(): g_obj null\n");
		return;
	}
	
	tmd2771x_enable_func(TAOS_TMD2771X_CNTL_PROX_ENBL, FALSE);
	
	g_obj->status &= (~TAOS_TMD2771X_STATUS_PROX_WORKING);

	if ( !(g_obj->status & TAOS_TMD2771X_STATUS_MASK )){
		if (g_obj->irq_enable){
			disable_irq(tmd2771x_i2c_client->irq);
			g_obj->irq_enable = FALSE;
		}	
		/*rc = g_platform_data->set_power_control(POWER_DOWN);
		if(rc < 0){
			printk("tmd2771 proximitysensor set poweroff failed\n");
		}*/
	}
    ps_work_flag = 0;

	DEBUG_LOG("TAOS: tmd2771x proximity poweroff OK!\n");

	return;
}

static void tmd2771x_proximity_report(void)
{
	if (!g_obj){
		DEBUG_ERR("tmd2771x_proximity_report(): g_obj null\n");
		return;
	}
	input_report_abs(g_obj->proximity_input, ABS_DISTANCE, g_obj->ps_intensity);
	input_sync(g_obj->proximity_input);
}

static void tmd2771x_ambient_poweron(void)
{	
	//int rc = 0;
	if (!g_obj){
		DEBUG_ERR("tmd2771x_ambient_poweron(): g_obj null\n");
		return;
	}

	if (!(g_obj->status & TAOS_TMD2771X_STATUS_MASK)){
		/*rc = g_platform_data->set_power_control(POWER_ON);
		if(rc < 0){
			printk("tmd2771 lightsensor set poweron failed\n");
		}*/
	}
	tmd2771x_enable_func(TAOS_TMD2771X_CNTL_ALS_ENBL, TRUE);

	if (!g_obj->irq_enable){
		enable_irq(tmd2771x_i2c_client->irq);
		g_obj->irq_enable = TRUE;
	}

	if (!tmd277x_irq_query()){
		tmd2771x_check_and_clear_intr();
	}
	
	g_obj->status |= TAOS_TMD2771X_STATUS_ALS_WORKING;

	tmd2771x_ambient_report();
	DEBUG_LOG("TAOS: tmd2771x ambient poweron OK!\n");
	return;
}

static void tmd2771x_ambient_report(void)
{
	if (!g_obj){
		DEBUG_ERR("tmd2771x_ambient_report(): g_obj null\n");
		return;
	}
     //modify begin by yeruiyong@20140114
     if (g_obj->als_intensity_old != g_obj->als_intensity) {
         input_report_abs(g_obj->light_input, ABS_MISC, g_obj->als_intensity);
         input_sync(g_obj->light_input);
         g_obj->als_intensity_old = g_obj->als_intensity;
     } else {
         DEBUG_WAR("%s:ambient not changed\n", __func__);
         return;
     }
    //modify end

}


static void tmd2771x_ambient_poweroff(void)
{
	//int rc = 0;
	if (!g_obj){
		DEBUG_ERR("tmd2771x_ambient_poweroff(): g_obj null\n");
		return;
	}
	tmd2771x_enable_func(TAOS_TMD2771X_CNTL_ALS_ENBL, FALSE);

	g_obj->status &= (~TAOS_TMD2771X_STATUS_ALS_WORKING);

	if (!(g_obj->status & TAOS_TMD2771X_STATUS_MASK)){
		if (g_obj->irq_enable){
			disable_irq(tmd2771x_i2c_client->irq);
			g_obj->irq_enable = FALSE;
		}
		/*rc = g_platform_data->set_power_control(POWER_DOWN);
		if(rc < 0){
			printk("tmd2771 lightysensor set poweroff failed\n");
		}*/
	}

	DEBUG_LOG("TAOS: tmd2771x ambient poweroff OK!\n");

	return;
}


static inline void tmd2771x_alsps_debounce_init(void)
{
	atomic_set(&g_obj->als_debounce, 1000);
    atomic_set(&g_obj->als_deb_on, 0);
    atomic_set(&g_obj->als_deb_end, 0);
    atomic_set(&g_obj->ps_debounce, 100);
    atomic_set(&g_obj->ps_deb_on, 0);
    atomic_set(&g_obj->ps_deb_end, 0);
}
static int tmd2771x_init_device(struct i2c_client *client)
{
	bool try_one_more_time = FALSE;

	if (!g_obj){
		DEBUG_ERR("tmd2771x_init_device(): g_obj null\n");
		return -EINVAL;
	}
	g_obj->als_intensity = LUX_DEFAULT;
        g_obj->als_intensity_old = LUX_DEFAULT_OLD;   //add by yeruiyong@20140114.

	g_obj->ps_intensity = PROX_DETECTION_FAR;
	
	g_obj->tmd2771x_reg_cfg.enable = taos_enable_param;
	g_obj->tmd2771x_reg_cfg.als_adc_time = taos_als_adc_time_param;
	g_obj->tmd2771x_reg_cfg.prox_adc_time = taos_prox_adc_time_param;
	g_obj->tmd2771x_reg_cfg.prox_wait_time = taos_prox_wait_time_param;
	g_obj->tmd2771x_reg_cfg.als_minthreshhi= ALS_THRESHOLD_LO_DEFAULT >> 8;
	g_obj->tmd2771x_reg_cfg.als_minthreshlo= ALS_THRESHOLD_LO_DEFAULT & 0x00ff;
	g_obj->tmd2771x_reg_cfg.als_maxthreshhi = ALS_THRESHOLD_HI_DEFAULT >> 8;
	g_obj->tmd2771x_reg_cfg.als_maxthreshlo = ALS_THRESHOLD_HI_DEFAULT & 0x00ff;
   	g_obj->tmd2771x_reg_cfg.prox_minthreshhi= prox_threshold_lo_param >> 8;
	g_obj->tmd2771x_reg_cfg.prox_minthreshlo= prox_threshold_lo_param & 0x00ff;
	g_obj->tmd2771x_reg_cfg.prox_maxthreshhi = prox_threshold_hi_param >> 8;
	g_obj->tmd2771x_reg_cfg.prox_maxthreshlo = prox_threshold_hi_param & 0x00ff;
	if(tmd277x_ftm_working){
             g_obj->tmd2771x_reg_cfg.prox_intr_filter = FTM_ALS_RATE;
             g_obj->als_intensity = 149;
	}else {
		g_obj->tmd2771x_reg_cfg.prox_intr_filter = prox_intr_filter_param;
	}
	g_obj->tmd2771x_reg_cfg.configuration = taos_configration_param;
	g_obj->tmd2771x_reg_cfg.prox_pulse_cnt = taos_prox_pulse_cnt_param;
	g_obj->tmd2771x_reg_cfg.gain_control = taos_gain_control_param;
	
retry_init_reg:
	if (i2c_smbus_write_i2c_block_data(client, (TAOS_TMD2771X_CMD_REG | TAOS_TMD2771X_CMD_AUTO_INCREMENT), sizeof(g_obj->tmd2771x_reg_cfg), (u8 *)&g_obj->tmd2771x_reg_cfg) < 0){
		printk(KERN_ERR "TAOS: tmd2771x_init_device: i2c_smbus_write_i2c_block_data failed in init devices\n");

		if (!try_one_more_time){
			mdelay(20);
			try_one_more_time = TRUE;
			goto retry_init_reg;
		}

		return -EAGAIN;
	}
    return 0;
}

/******************************************************************************
 								Proximity sysfs attributes
*******************************************************************************/
static ssize_t tmd2771x_proximity_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", (g_obj->status & TAOS_TMD2771X_STATUS_PROX_WORKING) ? "enable" : "disable");
}

static ssize_t tmd2771x_proximity_enable_store(struct device *dev,
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
			tmd2771x_proximity_poweron();
		}
		else
		{
                   tmd2771x_proximity_poweroff();
                   tmd277x_ftm_working = FALSE;
		}
	}
	
	return count;
}

static int tmd2771x_sysfs_create_proximity_group(void)
{
	int err;
	static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			tmd2771x_proximity_enable_show, tmd2771x_proximity_enable_store);
	
	static struct attribute *tmd2771x_proximity_attributes[] =
	{
		&dev_attr_enable.attr,
		NULL
	};
	
	static struct attribute_group tmd2771x_proximity_attribute_group =
	{
		.attrs = tmd2771x_proximity_attributes
	};
	
	err = sysfs_create_group(&g_obj->proximity_input->dev.kobj,
				&tmd2771x_proximity_attribute_group);
	if(err < 0)
	{
		DEBUG_ERR("tmd2771x sysfs create proximity sys group fail.\n");
		sysfs_remove_group(&g_obj->proximity_input->dev.kobj, 
				&tmd2771x_proximity_attribute_group);
	}

	return err;
}

/******************************************************************************
 								Light sysfs attributes
*******************************************************************************/
static ssize_t tmd2771x_light_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", (g_obj->status & TAOS_TMD2771X_STATUS_ALS_WORKING) ? "enable" : "disable");
}

static ssize_t tmd2771x_light_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;
    int	ret;
    
	ret = sscanf(buf, "%d", &enable);
	
	if(ret == 1)
	{
		tmd277x_ftm_working = FALSE;

		if(enable)
		{
			
			tmd2771x_ambient_poweron();
		}
		else
		{
			tmd2771x_ambient_poweroff();

		}
	}

	return count;
}


static ssize_t tmd2771x_light_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&g_obj->light_delay));
}

static ssize_t tmd2771x_light_poll_delay_store(struct device *dev,
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
        if(FTM_LIGHT_SENSOR_DELAY == delay) {
            if(g_obj->status & TAOS_TMD2771X_STATUS_ALS_WORKING) {
                tmd2771x_ambient_poweroff();
            }
            tmd277x_ftm_working = TRUE;
            atomic_set(&g_obj->light_delay, 200);
            tmd2771x_ambient_poweron();
        }
        else
        {
            atomic_set(&g_obj->light_delay, delay);
            tmd277x_ftm_working = FALSE;
        }
    }
	return count;
}

static int tmd2771x_sysfs_create_light_group(void)
{
	int err;
	static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			tmd2771x_light_enable_show, tmd2771x_light_enable_store);
	static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
			tmd2771x_light_poll_delay_show, tmd2771x_light_poll_delay_store);
	
	static struct attribute *tmd2771x_light_attributes[] =
	{
		&dev_attr_enable.attr,
		&dev_attr_poll_delay.attr,
		NULL
	};
	
	static struct attribute_group tmd2771x_light_attribute_group =
	{
		.attrs = tmd2771x_light_attributes
	};
	
	err = sysfs_create_group(&g_obj->light_input->dev.kobj,
				&tmd2771x_light_attribute_group);
	if(err < 0)
	{
		DEBUG_ERR("tmd2771x sysfs create light sys group fail.\n");
		sysfs_remove_group(&g_obj->light_input->dev.kobj, 
				&tmd2771x_light_attribute_group);
	}

	return err;

}



static int tmd2771_proximity_input_init(struct tmd2771x_priv *td2771ps)
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
	input_set_drvdata(dev, td2771ps);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		td2771ps->proximity_input = NULL;
		return err;
	}
	td2771ps->proximity_input = dev;

	return 0;
}

static int tmd2771_light_input_init(struct tmd2771x_priv *tmd2771als)
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
	input_set_drvdata(dev, tmd2771als);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		tmd2771als->light_input = NULL;
		return err;
	}
	tmd2771als->light_input = dev;

	return 0;
}

static void tmd2771_input_delete(struct tmd2771x_priv *tmd2771alsps)
{
	if (tmd2771alsps->proximity_input != NULL)
	{
		input_unregister_device(tmd2771alsps->proximity_input);
		input_free_device(tmd2771alsps->proximity_input);
	}

	if (tmd2771alsps->light_input != NULL)
	{
		input_unregister_device(tmd2771alsps->light_input);
		input_free_device(tmd2771alsps->light_input);
	}
}

#ifdef CONFIG_OF
 static int tmd277x_parse_dt(struct device *dev,
			 struct tmd2771x_platform_data *pdata)
 {
	 //int rc;
	 struct device_node *np = dev->of_node;
	 pdata->int_pin = of_get_named_gpio_flags(np, "tmd,irq-gpio",
				 0, &pdata->int_flags);
	 if (pdata->int_pin < 0) {
		 dev_err(dev, "Unable to read irq-gpio\n");
		 return pdata->int_pin;
	 }
 

	 pdata->use_fir = of_property_read_bool(np, "tmd,use-fir");
 
	 return 0;
 }
#else
 static int tmd277x_parse_dt(struct device *dev,
			 struct tmd2771x_platform_data *pdata)
 {
	 return -ENODEV;
 }
#endif /* !CONFIG_OF */


static int __devinit tmd2771x_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tmd2771x_platform_data *pdata;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "TAOS: tmd2771x_probe() - i2c smbus byte data functions unsupported\n");
		err = -EIO;
		goto exit;
	}

	g_obj = kzalloc(sizeof(struct tmd2771x_priv), GFP_KERNEL);
	if (!g_obj) {
		err = -ENOMEM;
	 	goto exit;
	}
	g_obj->client = client;
	
	if (client->dev.of_node) {
			pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
			if (!pdata) {
				dev_err(&client->dev, "Failed to allocate memory for pdata\n");
				err = -ENOMEM;
				goto exit;
			}

			err = tmd277x_parse_dt(&client->dev, pdata);
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
	
		printk("pdata->int_pin %d\n",pdata->int_pin);
		
		if (gpio_is_valid(pdata->int_pin )) {
		err = gpio_request(pdata->int_pin , "tmd2771x_irq_gpio");
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

	//memset(g_obj, 0, sizeof(*g_obj));
	wake_lock_init(&g_obj->alsps_wakelock, WAKE_LOCK_SUSPEND, "alsps");
	
	i2c_set_clientdata(client, g_obj);
	INIT_WORK(&g_obj->alsps_work, alsps_work_handler);

	//g_platform_data = client->dev.platform_data;
	//g_platform_data->init_irq();
	g_obj->irq_enable = TRUE;
	/*err = g_platform_data->set_power_control(POWER_ON);
	if(err < 0){
		printk("tmd2771 set poweron failed\n");
	}*/

	err = tmd277x_power_init(g_obj, true);
			if (err)
				goto exit;
		
			err = tmd277x_power_ctl(g_obj, true);
			if (err)
				goto exit;
	
	err = tmd2771x_init_device(client);
	if (err < 0){
	    tmd2771x_i2c_client = NULL;
		printk(KERN_ERR "[TMD2771X]: tmd2771x device init fail.\n");
		goto exit_kfree;
	}
	else{
		tmd2771x_i2c_client = client;
		printk(KERN_INFO "[TMD2771X]: tmd2771x device init OK.\n");
	}

	tmd2771x_alsps_debounce_init();
	
	err = request_irq(client->irq, alsps_irq_handler, IRQF_TRIGGER_FALLING, "alsps", client);
	if (err){
		printk(KERN_ERR "[TMD2771X]: Request IRQ for alsps failed, return:%d\n",err);
		goto exit_kfree;
	}

	disable_irq(tmd2771x_i2c_client->irq);
	g_obj->irq_enable = FALSE;
	
	err = tmd2771_light_input_init(g_obj);
	if (err < 0)
		goto error_exit;
	
	err = tmd2771_proximity_input_init(g_obj);
	if (err < 0)
		goto error_exit;
	
	err = tmd2771x_sysfs_create_proximity_group( );
	if (err < 0)
		goto error_exit;

	err = tmd2771x_sysfs_create_light_group( );
	if (err < 0)
		goto error_exit;

	/*err = g_platform_data->set_power_control(POWER_DOWN);
	if(err < 0){
		printk("tmd2771 set poweroff failed\n");
	}*/
	printk("[TMD2771X]: support ver. enabled\n");

	return 0;
 error_exit:
	tmd2771_input_delete(g_obj);
 exit_kfree:
 	wake_lock_destroy(&g_obj->alsps_wakelock);
	//g_platform_data->set_power_control(POWER_DOWN);
	kfree(g_obj);
 exit:
	return err;
}

static int __devexit tmd2771x_remove(struct i2c_client *client)
{
	wake_lock_destroy(&g_obj->alsps_wakelock);
	free_irq(client->irq, NULL);
	kfree(g_obj);
	
	return 0;
}
#ifdef CONFIG_PM
static int tmd2771x_suspend(struct i2c_client *client, pm_message_t state)
{
	if (g_obj->status & TAOS_TMD2771X_STATUS_PROX_WORKING){
		if (g_obj->irq_enable){
			disable_irq(client->irq);
			g_obj->irq_enable = FALSE;
		}
		
		enable_irq_wake(client->irq);
	}

	return 0;
}

static int tmd2771x_resume(struct i2c_client *client)
{
	if (g_obj->status & TAOS_TMD2771X_STATUS_PROX_WORKING){
		disable_irq_wake(client->irq);

		if (!g_obj->irq_enable){
			enable_irq(client->irq);
			g_obj->irq_enable = TRUE;
		}
	}
	
	return 0;
}

#else 
#define tmd2771x_suspend  NULL
#define tmd2771x_resume   NULL
#endif

static const struct i2c_device_id tmd2771x_id[] = {
	 { "tmd2771x", 0 },
	 { }
};

MODULE_DEVICE_TABLE(i2c, tmd2771x_id);

static struct of_device_id tmd277x_match_table[] = {
	{ .compatible = "tmd,tmd2771x",},
	{ },
};

static struct i2c_driver tmd2771x_driver = {
	.driver = {
		.owner = THIS_MODULE,	
		.name = "tmd2771x",
		.of_match_table = tmd277x_match_table,
	},
	.probe  = tmd2771x_probe,
	.remove = __devexit_p(tmd2771x_remove),
	.suspend = tmd2771x_suspend,
	.resume = tmd2771x_resume,
	.id_table = tmd2771x_id,
};

static int __init tmd2771x_init(void)
{ 
	return i2c_add_driver(&tmd2771x_driver);
}
 
static void __exit tmd2771x_exit(void)
{
	 i2c_del_driver(&tmd2771x_driver);
	 printk(KERN_INFO "TMD2771x exit\n");
}

MODULE_AUTHOR("liujiquan <liujiquan@gosomo.cn>");
MODULE_DESCRIPTION("TMD2771x ambient&light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

late_initcall(tmd2771x_init);
module_exit(tmd2771x_exit);

