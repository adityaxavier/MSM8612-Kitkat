/*
  *  apds9900.c - Linux kernel modules for ambient light sensor
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
#include "apds9900.h"

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


#define DRIVER_VERSION		"2.1"

#define DEBUG_TAG                  "[APDS9900]: "

//#define DEBUG
#ifdef DEBUG
#define DEBUG_ERR(fmt, args...)    printk(KERN_ERR  DEBUG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define DEBUG_LOG(fmt, args...)    printk(KERN_INFO DEBUG_TAG fmt, ##args)
#define DEBUG_DBG(fmt, args...)    printk(KERN_DEBUG DEBUG_TAG fmt, ##args) 
#define DEBUG_WAR(fmt, args...)    printk(KERN_WARNING DEBUG_TAG fmt, ##args) 
#else
#define DEBUG_ERR(fmt, args...)    printk(KERN_ERR  DEBUG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)

#define DEBUG_LOG(fmt, args...)    \
({if (0) printk(KERN_INFO DEBUG_TAG fmt, ##args);})

#define DEBUG_DBG(fmt, args...)	\
({if (0) printk(KERN_DEBUG fmt, ##args);})
#define DEBUG_WAR(fmt, args...)   \
({if (0) printk(KERN_WARNING fmt, ##args);})
#endif

#define PROX_DETECTION_CLOSE 3
#define PROX_DETECTION_FAR   8
#define DEFAULT_DELAY        4000
#define LUX_DEFAULT   400  

#define STK3X1X_VDD_MIN_UV	2000000
#define STK3X1X_VDD_MAX_UV	3300000
#define STK3X1X_VIO_MIN_UV	1750000
#define STK3X1X_VIO_MAX_UV	1950000

static struct i2c_client *apds9900_i2c_client = NULL;
struct apds9900_platform_data *g_platform_data = NULL;
static int apds9900_ftm_working = FALSE;

struct apds9900_priv {
	struct i2c_client *client;
	struct work_struct  alsps_work;
	struct apds9900_reg_cfg_t apds9900_reg_cfg;
	struct wake_lock alsps_wakelock;
	struct delayed_work light_work;
	
	u8 	status;
	u8	irq_enable;	
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

struct apds9900_priv *g_obj = NULL;

// device configuration
static u16 als_threshold_hi_param = 0x00;
static u16 als_threshold_lo_param = 0x00;
static u16 prox_threshold_value = 300;
static u16 prox_threshold_hi_param = 300;
static u16 prox_threshold_lo_param = 0;
static u8 taos_als_adc_time_param = 0xEE;
static u8 taos_prox_adc_time_param = 0xFF;
static u8 taos_prox_wait_time_param = 0xEE;
static u8 prox_intr_filter_param = 0x34;
static u8 taos_configration_param = 0x00;
static u8 taos_prox_pulse_cnt_param = 0x08;
static u8 taos_gain_control_param = 0x22;
static u8 taos_enable_param = 0x00;

struct taos_prox_info prox_cur_info;
struct taos_als_info als_cur_info;
unsigned int ps_work_flag = 0;

static int apds9900lux_table[] = {50, 200, 450, 650, 850};

static int apds9900_init_device(struct i2c_client *client);
static void apds9900_proximity_report(void);
static void apds9900_ambient_report(void);

static int apds9900_power_ctl(struct apds9900_priv * alsps, bool on)
{
	int ret = 0;
	printk("%s",__func__);

	if (!on && alsps->power_enabled) {
		ret = regulator_disable(alsps->vdd);
		if (ret) {
			dev_err(&alsps->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(alsps->vio);
		if (ret) {
			dev_err(&alsps->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			regulator_enable(alsps->vdd);
			return ret;
		}
		alsps->power_enabled = on;
		dev_dbg(&alsps->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else if (on && !alsps->power_enabled) {

		ret = regulator_enable(alsps->vdd);
		if (ret) {
			dev_err(&alsps->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(alsps->vio);
		if (ret) {
			dev_err(&alsps->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(alsps->vdd);
			return ret;
		}
		alsps->power_enabled = on;
		dev_dbg(&alsps->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&alsps->client->dev,
				"Power on=%d. enabled=%d\n",
				on, alsps->power_enabled);
	}

	return ret;
}

static int apds9900_power_init(struct apds9900_priv *alsps, bool on)
{
	int ret;
	printk("%s on%d\n",__func__,on);
	if (!on) {
		if (regulator_count_voltages(alsps->vdd) > 0)
			regulator_set_voltage(alsps->vdd,
					0, STK3X1X_VDD_MAX_UV);

		regulator_put(alsps->vdd);

		if (regulator_count_voltages(alsps->vio) > 0)
			regulator_set_voltage(alsps->vio,
					0, STK3X1X_VIO_MAX_UV);

		regulator_put(alsps->vio);
	} else {
		printk("regulator_get \n");
		
		alsps->vdd = regulator_get(&alsps->client->dev, "vdd");
		if (IS_ERR(alsps->vdd)) {
			ret = PTR_ERR(alsps->vdd);
			dev_err(&alsps->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		printk("regulator_count_voltages\n");

		if (regulator_count_voltages(alsps->vdd) > 0) {
			ret = regulator_set_voltage(alsps->vdd,
					STK3X1X_VDD_MIN_UV,
					STK3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&alsps->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}
		printk("regulator_count vio\n");

		alsps->vio = regulator_get(&alsps->client->dev, "vio");
		if (IS_ERR(alsps->vio)) {
			ret = PTR_ERR(alsps->vio);
			dev_err(&alsps->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(alsps->vio) > 0) {
			ret = regulator_set_voltage(alsps->vio,
					STK3X1X_VIO_MIN_UV,
					STK3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&alsps->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(alsps->vio);
reg_vdd_set:
	if (regulator_count_voltages(alsps->vdd) > 0)
		regulator_set_voltage(alsps->vdd, 0, STK3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(alsps->vdd);
	return ret;
}


static int apds9900_irq_query(void)
{
	return gpio_get_value(80) ? 1 : 0;
}



static int apds9900_enable_func(int func, int enable)
{
	int ret = 0;
	
	if (TAOS_APDS9900_CNTL_PROX_ENBL == func){
		if (enable){
			taos_enable_param |= (TAOS_APDS9900_CNTL_PROX_ENBL | TAOS_APDS9900_CNTL_PROX_INT_ENBL | TAOS_APDS9900_CNTL_WAIT_ENBL | TAOS_APDS9900_CNTL_PWRON);
			atomic_set(&g_obj->ps_deb_on, 1);
			atomic_set(&g_obj->ps_deb_end, jiffies + atomic_read(&g_obj->ps_debounce)/(1000/HZ));
		}else{
			taos_enable_param &= ((~TAOS_APDS9900_CNTL_PROX_ENBL) & (~TAOS_APDS9900_CNTL_PROX_INT_ENBL));
		}
	}
	else if (TAOS_APDS9900_CNTL_ALS_ENBL == func){
		if (enable){
			taos_enable_param |= (TAOS_APDS9900_CNTL_ALS_ENBL | TAOS_APDS9900_CNTL_ALS_INT_ENBL | TAOS_APDS9900_CNTL_WAIT_ENBL | TAOS_APDS9900_CNTL_PWRON);
			atomic_set(&g_obj->als_deb_on, 1);
			atomic_set(&g_obj->als_deb_end, jiffies + atomic_read(&g_obj->als_debounce)/(1000/HZ));
		}else{
			taos_enable_param &= ((~TAOS_APDS9900_CNTL_ALS_ENBL) & (~TAOS_APDS9900_CNTL_ALS_INT_ENBL));
		}
	}
	else{
		return -EINVAL;
	}

	ret = apds9900_init_device(apds9900_i2c_client);

	return ret;
}

static int apds9900_check_and_clear_intr(void)
{	
	int interrupt_id = 0;
    int ret = 0;
    
	if ((ret = i2c_smbus_read_byte_data(apds9900_i2c_client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_STATUS))) < 0){
		printk(KERN_ERR "TAOS: i2c_smbus_read_byte_data failed in apds9900_check_and_clear_intr() err %d\n", ret);

		ret = 0x30; 
	}

	interrupt_id = ret & 0x30;
	interrupt_id |= ((ret & 0x03) << 4);

	switch(interrupt_id)
	{
	case TAOS_APDS9900_STATUS_PINT:
		
	case TAOS_APDS9900_STATUS_AINT:
	
	case (TAOS_APDS9900_STATUS_PINT | TAOS_APDS9900_STATUS_AINT):
		if ((ret = (i2c_smbus_write_byte(apds9900_i2c_client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_CMD_SPL_FN | TAOS_APDS9900_CMD_ALS_AND_PS_INTCLR)))) < 0) {
			printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in apds9900_check_and_clear_intr() err %d\n", ret);
			return (ret);
		}
		break;

    default:
		DEBUG_LOG("TAOS: No interrupt bit need to be cleared in apds9900_check_and_clear_intr()\n");
		break;
	}
	
	return (interrupt_id);
}

static int apds9900_get_lux_value(struct taos_als_info *alsp, bool reset_thresh_enable) {

	u16 raw_clear = 0, raw_ir = 0;
	unsigned long endt;
	u8 i = 0;
	u8 rawdata[4];
	u8 chdata[4];
 
	if (i2c_smbus_read_i2c_block_data(apds9900_i2c_client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_CMD_AUTO_INCREMENT | TAOS_APDS9900_ALS_CHAN0LO ), 4, chdata) < 4){
		printk(KERN_ERR "TAOS: i2c_smbus_read_i2c_block_data() to als/prox data reg failed in apds9900_get_lux_value()\n");
		return -EBUSY;
	}
	
	raw_clear = chdata[1] << 8 | chdata[0];
	raw_ir = chdata[3] << 8 | chdata[2];
	
	DEBUG_LOG("\napds9900_get_lux() ch0=%d, ch1=%d.\n", raw_clear, raw_ir);

	if (reset_thresh_enable){
		if (raw_clear > 3500){
			als_threshold_hi_param = 5000;
			als_threshold_lo_param = 3500;
			i = 4;
		}else if (raw_clear > 1500){
			als_threshold_hi_param = 3500;
			als_threshold_lo_param = 1500;
			i = 3;
		}else if (raw_clear > 500){
			als_threshold_hi_param = 1500;
			als_threshold_lo_param = 500;
			i = 2;
		}else if (raw_clear  > 50){
			als_threshold_hi_param = 500;
			als_threshold_lo_param = 50;
			i = 1;
		}else {
			als_threshold_hi_param = 50;
			als_threshold_lo_param = 0;
			i = 0;
		}

		rawdata[0] = als_threshold_lo_param & 0x00ff;
		rawdata[1] = als_threshold_lo_param >> 8;
		rawdata[2] = als_threshold_hi_param & 0x00ff;
		rawdata[3] = als_threshold_hi_param >> 8;

		if (i2c_smbus_write_i2c_block_data(apds9900_i2c_client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_CMD_AUTO_INCREMENT | TAOS_APDS9900_ALS_MINTHRESHLO), 4, rawdata) < 0){
			printk(KERN_ERR "TAOS: reset als threshold: i2c_smbus_write_i2c_block_data failed\n");
			return -EAGAIN;	
		}
	}
	if (1 == atomic_read(&g_obj->als_deb_on)) {
		endt = atomic_read(&g_obj->als_deb_end);
		if (time_after(jiffies, endt)){
			atomic_set(&g_obj->als_deb_on, 0);
		}else{
			return g_obj->als_intensity;
		}
	}
	g_obj->als_intensity = apds9900lux_table[i];
	return g_obj->als_intensity;	
}

static int apds9900_get_ps_value(struct taos_prox_info *prxp, bool reset_thresh_enable) 
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

	if (i2c_smbus_read_i2c_block_data(apds9900_i2c_client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_CMD_AUTO_INCREMENT | TAOS_APDS9900_ALS_CHAN0LO ), 6, chdata) < 6){
		printk(KERN_ERR "TAOS: i2c_smbus_read_i2c_block_data() to als/prox data reg failed in apds9900_get_ps_value()\n");
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

		if (i2c_smbus_write_i2c_block_data(apds9900_i2c_client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_CMD_AUTO_INCREMENT | TAOS_APDS9900_PRX_MINTHRESHLO), 4, rawdata) < 0){
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
	
	if (!g_obj){
		DEBUG_ERR("alsps_work_handler(): g_obj null\n");
		return;
	}

	interrupt_status = apds9900_check_and_clear_intr();
	printk(KERN_ERR "TAOS PS status = 0x%x\n", interrupt_status);
	switch (interrupt_status){
		case TAOS_APDS9900_STATUS_AINT:
			if ((ret = apds9900_get_ps_value(&prox_cur_info, TRUE)) > 0){
				apds9900_proximity_report();
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS PS Interrupt get_ps_value err in alsps_work_handler() err %d\n", ret);
			}
			break;
		
		case TAOS_APDS9900_STATUS_PINT:
			
			if ((ret = apds9900_get_lux_value(&als_cur_info, TRUE)) > 0){
				DEBUG_LOG(KERN_INFO "TAOS LAS Interrupt get_als_value = %d\n", g_obj->als_intensity);
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS ALS Interrupt get_als_value err in alsps_work_handler() err %d\n",ret);
			}
			
			break;

		case (TAOS_APDS9900_STATUS_AINT | TAOS_APDS9900_STATUS_PINT):

			if ((ret = apds9900_get_ps_value(&prox_cur_info, TRUE)) >= 0){
				apds9900_proximity_report();
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS ALS & PS Interrupt get_ps_value err in alsps_work_handler() err %d\n", ret);
			}
			
			if ((ret = apds9900_get_lux_value(&als_cur_info, TRUE)) > 0){
				DEBUG_LOG(KERN_INFO "TAOS ALS Interrupt get_als_value = %d\n",g_obj->als_intensity);
			}
			else{
				DEBUG_LOG(KERN_ERR "TAOS ALS Interrupt get_als_value err in alsps_work_handler() err %d\n", ret);
			}
			break;
			
		default:
			break;
	}
	
	return;
}

static irqreturn_t alsps_irq_handler(int irq, void *dev_id)
{   
    if (!g_obj){
		DEBUG_ERR("alsps_irq_handler(): g_obj null\n");
    	return IRQ_NONE;
    }
	
	wake_lock_timeout(&g_obj->alsps_wakelock, 2*HZ);

	if (apds9900_irq_query())		
		return IRQ_NONE;				
	
    schedule_work(&g_obj->alsps_work);
	
	return IRQ_HANDLED;
}

static void apds9900_proximity_poweron(void)
{	
	if (!g_obj){
		DEBUG_ERR("apds9900_proximity_poweron(): g_obj null\n");
		return;
	}
	
	apds9900_enable_func(TAOS_APDS9900_CNTL_PROX_ENBL, TRUE);
	ps_work_flag = 1;

	if (!apds9900_irq_query()){
		apds9900_check_and_clear_intr();
	}
	
	if (!g_obj->irq_enable){
		enable_irq(apds9900_i2c_client->irq);
		g_obj->irq_enable = TRUE;
	}

	g_obj->ps_intensity = PROX_DETECTION_FAR;
	g_obj->status |=  TAOS_APDS9900_STATUS_PROX_WORKING;
	apds9900_proximity_report();
	DEBUG_LOG("TAOS: apds9900 proximity poweron OK!\n");
	return;
}

static void apds9900_proximity_poweroff(void)
{
	if (!g_obj){
		DEBUG_ERR("apds9900_proximity_poweroff(): g_obj null\n");
		return;
	}
	
	apds9900_enable_func(TAOS_APDS9900_CNTL_PROX_ENBL, FALSE);
	
	g_obj->status &= (~TAOS_APDS9900_STATUS_PROX_WORKING);

	if ( !(g_obj->status & TAOS_APDS9900_STATUS_MASK )){
		if (g_obj->irq_enable){
			disable_irq(apds9900_i2c_client->irq);
			g_obj->irq_enable = FALSE;
		}	
	}
    ps_work_flag = 0;

	DEBUG_LOG("TAOS: apds9900 proximity poweroff OK!\n");

	return;
}

static void apds9900_proximity_report(void)
{
	if (!g_obj){
		DEBUG_ERR("apds9900_proximity_report(): g_obj null\n");
		return;
	}
	input_report_abs(g_obj->proximity_input, ABS_DISTANCE, g_obj->ps_intensity);
	input_sync(g_obj->proximity_input);
}

static void apds9900_ambient_poweron(void)
{	
	if (!g_obj){
		DEBUG_ERR("apds9900_ambient_poweron(): g_obj null\n");
		return;
	}

	apds9900_enable_func(TAOS_APDS9900_CNTL_ALS_ENBL, TRUE);
	
	if (!apds9900_irq_query()){
		apds9900_check_and_clear_intr();
	}
	
	if (!g_obj->irq_enable){
		enable_irq(apds9900_i2c_client->irq);
		g_obj->irq_enable = TRUE;
	}
	
	//apds9900_get_lux_value(&als_cur_info, TRUE);
	g_obj->status |= TAOS_APDS9900_STATUS_ALS_WORKING;
	schedule_delayed_work(&g_obj->light_work,\
					msecs_to_jiffies(atomic_read(&g_obj->light_delay)));
	apds9900_ambient_report();
	DEBUG_LOG("TAOS: apds9900 ambient poweron OK!\n");
	return;
}

static void apds9900_ambient_report(void)
{
	if (!g_obj){
		DEBUG_ERR("apds9900_ambient_report(): g_obj null\n");
		return;
	}
	input_report_abs(g_obj->light_input, ABS_MISC, g_obj->als_intensity);
	input_sync(g_obj->light_input);
}


static void apds9900_ambient_poweroff(void)
{
	if (!g_obj){
		DEBUG_ERR("apds9900_ambient_poweroff(): g_obj null\n");
		return;
	}
	apds9900_enable_func(TAOS_APDS9900_CNTL_ALS_ENBL, FALSE);

	g_obj->status &= (~TAOS_APDS9900_STATUS_ALS_WORKING);

	if (!(g_obj->status & TAOS_APDS9900_STATUS_MASK)){
		if (g_obj->irq_enable){
			disable_irq(apds9900_i2c_client->irq);
			g_obj->irq_enable = FALSE;
		}
	}
	cancel_delayed_work_sync(&g_obj->light_work);
	//atomic_set(&g_obj->light_delay, DEFAULT_DELAY);
	DEBUG_LOG("TAOS: apds9900 ambient poweroff OK!\n");

	return;
}


static inline void apds9900_alsps_debounce_init(void)
{
	atomic_set(&g_obj->als_debounce, 1000);
    atomic_set(&g_obj->als_deb_on, 0);
    atomic_set(&g_obj->als_deb_end, 0);
    atomic_set(&g_obj->ps_debounce, 100);
    atomic_set(&g_obj->ps_deb_on, 0);
    atomic_set(&g_obj->ps_deb_end, 0);
}
static int apds9900_init_device(struct i2c_client *client)
{
	bool try_one_more_time = FALSE;
	DEBUG_LOG ("%s\n",__func__);

	if (!g_obj){
		DEBUG_ERR("apds9900_init_device(): g_obj null\n");
		return -EINVAL;
	}
	g_obj->als_intensity = LUX_DEFAULT;
	g_obj->ps_intensity = PROX_DETECTION_FAR;
	
	g_obj->apds9900_reg_cfg.enable = taos_enable_param;
	g_obj->apds9900_reg_cfg.als_adc_time = taos_als_adc_time_param;
	g_obj->apds9900_reg_cfg.prox_adc_time = taos_prox_adc_time_param;
	g_obj->apds9900_reg_cfg.prox_wait_time = taos_prox_wait_time_param;
	g_obj->apds9900_reg_cfg.als_minthreshhi= als_threshold_lo_param >> 8;
	g_obj->apds9900_reg_cfg.als_minthreshlo= als_threshold_lo_param & 0x00ff;
	g_obj->apds9900_reg_cfg.als_maxthreshhi = als_threshold_hi_param >> 8;
	g_obj->apds9900_reg_cfg.als_maxthreshlo = als_threshold_hi_param & 0x00ff;
   	g_obj->apds9900_reg_cfg.prox_minthreshhi= prox_threshold_lo_param >> 8;
	g_obj->apds9900_reg_cfg.prox_minthreshlo= prox_threshold_lo_param & 0x00ff;
	g_obj->apds9900_reg_cfg.prox_maxthreshhi = prox_threshold_hi_param >> 8;
	g_obj->apds9900_reg_cfg.prox_maxthreshlo = prox_threshold_hi_param & 0x00ff;
	if(apds9900_ftm_working){
		g_obj->apds9900_reg_cfg.prox_intr_filter = FTM_ALS_RATE;
	}else {
		g_obj->apds9900_reg_cfg.prox_intr_filter = prox_intr_filter_param;
	}
	g_obj->apds9900_reg_cfg.configuration = taos_configration_param;
	g_obj->apds9900_reg_cfg.prox_pulse_cnt = taos_prox_pulse_cnt_param;
	g_obj->apds9900_reg_cfg.gain_control = taos_gain_control_param;
	
retry_init_reg:
	if (i2c_smbus_write_i2c_block_data(client, (TAOS_APDS9900_CMD_REG | TAOS_APDS9900_CMD_AUTO_INCREMENT), sizeof(g_obj->apds9900_reg_cfg), (u8 *)&g_obj->apds9900_reg_cfg) < 0){
		printk(KERN_ERR "TAOS: apds9900_init_device: i2c_smbus_write_i2c_block_data failed in init devices\n");

		if (!try_one_more_time){
			mdelay(20);
			try_one_more_time = TRUE;
			goto retry_init_reg;
		}

		return -EAGAIN;
	}
    return 0;
}

static void light_work_func(struct work_struct *work){
	
	unsigned long delay = msecs_to_jiffies(atomic_read(&g_obj->light_delay));
	
	apds9900_ambient_report();
	schedule_delayed_work(&g_obj->light_work, delay);  
}
	
	
/******************************************************************************
									Light sysfs attributes
*******************************************************************************/
static ssize_t apds9900_light_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf){
	u8 light_enable = 0;
	if(g_obj->status & TAOS_APDS9900_STATUS_ALS_WORKING){
		light_enable = 1;
	}
	return sprintf(buf, "%s\n", light_enable ? "enable" : "disable");
}
	
static ssize_t apds9900_light_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count){
			
	int enable;
	int ret;	
	ret = sscanf(buf, "%d", &enable);
		
	if (ret == 1)
	{
		if (enable)
		{
			apds9900_ambient_poweron();
			apds9900_ftm_working = FALSE;
		}else
		{
			apds9900_ambient_poweroff();
		}
	}
	return count;
}
	
static ssize_t apds9900_light_poll_delay_show(struct device *dev,
			struct device_attribute *attr, char *buf){
			
	return sprintf(buf, "%d\n",atomic_read(&g_obj->light_delay));
}
	
static ssize_t apds9900_light_Poll_delay_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count){
			
	int ms;
	int delay;
	int ret;
	ret = sscanf(buf, "%d", &ms);
	delay = ms;
	
	if (ret == 1)
	{
		if (FTM_LIGHT_SENSOR_DELAY == delay)
		   {
               apds9900_ftm_working = TRUE;
                
               if(g_obj->status & TAOS_APDS9900_STATUS_ALS_WORKING){
                   apds9900_ambient_poweroff();
           }

               atomic_set(&g_obj->light_delay, 120);

               apds9900_ambient_poweron();
		    }
		    else
		    {   
			    atomic_set(&g_obj->light_delay, delay);
			}
	}
	return count;
}

static int apds9900_sysfs_create_light_group(void)
{
	int err;
	static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
				apds9900_light_enable_show, apds9900_light_enable_store);
	static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
				apds9900_light_poll_delay_show, apds9900_light_Poll_delay_store);
		
	static struct attribute *apds9900_light_attributes[] = {
			&dev_attr_enable.attr,
			&dev_attr_poll_delay.attr,
			NULL
	};
		
	static struct attribute_group apds9900_light_attribute_group = {
			.attrs = apds9900_light_attributes
	};
	
	err = sysfs_create_group(&g_obj->light_input->dev.kobj,
				&apds9900_light_attribute_group);
	if (err < 0)
	{
		sysfs_remove_group(&g_obj->light_input->dev.kobj, 
				&apds9900_light_attribute_group);
	}

	return err;



}

/******************************************************************************
									Proximity sysfs attributes
*******************************************************************************/
static ssize_t apds9900_proximity_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf){
	u8 proximity_enable = 0;
	
	if(g_obj->status & TAOS_APDS9900_STATUS_PROX_WORKING){
		proximity_enable = 1;
	}
	return sprintf(buf, "%s\n", proximity_enable ? "enable" : "disable");
}
	
static ssize_t apds9900_proximity_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count){
			
	int enable;
	int ret;	
	ret = sscanf(buf, "%d", &enable);
		
	if (ret == 1)
	{
		if (enable)
		{
			apds9900_proximity_poweron();
		}else
		{
			apds9900_proximity_poweroff();
		}
	}
	return count;
}

static int apds9900_sysfs_create_proximity_group(void)
{
	int err;
	static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
				apds9900_proximity_enable_show, apds9900_proximity_enable_store);
		
	static struct attribute *apds9900_proximity_attributes[] = {
			&dev_attr_enable.attr,
			NULL
	};
		
	static struct attribute_group apds9900_proximity_attribute_group = {
			.attrs = apds9900_proximity_attributes
	};
	
	err = sysfs_create_group(&g_obj->proximity_input->dev.kobj,
				&apds9900_proximity_attribute_group);
	if (err < 0)
	{	
		sysfs_remove_group(&g_obj->proximity_input->dev.kobj, 
				&apds9900_proximity_attribute_group);
	}
	
	return err;

}



static int apds9900_proximity_input_init(struct apds9900_priv *apds9900)
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
	input_set_drvdata(dev, apds9900);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		apds9900->proximity_input = NULL;
		return err;
	}
	apds9900->proximity_input = dev;

	return 0;
}

static int apds9900_light_input_init(struct apds9900_priv *apds9900als)
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
	input_set_drvdata(dev, apds9900als);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		apds9900als->light_input = NULL;
		return err;
	}
	apds9900als->light_input = dev;

	return 0;
}

static void apds9900_input_delete(struct apds9900_priv *apds9900alsps)
{
	if (apds9900alsps->proximity_input != NULL)
	{
		input_unregister_device(apds9900alsps->proximity_input);
		input_free_device(apds9900alsps->proximity_input);
	}

	if (apds9900alsps->light_input != NULL)
	{
		input_unregister_device(apds9900alsps->light_input);
		input_free_device(apds9900alsps->light_input);
	}
}

#ifdef CONFIG_OF
 static int apds_parse_dt(struct device *dev,
			 struct apds9900_platform_data *pdata)
 {
	 //int rc;
	 struct device_node *np = dev->of_node;
	 pdata->int_pin = of_get_named_gpio_flags(np, "apds,irq-gpio",
				 0, &pdata->int_flags);
	 if (pdata->int_pin < 0) {
		 dev_err(dev, "Unable to read irq-gpio\n");
		 return pdata->int_pin;
	 }
 

	 pdata->use_fir = of_property_read_bool(np, "apds,use-fir");
 
	 return 0;
 }
#else
 static int apds_parse_dt(struct device *dev,
			 struct apds9900_platform_data *pdata)
 {
	 return -ENODEV;
 }
#endif /* !CONFIG_OF */



static int __devinit apds9900_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9900_platform_data *pdata;
	int err = 0;
	DEBUG_LOG("%s\n",__func__);

	client->addr =0x39;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "TAOS: apds9900_probe() - i2c smbus byte data functions unsupported\n");
		err = -EIO;
		goto exit;
	}

	if (!i2c_check_functionality(adapter, I2C_SMBUS_I2C_BLOCK_DATA)) {
		printk(KERN_ERR "TAOS: apds9900_probe() - i2c smbus i2c block data functions unsupported\n");
	}

	g_obj = kzalloc(sizeof(struct apds9900_priv), GFP_KERNEL);
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

		err =apds_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "Failed to get pdata from device tree\n");
			goto exit;
		}
	} 
	else {
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

	
	wake_lock_init(&g_obj->alsps_wakelock, WAKE_LOCK_SUSPEND, "alsps9900");
	
	i2c_set_clientdata(client, g_obj);
	INIT_WORK(&g_obj->alsps_work, alsps_work_handler);
	
	g_obj->irq_enable = TRUE;


	err = apds9900_power_init(g_obj, true);
		if (err)
			goto exit;

	err = apds9900_power_ctl(g_obj, true);
	if (err)
		goto exit;


	err = apds9900_init_device(client);
	if (err < 0){
	    apds9900_i2c_client = NULL;
		printk(KERN_ERR "[apds9900]: apds9900 device init fail.\n");
		goto exit_kfree;
	}
	else{
		apds9900_i2c_client = client;
		printk(KERN_INFO "[apds9900]: apds9900 device init OK.\n");
	}

	apds9900_alsps_debounce_init();
	
	err = request_irq(client->irq, alsps_irq_handler, IRQF_TRIGGER_FALLING, "alsps9900", client);
	if (err){
		printk(KERN_ERR "[apds9900]: Request IRQ for alsps failed, return:%d\n",err);
		goto exit_kfree;
	}

	disable_irq(apds9900_i2c_client->irq);
	g_obj->irq_enable = FALSE;
	
	err = apds9900_light_input_init(g_obj);
	if (err < 0)
		goto error_exit;
	
	err = apds9900_proximity_input_init(g_obj);
	if (err < 0)
		goto error_exit;

	err = apds9900_sysfs_create_proximity_group();
	if (err < 0)
		goto error_exit;

	err = apds9900_sysfs_create_light_group();
	if (err < 0)
		goto error_exit;

	INIT_DELAYED_WORK(&g_obj->light_work, light_work_func);
	atomic_set(&g_obj->light_delay, DEFAULT_DELAY);
	

	printk(KERN_INFO "[apds9900]: support ver. %s enabled\n", DRIVER_VERSION);

	return 0;
 error_exit:
	apds9900_input_delete(g_obj);
 exit_kfree:
 	wake_lock_destroy(&g_obj->alsps_wakelock);
	kfree(g_obj);
 exit:
	return err;
}

static int __devexit apds9900_remove(struct i2c_client *client)
{
	wake_lock_destroy(&g_obj->alsps_wakelock);
	free_irq(client->irq, NULL);
	kfree(g_obj);
	
	return 0;
}
#ifdef CONFIG_PM
static int apds9900_suspend(struct i2c_client *client, pm_message_t state)
{
	if (g_obj->status & TAOS_APDS9900_STATUS_PROX_WORKING){
		if (g_obj->irq_enable){
			disable_irq(client->irq);
			g_obj->irq_enable = FALSE;
		}
		
		enable_irq_wake(client->irq);
	}

	return 0;
}

static int apds9900_resume(struct i2c_client *client)
{
	if (g_obj->status & TAOS_APDS9900_STATUS_PROX_WORKING){
		disable_irq_wake(client->irq);

		if (!g_obj->irq_enable){
			enable_irq(client->irq);
			g_obj->irq_enable = TRUE;
		}
	}
	
	return 0;
}

#else 
#define apds9900_suspend  NULL
#define apds9900_resume   NULL
#endif

static const struct i2c_device_id apds9900_id[] = {
	 { "apds9900", 0 },
	 { }
};

MODULE_DEVICE_TABLE(i2c, apds9900_id);

static struct of_device_id apds9900_match_table[] = {
	{ .compatible = "apds,apds9900",},
	{ },
};

static struct i2c_driver apds9900_driver = {
	.driver = {
		.owner = THIS_MODULE,	
		.name = "apds9900",
		.of_match_table = apds9900_match_table,
	},
	.probe  = apds9900_probe,
	.remove = __devexit_p(apds9900_remove),
	.suspend = apds9900_suspend,
	.resume = apds9900_resume,
	.id_table = apds9900_id,
};

static int __init apds9900_init(void)
{ 

	return i2c_add_driver(&apds9900_driver);
}
 
static void __exit apds9900_exit(void)
{
	 i2c_del_driver(&apds9900_driver);
	 printk(KERN_INFO "apds9900 exit\n");
}

MODULE_AUTHOR("liujiquan <liujiquan@gosomo.cn>");
MODULE_DESCRIPTION("apds9900 ambient&light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

late_initcall(apds9900_init);
module_exit(apds9900_exit);

