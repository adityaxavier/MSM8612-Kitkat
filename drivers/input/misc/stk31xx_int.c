/*
 *  stk31xx_int.c - Linux kernel modules for proximity/ambient light sensor
 *  (Intrrupt Mode)
 *
 *  Copyright (C) 2011 Patrick Chang / sensortek <patrick_chang@sitronix.com.tw>
 *  Copyright (C) 2012 Lex Hsieh     / sensortek <lex_hsieh@sitronix.com.tw>
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
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/ktime.h>

#include "stk31xx.h"
#include "stk_defines.h"
#include "stk_lk_defs.h"
#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU))
#include   <linux/fs.h>   
#include  <asm/uaccess.h> 
#endif	
#ifdef SITRONIX_PERMISSION_THREAD
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#endif 	//	#ifdef SITRONIX_PERMISSION_THREAD

#define ADDITIONAL_GPIO_CFG 1

/* // Additional GPIO CFG Header */
#if ADDITIONAL_GPIO_CFG
#include <linux/gpio.h>
#ifdef SPREADTRUM_DRIVER
extern int sprd_3rdparty_gpio_pls_irq;
#endif
// Use irq_to_gpio() if it is possible
// #include <plat/gpio.h>
// #define EINT_GPIO (irq_to_gpio(client->irq))
#endif

#define MIN_ALS_POLL_DELAY_NS	110000000


#define STKALS_DRV_NAME	"stk_als"
#define STKPS_DRV_NAME "stk_ps"
#define DEVICE_NAME		"stk-oss"
#define DRIVER_VERSION  STK_DRIVER_VER
#define LightSensorDevName "stk_als"
#define ProximitySensorDevName "stk_ps"

#define STK_LOCK0 mutex_unlock(&stkps_io_lock)
#define STK_LOCK1 mutex_lock(&stkps_io_lock)

#if( !defined(CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD) && defined(CONFIG_STK31XX_INT_MODE))
static uint32_t lux_threshold_table[] =
{
	3,
	10,
	40,
	65,
	145,
	300,
	550,
	930,
	1250,
	1700,
};

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table)/sizeof(uint32_t)+1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE+1];
#endif 	//#if( !defined(CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD) && defined(CONFIG_STK31XX_INT_MODE))

static int32_t init_all_setting(struct stk31xx_platform_data* plat_data);
static int32_t enable_ps(uint8_t enable);
static int32_t enable_als(uint8_t enable);
static int32_t software_reset(void);

static int32_t set_als_it(uint8_t it);
static int32_t set_als_gain(uint8_t gain);
static int32_t set_ps_it(uint8_t it);
static int32_t set_ps_slp(uint8_t slp);
static int32_t set_ps_led_driving_current(uint8_t irdr);
static int32_t set_ps_gc(uint8_t gc);

static int32_t set_ps_thd_l(uint8_t thd_l);
static int32_t set_ps_thd_h(uint8_t thd_h);

static int32_t set_als_thd_l(uint16_t thd_l);
static int32_t set_als_thd_h(uint16_t thd_h);

static int32_t reset_int_flag(uint8_t org_status,uint8_t disable_flag);
static int32_t enable_ps_int(uint8_t enable);
static int32_t enable_als_int(uint8_t enable);
static struct mutex stkps_io_lock;
static struct stkps31xx_data* pStkPsData = NULL;
static uint8_t ps_code_low_thd;
static uint8_t ps_code_high_thd;
static int32_t als_transmittance;


inline uint32_t alscode2lux(uint32_t alscode)
{
   alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));     // 137.5
      //x1       //   x128         x8            x0.5
    alscode<<=3; // x 8 (software extend to 19 bits)
    // Gain & IT setting ==> x8
    // ==> i.e. code x 8800
    // Org : 1 code = 0.88 Lux
    // 8800 code = 0.88 lux --> this means it must be * 1/10000

    alscode/=als_transmittance;
    return alscode;
}

inline uint32_t lux2alscode(uint32_t lux)
{
    lux*=als_transmittance;
    lux/=1100;
    if (unlikely(lux>=(1<<16)))
        lux = (1<<16) -1;
    return lux;

}

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
static void init_code_threshold_table(void)
{
    uint32_t i,j;
    uint32_t alscode;

    code_threshold_table[0] = 0;
    INFO("alscode[0]=%d\n",0);
    for (i=1,j=0;i<LUX_THD_TABLE_SIZE;i++,j++)
    {
        alscode = lux2alscode(lux_threshold_table[j]);
        INFO("alscode[%d]=%d\n",i,alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }
    code_threshold_table[i] = 0xffff;
    INFO("alscode[%d]=%d\n",i,alscode);
}

static uint32_t get_lux_interval_index(uint16_t alscode)
{
    uint32_t i;
    for (i=1;i<=LUX_THD_TABLE_SIZE;i++)
    {
        if ((alscode>=code_threshold_table[i-1])&&(alscode<code_threshold_table[i]))
        {
            return i;
        }
    }
    return LUX_THD_TABLE_SIZE;
}
#else
inline void set_als_new_thd_by_reading(uint16_t alscode)
{
    int32_t high_thd,low_thd;
    high_thd = alscode + lux2alscode(CONFIG_STK_ALS_CHANGE_THRESHOLD);
    low_thd = alscode - lux2alscode(CONFIG_STK_ALS_CHANGE_THRESHOLD);
    if (high_thd >= (1<<16))
        high_thd = (1<<16) -1;
    if (low_thd <0)
        low_thd = 0;
    set_als_thd_h((uint16_t)high_thd);
    set_als_thd_l((uint16_t)low_thd);
}

#endif // CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

static int32_t init_all_setting(struct stk31xx_platform_data* plat_data)
{
    if (software_reset()<0)
    {
        ERR("STK PS : error --> device not found\n");
        return 0;
    }
	pStkPsData->ps_cmd_reg = plat_data->ps_cmd;
	pStkPsData->als_cmd_reg = plat_data->als_cmd;
	pStkPsData->ps_gc_reg = plat_data->ps_gain;
	
    enable_ps(0);
    enable_als(0);
    set_ps_slp((plat_data->ps_cmd & STK_PS_CMD_SLP_MASK) >> STK_PS_CMD_SLP_SHIFT);
    set_ps_gc(plat_data->ps_gain);
    set_ps_it((plat_data->ps_cmd & STK_PS_CMD_IT_MASK) >> STK_PS_CMD_IT_SHIFT);
    set_ps_led_driving_current((plat_data->ps_cmd & STK_PS_CMD_DR_MASK) >> STK_PS_CMD_DR_SHIFT);
    set_als_gain((plat_data->als_cmd & STK_ALS_CMD_GAIN_MASK) >> STK_ALS_CMD_GAIN_SHIFT); 
    set_als_it((plat_data->als_cmd & STK_ALS_CMD_IT_MASK) >> STK_ALS_CMD_IT_SHIFT); 
    set_ps_thd_h(plat_data->ps_high_thd);
    set_ps_thd_l(plat_data->ps_low_thd);

    enable_ps_int(1);
    enable_als_int(1);
	pStkPsData->ps_distance_last = 1;
    return 1;
}

static int32_t software_reset(void)
{
    // software reset and check stk 83xx is valid
    int32_t r;
    uint8_t w_reg;
    uint8_t org_reg;

    r = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_GC_REG);
    if (r<0)
    {
        ERR("STK PS software reset: read i2c error\n");
        return r;
    }
    org_reg = (uint8_t)(r&0xf0);
    w_reg = ~((uint8_t)(r&0xff));
    r = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_GC_REG,w_reg);
    if (r<0)
    {
        ERR("STK PS software reset: write i2c error\n");
        return r;
    }
    r = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_GC_REG);
    if (w_reg!=(uint8_t)(r&0xff))
    {
        ERR("STK PS software reset: read-back value is not  the same\n");
        return -1;
    }
    r = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_SOFTWARE_RESET_REG,0);
    msleep(5);
    if (r<0)
    {
        ERR("STK PS software reset: read error after reset\n");
        return r;
    }
    return 0;
}

static int32_t enable_ps_int(uint8_t enable)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_INT_MASK);
    pStkPsData->ps_cmd_reg |= STK_PS_CMD_INT(enable);
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
}
static int32_t enable_als_int(uint8_t enable)
{
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_INT_MASK);
    pStkPsData->als_cmd_reg |= STK_ALS_CMD_INT(enable);
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);
}


static int32_t set_als_it(uint8_t it)
{
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_IT_MASK);
    pStkPsData->als_cmd_reg |= (STK_ALS_CMD_IT_MASK & STK_ALS_CMD_IT(it));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);

}
static int32_t set_als_gain(uint8_t gain)
{
	if(gain >= 2)
	{
		INFO("STK PS : als_gain = %d\n", gain);		
	}	
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_GAIN_MASK);
    pStkPsData->als_cmd_reg |= (STK_ALS_CMD_GAIN_MASK & STK_ALS_CMD_GAIN(gain));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);
}
static int32_t set_ps_it(uint8_t it)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_IT_MASK);
    pStkPsData->ps_cmd_reg |= (STK_PS_CMD_IT_MASK & STK_PS_CMD_IT(it));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
}
static int32_t set_ps_slp(uint8_t slp)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_SLP_MASK);
    pStkPsData->ps_cmd_reg |= (STK_PS_CMD_SLP_MASK & STK_PS_CMD_SLP(slp));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);

}
static int32_t set_ps_led_driving_current(uint8_t irdr)
{
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_DR_MASK);
    pStkPsData->ps_cmd_reg |= (STK_PS_CMD_DR_MASK & STK_PS_CMD_DR(irdr));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
}
static int32_t set_ps_gc(uint8_t gc)
{
    int32_t retval;

    retval = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_GC_REG);
    if (retval<0)
        return retval;
    pStkPsData->ps_gc_reg = (uint8_t)retval;
    pStkPsData->ps_gc_reg &= (~STK_PS_GC_GAIN_MASK);
    pStkPsData->ps_gc_reg |= (STK_PS_GC_GAIN(gc)&STK_PS_GC_GAIN_MASK);

    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_GC_REG,pStkPsData->ps_gc_reg);
}


static int32_t set_ps_thd_l(uint8_t thd_l)
{
    ps_code_low_thd = thd_l;
	return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_THD_L_REG,thd_l);
}
static int32_t set_ps_thd_h(uint8_t thd_h)
{
    ps_code_high_thd = thd_h;
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_THD_H_REG,thd_h);
}

static int32_t set_als_thd_l(uint16_t thd_l)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_l;
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    return i2c_smbus_write_word_data(pStkPsData->client,STK_ALS_THD_L1_REG,thd_l);
}
static int32_t set_als_thd_h(uint16_t thd_h)
{
    uint8_t temp;
    uint8_t* pSrc = (uint8_t*)&thd_h;
    temp = *pSrc;
    *pSrc = *(pSrc+1);
    *(pSrc+1) = temp;
    return i2c_smbus_write_word_data(pStkPsData->client,STK_ALS_THD_H1_REG,thd_h);
}

inline int32_t get_status_reg(void)
{
    return i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_STATUS_REG);
}

static int32_t reset_int_flag(uint8_t org_status,uint8_t disable_flag)
{
	uint8_t val;
	
	org_status &= (STK_PS_STATUS_PS_INT_FLAG_MASK | STK_PS_STATUS_ALS_INT_FLAG_MASK);
    val = (uint8_t)(org_status&(~disable_flag));
    return i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_STATUS_REG,val);
}

static inline int32_t get_als_reading(void)
{
    int32_t word_data, tmp_word_data;
	// make sure MSB and LSB are the same data set
	tmp_word_data = i2c_smbus_read_word_data(pStkPsData->client, STK_ALS_DT1_REG);
	if(tmp_word_data < 0)
	{
		ERR("STK PS :%s fail, err=0x%x\n", __func__, tmp_word_data);
		return tmp_word_data;	   
	}
	else
	{
		word_data = ((tmp_word_data & 0xFF00) >> 8) | ((tmp_word_data & 0x00FF) << 8) ;
		//INFO("%s: word_data=0x%4x\n", __func__, word_data);
		return word_data;
	}
}

static inline int32_t get_ps_reading(void)
{
	int32_t ps;
	ps = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_READING_REG);
    if(ps < 0)
	{
		ERR("STK PS :%s fail, err=0x%x\n", __func__, ps);
		return -EINVAL;
	}
	return ps;
}

inline void als_report_event(struct input_dev* dev,int32_t report_value)
{
    pStkPsData->als_lux_last = report_value;
	input_report_abs(dev, ABS_MISC, report_value);
	input_sync(dev);
	INFO("STK PS : als input event %d lux\n",report_value);
}

inline void ps_report_event(struct input_dev* dev,int32_t report_value)
{
    pStkPsData->ps_distance_last = report_value;
	input_report_abs(dev, ABS_DISTANCE, report_value);
	input_sync(dev);
	wake_lock_timeout(&pStkPsData->alsps_wakelock, 2*HZ);
	INFO("STK PS : ps input event %d cm\n",report_value);
}


static int32_t enable_ps(uint8_t enable)
{
    int32_t ret;
    pStkPsData->ps_cmd_reg &= (~STK_PS_CMD_SD_MASK);
    pStkPsData->ps_cmd_reg |= STK_PS_CMD_SD(enable?0:1);
    ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_PS_CMD_REG,pStkPsData->ps_cmd_reg);
    if(enable)
		ps_report_event(pStkPsData->ps_input_dev,8); 
    return ret;

}
static int32_t enable_als(uint8_t enable)
{
    int32_t ret;
    if (enable)
        enable_als_int(0);
    pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_SD_MASK);
    pStkPsData->als_cmd_reg |= STK_ALS_CMD_SD(enable?0:1);
    ret = i2c_smbus_write_byte_data(pStkPsData->client,STK_ALS_CMD_REG,pStkPsData->als_cmd_reg);

    if(enable)
    {		
        set_als_thd_h(0x0000);
        set_als_thd_l(0xFFFF);
        enable_als_int(1);
    }
    return ret;
}



static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t enable, ret;   
    STK_LOCK(1);
    enable = (pStkPsData->ps_cmd_reg & STK_PS_CMD_SD_MASK)?0:1;
    STK_LOCK(0);

    ret = i2c_smbus_read_byte_data(pStkPsData->client,STK_PS_CMD_REG);
    ret &= STK_PS_CMD_SD_MASK;
	ret = !ret;
	
	if(enable == ret)
		return sprintf(buf, "%d\n", ret);
	else
	{
		ERR("STK PS: %s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);
		return sprintf(buf, "%d\n", ret);
	}
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t len)
{
    uint32_t value = simple_strtoul(buf, NULL, 10);
    INFO("STK PS31xx Driver : Enable PS : %d\n",value);
    STK_LOCK(1);
    enable_ps(value);
    STK_LOCK(0);
    return len;
}

static ssize_t als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t enable, ret;
    STK_LOCK(1);
    enable = (pStkPsData->als_cmd_reg & STK_ALS_CMD_SD_MASK)?0:1;
    STK_LOCK(0);
    ret = i2c_smbus_read_byte_data(pStkPsData->client,STK_ALS_CMD_REG);
    ret &= STK_ALS_CMD_SD_MASK;
	ret = !ret;
	
	if(enable == ret)
		return sprintf(buf, "%d\n", ret);
	else
	{
		ERR("STK PS: %s: driver and sensor mismatch! driver_enable=0x%x, sensor_enable=%x\n", __func__, enable, ret);
		return sprintf(buf, "%d\n", ret);
	}
}

static ssize_t als_enable_store(struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t len)
{
    uint32_t value = simple_strtoul(buf, NULL, 10);
    INFO("STK PS31xx Driver : Enable ALS : %d\n",value);
    STK_LOCK(1);
    enable_als(value);
    STK_LOCK(0);
    return len;
}
static ssize_t stk_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%lld\n", ktime_to_ns(pStkPsData->als_poll_delay));
}


static ssize_t stk_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
#if 1
	uint64_t value = 0;
	int ret;
	int	delay;
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoull failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
	delay = value;

	printk(KERN_INFO "%s: set als poll delay=%lld\n", __func__, value);

if (ret == 1)
	{
		if(FTM_LIGHT_SENSOR_DELAY == delay)
		{
		    //if(atomic_read(&pStkPsData->light_enable))
			//{
		       // enable_als(0);
		   // }
				
		       pStkPsData->als_poll_delay = ns_to_ktime(200);
		        enable_als(1);
		}
	    else
	    {   
		    pStkPsData->als_poll_delay = ns_to_ktime(value);
		}

	
	}

#if 0
	if(value < MIN_ALS_POLL_DELAY_NS)
	{
		printk(KERN_ERR "%s: delay is too small\n", __func__);
		value = MIN_ALS_POLL_DELAY_NS;
	}
//	mutex_lock(&ps_data->io_lock);
	if(value != ktime_to_ns(pStkPsData->als_poll_delay))
		pStkPsData->als_poll_delay = ns_to_ktime(value);

//	if (ps_data->use_fir)
//		stk_als_delay_store_fir(ps_data);

//	mutex_unlock(&ps_data->io_lock);
#endif
	return size;

#else
	int ns;
	int	delay;
	int ret;

	ret = sscanf(buf, "%d", &ns);
	delay = ns ; 

	if(ret == 1)
	{
		if(FTM_LIGHT_SENSOR_DELAY == delay)
		{
		    if(atomic_read(&pStkPsData->light_enable))
			{
		        enable_als(0);
		    }
				
		        atomic_set(&pStkPsData->als_poll_delay, 200);
		        enable_als(1);
		}
	    else
	    {   
		    atomic_set(&pStkPsData->als_poll_delay, delay);
		}
	}

	return size;
#endif

}




static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,ps_enable_show,ps_enable_store);

static struct attribute *stk31xx_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    NULL
};
static struct attribute_group stk31xx_ps_attribute_group = {
	.attrs = stk31xx_ps_attrs,
};

static struct device_attribute als_enable_attribute = __ATTR(enable,0664,als_enable_show,als_enable_store);
static struct device_attribute als_poll_delay_attribute =__ATTR(poll_delay, 0664, stk_als_delay_show, stk_als_delay_store);

static struct attribute *stk31xx_als_attrs [] =
{
    &als_enable_attribute.attr,	
	&als_poll_delay_attribute.attr,
    NULL
};
static struct attribute_group stk31xx_als_attribute_group = {
	.attrs = stk31xx_als_attrs,
};

static struct platform_device *stk_oss_dev = NULL; /* Device structure */
static struct workqueue_struct *stk_oss_work_queue = NULL;


static void stk_oss_work(struct work_struct *work)
{

    int32_t ret,reading;
    uint8_t disable_flag = 0;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD	
	uint32_t nLuxIndex;	
#endif
	
    STK_LOCK(1);
    ret = get_status_reg();

	if(ret < 0)
	{
		STK_LOCK(0);
		ERR("stk_oss_work:get_status_reg fail, ret=%d\n", ret);
		msleep(30);
		enable_irq(pStkPsData->irq);
		return; 		
	}	
	
    if (ret&STK_PS_STATUS_ALS_INT_FLAG_MASK)
    {
		disable_flag = STK_PS_STATUS_ALS_INT_FLAG_MASK;
        reading = get_als_reading();
		if(reading < 0)
		{
			STK_LOCK(0);
			ERR("stk_oss_work:get_als_reading fail, ret=%d\n", reading);
			msleep(30);
			enable_irq(pStkPsData->irq);
			return; 
		}
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        nLuxIndex = get_lux_interval_index(reading);
        set_als_thd_h(code_threshold_table[nLuxIndex]);
        set_als_thd_l(code_threshold_table[nLuxIndex-1]);
#else
        set_als_new_thd_by_reading(reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        als_report_event(pStkPsData->als_input_dev,alscode2lux(reading));

    }
    if (ret&STK_PS_STATUS_PS_INT_FLAG_MASK)
    {

        reading = get_ps_reading();
        INFO("%s : ps code = %d\n",__func__,reading);
		if(reading < 0)
		{
			STK_LOCK(0);
			ERR("stk_oss_work:get_ps_reading fail, ret=%d\n", reading);
			msleep(30);
			enable_irq(pStkPsData->irq);
			return; 			
		}
        if (reading>=ps_code_high_thd)
        {
			disable_flag |= STK_PS_STATUS_PS_INT_FLAG_MASK;
            ps_report_event(pStkPsData->ps_input_dev,3);
        }
        else if (reading<ps_code_high_thd)
        {
			disable_flag |= STK_PS_STATUS_PS_INT_FLAG_MASK;
            ps_report_event(pStkPsData->ps_input_dev,8);
		}
		else
		    msleep(10);
    }

    ret = reset_int_flag(ret,disable_flag);
	if(ret < 0)
	{
		STK_LOCK(0);
		ERR("stk_oss_work:reset_int_flag fail, ret=%d\n", ret);
		msleep(30);
		enable_irq(pStkPsData->irq);
		return; 		
	}		
	
	msleep(1);
    enable_irq(pStkPsData->irq);

    STK_LOCK(0);
}



static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stkps31xx_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(stk_oss_work_queue,&pData->work);
	return IRQ_HANDLED;
}

static int stk31xx_suspend(struct device *dev)
{
	int32_t enable;
	int err;
    struct i2c_client *client = to_i2c_client(dev);
	   
	INFO("%s\n", __func__);
	STK_LOCK(1);
    enable = (pStkPsData->als_cmd_reg & STK_ALS_CMD_SD_MASK)?0:1;    		
	if(enable)
	{
		enable_als(0);		
		pStkPsData->als_cmd_reg &= (~STK_ALS_CMD_SD_MASK);	
	}	
	
    enable = (pStkPsData->ps_cmd_reg & STK_PS_CMD_SD_MASK)?0:1;    		
	if(enable)
	{
		/*
		if (pStkPsData->ps_distance_last) {
			//pStkPsData->ps_distance_last = 0;
			WARNING("%s: PS is far\n", __func__);
			STK_LOCK(0);
			return -EAGAIN;
		}
		*/
		//disable_irq(pStkPsData->irq);	
		if(device_may_wakeup(&client->dev))
		{
			err = enable_irq_wake(pStkPsData->irq);	
			if (err)
			{
				WARNING("%s: set_irq_wake(%d,1) failed for (%d)\n",
				__func__, pStkPsData->irq, err);
			}
		}
		else
		{
			ERR("%s: not support wakeup source", __func__);
		}
	}
	STK_LOCK(0);
	return 0;
}

static int stk31xx_resume(struct device *dev)
{
	int32_t enable;
	int err;
    struct i2c_client *client = to_i2c_client(dev);
	
	INFO("%s\n", __func__);
	STK_LOCK(1);
    enable = (pStkPsData->als_cmd_reg & STK_ALS_CMD_SD_MASK)?0:1;    		
	if(enable)
	{
		enable_als(1);		
	}	
	
    enable = (pStkPsData->ps_cmd_reg & STK_PS_CMD_SD_MASK)?0:1;    		
	if(enable)
	{
		if(device_may_wakeup(&client->dev))
		{		
			err = disable_irq_wake(pStkPsData->irq);	
			//enable_irq(pStkPsData->irq);	
			if (err)
			{
				WARNING("%s: disable_irq_wake(%d,1) failed for (%d)\n",
				__func__, pStkPsData->irq, err);
			}
		}		
	}
	STK_LOCK(0);
	return 0;
}

static const struct dev_pm_ops stk31xx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk31xx_suspend, stk31xx_resume)
};

//#ifdef CONFIG_HAS_EARLYSUSPEND

static int stk3x1x_power_ctl(struct stkps31xx_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else if (on && !data->power_enabled) {

		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		dev_dbg(&data->client->dev, "stk3x1x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int stk3x1x_power_init(struct stkps31xx_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, STK3X1X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, STK3X1X_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					STK3X1X_VDD_MIN_UV,
					STK3X1X_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					STK3X1X_VIO_MIN_UV,
					STK3X1X_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, STK3X1X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

#ifdef CONFIG_OF
static int stk3x1x_parse_dt(struct device *dev,
			struct stk31xx_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	pdata->int_pin = of_get_named_gpio_flags(np, "stk,irq-gpio",
				0, &pdata->int_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}
	else
	{
		printk("%s %d\n",__func__,pdata->int_pin);
	}

	rc = of_property_read_u32(np, "stk,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_high_thd = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_low_thd = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}
	pdata->ps_gain = 0x0D;
	pdata->ps_high_thd = 0x41;
	pdata->ps_low_thd = 0x32;
	pdata->als_cmd = 0x49;
	pdata->ps_cmd = 0x21;

	//pdata->use_fir = of_property_read_bool(np, "stk,use-fir");

	return 0;
}
#else
static int stk3x1x_parse_dt(struct device *dev,
			struct stk3x1x_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int stk_ps_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err;
    struct stkps31xx_data* ps_data;
	struct stk31xx_platform_data* plat_data;	
    INFO("STK PS -- %s: I2C is probing, driver version=%s\n", __func__, STK_DRIVER_VER);	
   
   if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	  {
		  printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_BYTE_DATA\n", __func__);
		  return -ENODEV;
	  }
	  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
	  {
		  printk(KERN_ERR "%s: No Support for I2C_FUNC_SMBUS_WORD_DATA\n", __func__);
		  return -ENODEV;
	  }
   printk("%s\n",__func__);
	
    
		err = i2c_smbus_read_byte_data(client,STK_PS_STATUS_REG);
		if (err < 0)
		{
			ERR("STK PS %s: read i2c error, err=0x%x\n", __func__, err);
			return err;
		}
		else
			printk("read ps status ok\n");
		if ((err&STK_PS_STATUS_ID_MASK)!=STK_PS31xx_ID)
		{
			ERR("STK PS %s: invalid ID number\n", __func__);
			return -EINVAL;
		}
		else
			printk(" status ID %d\n",((err&STK_PS_STATUS_ID_MASK)!=STK_PS31xx_ID));
        ps_data = kzalloc(sizeof(struct stkps31xx_data),GFP_KERNEL);

		if(!ps_data)
		{
			printk(KERN_ERR "%s: failed to allocate stk3x1x_data\n", __func__);
			return -ENOMEM;
		}else{
			printk(KERN_ERR "%s:  allocate stk3x1x_data ok\n ", __func__);

    	}
		pStkPsData = ps_data;
        ps_data->client = client;
        i2c_set_clientdata(client,ps_data);
		
		if (client->dev.of_node) {
				plat_data = devm_kzalloc(&client->dev,
					sizeof(struct stk31xx_platform_data), GFP_KERNEL);
				if (!plat_data) {
					dev_err(&client->dev, "Failed to allocate memory\n");
					return -ENOMEM;
				}

				err = stk3x1x_parse_dt(&client->dev, plat_data);
				dev_err(&client->dev,
					"%s: stk31xx_parse_dt ret=%d\n", __func__, err);
				if (err)
					return err;
			} else
				plat_data = client->dev.platform_data;

			if (!plat_data) {
				dev_err(&client->dev,
					"%s: no stk3x1x platform data!\n", __func__);
				 return -ENOMEM;
			}
			else
			{
				printk("plat_data ok \n");
			}

			if (gpio_is_valid(plat_data->int_pin )) {
				err = gpio_request(plat_data->int_pin , "stk_irq_gpio");
			if (err) {
				dev_err(&client->dev, "irq gpio request failed");
				return -ENOMEM;
			}
			err = gpio_direction_input(plat_data->int_pin);
			if (err) {
				dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
				return -ENOMEM;
			}
			}

		err = stk3x1x_power_init(ps_data, true);
			if (err)
				 return -ENOMEM;
		
			err = stk3x1x_power_ctl(ps_data, true);
			if (err)
				 return -ENOMEM;			

		als_transmittance = plat_data->transmittance;			
	
        mutex_init(&stkps_io_lock);
        ps_data->als_input_dev = input_allocate_device();
        ps_data->ps_input_dev = input_allocate_device();
        if ((ps_data->als_input_dev==NULL)||(ps_data->ps_input_dev==NULL))
        {
            if (ps_data->als_input_dev==NULL)
                input_free_device(ps_data->ps_input_dev);                
            if (ps_data->ps_input_dev==NULL)
				input_free_device(ps_data->als_input_dev);
            ERR("%s: could not allocate input device\n", __func__);
            mutex_destroy(&stkps_io_lock);
            kfree(ps_data);
            return -ENOMEM;
        }
        ps_data->als_input_dev->name = ALS_NAME;
        ps_data->ps_input_dev->name = PS_NAME;
        set_bit(EV_ABS, ps_data->als_input_dev->evbit);
        set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
        input_set_abs_params(ps_data->als_input_dev, ABS_MISC, 0, alscode2lux((1<<16)-1), 0, 0);
        input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
        err = input_register_device(ps_data->als_input_dev);
        if (err<0)
        {
            ERR("STK PS : can not register als input device\n");
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);

            return err;
        }
        INFO("STK PS : register als input device OK\n");
        err = input_register_device(ps_data->ps_input_dev);
        if (err<0)
        {
            ERR("STK PS : can not register ps input device\n");
            input_unregister_device(ps_data->als_input_dev);
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);
            return err;

        }
        
		
	
        printk("STK PS : gpio #=%d, irq=%d\n",plat_data->int_pin, gpio_to_irq(plat_data->int_pin));
        if (client->irq<=0)
        {
            ERR("STK PS : fail --> you don't(or forget to) specify a irq number, irq # = %d\n or irq_to_gpio fail\n",client->irq);
            client->irq = 0;
	
#if ADDITIONAL_GPIO_CFG // Additional GPIO CFG
			gpio_free(plat_data->int_pin);
#endif // Additional GPIO CFG
			
			input_unregister_device(ps_data->als_input_dev);
			input_unregister_device(ps_data->ps_input_dev);
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);
            return -EINVAL;
        }

		
        stk_oss_work_queue = create_singlethread_workqueue("stk_oss_wq");
        if (!stk_oss_work_queue)
        {
            ERR("STK PS : create_singlethread_workqueue fail\n");
			client->irq = 0;
		
#if ADDITIONAL_GPIO_CFG // Additional GPIO CFG
			gpio_free(plat_data->int_pin);
#endif // Additional GPIO CFG
				
            input_unregister_device(ps_data->als_input_dev);
            input_unregister_device(ps_data->ps_input_dev);
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);          
			kfree(ps_data);
            return -EINVAL;
        }		
        INIT_WORK(&ps_data->work, stk_oss_work);
        enable_als(0);
        enable_ps(0);

        err = request_irq(client->irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, ps_data);
        if (err < 0) {
            WARNING("%s: request_irq(%d) failed for (%d)\n",
                __func__, client->irq, err);
			client->irq = 0;
		
#if ADDITIONAL_GPIO_CFG // Additional GPIO CFG
			gpio_free(plat_data->int_pin);
#endif // Additional GPIO CFG
			
			destroy_workqueue(stk_oss_work_queue);
            input_unregister_device(ps_data->als_input_dev);
            input_unregister_device(ps_data->ps_input_dev);
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);				
            return err;
        }
        pStkPsData->irq = client->irq;

		err = sysfs_create_group(&pStkPsData->als_input_dev->dev.kobj, &stk31xx_als_attribute_group);
			if (err < 0)
			{
				printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
				return -ENOMEM;
			}
		err = sysfs_create_group(&pStkPsData->ps_input_dev->dev.kobj, &stk31xx_ps_attribute_group);
			if (err < 0)
			{
				printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
				return -ENOMEM;
			}


        wake_lock_init(&pStkPsData->alsps_wakelock,WAKE_LOCK_SUSPEND,"stk_ps_wakelock");
		device_init_wakeup(&client->dev, true);
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        init_code_threshold_table();
#endif
        if (!init_all_setting(plat_data))
        {
			device_init_wakeup(&client->dev, false);
			wake_lock_destroy(&pStkPsData->alsps_wakelock);			
			free_irq(client->irq, NULL);
			client->irq = 0;
		
#if ADDITIONAL_GPIO_CFG // Additional GPIO CFG
			gpio_free(plat_data->int_pin);
#endif // Additional GPIO CFG
			
			destroy_workqueue(stk_oss_work_queue);
            input_unregister_device(pStkPsData->als_input_dev);
            input_unregister_device(pStkPsData->ps_input_dev);
			mutex_destroy(&stkps_io_lock);
            input_free_device(pStkPsData->als_input_dev);
            input_free_device(pStkPsData->ps_input_dev);
            kfree(pStkPsData);
            pStkPsData = NULL;
            return -EINVAL;
        }
//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
		ps_data->stk_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ps_data->stk_early_suspend.suspend = stk31xx_early_suspend;
		ps_data->stk_early_suspend.resume = stk31xx_late_resume;
		register_early_suspend(&ps_data->stk_early_suspend);
#endif
        return 0;
   

    return -EINVAL;
}


static int stk_ps_remove(struct i2c_client *client)
{
	struct stk31xx_platform_data* plat_data;
	plat_data = client->dev.platform_data;

	device_init_wakeup(&client->dev, false);
    wake_lock_destroy(&pStkPsData->alsps_wakelock);
    mutex_destroy(&stkps_io_lock);
    if (pStkPsData)
    {
		free_irq(pStkPsData->irq, NULL);
		
#if ADDITIONAL_GPIO_CFG // Additional GPIO CFG
		gpio_free(plat_data->int_pin);
#endif // Additional GPIO CFG

        if (stk_oss_work_queue)
            destroy_workqueue(stk_oss_work_queue);
        input_unregister_device(pStkPsData->als_input_dev);
        input_unregister_device(pStkPsData->ps_input_dev);
        input_free_device(pStkPsData->als_input_dev);
        input_free_device(pStkPsData->ps_input_dev);
        kfree(pStkPsData);
        pStkPsData = NULL;
    }
    return 0;
}

static const struct i2c_device_id stk_ps_id[] =
{
    { "stk_ps", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] = {
	{ .compatible = "stk,stk3x1x", },
	{ },
};

static struct i2c_driver stk_ps_driver =
{
    .driver = {
        .name = STKPS_DRV_NAME,
		.pm = &stk31xx_pm_ops,	
		.owner = THIS_MODULE,
		.of_match_table = stk_match_table,
    },	
    .probe = stk_ps_probe,
    .remove = stk_ps_remove,
    .id_table = stk_ps_id,
};


static int __init stk_i2c_ps31xx_init(void)
{

	int ret;
    ret = i2c_add_driver(&stk_ps_driver);
    if (ret)
    {
        i2c_del_driver(&stk_ps_driver);
		
        return ret;
    }	
    if (pStkPsData == NULL)
    {
        i2c_del_driver(&stk_ps_driver);
        return -EINVAL;
    }

    stk_oss_dev = platform_device_alloc(DEVICE_NAME,-1);
    if (!stk_oss_dev)
    {
       i2c_del_driver(&stk_ps_driver);
       return -ENOMEM;
    }
    if (platform_device_add(stk_oss_dev))
    {
		platform_device_put(stk_oss_dev);
       i2c_del_driver(&stk_ps_driver);
       return -ENOMEM;
    }
	
#ifdef SITRONIX_PERMISSION_THREAD
	STKPermissionThread = kthread_run(stk_permission_thread,"Sitronix","Permissionthread");
	if(IS_ERR(STKPermissionThread))
		STKPermissionThread = NULL;
#endif // SITRONIX_PERMISSION_THREAD	
	INFO("STK PS Module initialized.\n");
    return 0;
}

static void __exit stk_i2c_ps31xx_exit(void)
{
    if (stk_oss_dev);
    {
#ifdef CONFIG_STK_SYSFS_DBG
        sysfs_remove_group(&(stk_oss_dev->dev.kobj), &sensortek_optics_sensors_attrs_group);
#endif
    }
    i2c_del_driver(&stk_ps_driver);
    platform_device_unregister(stk_oss_dev);
#ifdef SITRONIX_PERMISSION_THREAD
	if(STKPermissionThread)
		STKPermissionThread = NULL;
#endif // SITRONIX_PERMISSION_THREAD		
}

MODULE_AUTHOR("Patrick Chang <patrick_chang@sitronix.com.tw>");
MODULE_DESCRIPTION("sensortek Proximity Sensor driver");
MODULE_LICENSE("GPL");
module_init(stk_i2c_ps31xx_init);
module_exit(stk_i2c_ps31xx_exit);
