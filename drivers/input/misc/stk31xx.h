/*
 *
 * $Id: stk31xx.h,v 1.0 2011/02/26 11:12:08 jsgood Exp $
 *
 * Copyright (C) 2011 Patrick Chang <patrick_chang@sitronix.com.tw>
 * Copyright (C) 2012 Lex Hsieh     <lex_hsieh@sitronix.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __STK_I2C_PS31XX
#define __STK_I2C_PS31XX

#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif
#include <linux/ktime.h>

/* Driver Settings */
#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#define CONFIG_STK_ALS_CHANGE_THRESHOLD	20	/* The threshold to trigger ALS interrupt, unit: lux */	
#endif	/* #ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD */

//#define CONFIG_STK_SYSFS_DBG
//#define CONFIG_STK_PS_ENGINEER_TUNING	/* Mount nodes on SYSFS to tune proximity sensor's parameter for debug purpose */ 
//#define SPREADTRUM_DRIVER

/* STK calibration */
//#define STK_AUTO_CT_CALI_SATU
//#define STK_MANUAL_GREYCARD_CALI
//#define STK_AUTO_CT_CALI_NO_SATU


/* Define Register Map */
#define STK_ALS_CMD_REG 0x01
#define STK_ALS_DT1_REG 0x02
#define STK_ALS_DT2_REG 0x03

#define STK_ALS_THD_H1_REG 0x04
#define STK_ALS_THD_H0_REG 0x05
#define STK_ALS_THD_L1_REG 0x06
#define STK_ALS_THD_L0_REG 0x07

#define STK_PS_STATUS_REG 0x08
#define STK_PS_CMD_REG 0x09
#define STK_PS_READING_REG 0x0A
#define STK_PS_THD_H_REG 0x0B
#define STK_PS_THD_L_REG 0x0C

#define STK_PS_SOFTWARE_RESET_REG 0x80
#define STK_PS_GC_REG 0x82

/* Define ALS CMD */

#define STK_ALS_CMD_GAIN_SHIFT  6
#define STK_ALS_CMD_IT_SHIFT    2
#define STK_ALS_CMD_SD_SHIFT    0
#define STK_ALS_CMD_INT_SHIFT   1

#define STK_ALS_CMD_GAIN(x) ((x)<<STK_ALS_CMD_GAIN_SHIFT)
#define STK_ALS_CMD_IT(x) ((x)<<STK_ALS_CMD_IT_SHIFT)
#define STK_ALS_CMD_INT(x) ((x)<<STK_ALS_CMD_INT_SHIFT)
#define STK_ALS_CMD_SD(x) ((x)<<STK_ALS_CMD_SD_SHIFT)

#define STK_ALS_CMD_GAIN_MASK 0xC0
#define STK_ALS_CMD_IT_MASK 0x0C
#define STK_ALS_CMD_INT_MASK 0x02
#define STK_ALS_CMD_SD_MASK 0x1

/* Define PS Status */
#define STK_PS_STATUS_ID_SHIFT  6

#define STK_PS_STATUS_ID(x) ((x)<<STK_PS_STATUS_ID_SHIFT)

#define STK_PS_STATUS_ID_MASK  0xC0
#define STK_PS_STATUS_PS_INT_FLAG_MASK 0x20
#define STK_PS_STATUS_ALS_INT_FLAG_MASK 0x10

#define STK_PS31xx_ID 0x00


/* Define PS CMD */

#define STK_PS_CMD_INTCTRL_SHIFT 7
#define STK_PS_CMD_SLP_SHIFT    5
#define STK_PS_CMD_DR_SHIFT     4
#define STK_PS_CMD_IT_SHIFT     2
#define STK_PS_CMD_INT_SHIFT    1
#define STK_PS_CMD_SD_SHIFT     0

#define STK_PS_CMD_INTCTRL(x) ((x)<<STK_PS_CMD_INTCTRL_SHIFT)
#define STK_PS_CMD_SLP(x) ((x)<<STK_PS_CMD_SLP_SHIFT)
#define STK_PS_CMD_DR(x) ((x)<<STK_PS_CMD_DR_SHIFT)
#define STK_PS_CMD_IT(x) ((x)<<STK_PS_CMD_IT_SHIFT)
#define STK_PS_CMD_INT(x) ((x)<<STK_PS_CMD_INT_SHIFT)
#define STK_PS_CMD_SD(x) ((x)<<STK_PS_CMD_SD_SHIFT)

#define STK_PS_CMD_INTCTRL_MASK 0x80
#define STK_PS_CMD_SLP_MASK 0x60
#define STK_PS_CMD_DR_MASK 0x10
#define STK_PS_CMD_IT_MASK 0x0C
#define STK_PS_CMD_INT_MASK 0x2
#define STK_PS_CMD_SD_MASK 0x1

/* Define PS GC */
#define STK_PS_GC_GAIN(x) ((x)<<0)
#define STK_PS_GC_GAIN_MASK 0x0f

#define STK3X1X_VDD_MIN_UV	2000000
#define STK3X1X_VDD_MAX_UV	3300000
#define STK3X1X_VIO_MIN_UV	1750000
#define STK3X1X_VIO_MAX_UV	1950000


struct stkps31xx_data {
	struct i2c_client *client;

    uint8_t ps_reading;
    uint16_t als_reading;

	uint8_t ps_gc_reg;
	uint8_t ps_cmd_reg;
	uint8_t als_cmd_reg;

	struct input_dev* als_input_dev;
	struct input_dev* ps_input_dev;
	int32_t ps_distance_last;
	int32_t ps_code_last;
	int32_t als_lux_last;
	uint8_t recv_reg;

	
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend stk_early_suspend;
#endif

	struct work_struct	alsps_work;
	struct wake_lock	alsps_wakelock;
	struct delayed_work light_work;

    struct work_struct work;
    int32_t irq;

    uint8_t ps_enabled;
	uint8_t als_enabled;
	
    struct work_struct stk_als_work;
	struct workqueue_struct *stk_als_wq;
    struct work_struct stk_ps_work;
	struct workqueue_struct *stk_ps_wq;	
	ktime_t als_poll_delay;
	ktime_t ps_poll_delay;
	
	struct hrtimer als_timer;	
	struct hrtimer ps_timer;	
#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU) || defined(STK_AUTO_CT_CALI_NO_SATU))
	bool ps_cali_done;
	bool cali_file_exist;
	bool first_boot;
	int32_t ps_ctk_val;
#endif
	
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;

	u8	irq_enable;	
	u8  light_delay;

};

#define ALS_MIN_DELAY   100
#define PS_MIN_DELAY    10

#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU) || defined(STK_AUTO_CT_CALI_NO_SATU))
#define STK_CALI_SAMPLE_NO		5
#define STK_THD_H_MAX			250
#define STK_THD_H_MIN			35
#define STK_THD_L_MAX			235
#define STK_THD_L_MIN			20

#define STK_HIGH_THD				0
#define STK_LOW_THD				1

#define STK_DATA_MAX			0
#define STK_DATA_AVE				1
#define STK_DATA_MIN				2

const static uint16_t cali_sample_time_table[4] = {20, 40, 100, 300};
#define STK_CALI_VER0			0x48
#define STK_CALI_VER1			0x03
#define STK_CALI_FILE "/data/misc/stkpscali.conf"
#define STK_CALI_FILE_SIZE 		10

#define STK_MAX_PS_CROSSTALK	200
#define STK_THD_H_ABOVE_CT		30
#define STK_THD_L_ABOVE_CT		15
#define STK_CT_DENOMINATOR		10
#define STK_CT_NUMERATOR		13

#define STK_K_OFFSET			70
#define STK_A_OFFSET			120
#define STK_DEF_CTK				50
#define STK_CTK_MAX				150
#define STK_CTA_MAX				220
#endif	/*	#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU) || defined(STK_AUTO_CT_CALI_NO_SATU))	*/

#ifdef STK_MANUAL_GREYCARD_CALI
#define STK_MIN_GREY_PS_DATA			50
#define STK_DIFF_GREY_N_THD_H		15
#define STK_DIFF_GREY_N_THD_L		30
#endif	/*	#ifdef STK_MANUAL_GREYCARD_CALI	*/

#if (defined(STK_MANUAL_GREYCARD_CALI) || defined(STK_AUTO_CT_CALI_SATU))
	#define SITRONIX_PERMISSION_THREAD
#endif

struct stk31xx_platform_data{
	uint8_t als_cmd;
	uint8_t ps_cmd;
	uint8_t ps_gain;
	uint8_t ps_high_thd;
	uint8_t ps_low_thd;
	int32_t transmittance;	
	int 	int_pin;
	uint32_t int_flags;
};
	

#endif // __STK_I2C_PS31XX
