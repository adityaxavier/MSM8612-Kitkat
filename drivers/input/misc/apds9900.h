#ifndef _APDS9900_H_
#define _APDS9900_H_

/*ioctl cmd*/
#define APDS9900_IOC_MAGIC 'C'

#define APDS9900_IOCS_PRO_MODE 			_IOWR(APDS9900_IOC_MAGIC,0, unsigned char)
#define APDS9900_IOCG_PRO_MODE 			_IOWR(APDS9900_IOC_MAGIC,1, unsigned char)
#define APDS9900_IOCG_PRO_DATA 			_IOWR(APDS9900_IOC_MAGIC,2, unsigned char)
#define APDS9900_IOCG_PRO_RAW 			_IOWR(APDS9900_IOC_MAGIC,3, unsigned char)
#define APDS9900_IOCS_LIG_MODE 			_IOWR(APDS9900_IOC_MAGIC,4, unsigned char)
#define APDS9900_IOCG_LIG_MODE 			_IOWR(APDS9900_IOC_MAGIC,5, unsigned char)
#define APDS9900_IOCG_LIG_DATA 			_IOWR(APDS9900_IOC_MAGIC,6, unsigned char)
#define APDS9900_IOCG_LIG_RAW 			_IOWR(APDS9900_IOC_MAGIC,7, unsigned char)

#define APDS9900_IOC_MAXNR	8	


// APDS9900 register offsets
#define TAOS_APDS9900_CNTRL 			0x00
#define TAOS_APDS9900_ALS_TIME 			0x01
#define TAOS_APDS9900_PRX_TIME			0x02
#define TAOS_APDS9900_WAIT_TIME			0x03
#define TAOS_APDS9900_ALS_MINTHRESHLO	0x04
#define TAOS_APDS9900_ALS_MINTHRESHHI 	0x05
#define TAOS_APDS9900_ALS_MAXTHRESHLO	0x06
#define TAOS_APDS9900_ALS_MAXTHRESHHI	0x07
#define TAOS_APDS9900_PRX_MINTHRESHLO 	0x08
#define TAOS_APDS9900_PRX_MINTHRESHHI 	0x09
#define TAOS_APDS9900_PRX_MAXTHRESHLO 	0x0A
#define TAOS_APDS9900_PRX_MAXTHRESHHI 	0x0B
#define TAOS_APDS9900_INTERRUPT			0x0C
#define TAOS_APDS9900_CONFIGURATION	    0x0D
#define TAOS_APDS9900_PRX_PLUSE_COUNT   0x0E
#define TAOS_APDS9900_GAIN_CONTROL		0x0F
#define TAOS_APDS9900_REVID				0x11
#define TAOS_APDS9900_CHIPID      		0x12
#define TAOS_APDS9900_STATUS			0x13
#define TAOS_APDS9900_ALS_CHAN0LO		0x14
#define TAOS_APDS9900_ALS_CHAN0HI		0x15
#define TAOS_APDS9900_ALS_CHAN1LO		0x16
#define TAOS_APDS9900_ALS_CHAN1HI		0x17
#define TAOS_APDS9900_PRX_LO			0x18
#define TAOS_APDS9900_PRX_HI			0x19
#define TAOS_APDS9900_TEST_STATUS		0x1F
#define TAOS_APDS9900_REG_NUM           0x20
	
// APDS9900 cmd reg masks
#define TAOS_APDS9900_CMD_REG			0x80
#define TAOS_APDS9900_CMD_AUTO_INCREMENT 0x20
#define TAOS_APDS9900_CMD_SPL_FN		0x60
#define TAOS_APDS9900_CMD_PROX_INTCLR	0x05
#define TAOS_APDS9900_CMD_ALS_INTCLR	0x06
#define TAOS_APDS9900_CMD_ALS_AND_PS_INTCLR	0x07


// APDS9900 cntrl reg masks
#define TAOS_APDS9900_CNTL_PROX_INT_ENBL	0x20
#define TAOS_APDS9900_CNTL_ALS_INT_ENBL		0x10
#define TAOS_APDS9900_CNTL_WAIT_ENBL		0x08
#define TAOS_APDS9900_CNTL_PROX_ENBL		0x04
#define TAOS_APDS9900_CNTL_ALS_ENBL			0x02
#define TAOS_APDS9900_CNTL_PWRON			0x01

//APDS9900  status reg masks
#define TAOS_APDS9900_STATUS_ADCVALID	0x01
#define TAOS_APDS9900_STATUS_PRXVALID	0x02
#define TAOS_APDS9900_STATUS_AINT	0x20
#define TAOS_APDS9900_STATUS_PINT	0x10


//APDS9900 status masks
#define TAOS_APDS9900_STATUS_PROX_WORKING			0x01
#define TAOS_APDS9900_STATUS_ALS_WORKING			0x02
#define TAOS_APDS9900_STATUS_MASK                   0x03

#define TRUE                 1
#define FALSE                0
#define POWER_ON             1
#define POWER_DOWN           0
#define PATH_LEN                  64
#define LIGHT_SENSOR_NAME         "light"
#define PROXIMITY_SENSOR_NAME     "proximity" 
#define FTM_LIGHT_SENSOR_DELAY    0xA8

#define FTM_ALS_RATE         0x30

struct apds9900_platform_data {
	//int (*set_power_control)(int poweron);
	int int_pin;
	uint32_t int_flags;
	bool use_fir;
};

// proximity data
struct taos_prox_info {
	u16 prox_clear;
	u16 prox_data;
	int prox_event;
};

struct taos_als_info{
	u16 als_clear;
	u16 als_ir;
	u16 als_lux;
	u16 als_threshold_lo;
	u16 als_threshold_hi;
	int als_event;
};

// device configuration
struct apds9900_reg_cfg_t{
	u8 enable;             //Reg 0x00
	u8 als_adc_time;       //Reg 0x01
	u8 prox_adc_time;      //Reg 0x02
	u8 prox_wait_time;     //Reg 0x03
	u8 als_minthreshlo;    //Reg 0x04
	u8 als_minthreshhi;    //Reg 0x05
	u8 als_maxthreshlo;    //Reg 0x06
	u8 als_maxthreshhi;    //Reg 0x07
	u8 prox_minthreshlo;   //Reg 0x08
	u8 prox_minthreshhi;   //Reg 0x09
	u8 prox_maxthreshlo;   //Reg 0x0A
	u8 prox_maxthreshhi;   //Reg 0x0B
	u8 prox_intr_filter;   //Reg 0x0C
	u8 configuration;      //Reg 0x0D
	u8 prox_pulse_cnt;     //Reg 0x0E
	u8 gain_control;       //Reg 0x0F
};


#endif
