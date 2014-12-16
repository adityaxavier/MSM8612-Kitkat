#ifndef _TMD2771X_H_
#define _TMD2771X_H_

/*ioctl cmd*/
#define TMD2771X_IOC_MAGIC 'C'

#define TMD2771X_IOCS_PRO_MODE 			_IOWR(TMD2771X_IOC_MAGIC,0, unsigned char)
#define TMD2771X_IOCG_PRO_MODE 			_IOWR(TMD2771X_IOC_MAGIC,1, unsigned char)
#define TMD2771X_IOCG_PRO_DATA 			_IOWR(TMD2771X_IOC_MAGIC,2, unsigned char)
#define TMD2771X_IOCG_PRO_RAW 			_IOWR(TMD2771X_IOC_MAGIC,3, unsigned char)
#define TMD2771X_IOCS_LIG_MODE 			_IOWR(TMD2771X_IOC_MAGIC,4, unsigned char)
#define TMD2771X_IOCG_LIG_MODE 			_IOWR(TMD2771X_IOC_MAGIC,5, unsigned char)
#define TMD2771X_IOCG_LIG_DATA 			_IOWR(TMD2771X_IOC_MAGIC,6, unsigned char)
#define TMD2771X_IOCG_LIG_RAW 			_IOWR(TMD2771X_IOC_MAGIC,7, unsigned char)

#define TMD2771X_IOC_MAXNR	8	


// TMD2771X register offsets
#define TAOS_TMD2771X_CNTRL 			0x00
#define TAOS_TMD2771X_ALS_TIME 			0x01
#define TAOS_TMD2771X_PRX_TIME			0x02
#define TAOS_TMD2771X_WAIT_TIME			0x03
#define TAOS_TMD2771X_ALS_MINTHRESHLO	0x04
#define TAOS_TMD2771X_ALS_MINTHRESHHI 	0x05
#define TAOS_TMD2771X_ALS_MAXTHRESHLO	0x06
#define TAOS_TMD2771X_ALS_MAXTHRESHHI	0x07
#define TAOS_TMD2771X_PRX_MINTHRESHLO 	0x08
#define TAOS_TMD2771X_PRX_MINTHRESHHI 	0x09
#define TAOS_TMD2771X_PRX_MAXTHRESHLO 	0x0A
#define TAOS_TMD2771X_PRX_MAXTHRESHHI 	0x0B
#define TAOS_TMD2771X_INTERRUPT			0x0C
#define TAOS_TMD2771X_CONFIGURATION	    0x0D
#define TAOS_TMD2771X_PRX_PLUSE_COUNT   0x0E
#define TAOS_TMD2771X_GAIN_CONTROL		0x0F
#define TAOS_TMD2771X_REVID				0x11
#define TAOS_TMD2771X_CHIPID      		0x12
#define TAOS_TMD2771X_STATUS			0x13
#define TAOS_TMD2771X_ALS_CHAN0LO		0x14
#define TAOS_TMD2771X_ALS_CHAN0HI		0x15
#define TAOS_TMD2771X_ALS_CHAN1LO		0x16
#define TAOS_TMD2771X_ALS_CHAN1HI		0x17
#define TAOS_TMD2771X_PRX_LO			0x18
#define TAOS_TMD2771X_PRX_HI			0x19
#define TAOS_TMD2771X_TEST_STATUS		0x1F
#define TAOS_TMD2771X_REG_NUM           0x20
	
// TMD2771X cmd reg masks
#define TAOS_TMD2771X_CMD_REG			0x80
#define TAOS_TMD2771X_CMD_AUTO_INCREMENT 0x20
#define TAOS_TMD2771X_CMD_SPL_FN		0x60
#define TAOS_TMD2771X_CMD_PROX_INTCLR	0x05
#define TAOS_TMD2771X_CMD_ALS_INTCLR	0x06
#define TAOS_TMD2771X_CMD_ALS_AND_PS_INTCLR	0x07


// TMD2771X cntrl reg masks
#define TAOS_TMD2771X_CNTL_PROX_INT_ENBL	0x20
#define TAOS_TMD2771X_CNTL_ALS_INT_ENBL		0x10
#define TAOS_TMD2771X_CNTL_WAIT_ENBL		0x08
#define TAOS_TMD2771X_CNTL_PROX_ENBL		0x04
#define TAOS_TMD2771X_CNTL_ALS_ENBL			0x02
#define TAOS_TMD2771X_CNTL_PWRON			0x01

//TMD2771X  status reg masks
#define TAOS_TMD2771X_STATUS_ADCVALID	0x01
#define TAOS_TMD2771X_STATUS_PRXVALID	0x02
#define TAOS_TMD2771X_STATUS_AINT	0x10
#define TAOS_TMD2771X_STATUS_PINT	0x20


//TMD2771X status masks
#define TAOS_TMD2771X_STATUS_PROX_WORKING			0x01
#define TAOS_TMD2771X_STATUS_ALS_WORKING			0x02
#define TAOS_TMD2771X_STATUS_MASK                   0x03

#define TRUE                 1
#define FALSE                0
#define POWER_ON             1
#define POWER_DOWN           0
#define PATH_LEN                  64
#define LIGHT_SENSOR_NAME         "light"
#define PROXIMITY_SENSOR_NAME     "proximity" 
#define FTM_LIGHT_SENSOR_DELAY    0xA8

#define FTM_ALS_RATE         0x33

struct tmd2771x_platform_data {
	//int (*set_power_control)(int poweron);
	//int	(*init_irq)(void);
	//int (*irq_query)(void);
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
struct tmd2771x_reg_cfg_t{
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
