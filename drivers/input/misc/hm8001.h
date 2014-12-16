#ifndef _HM8001_H_
#define _HM8001_H_

// register offsets
#define HM8001_DEVICE_ID			0x00
#define HM8001_OPCON				0x01
#define HM8001_INT_FLAG				0x02
#define HM8001_ALS_WAIT_TIME		0x03
#define HM8001_PS_WAIT_TIME			0x04
#define HM8001_ALS_ANA_WAKEUP_TIME	0x05
#define HM8001_PS_ANA_WAKEUP_TIME	0x06
#define HM8001_ALS_CON				0x08
#define HM8001_ALS_STATUS			0x09
#define HM8001_ALS_CNT_H			0x0A
#define HM8001_ALS_CNT_M			0x0B
#define HM8001_ALS_CNT_L			0x0C
#define HM8001_ALS_IHTH				0x0D
#define HM8001_ALS_IHTL				0x0E
#define HM8001_ALS_ILTH				0x0F
#define HM8001_ALS_ILTL				0x10
#define HM8001_ALS_INTG				0x11
#define HM8001_ALS_STEPH			0x12
#define HM8001_ALS_STEPL			0x13
#define HM8001_ALS_ANA_CON1			0x14
#define HM8001_ALS_ANA_CON2			0x15
#define HM8001_PS_CON				0x19
#define HM8001_PS_BIT_CHECK			0x1A
#define HM8001_PS_CNT_STEP			0x1B
#define HM8001_PS_MTIME				0x1C
#define HM8001_PS_CYC_THR	 		0x1D
#define HM8001_PS_LED_DET			0x1E
#define HM8001_PS_LED_REL			0x1F
#define HM8001_PS_LED_PAT			0x20
#define HM8001_PS_DET				0x21
#define HM8001_PS_ANA_CON1			0x22
#define HM8001_PS_ANA_CON2			0x23
#define HM8001_PS_ANA_CON3			0x24
#define HM8001_REGISTER_NUM			0x25


// HM8001_OPCON
#define ASDS_MODE_ENABLE			0x40
#define POWER_DOWN_MODE				0x30
#define INT_OUTPUT_ENABLE			0x10
#define ALS_INT_MODE				0x08
#define PS_INT_MODE					0x04
#define ALS_CONTROL_ENABLE			0x02
#define PS_CONTROL_ENABLE			0x01

#define ALS_ACTIVE_MODE				0x00

// INT_FLAG
#define HM8001_ALS_INT				0x02
#define HM8001_PS_INT				0x01
#define HM8001_ALSPS_INT			0x03

#define MAX_ALS_IHTH				0xff
#define MAX_ALS_IHTL				0xff

#define MIN_ALS_ILTH				0
#define MIN_ALS_ILTL				0

#define ALS_IHTH_DEFAULT			0x01
#define ALS_IHTL_DEFAULT			0x2C
#define ALS_ILTH_DEFAULT			0x00
#define ALS_ILTL_DEFAULT			0x32

#define PS_LED_ALWAYS_HIGH       	0x20
#define PS_INT_INVERT_ENABLE     	0x08
#define PS_POLLING_BIT_HIGH      	0x04


// ALS_WAIT_TIME
#define ALS_WAIT_TIME_10MS					0x01	// wait time = ALS_WAIT_TIME[7:0]*10ms
#define ALS_WAIT_TIME_50MS					0x05	// wait time = ALS_WAIT_TIME[7:0]*10ms

// PS_WAIT_TIME
#define PS_WAIT_TIME_80MS					0x08	// wait time = PS_WAIT_TIME[7:0]*10ms

// ALS_ANA_WAKEUP_TIME
#define ALS_ANA_WAKEUP_TIME_20US			0x0e	// wait time = (ALS_ANA_WAKEUP_TIME[4:0]+1)*1.33us 

// PS_ANA_WAKEUP_TIME
#define DEFAULT_PS_ANA_WAKEUP_TIME_100US	0x4a	// wait time = (PS_ANA_WAKEUP_TIME[6:0]+1)*1.33us

#define ALS_INT_HOLD_ENABLE			0x40
#define ALS_INT_MULTI_FRAME_ENABLE	0x20
#define ALS_WEIGHTED_AVR_ENABLE		0x10
#define ALS_INT_FRAME_2				0x00
#define ALS_INT_FRAME_4				0x04
#define ALS_INT_FRAME_8				0x08
#define ALS_INT_FRAME_16			0x0C
#define ALS_WEIGHTED_AVR_1D2		0x00
#define ALS_WEIGHTED_AVR_1D4		0x01
#define ALS_WEIGHTED_AVR_1D8		0x02
#define ALS_WEIGHTED_AVR_1D16		0x03

// ALS_STATUS Read-only
#define ALS_INT_FLAG			0x80
#define ALS_COUNT_OV_FLAG		0x40
#define ALS_PRE_POS				0x03	// [0:1]

#define LOW_THAN_LOW						0x00
#define UPPER_THAN_LOW_AND_LOW_THAN_HIGH	0x01
#define UPPER_THAN_HIGH						0x02

// ALS_INTG
#define DEFAULT_ALS_INTG		0x01	// Int. Time = ALS_INTG[7:0]*({ALS_STEPH[7:0],ALS_STEPL[7:0]}+1)

// ALS_STEP
#define DEFAULT_ALS_STEPH		0x05
#define DEFAULT_ALS_STEPL		0xdf 	// for Int. Time (ALS_INTG)

// PS_CON
#define LED_ON					0x20
#define INT_PS_OUT_INV			0x08
#define POLL_HIGH				0x04

// PS_BITE_CHECK
#define BIT_CHECK_OBJ			0x22

// PS_CNT_STEP
#define DEFAULT_PS_CNT_STEP		0x02

// PS_MTIME
#define DEFAULT_PS_MTINE		0x08	// [7:0] : Margin time (1 step = margin counter step * 1.333us)

// PS_CYC_THR
#define CYC_THR_COUNT			0x11

// PS_LED_DET  set the led current and the plus width when obj detetion
#define PS_DET_ILED25MA			0x00
#define PS_DET_ILED50MA			0x10
#define PS_DET_ILED75MA			0x20
//#define PS_DET_ILED100MA		0x30
#define PS_DET_ILED125MA		0x40
#define PS_DET_ILED150MA		0x50
#define PS_DET_ILED175MA		0x60

#define PS_DET_ILED20MA			(0x80 | 0x00)
#define PS_DET_ILED40MA			(0x80 | 0x10)
#define PS_DET_ILED60MA			(0x80 | 0x20)
#define PS_DET_ILED80MA		    (0x80 | 0x30)
#define PS_DET_ILED100MA		(0x80 | 0x40)
#define PS_DET_ILED120MA		(0x80 | 0x50)
#define PS_DET_ILED140MA		(0x80 | 0x60)
#define PS_DET_ILED160MA		(0x80 | 0x70)

#define PS_DET_ILED200MA		0x70
#define PS_DET_WIDTH3US999		0x00
#define PS_DET_WIDTH5US332		0x01
#define PS_DET_WIDTH6US665		0x02
#define PS_DET_WIDTH7US998		0x03
#define PS_DET_WIDTH9US331		0x04
#define PS_DET_WIDTH10US664		0x05
#define PS_DET_WIDTH11US997		0x06
#define PS_DET_WIDTH13US330		0x07
#define PS_DET_WIDTH14US663		0x08
#define PS_DET_WIDTH15US996		0x09
#define PS_DET_WIDTH17US329		0x0a
#define PS_DET_WIDTH18US662		0x0b
#define PS_DET_WIDTH19US995		0x0c
#define PS_DET_WIDTH21US328		0x0d
#define PS_DET_WIDTH22US661		0x0e
#define PS_DET_WIDTH23US994		0x0f


// PS_LED_REL  set the led current and the plus width when obj release
#define PS_REL_ILED25MA			0x00
#define PS_REL_ILED50MA			0x10
#define PS_REL_ILED75MA			0x20
//#define PS_REL_ILED100MA		0x30
#define PS_REL_ILED125MA		0x40
#define PS_REL_ILED150MA		0x50
#define PS_REL_ILED175MA		0x60
#define PS_REL_ILED200MA		0x70

#define PS_REL_ILED20MA			(0x80 | 0x00)
#define PS_REL_ILED40MA			(0x80 | 0x10)
#define PS_REL_ILED60MA			(0x80 | 0x20)
#define PS_REL_ILED80MA		    (0x80 | 0x30)
#define PS_REL_ILED100MA		(0x80 | 0x40)
#define PS_REL_ILED120MA		(0x80 | 0x50)
#define PS_REL_ILED140MA		(0x80 | 0x60)
#define PS_REL_ILED160MA		(0x80 | 0x70)

#define PS_REL_WIDTH3US999		0x00
#define PS_REL_WIDTH5US332		0x01
#define PS_REL_WIDTH6US665		0x02
#define PS_REL_WIDTH7US998		0x03
#define PS_REL_WIDTH9US331		0x04
#define PS_REL_WIDTH10US664		0x05
#define PS_REL_WIDTH11US997		0x06
#define PS_REL_WIDTH13US330		0x07
#define PS_REL_WIDTH14US663		0x08
#define PS_REL_WIDTH15US996		0x09
#define PS_REL_WIDTH17US329		0x0a
#define PS_REL_WIDTH18US662		0x0b
#define PS_REL_WIDTH19US995		0x0c
#define PS_REL_WIDTH21US328		0x0d
#define PS_REL_WIDTH22US661		0x0e
#define PS_REL_WIDTH23US994		0x0f


// PS_LED_PAT
#define DEFUALT_PS_LED_PAT 0xc1

// PS_ANA_CON1
#define DET_ANA57MV		0x00
#define DET_ANA59MV		0x10
#define DET_ANA61MV		0x20
#define DET_ANA63MV		0x30
#define DET_ANA64MV		0x40
#define DET_ANA66MV		0x50
#define DET_ANA67MV		0x60
#define DET_ANA69MV		0x70
#define DET_ANA71MV		0x80
#define DET_ANA73MV		0x90
#define DET_ANA75MV		0xa0
#define DET_ANA77MV		0xb0
#define DET_ANA79MV		0xc0
#define DET_ANA81MV		0xd0
#define DET_ANA83MV		0xe0

#define DET_ANA12MV		0x00
#define DET_ANA15MV		0x10
#define DET_ANA18MV		0x20
#define DET_ANA21MV		0x30
#define DET_ANA24MV		0x40
#define DET_ANA27MV		0x50
#define DET_ANA30MV		0x60
#define DET_ANA33MV		0x70
#define DET_ANA36MV		0x80
#define DET_ANA39MV		0x90
#define DET_ANA42MV		0xa0
#define DET_ANA45MV		0xb0
#define DET_ANA48MV		0xc0
#define DET_ANA51MV		0xd0
#define DET_ANA54MV		0xe0


#define REL_ANA57MV		0x00
#define REL_ANA59MV		0x01
#define REL_ANA61MV		0x02
#define REL_ANA63MV		0x03
#define REL_ANA64MV		0x04
#define REL_ANA66MV		0x05
#define REL_ANA67MV		0x06
#define REL_ANA69MV		0x07
#define REL_ANA71MV		0x08
#define REL_ANA73MV		0x09
#define REL_ANA75MV		0x0a
#define REL_ANA77MV		0x0b
#define REL_ANA79MV		0x0c
#define REL_ANA81MV		0x0d
#define REL_ANA83MV		0x0e

#define REL_ANA12MV		0x00
#define REL_ANA15MV		0x10
#define REL_ANA18MV		0x20
#define REL_ANA21MV		0x30
#define REL_ANA24MV		0x40
#define REL_ANA27MV		0x50
#define REL_ANA30MV		0x60
#define REL_ANA33MV		0x70
#define REL_ANA36MV		0x80
#define REL_ANA39MV		0x90
#define REL_ANA42MV		0xa0
#define REL_ANA45MV		0xb0
#define REL_ANA48MV		0xc0
#define REL_ANA51MV		0xd0
#define REL_ANA54MV		0xe0



// ALS_ANA_CON1
#define DEFUALT_ALS_ANA_CON1					0x27			
#define ALS_ANA_VDD_PUL_RET_PLUS_WIDTH250NS		0x20// Reset pulse width control,default 250ns (0b10)

#define ALS_ANA_VHI1V62			0x0c				// High comparison voltage selection for Amp output
#define ALS_ANA_VHI1V39			0x08
#define ALS_ANA_VHI1V16			0x04
#define ALS_ANA_VHI1V04			0x00 
			
#define ALS_ANA_VLO0V93			0x03				// Low comparison voltage selection for Amp output
#define ALS_ANA_VLO0V81			0x02
#define ALS_ANA_VLO0V70			0x01
#define ALS_ANA_VLO0V58			0x00

#define DEFUALT_ALS_ANA_CON2	0x02
#define ALS_ANA_REF_DATA_EN		0x20
#define ALS_ANA_FUSE_DIS		0x10
#define ALS_ANA_SUBTRC_DIS		0x08				// Pulse subtraction disable used in test mode
			
#define ALS_ANA_CAP3P5			0x07				// Capacitance selection (code x 160fF)
#define ALS_ANA_CAP3P			0x06
#define ALS_ANA_CAP2P5			0x05
#define ALS_ANA_CAP2P			0x04
#define ALS_ANA_CAP1P5			0x03
#define ALS_ANA_CAP1P			0x02
#define ALS_ANA_CAP0P5			0x01
#define ALS_ANA_CAP0P			0x00


// PS_ANA_CON2
#define DEFUALT_PS_ANA_CON2		0x65
#define PS_REF_DATA_READ_EN	0x80
#define PS_NOISE_CANCEL		0x40

#define PS_CONV_GAIN2M			0x00
#define PS_CONV_GAIN800K		0x10
#define PS_CONV_GAIN600K		0x20
#define PS_CONV_GAIN400K		0x30
#define PS_COMMON_VOL			0x05

// PS_ANA_CON3
#define DEFUALT_PS_ANA_CON3			0xa4
#define PS_ANA_FUSE_ENABLE			0x80

#define PS_R3SEL_BANDWIDTH500K		0x00
#define PS_R3SEL_BANDWIDTH200K		0x20
#define PS_R3SEL_BANDWIDTH150K		0x40
#define PS_R3SEL_BANDWIDTH100K		0x60
#define PS_ANA_CAP1_SEL				0x10
#define PS_ANA_CAP2_SEL				0x08
#define PS_ANA_CAP3_SEL				0x04
#define PS_DET_THR_VOL_RANGE_SEL	0x02
#define PS_REL_THR_VOL_RANGE_SEL	0x01

/* Power On response time in ms */
#define PON_DELAY		300
#define WAKEUP_DELAY	10


#define POWER_SLEEP_MODE		0x00


#define ALS						0
#define PS						1
#define PROX_DETECTION_CLOSE	3
#define PROX_DETECTION_FAR		8
#define LUX_DEFAULT				200

#define PS_NEWDATA				0x01 
#define ALS_NEWDATA				0x04
#define TRUE					1
#define FALSE					0
#define POWER_ON				1
#define POWER_DOWN				0
#define DEFAULT_DELAY			100
#define FTM_LIGHT_SENSOR_DELAY	0xa8

struct hm8001alsps_platform_data {
	void (*init_irq)(void);
	int (*irq_query)(void);
	//int (*power_control)(int poweron);
};

struct hm8001_platform_data {
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t alsctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t wait_reg;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	int int_pin;
	uint32_t transmittance;
	uint32_t int_flags;
	bool use_fir;
};



#endif

