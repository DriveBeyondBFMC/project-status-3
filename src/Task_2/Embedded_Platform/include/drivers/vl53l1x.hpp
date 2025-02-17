

#ifndef VL53L1X_HPP
#define VL53L1X_HPP


/*signed integer types*/
typedef signed char s8; /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32; /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8; /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32; /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
// TODO: Add your code here

#define VL53L1X_WR_FUNC_PTR       		s8 \
    (*bus_write)(u8, u16, u8, u8)

#define VL53L1X_WR_BYTE_FUNC_PTR       	s8 \
    (*bus_write_byte)(u8, u16, u8)

#define VL53L1X_WR_WORD_FUNC_PTR		s8 \
	(*bus_write_word)(u8, u16, u16)


#define VL53L1X_RD_FUNC_PTR       		s8 \
    (*bus_read)(u8, u16, u8 *, u16)

#define VL53L1X_RD_BYTE_FUNC_PTR       	s8 \
    (*bus_read_byte)(u8, u16, u8 *)

#define VL53L1X_RD_WORD_FUNC_PTR		s8 \
	(*bus_read_word)(u8, u16, u16 *)

#define VL53L1X_RD_DB_WORD_FUNC_PTR		s8 \
	(*bus_read_double_word)(u8, u16, u32 *)



#define VL53L1X_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, cnt) \
    bus_write(dev_addr, reg_addr, reg_data, cnt) \

#define VL53L1X_BUS_WRITE_BYTE_FUNC(dev_addr, reg_addr, reg_data) \
    bus_write_byte(dev_addr, reg_addr, reg_data) \    

#define VL53L1X_BUS_WRITE_WORD_FUNC(dev_addr, reg_addr, reg_data) \
    bus_write_word(dev_addr, reg_addr, reg_data) \  



#define VL53L1X_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len) \
    bus_read(dev_addr, reg_addr, reg_data, r_len)

#define VL53L1X_BUS_READ_BYTE_FUNC(dev_addr, reg_addr, data) \
    bus_read_byte(dev_addr, reg_addr, data) \

#define VL53L1X_BUS_READ_WORD_FUNC(dev_addr, reg_addr, data) \
    bus_read_word(dev_addr, reg_addr, data) \

#define VL53L1X_BUS_READ_DOUBLE_WORD_FUNC(dev_addr, reg_addr, data) \
    bus_read_double_word(dev_addr, reg_addr, data) \




#define VL53L1X_DELAY_RETURN_TYPE void

#define VL53L1X_RETURN_FUNCTION_TYPE  s8

#define VL53L1X_DELAY_PARAM_TYPES u32

#define VL53L1X_DELAY_FUNC(delay_in_msec) \
    delay_func(delay_in_msec)


#define VL53L1X_IMPLEMENTATION_VER_MAJOR       1
#define VL53L1X_IMPLEMENTATION_VER_MINOR       0
#define VL53L1X_IMPLEMENTATION_VER_SUB         1
#define VL53L1X_IMPLEMENTATION_VER_REVISION  0000

#define SOFT_RESET											                0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS					                0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		                0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	                0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	                0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					                0x001E
#define MM_CONFIG__INNER_OFFSET_MM							                0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							                0x0022
#define GPIO_HV_MUX__CTRL									                0x0030
#define GPIO__TIO_HV_STATUS       							                0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						                0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				                0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				                0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				                0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						                0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					                0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					                0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							                0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			                0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				                0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD				                0x006C
#define SYSTEM__THRESH_HIGH 								                0x0072
#define SYSTEM__THRESH_LOW 									                0x0074
#define SD_CONFIG__WOI_SD0                  				                0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				                0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					                0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		                0x0080
#define SYSTEM__SEQUENCE_CONFIG								                0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 				                0x0082
#define SYSTEM__INTERRUPT_CLEAR       						                0x0086
#define SYSTEM__MODE_START                 					                0x0087
#define VL53L1_RESULT__RANGE_STATUS							                0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		                0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					                0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL					                0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD				                0x013E


#define VL53L1X_I2C_ADDR													0x29
#define VL53L1X_ENABLE_VALUE												0x40
#define VL53L1X_DISABLE_VALUE												0x00
#define VL53L1X_CLEAR_INTERUPT_TRIGGER_VALUE								0x01
#define VL53L1X_ONE_SHOT_RANGING_ENABLE_VALUE								0x10


#define VL53L1X_MDELAY_DATA_TYPE                                            u32
#define VL53L1X_ERROR                                                       ((s8) - 1)
#define VL53L1X_SUCCESS                             						((u8)0)
#define VL53L1X_INIT_VALUE                         							((u8)0)



const u8 VL53L1X_DEFAULT_CONFIGURATION[] = {
	0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
	0x01, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
	0x01, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
	0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
	0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
	0x00, /* 0x32 : not user-modifiable */
	0x02, /* 0x33 : not user-modifiable */
	0x08, /* 0x34 : not user-modifiable */
	0x00, /* 0x35 : not user-modifiable */
	0x08, /* 0x36 : not user-modifiable */
	0x10, /* 0x37 : not user-modifiable */
	0x01, /* 0x38 : not user-modifiable */
	0x01, /* 0x39 : not user-modifiable */
	0x00, /* 0x3a : not user-modifiable */
	0x00, /* 0x3b : not user-modifiable */
	0x00, /* 0x3c : not user-modifiable */
	0x00, /* 0x3d : not user-modifiable */
	0xff, /* 0x3e : not user-modifiable */
	0x00, /* 0x3f : not user-modifiable */
	0x0F, /* 0x40 : not user-modifiable */
	0x00, /* 0x41 : not user-modifiable */
	0x00, /* 0x42 : not user-modifiable */
	0x00, /* 0x43 : not user-modifiable */
	0x00, /* 0x44 : not user-modifiable */
	0x00, /* 0x45 : not user-modifiable */
	0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
	0x0b, /* 0x47 : not user-modifiable */
	0x00, /* 0x48 : not user-modifiable */
	0x00, /* 0x49 : not user-modifiable */
	0x02, /* 0x4a : not user-modifiable */
	0x0a, /* 0x4b : not user-modifiable */
	0x21, /* 0x4c : not user-modifiable */
	0x00, /* 0x4d : not user-modifiable */
	0x00, /* 0x4e : not user-modifiable */
	0x05, /* 0x4f : not user-modifiable */
	0x00, /* 0x50 : not user-modifiable */
	0x00, /* 0x51 : not user-modifiable */
	0x00, /* 0x52 : not user-modifiable */
	0x00, /* 0x53 : not user-modifiable */
	0xc8, /* 0x54 : not user-modifiable */
	0x00, /* 0x55 : not user-modifiable */
	0x00, /* 0x56 : not user-modifiable */
	0x38, /* 0x57 : not user-modifiable */
	0xff, /* 0x58 : not user-modifiable */
	0x01, /* 0x59 : not user-modifiable */
	0x00, /* 0x5a : not user-modifiable */
	0x08, /* 0x5b : not user-modifiable */
	0x00, /* 0x5c : not user-modifiable */
	0x00, /* 0x5d : not user-modifiable */
	0x01, /* 0x5e : not user-modifiable */
	0xdb, /* 0x5f : not user-modifiable */
	0x0f, /* 0x60 : not user-modifiable */
	0x01, /* 0x61 : not user-modifiable */
	0xf1, /* 0x62 : not user-modifiable */
	0x0d, /* 0x63 : not user-modifiable */
	0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
	0x68, /* 0x65 : Sigma threshold LSB */
	0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
	0x80, /* 0x67 : Min count Rate LSB */
	0x08, /* 0x68 : not user-modifiable */
	0xb8, /* 0x69 : not user-modifiable */
	0x00, /* 0x6a : not user-modifiable */
	0x00, /* 0x6b : not user-modifiable */
	0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
	0x00, /* 0x6d : Intermeasurement period */
	0x0f, /* 0x6e : Intermeasurement period */
	0x89, /* 0x6f : Intermeasurement period LSB */
	0x00, /* 0x70 : not user-modifiable */
	0x00, /* 0x71 : not user-modifiable */
	0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
	0x00, /* 0x73 : distance threshold high LSB */
	0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
	0x00, /* 0x75 : distance threshold low LSB */
	0x00, /* 0x76 : not user-modifiable */
	0x01, /* 0x77 : not user-modifiable */
	0x0f, /* 0x78 : not user-modifiable */
	0x0d, /* 0x79 : not user-modifiable */
	0x0e, /* 0x7a : not user-modifiable */
	0x0e, /* 0x7b : not user-modifiable */
	0x00, /* 0x7c : not user-modifiable */
	0x00, /* 0x7d : not user-modifiable */
	0x02, /* 0x7e : not user-modifiable */
	0xc7, /* 0x7f : ROI center, use SetROI() */
	0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
	0x9B, /* 0x81 : not user-modifiable */
	0x00, /* 0x82 : not user-modifiable */
	0x00, /* 0x83 : not user-modifiable */
	0x00, /* 0x84 : not user-modifiable */
	0x01, /* 0x85 : not user-modifiable */
	0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
	0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

/*!
 *  @brief vl53l1x struct
 */
struct vl53l1x_t
{
    u8 chip_id; 
    u8 dev_addr; 
	
    VL53L1X_WR_FUNC_PTR; 
	VL53L1X_WR_BYTE_FUNC_PTR; 
	VL53L1X_WR_WORD_FUNC_PTR; 

    VL53L1X_RD_FUNC_PTR; 
    VL53L1X_RD_BYTE_FUNC_PTR; 
	VL53L1X_RD_WORD_FUNC_PTR;
	VL53L1X_RD_DB_WORD_FUNC_PTR;


    void (*delay_msec)(VL53L1X_MDELAY_DATA_TYPE);
};

struct vl53l1x_version_t {
	u8      major;    /*!< major number */
	u8      minor;    /*!< minor number */
	u8      build;    /*!< build number */
	u32     revision; /*!< revision number */
};



VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_init(struct vl53l1x_t *vl53l1x);

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_distance(u16 *distance);

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_start_ranging();

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_sw_version(vl53l1x_version_t *pVersion);

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_set_I2C_address(u8 new_address);

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_set_interrupt_polarity(u8 new_polarity);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_get_interrupt_polarity(u8 *p_interrupt_polarity);

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_stop_ranging();

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_start_one_shot_ranging();

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_clear_interupt();

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_sensor_id(u16 *sensor_id);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_check_for_data_ready(u8 *is_data_ready);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_get_distance_mode(u16 *DM);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_set_inter_measurement_in_ms(u16 inter_meas_ms);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_set_timing_budget_in_ms(u16 timing_budget_in_ms);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_get_timing_budget_in_ms(u16 *p_timing_budget);

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_set_distance_mode(u16 DM);


#endif // VL53L1X_HPP
