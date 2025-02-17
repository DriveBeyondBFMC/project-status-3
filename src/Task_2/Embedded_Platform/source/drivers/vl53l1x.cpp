#include "drivers/vl53l1x.hpp"
#include <stdint.h>
#include <stddef.h>

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif


static struct vl53l1x_t *p_vl53l1x;
static struct vl53l1x_version_t *p_version;

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_init(struct vl53l1x_t *vl53l1x) {
    p_vl53l1x = vl53l1x;

    VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR; 

	uint8_t Addr = 0x00, dataReady = 0, timeout = 0;

	for (Addr = 0x2D; Addr <= 0x87; Addr++)
	{
		com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, Addr, VL53L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
	}
	com_rslt = vl53l1x_start_ranging();

	//We need to wait at least the default intermeasurement period of 103ms before dataready will occur
	//But if a unit has already been powered and polling, it may happen much faster
	// while (dataReady == 0)
	// {
	// 	com_rslt = VL53L1X_CheckForDataReady(&dataReady);
	// 	if (timeout++ > 150)
	// 		return -1;
	// 	p_vl53l1x->delay_msec(10);
	// }
	com_rslt = vl53l1x_clear_interupt();
	com_rslt = vl53l1x_stop_ranging();
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, 0x0B, 0);											/* start VHV from the previous temperature */
	return com_rslt;
}


// VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_check_for_data_ready(uint8_t *isDataReady)
// {
// 	uint8_t Temp;
// 	uint8_t IntPol;
// 	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

// 	com_rslt = VL53L1X_get_interrupt_polarity(&IntPol);
// 	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, GPIO__TIO_HV_STATUS, &Temp);
// 	/* Read in the register to check if a new value is available */
// 	if (com_rslt == 0)
// 	{
// 		if ((Temp & 1) == IntPol)
// 			*isDataReady = 1;
// 		else
// 			*isDataReady = 0;
// 	}
// 	return status;
// }

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_distance(u16 *distance) {
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR; 
	u16 tmp;
	com_rslt = (p_vl53l1x->VL53L1X_BUS_READ_WORD_FUNC(p_vl53l1x->dev_addr,
							VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
	*distance = tmp;
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_start_ranging()
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, SYSTEM__INTERRUPT_CLEAR, 0x01); /* clear interrupt trigger */
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, SYSTEM__MODE_START, 0x40); /* Enable VL53L1X */
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_sw_version(vl53l1x_version_t *pVersion)
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	p_version->major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
	p_version->minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
	p_version->build = VL53L1X_IMPLEMENTATION_VER_SUB;
	p_version->revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_set_I2C_address(u8 new_address)
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
	p_vl53l1x->dev_addr = new_address;

	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_set_interrupt_polarity(u8 new_polarity)
{
	u8 Temp;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_FUNC(p_vl53l1x->dev_addr, GPIO_HV_MUX__CTRL, &Temp, 1);
	Temp = Temp & 0xEF;
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, GPIO_HV_MUX__CTRL, Temp | (!(new_polarity & 1)) << 4);
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_interrupt_polarity(u8 *p_interrupt_polarity)
{
	u8 Temp;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0x10;
	*p_interrupt_polarity = !(Temp >> 4);
	return com_rslt;
} 


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_stop_ranging()
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, SYSTEM__MODE_START, VL53L1X_DISABLE_VALUE); 
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_start_one_shot_ranging()
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, SYSTEM__INTERRUPT_CLEAR, VL53L1X_CLEAR_INTERUPT_TRIGGER_VALUE); 
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, SYSTEM__MODE_START, VL53L1X_ONE_SHOT_RANGING_ENABLE_VALUE); 
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_clear_interupt()
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, SYSTEM__INTERRUPT_CLEAR, VL53L1X_CLEAR_INTERUPT_TRIGGER_VALUE);
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_sensor_id(u16 *sensor_id)
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	u16 tmp = 0;
	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_WORD_FUNC(p_vl53l1x->dev_addr, VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
	*sensor_id = tmp;
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_check_for_data_ready(u8 *is_data_ready)
{
	u8 temp;
	u8 int_pol;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = VL53L1X_get_interrupt_polarity(&int_pol);
	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, GPIO__TIO_HV_STATUS, &temp);
	/* Read in the register to check if a new value is available */
	if (com_rslt == 0)
	{
		if ((temp & 1) == temp)
			*is_data_ready = 1;
		else
			*is_data_ready = 0;
	}
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_distance_mode(u16 *DM)
{
	u8 TempDM;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
	if (TempDM == 0x14)
		*DM = 1;
	if (TempDM == 0x0A)
		*DM = 2;
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_set_inter_measurement_in_ms(u16 inter_meas_ms)
{
	u16 ClockPLL;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_WORD_FUNC(p_vl53l1x->dev_addr, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
	ClockPLL = ClockPLL & 0x3FF;
	p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
				   (uint32_t)(ClockPLL * inter_meas_ms * 1.075));
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE vl513l1x_set_timing_budget_in_ms(u16 timing_budget_in_ms)
{
	u16 DM;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	com_rslt = VL53L1X_get_distance_mode(&DM);
	if (DM == 0)
		return 1;
	else if (DM == 1) { /* Short DistanceMode */
		switch (timing_budget_in_ms) {
		case 15: /* only available in short distance mode */
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
			break;
		case 20:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
			break;
		case 33:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
			break;
		case 50:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
			break;
		case 100:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
			break;
		case 200:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
			break;
		case 500:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
			break;
		default:
			com_rslt = 1;
			break;
		}
	}
	else {
		switch (timing_budget_in_ms) {
		case 20:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
			break;
		case 33:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
			break;
		case 50:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
			break;
		case 100:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
			break;
		case 200:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
			break;
		case 500:
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
			p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
			break;
		default:
			com_rslt = 1;
			break;
		}
	}
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE VL53L1X_get_timing_budget_in_ms(u16 *p_timing_budget)
{
	uint16_t Temp;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_WORD_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
	switch (Temp)
	{
	case 0x001D:
		*p_timing_budget = 15;
		break;
	case 0x0051:
	case 0x001E:
		*p_timing_budget = 20;
		break;
	case 0x00D6:
	case 0x0060:
		*p_timing_budget = 33;
		break;
	case 0x1AE:
	case 0x00AD:
		*p_timing_budget = 50;
		break;
	case 0x02E1:
	case 0x01CC:
		*p_timing_budget = 100;
		break;
	case 0x03E1:
	case 0x02D9:
		*p_timing_budget = 200;
		break;
	case 0x0591:
	case 0x048F:
		*p_timing_budget = 500;
		break;
	default:
		*p_timing_budget = 0;
		break;
	}
	return com_rslt;
}



VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_set_distance_mode(u16 DM)
{
	u16 TB, word_value;
	u8 byte_value;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	com_rslt = VL53L1X_get_timing_budget_in_ms(&TB);
	switch (DM)
	{
	case 1:
		byte_value = 0x14;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, PHASECAL_CONFIG__TIMEOUT_MACROP, &byte_value);
		byte_value = 0x07;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__VCSEL_PERIOD_A, &byte_value);
		byte_value = 0x05;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__VCSEL_PERIOD_B, &byte_value);
		byte_value = 0x38;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__VALID_PHASE_HIGH, &byte_value);
		word_value = 0x0705;
		com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, SD_CONFIG__WOI_SD0, word_value);
		word_value = 0x0606;
		com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, SD_CONFIG__INITIAL_PHASE_SD0, word_value);
		break;
	case 2:
		byte_value = 0x0A;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, PHASECAL_CONFIG__TIMEOUT_MACROP, &byte_value);
		byte_value = 0x0F;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__VCSEL_PERIOD_A, &byte_value);
		byte_value = 0x0D;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__VCSEL_PERIOD_B, &byte_value);
		byte_value = 0xB8;
		com_rslt = p_vl53l1x->VL53L1X_BUS_READ_BYTE_FUNC(p_vl53l1x->dev_addr, RANGE_CONFIG__VALID_PHASE_HIGH, &byte_value);
		word_value = 0x0F0D;
		com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, SD_CONFIG__WOI_SD0, word_value);
		word_value = 0x0E0E;
		com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_WORD_FUNC(p_vl53l1x->dev_addr, SD_CONFIG__INITIAL_PHASE_SD0, word_value);
		break;
	default:
		break;
	}
	com_rslt = VL53L1X_set_timing_budget_in_ms(TB);
	return com_rslt;
}

VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_get_inter_measurement_in_ms(u16 *pIM)
{
	u16 ClockPLL;
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;

	uint32_t tmp;

	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_DOUBLE_WORD_FUNC(p_vl53l1x->dev_addr, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
	*pIM = (u16)tmp;
	com_rslt = p_vl53l1x->VL53L1X_BUS_READ_WORD_FUNC(p_vl53l1x->dev_addr, VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
	ClockPLL = ClockPLL & 0x3FF;
	*pIM = (u16)(*pIM / (ClockPLL * 1.065));
	return com_rslt;
}


VL53L1X_RETURN_FUNCTION_TYPE vl53l1x_boot_state(u8 *state)
{
	VL53L1X_RETURN_FUNCTION_TYPE com_rslt = VL53L1X_ERROR;
	u8 tmp = 0;

	com_rslt = p_vl53l1x->VL53L1X_BUS_WRITE_BYTE_FUNC(p_vl53l1x->dev_addr, VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
	*state = tmp;
	return com_rslt;
}
