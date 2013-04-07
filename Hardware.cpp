#include "Hardware.h"
#include "core_pins.h"



void pwmWrite(int pin, float value)
{
	uint32_t cval, val;

	// Constrain input to be between 1.0f and 0.0f
    value = value > 1.0f ? 1.0f : (value < 0.0f ? 0.0f : value);
    
    val = value * 65535;

	if (pin == 3 || pin == 4)
    {
		cval = ((uint32_t)val * (uint32_t)(FTM1_MOD + 1)) >> 16;
	}
    else
    {
		cval = ((uint32_t)val * (uint32_t)(FTM0_MOD + 1)) >> 16;
	}

	switch (pin)
    {
	case 3: // PTA12, FTM1_CH0
		FTM1_C0V = cval;
		CORE_PIN3_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 4: // PTA13, FTM1_CH1
		FTM1_C1V = cval;
		CORE_PIN4_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 5: // PTD7, FTM0_CH7
		FTM0_C7V = cval;
		CORE_PIN5_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 6: // PTD4, FTM0_CH4
		FTM0_C4V = cval;
		CORE_PIN6_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 9: // PTC3, FTM0_CH2
		FTM0_C2V = cval;
		CORE_PIN9_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 10: // PTC4, FTM0_CH3
		FTM0_C3V = cval;
		CORE_PIN10_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 20: // PTD5, FTM0_CH5
		FTM0_C5V = cval;
		CORE_PIN20_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 21: // PTD6, FTM0_CH6
		FTM0_C6V = cval;
		CORE_PIN21_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 22: // PTC1, FTM0_CH0
		FTM0_C0V = cval;
		CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	case 23: // PTC2, FTM0_CH1
		FTM0_C1V = cval;
		CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
		break;
	default:
		// Do nothing
        break;
	}
}



