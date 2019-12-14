#include <stdint.h>

#include <inc/hw_nvic.h>
#include <inc/hw_types.h>

#include <mcu.h>

void MCU_SoftwareReset(void)
{
	/* Access directly the registers */
	HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}
