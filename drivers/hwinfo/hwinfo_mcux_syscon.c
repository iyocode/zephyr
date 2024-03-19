/*
 * Copyright (c) 2021 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_uid

#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#if CONFIG_SOC_SERIES_IMX_RT6XX
#include <fsl_device_registers.h>
#endif
#define UID_WORD_COUNT (DT_INST_REG_SIZE(0) / sizeof(uint32_t))

struct uid {
	uint32_t id[UID_WORD_COUNT];
};

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	volatile const uint32_t * const uid_addr = (uint32_t *) DT_INST_REG_ADDR(0);
	struct uid dev_id;

	if (buffer == NULL) {
		return 0;
	}

	for (size_t i = 0 ; i < UID_WORD_COUNT ; i++) {
		dev_id.id[i] = sys_cpu_to_be32(uid_addr[i]);
	}

	if (length > sizeof(dev_id.id)) {
		length = sizeof(dev_id.id);
	}

	memcpy(buffer, dev_id.id, length);

	return length;
}

#if CONFIG_SOC_SERIES_IMX_RT6XX
int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t flags = 0;
	if(RSTCTL0->SYSRSTSTAT & RSTCTL0_SYSRSTSTAT_VDD_POR_MASK)
	{
		flags |= RESET_POR;
	}
	if(RSTCTL0->SYSRSTSTAT & RSTCTL0_SYSRSTSTAT_PAD_RESET_MASK)
	{
		flags |= RESET_PIN;
	}
	if(RSTCTL0->SYSRSTSTAT & RSTCTL0_SYSRSTSTAT_ARM_APD_RESET_MASK)
	{
		flags |= RESET_SOFTWARE;
	}
	if(RSTCTL0->SYSRSTSTAT & RSTCTL0_SYSRSTSTAT_WDT0_RESET_MASK)
	{
		flags |= RESET_WATCHDOG;
	}
	if(RSTCTL0->SYSRSTSTAT & RSTCTL0_SYSRSTSTAT_WDT1_RESET_MASK)
	{
		flags |= RESET_WATCHDOG;
	}
	*cause = flags;
	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	RSTCTL0->SYSRSTSTAT = 0;
	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = RESET_PIN | RESET_SOFTWARE | RESET_POR | RESET_WATCHDOG;
	return 0;
}
#endif