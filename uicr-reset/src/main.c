/*
 * Nordic User Information Configuration Registers (UICR) Reset Example
 * Demonstrates disabling the reset pin on the nRF52832.
 * 
 * Copyright (c) 2023 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>


void disable_reset_pin()
{
	printk("Disabling reset pin. ");

	// When UICR memory is erased, it is set to 1. When programmed, 
	// bits can be set to 0, but a 0 cannot be reset to a 1 without
	// an erase operation. To disable the reset pin, we need to set
	// the connect bit (Bit 31) to zero, hence we don't need to 
	// perform an erase operation.

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	NRF_UICR->PSELRESET[1] = 0;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
	NRF_UICR->PSELRESET[0] = 0;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	printk("Done. Rebooting. \n");
	k_sleep(K_MSEC(200));

	// System reset is needed to update UICR registers.
	NVIC_SystemReset();
}

void remap_reset_pin()
{
	printk("Remapping reset pin to P0.06. ");

	// When UICR memory is erased, it is set to 1. When programmed, 
	// bits can be set to 0, but a 0 cannot be reset to a 1 without
	// an erase operation. To disable the reset pin, we need to set
	// the connect bit (Bit 31) to zero, hence we don't need to 
	// perform an erase operation.

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	NRF_NVMC->ERASEUICR = 0x01;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	NRF_UICR->PSELRESET[1] = 6;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
	NRF_UICR->PSELRESET[0] = 6;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

	printk("Done. Rebooting. \n");
	k_sleep(K_MSEC(200));

	// System reset is needed to update UICR registers.
	NVIC_SystemReset();
}


void main(void)
{
	printk("Nordic User Information Configuration Registers (UICR) Reset Example\nBoard: %s\n", CONFIG_BOARD);

#if CONFIG_BOARD_LEMON_IOT_BLE_NRF52832

	printk("PSELRESET[0] = %d\n",NRF_UICR->PSELRESET[0]);
	printk("PSELRESET[1] = %d\n",NRF_UICR->PSELRESET[1]);

	// Only perform if needed (which is the first time this code runs)
	//if ((NRF_UICR->PSELRESET[0] != 0) || (NRF_UICR->PSELRESET[1] != 0))	disable_reset_pin();
	if ((NRF_UICR->PSELRESET[0] != 6) || (NRF_UICR->PSELRESET[1] != 6)) remap_reset_pin();

#endif

	while (1) {
		printk("*");
		k_sleep(K_MSEC(1000));
	}

}
