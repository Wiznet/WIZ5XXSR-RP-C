/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __W5X00_DHCP_CB_H__
#define __W5X00_DHCP_CB_H__

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdint.h>
#include "dhcp.h"
#include "ConfigData.h"


void w5x00_dhcp_assign(void);
void w5x00_dhcp_conflict(void);

#endif
