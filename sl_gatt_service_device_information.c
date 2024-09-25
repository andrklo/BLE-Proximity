/***************************************************************************//**
 * @file
 * @brief Device Information GATT Service
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include "sl_status.h"
#include "sl_gsdk_version.h"
#include "gatt_db.h"
#include "app_assert.h"
#include "sl_gatt_service_device_information.h"

#define GATTDB_MODEL_NUMBER_STRING_LEN          8
#define GATTDB_HARDWARE_REVISION_STRING_LEN     3
#define GATTDB_FIRMWARE_REVISION_STRING_LEN     5
#define GATTDB_SYSTEM_ID_LEN                    8

/**************************************************************************//**
 * Bluetooth stack event handler.
 *****************************************************************************/
void sl_gatt_service_device_information_on_event(sl_bt_msg_t *evt)
{
  (void)evt;
}
