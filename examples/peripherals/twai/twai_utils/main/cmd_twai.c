/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cmd_twai.h"
#include "esp_log.h"
#include "esp_console.h"

static const char *TAG = "cmd_twai";

/* =============================================================================
 * GLOBAL STATE DEFINITION
 * =============================================================================*/

/* =============================================================================
 * COMMAND REGISTRATION
 * =============================================================================*/

void register_twai_commands(void)
{
    ESP_LOGI(TAG, "TWAI commands registered successfully");
}

void unregister_twai_commands(void)
{
    ESP_LOGI(TAG, "TWAI commands unregistered successfully");
}
