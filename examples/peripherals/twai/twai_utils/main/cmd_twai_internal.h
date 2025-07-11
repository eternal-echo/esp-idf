/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include <stdatomic.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

/* =============================================================================
 * MACRO DEFINITIONS
 * =============================================================================*/

#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
#define DEFAULT_FD_DATA_BITRATE     1000000             // Safe default: 1 Mbps for FD data
#define MAX_FRAME_DATA_LEN          TWAIFD_FRAME_MAX_LEN
#define TWAI_FRAME_BUFFER_SIZE      TWAIFD_FRAME_MAX_LEN
#else
#define DEFAULT_FD_DATA_BITRATE     0
#define MAX_FRAME_DATA_LEN          TWAI_FRAME_MAX_LEN
#define TWAI_FRAME_BUFFER_SIZE      TWAI_FRAME_MAX_LEN
#endif

typedef struct {
    twai_onchip_node_config_t driver_config;     // Cached driver configuration
    twai_onchip_node_config_t default_config;    // Default configuration
    twai_node_handle_t driver_handle;
    twai_event_callbacks_t driver_cbs;
    atomic_bool is_initialized;
} twai_core_ctx_t;

/**
 * @brief Core state machine for the TWAI console.
 *
 * This structure manages core driver resources, synchronization primitives,
 * and resources for different functional modules (send, dump, player).
 * It embeds twai_utils_status_t to handle bus status and statistics.
 */
typedef struct {
    /* Core Driver Resources */
    twai_core_ctx_t core_ctx;
    twai_node_handle_t node_handle;
    /* Synchronization and Concurrency Control */
    SemaphoreHandle_t mutex;                     // Main protection mutex
} twai_controller_ctx_t;

extern twai_controller_ctx_t s_twai_controller_ctx[SOC_TWAI_CONTROLLER_NUM];

twai_controller_ctx_t* get_controller_by_id(int controller_id);

void register_twai_core_commands(void);

#ifdef __cplusplus
}
#endif
