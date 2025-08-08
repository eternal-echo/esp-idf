/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "hal/twai_types.h"
#include "esp_twai.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TWAI frame constants */
#define TWAI_MAX_DATA_LEN    8
#define TWAI_FD_MAX_DATA_LEN 64
#define TWAI_STD_ID_CHAR_LEN      3
#define TWAI_EXT_ID_CHAR_LEN      8

/** @brief Parser return codes */
#define PARSE_OK            0
#define PARSE_ERROR         -1
#define PARSE_INVALID_ARG   -2
#define PARSE_OUT_OF_RANGE  -3
#define PARSE_TOO_LONG      -4

/* Additional constants */
#define TWAI_RTR_DEFAULT_DLC        8
#define TWAI_FD_FLAGS_MAX_VALUE     15
#define TWAI_FD_BRS_FLAG_MASK       0x01
#define TWAI_FD_ESI_FLAG_MASK       0x02
#define MAX_INPUT_LEN               256

/**
 * @brief Parse TWAI ID from string
 *
 * @param[in] str  Pointer to the start of the ID string
 * @param[in] len  Length of the ID string
 * @param[out] f   Pointer to frame structure to fill
 *
 * @return PARSE_OK on success;
 *         PARSE_INVALID_ARG if pointers are NULL;
 *         PARSE_ERROR or PARSE_OUT_OF_RANGE on format or range error
 */
int parse_twai_id(const char *str, size_t len, twai_frame_t *f);

#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
/**
 * @brief Parse TWAI-FD frames with flags and extended payload
 *
 * Body format: <flags>{data}
 *   flags: single hex nibble (0..F)
 *   data: up to 64 bytes hex pairs
 *
 * @param[in] body  Pointing to the substring after '#'
 * @param[out] f     Pointer to frame structure to fill
 *
 * @return PARSE_OK on success;
 *         PARSE_INVALID_ARG if arguments are NULL;
 *         PARSE_ERROR or PARSE_OUT_OF_RANGE on format or range error
 */
int parse_twaifd_frame(const char *str, twai_frame_t *f);
#endif

/**
 * @brief Parse Classical TWAI data and RTR frames
 *
 * Supports:
 *   <twai_id>#{data}          Data frame with up to 8 bytes
 *   <twai_id>#R{len}          RTR frame with specified length
 *   <twai_id>#{data}_{dlc}    Data frame with extended DLC (9..F)
 *
 * @param[in]  body  Pointing to the substring after '#'
 * @param[out] f     Pointer to frame structure to fill
 *
 * @return PARSE_OK on success;
 *         PARSE_INVALID_ARG if arguments are NULL;
 *         PARSE_ERROR or PARSE_OUT_OF_RANGE on format or range error
 */
int parse_classic_frame(const char *str, twai_frame_t *f);

/**
 * @brief Parse a single hex nibble character
 *
 * @param[in]  c    Input character (0-9, A-F, a-f)
 * @param[out] out  Output pointer to store the parsed nibble value (0-15)
 *
 * @return PARSE_OK on success;
 *         PARSE_INVALID_ARG if out pointer is NULL;
 *         PARSE_ERROR if character is not a valid hex digit
 */
int parse_nibble(char c, uint8_t *out);

/**
 * @brief Parse controller string and return controller ID
 *
 * @param[in] controller_str  Controller string (e.g., "twai0")
 *
 * @return Controller ID (0-9) on success, PARSE_ERROR on failure
 */
int parse_controller_string(const char *controller_str);

/**
 * @brief Convert TWAI state to string
 *
 * @param[in] state  TWAI error state
 *
 * @return Pointer to the string representation of the state
 */
const char *twai_state_to_string(twai_error_state_t state);

/**
 * @brief Format GPIO pin display
 *
 * @param[in] gpio_pin  GPIO pin number
 * @param[out] buffer   Buffer to store the formatted string
 * @param[in] buffer_size  Size of the buffer
 *
 * @return Number of characters written to buffer
 */
int format_gpio_pin(int gpio_pin, char *buffer, size_t buffer_size);

/**
 * @brief Parse filter string and populate filter configurations
 *
 * Parses comma-separated filter tokens and populates mask and range filter
 * configurations. Supports both mask filters (id:mask) and range filters (low-high).
 *
 * @param[in] filter_str  Comma-separated filter string
 * @param[out] masks  Array to store mask filter configurations
 * @param[out] mask_count  Number of mask filters parsed
#if SOC_TWAI_SUPPORT_FD
 * @param[out] ranges  Array to store range filter configurations
 * @param[out] range_count  Number of range filters parsed
#endif
 *
 * @return PARSE_OK on success, PARSE_ERROR on format error
 */
int parse_filters(const char *filter_str,
                  twai_mask_filter_config_t *masks, int *mask_count
#if SOC_TWAI_SUPPORT_FD
                  , twai_range_filter_config_t *ranges, int *range_count
#endif
                 );

/**
 * @brief Locate first '#' and count consecutives
 *
 * @param[in] input Input string
 * @param[out] sep Pointer to the separator
 * @param[out] hash_count Pointer to the hash count
 *
 * @return PARSE_OK if successful, PARSE_INVALID_ARG if input is NULL, PARSE_ERROR if no '#' is found
 */
static inline int locate_hash(const char *input, const char **sep, int *hash_count)
{
    if (!input || !sep || !hash_count) {
        return PARSE_INVALID_ARG;
    }
    const char *s = strchr(input, '#');
    if (!s) {
        return PARSE_ERROR;
    }
    *sep = s;
    *hash_count = 1;
    while (s[*hash_count] == '#') {
        (*hash_count)++;
    }
    return PARSE_OK;
}

/**
 * @brief Parse the controller ID string and return the end of the controller substring
 *
 * This function parses a controller string in the format "twai0", "twai1", ..., "twaix"
 * and extracts the controller ID (0-x). It also supports controller strings with filters,
 * such as "twai0,123:7FF", and returns a pointer to the end of the substring(e.g. the ',' or '\0').
 *
 * @param[in] controller_str Input controller string (e.g., "twai0" or "twai0,123:7FF")
 * @param[out] controller_id Output pointer to store the parsed controller ID
 *
 * @return Pointer to the end of the controller substring (e.g., the ',' or '\0'), or NULL on error
 */
static inline const char *parse_controller_id(const char *controller_str, int *controller_id)
{
    if (!controller_str || !controller_id) {
        return NULL;
    }

    /* Support "twai0" ~ "twaix" format (which is dependent on SOC_TWAI_CONTROLLER_NUM) */
    if (strncmp(controller_str, "twai", 4) == 0 && strlen(controller_str) >= 5) {
        char id_char = controller_str[4];
        if (id_char >= '0' && id_char <= '9' && id_char < '0' + SOC_TWAI_CONTROLLER_NUM) {
            *controller_id = id_char - '0';
            /* Return pointer to character after the ID digit */
            return controller_str + 5;
        }
    }

    return NULL;
}

#ifdef __cplusplus
}
#endif
