/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "twai_utils_parser.h"
#include <string.h>
#include <ctype.h>
#include "hal/twai_types.h"
#include "driver/gpio.h"

int parse_nibble(char c, uint8_t *out)
{
    if (!out) {
        return PARSE_INVALID_ARG;
    }

    if (c >= '0' && c <= '9') {
        *out = (uint8_t)(c - '0');
        return PARSE_OK;
    }
    if (c >= 'A' && c <= 'F') {
        *out = (uint8_t)(c - 'A' + 10);
        return PARSE_OK;
    }
    if (c >= 'a' && c <= 'f') {
        *out = (uint8_t)(c - 'a' + 10);
        return PARSE_OK;
    }

    return PARSE_ERROR;
}

/**
 * @brief Parse hex string with specified length (no null terminator required)
 *
 * @param[in] str Input string pointer
 * @param[in] len Length of hex string to parse
 * @param[out] out Output value pointer
 *
 * @return PARSE_OK on success, PARSE_ERROR on format error
 */
static int parse_hex_segment(const char *str, size_t len, uint32_t *out)
{
    if (!str || len == 0 || len > TWAI_EXT_ID_CHAR_LEN || !out) {
        return PARSE_INVALID_ARG;
    }

    uint32_t result = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t nibble;
        if (parse_nibble(str[i], &nibble) != PARSE_OK) {
            return PARSE_ERROR;
        }
        result = (result << 4) | nibble;
    }

    *out = result;
    return PARSE_OK;
}

/**
 * @brief Parse payload bytes (hex pairs) up to max length, skipping '.' separators
 *
 * This function reads up to max bytes from the ASCII hex string s,
 * ignoring any '.' separators. Each pair of hex digits is converted
 * into one byte and stored into buf.
 *
 * @param[in]  s    Null-terminated input string containing hex digits and optional '.' separators
 * @param[out] buf  Buffer to store parsed byte values
 * @param[in]  max  Maximum number of bytes to parse (buffer capacity)
 *
 * @return On success, returns the number of bytes parsed (0..max).
 *         Returns PARSE_INVALID_ARG if input pointers are NULL or max <= 0.
 *         Returns PARSE_ERROR if a non-hex digit is encountered before parsing max bytes
 */
static inline int parse_payload(const char *s, uint8_t *buf, int max)
{
    if (!s || !buf || max <= 0) {
        return PARSE_INVALID_ARG;
    }
    int cnt = 0;
    while (*s && cnt < max) {
        if (*s == '.') {
            s++;
            continue;
        }
        /* Check if we have valid hex pair */
        if (!isxdigit((unsigned char)s[0])) {
            if (cnt == 0 && *s != '\0') {
                return PARSE_ERROR;
            }
            break;
        }
        if (!isxdigit((unsigned char)s[1])) {
            return PARSE_ERROR;
        }
        uint8_t high, low;
        if (parse_nibble(s[0], &high) != PARSE_OK || parse_nibble(s[1], &low) != PARSE_OK) {
            return PARSE_ERROR;
        }
        buf[cnt++] = (high << 4) | low;
        s += 2;
    }
    return cnt;
}

/**
 * @brief Parse hex ID substring of given length
 *
 * @param[in] str   Pointer to the start of the hex substring
 * @param[in] len   Number of characters in the hex substring (3 or 8)
 * @param[out] out  Pointer to the variable to receive the parsed ID value
 * @param[out] is_ext Pointer to store whether the ID is extended format
 *
 * @return PARSE_OK on success;
 *         PARSE_INVALID_ARG if pointers are NULL or len is out of range;
 *         PARSE_ERROR if any character is not a valid hex digit or length mismatch
 */
static inline int parse_hex_id(const char *str, size_t len, uint32_t *out, bool *is_ext)
{
    if (!str || len == 0 || len > TWAI_EXT_ID_CHAR_LEN) {
        return PARSE_INVALID_ARG;
    }
    char *end;
    unsigned long val = strtoul(str, &end, 16);
    if ((size_t)(end - str) != len) {
        return PARSE_ERROR;
    }
    *is_ext = (len > TWAI_STD_ID_CHAR_LEN);
    if (*is_ext && val > TWAI_EXT_ID_MASK) {
        return PARSE_OUT_OF_RANGE;
    }
    if (!*is_ext && val > TWAI_STD_ID_MASK) {
        return PARSE_OUT_OF_RANGE;
    }
    *out = (uint32_t)val;
    return PARSE_OK;
}

int parse_twai_id(const char *str, size_t len, twai_frame_t *f)
{
    if (!str || !f) {
        return PARSE_INVALID_ARG;
    }
    bool is_ext = false;
    uint32_t id = 0;
    int res = parse_hex_id(str, len, &id, &is_ext);
    if (res != PARSE_OK) {
        return res;
    }
    f->header.id = id;
    f->header.ide = is_ext ? 1 : 0;
    return PARSE_OK;
}

int parse_classic_frame(const char *body, twai_frame_t *f)
{
    if (!body || !f) {
        return PARSE_INVALID_ARG;
    }

    /* Handle RTR frame */
    if (*body == 'R' || *body == 'r') {
        f->header.rtr = true;
        body++;
        if (isxdigit((unsigned char) * body)) {
            f->header.dlc = (uint8_t)strtoul(body, NULL, 16);
        } else {
            f->header.dlc = TWAI_RTR_DEFAULT_DLC;
        }
        f->buffer_len = 0;
        return PARSE_OK;
    }

    /* Handle data frame */
    int dl = parse_payload(body, f->buffer, TWAI_MAX_DATA_LEN);
    if (dl < 0) {
        return dl;
    }

    /* Check for optional _dlc suffix */
    const char *p = body;
    int seen = 0;
    while (*p && seen < dl) {
        if (*p == '.') {
            p++;
            continue;
        }
        if (!isxdigit((unsigned char)p[0]) || !isxdigit((unsigned char)p[1])) {
            break;
        }
        p += 2; seen++;
    }

    if (*p == '_') {
        p++;
        uint8_t code;
        if (parse_nibble(*p, &code) == PARSE_OK && code > TWAI_MAX_DATA_LEN) {
            f->header.dlc = TWAI_MAX_DATA_LEN;
        } else {
            f->header.dlc = (uint8_t)dl;
        }
    } else {
        f->header.dlc = (uint8_t)dl;
    }
    f->buffer_len = dl;
    return PARSE_OK;
}

#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
int parse_twaifd_frame(const char *body, twai_frame_t *f)
{
    if (!body || !f) {
        return PARSE_INVALID_ARG;
    }
    uint8_t flags;
    if (parse_nibble(*body++, &flags) != PARSE_OK || flags > TWAI_FD_FLAGS_MAX_VALUE) {
        return PARSE_OUT_OF_RANGE;
    }
    f->header.fdf = true;
    f->header.brs = !!(flags & TWAI_FD_BRS_FLAG_MASK);
    f->header.esi = !!(flags & TWAI_FD_ESI_FLAG_MASK);
    int dl = parse_payload(body, f->buffer, TWAI_FD_MAX_DATA_LEN);
    if (dl < 0) {
        return dl;
    }
    f->buffer_len = dl;
    f->header.dlc = (uint8_t)twaifd_len2dlc((uint16_t)dl);
    return PARSE_OK;
}
#endif

/**
 * @brief Parse a single filter token (id:mask or low-high)
 *
 * @param[in] tok  Pointing to the substring after '#'
 * @param[in] tok_len  Length of the token
 * @param[out] mask_cfgs  Pointer to mask filter configurations
 * @param[in,out] mask_idx  Pointer to the mask index, it will be updated
 *                        if the token is parsed to a mask filter
 * @param[out] range_cfgs  Pointer to range filter configurations
 * @param[in,out] range_idx  Pointer to the range index, it will be updated
 *                        if the token is parsed to a range filter
 *
 * @return PARSE_OK on success;
 *         PARSE_ERROR on format error;
 *         PARSE_OUT_OF_RANGE if index exceeds filter limits
 */
static int parse_filter_token(const char *tok, size_t tok_len,
                              twai_mask_filter_config_t *mask_cfgs, int *mask_idx
#if SOC_TWAI_SUPPORT_FD
                              ,
                              twai_range_filter_config_t *range_cfgs, int *range_idx
#endif
                             )
{
    const char *sep;
    uint32_t a, b;
    size_t sep_pos;

    /* Parse mask filter: id:mask */
    sep = memchr(tok, ':', tok_len);
    if (sep) {
        sep_pos = sep - tok;
        size_t mask_len = tok_len - sep_pos - 1;

        if (parse_hex_segment(tok, sep_pos, &a) != PARSE_OK ||
                parse_hex_segment(tok + sep_pos + 1, mask_len, &b) != PARSE_OK) {
            return PARSE_ERROR;
        }
        if (*mask_idx >= SOC_TWAI_MASK_FILTER_NUM) {
            printf("Mask filter index exceeds limit: mask_idx=%d", *mask_idx);
            return PARSE_OUT_OF_RANGE;
        }
        mask_cfgs[(*mask_idx)++] = (twai_mask_filter_config_t) {
            .id = a, .mask = b, .is_ext = false
        };
        return PARSE_OK;
    }

#if SOC_TWAI_SUPPORT_FD
    /* Parse range filter: low-high */
    sep = memchr(tok, '-', tok_len);
    if (sep) {
        sep_pos = sep - tok;
        size_t high_len = tok_len - sep_pos - 1;

        if (parse_hex_segment(tok, sep_pos, &a) != PARSE_OK ||
                parse_hex_segment(tok + sep_pos + 1, high_len, &b) != PARSE_OK ||
                a > b) {
            return PARSE_ERROR;
        }
        if (*range_idx >= SOC_TWAI_RANGE_FILTER_NUM) {
            printf("Range filter index exceeds limit: range_idx=%d", *range_idx);
            return PARSE_OUT_OF_RANGE;
        }
        range_cfgs[(*range_idx)++] = (twai_range_filter_config_t) {
            .range_low = a,
            .range_high = b,
            .is_ext = false
        };
        return PARSE_OK;
    }
#endif

    return PARSE_ERROR;
}

int parse_filters(const char *filter_str,
                  twai_mask_filter_config_t *masks, int *mask_count
#if SOC_TWAI_SUPPORT_FD
                  , twai_range_filter_config_t *ranges, int *range_count
#endif
                 )
{
    if (!filter_str || strlen(filter_str) >= MAX_INPUT_LEN) {
        return PARSE_ERROR;
    }

    /* Initialize counts */
    *mask_count = 0;
#if SOC_TWAI_SUPPORT_FD
    *range_count = 0;
#endif

    /* Empty filter string is valid - means accept all frames */
    if (strlen(filter_str) == 0) {
        return PARSE_OK;
    }

    const char *start = filter_str;
    const char *comma;

    while (start && *start) {
        /* Find next comma or end of string */
        comma = strchr(start, ',');
        size_t token_len = comma ? (size_t)(comma - start) : strlen(start);

        /* Skip empty tokens */
        if (token_len == 0) {
            start = comma ? comma + 1 : NULL;
            continue;
        }

        int ret = parse_filter_token(start, token_len, masks, mask_count
#if SOC_TWAI_SUPPORT_FD
                                     , ranges, range_count
#endif
                                    );
        if (ret != PARSE_OK) {
            return ret;
        }

        /* Move to next token */
        start = comma ? comma + 1 : NULL;
    }
    return PARSE_OK;
}

const char *twai_state_to_string(twai_error_state_t state)
{
    switch (state) {
    case TWAI_ERROR_ACTIVE:  return "Error Active";
    case TWAI_ERROR_WARNING: return "Error Warning";
    case TWAI_ERROR_PASSIVE: return "Error Passive";
    case TWAI_ERROR_BUS_OFF:  return "Bus Off";
    default: return "Unknown";
    }
}

int format_gpio_pin(int gpio_pin, char *buffer, size_t buffer_size)
{
    if (gpio_pin == GPIO_NUM_NC || gpio_pin < 0) {
        return snprintf(buffer, buffer_size, "Disabled");
    } else {
        return snprintf(buffer, buffer_size, "GPIO%d", gpio_pin);
    }
}

int parse_controller_string(const char *controller_str)
{
    int controller_id;
    const char *end = parse_controller_id(controller_str, &controller_id);
    return end ? controller_id : PARSE_ERROR;
}
