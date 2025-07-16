/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <inttypes.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "argtable3/argtable3.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "cmd_twai_internal.h"

static const char *TAG = "cmd_twai_send";

/* Command line arguments for sending frames - supports both legacy and frame string formats */
static struct {
    struct arg_int *controller;   /* Controller ID (required) */
    struct arg_str *id;           /* Message ID (hex) or frame string (e.g., 123#AABBCC) */
    struct arg_str *data;         /* Data bytes (hex) or frame string if not provided with -i */
    struct arg_lit *rtr;          /* RTR flag (legacy mode) */
    struct arg_lit *ext;          /* Extended ID flag (legacy mode) */
    struct arg_lit *fd;           /* FD flag (legacy mode) */
    struct arg_lit *brs;          /* BRS flag (legacy mode) */
    struct arg_end *end;
} twai_send_args;

/* TX Callback for TWAI event handling */
static bool twai_send_tx_done_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t *event_data, void *user_ctx)
{
    twai_controller_ctx_t *controller = (twai_controller_ctx_t *)user_ctx;

    // Signal TX completion
    if (atomic_load(&controller->send_ctx.is_tx_pending)) {
        atomic_store(&controller->send_ctx.is_tx_pending, false);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(controller->send_ctx.tx_done_sem, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    return false;
}

/**
 * @brief Initialize the send module for a controller
 *
 * @param controller Pointer to the controller context
 * @return esp_err_t ESP_OK on success, or an error code
 */
static esp_err_t twai_send_init_controller(twai_controller_ctx_t *controller)
{
    int controller_idx = controller - &s_twai_controller_ctx[0];

    // Create TX completion semaphore
    controller->send_ctx.tx_done_sem = xSemaphoreCreateBinary();
    if (controller->send_ctx.tx_done_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create TX semaphore for controller %d", controller_idx);
        return ESP_ERR_NO_MEM;
    }

    // Initialize TX pending flag
    atomic_store(&controller->send_ctx.is_tx_pending, false);

    // Register TX done callback
    twai_core_ctx_t *core_ctx = &controller->core_ctx;
    core_ctx->driver_cbs.on_tx_done = twai_send_tx_done_cb;

    ESP_LOGI(TAG, "Send module initialized for TWAI%d", controller_idx);
    return ESP_OK;
}

/* Helpers for frame parsing */
#define TWAI_STD_DELIMITER_POS    3  /* Position of # in standard frame: 123# */
#define TWAI_EXT_DELIMITER_POS    8  /* Position of # in extended frame: 12345678# */
#define TWAI_STD_DATA_START_POS   4  /* Start position of data after 123# */
#define TWAI_EXT_DATA_START_POS   9  /* Start position of data after 12345678# */
#define TWAI_STD_ID_CHAR_LEN      3  /* Standard ID is 3 hex chars */
#define TWAI_EXT_ID_CHAR_LEN      8  /* Extended ID is 8 hex chars */
// 不再重复定义这些掩码，使用头文件中已有的定义
// #define TWAI_STD_ID_MASK        0x7FF
// #define TWAI_EXT_ID_MASK   0x1FFFFFFF
#define TWAI_MIN_FRAME_LEN        5  /* Minimum length: "123#R" */
#define TWAI_RTR_DEFAULT_DLC      0  /* Default DLC for RTR frames */
#define HEX_NIBBLE_MASK        0x0F
#define HEX_NIBBLE_SHIFT          4
#define TWAI_FD_FLAGS_MAX_VALUE   3
#define TWAI_FD_BRS_FLAG_MASK     1  /* 0b01 */
#define TWAI_FD_ESI_FLAG_MASK     2  /* 0b10 */

/**
 * @brief Convert ASCII hex character to nibble
 *
 * @param c ASCII character
 * @return uint8_t Nibble value (0-15) or 0xFF if invalid
 */
static uint8_t asc2nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    } else {
        return 0xFF;
    }
}

/* parse_hex_string function removed - no longer needed as we only support frame string format */

/* Frame parsing */
static bool parse_frame(const char *frame_str, twai_frame_t *frame)
{
    int len = strlen(frame_str);
    int idx, dlen;
    unsigned char tmp;
    bool is_fd_frame = false;

    if (len < TWAI_MIN_FRAME_LEN) {
        ESP_LOGE(TAG, "Invalid frame format: %s", frame_str);
        ESP_LOGE(TAG, "Expected format: <ID>#<DATA> or <ID>#R[DLC]");
        return false;
    }

    memset(&frame->header, 0, sizeof(frame->header));

    /* Parse CAN ID */
    if (frame_str[TWAI_STD_DELIMITER_POS] == '#') {
        /* Standard frame: "123#..." */
        idx = TWAI_STD_DATA_START_POS;
        for (int i = 0; i < TWAI_STD_ID_CHAR_LEN; i++) {
            if ((tmp = asc2nibble(frame_str[i])) > HEX_NIBBLE_MASK) {
                ESP_LOGE(TAG, "Invalid ID character: %c", frame_str[i]);
                return false;
            }
            frame->header.id |= tmp << (2 - i) * HEX_NIBBLE_SHIFT;
        }
        frame->header.ide = false;

    } else if (frame_str[TWAI_EXT_DELIMITER_POS] == '#') {
        /* Extended frame: "12345678#..." */
        idx = TWAI_EXT_DATA_START_POS;
        for (int i = 0; i < TWAI_EXT_ID_CHAR_LEN; i++) {
            if ((tmp = asc2nibble(frame_str[i])) > HEX_NIBBLE_MASK) {
                ESP_LOGE(TAG, "Invalid ID character: %c", frame_str[i]);
                return false;
            }
            frame->header.id |= tmp << (7 - i) * HEX_NIBBLE_SHIFT;
        }
        frame->header.ide = true;

        /* Validate extended ID range */
        if (frame->header.id > TWAI_EXT_ID_MASK) {
            ESP_LOGE(TAG, "Extended ID out of range: 0x%" PRIx32, frame->header.id);
            return false;
        }

    } else {
        ESP_LOGE(TAG, "Invalid frame format: %s", frame_str);
        ESP_LOGE(TAG, "Expected format: <ID>#<DATA> where ID is %d digits (standard) or %d digits (extended)",
                 TWAI_STD_ID_CHAR_LEN, TWAI_EXT_ID_CHAR_LEN);
        return false;
    }

    /* Validate standard ID range */
    if (!frame->header.ide && frame->header.id > TWAI_STD_ID_MASK) {
        ESP_LOGE(TAG, "Standard ID out of range: 0x%" PRIx32, frame->header.id);
        return false;
    }

    /* Handle RTR frames */
    if ((frame_str[idx] == 'R') || (frame_str[idx] == 'r')) {
        frame->header.rtr = true;
        idx++;

        /* Parse optional DLC */
        if (frame_str[idx] && (tmp = asc2nibble(frame_str[idx])) <= TWAI_FRAME_MAX_LEN) {
            frame->header.dlc = tmp;
            idx++;
        } else {
            frame->header.dlc = TWAI_RTR_DEFAULT_DLC; /* Default DLC for RTR */
        }

        /* RTR frames have no data, set buffer_len to 0 */
        frame->buffer_len = 0;

        return true; /* RTR frame processing complete */
    }

    /* Handle TWAI-FD frames */
    if (frame_str[idx] == '#') {
#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
        idx++; /* Skip second # */

        /* Parse FD flags */
        if (!frame_str[idx] || (tmp = asc2nibble(frame_str[idx])) > HEX_NIBBLE_MASK) {
            ESP_LOGE(TAG, "Missing or invalid FD flags");
            return false;
        }

        /* Validate FD flags value (must be 0-3) */
        if (tmp > TWAI_FD_FLAGS_MAX_VALUE) {
            ESP_LOGE(TAG, "Invalid FD flags value: %d (valid range: 0-3)", tmp);
            return false;
        }

        frame->header.fdf = true;                    /* This is an FD frame */
        frame->header.brs = (tmp & TWAI_FD_BRS_FLAG_MASK) ? true : false;  /* BRS flag */
        frame->header.esi = (tmp & TWAI_FD_ESI_FLAG_MASK) ? true : false;  /* ESI flag */
        is_fd_frame = true;
        idx++;
#else
        ESP_LOGE(TAG, "TWAI-FD frames not supported in this build");
        return false;
#endif
    }

    /* Parse data bytes with support for dot separators */
    int max_data_len = MAX_FRAME_DATA_LEN;
    /* For classic frames in FD mode, limit to 8 bytes unless it's an FD frame */
    if (!is_fd_frame) {
        max_data_len = TWAI_FRAME_MAX_LEN;
    }

    dlen = 0;
    while (idx < len && dlen < max_data_len) {
        /* Skip optional dot separator */
        if (frame_str[idx] == '.') {
            idx++;
            continue;
        }

        /* Check if we have at least 2 characters for a complete byte */
        if (idx + 1 >= len) {
            break;
        }

        /* Parse high nibble */
        if ((tmp = asc2nibble(frame_str[idx])) > 0x0F) {
            ESP_LOGE(TAG, "Invalid hex character: %c", frame_str[idx]);
            return false;
        }
        frame->buffer[dlen] = tmp << 4;
        idx++;

        /* Parse low nibble */
        if ((tmp = asc2nibble(frame_str[idx])) > 0x0F) {
            ESP_LOGE(TAG, "Invalid hex character: %c", frame_str[idx]);
            return false;
        }
        frame->buffer[dlen] |= tmp;
        idx++;

        dlen++;
    }

    /* Check for remaining unparsed data and provide better error message */
    if (idx < len && frame_str[idx] != '_') {
        /* Count remaining hex characters */
        int remaining_chars = 0;
        for (int i = idx; i < len; i++) {
            /* Fix: cast to unsigned char to avoid char subscript warning */
            if (frame_str[i] != '.' && frame_str[i] != '_' && isxdigit((unsigned char)frame_str[i])) {
                remaining_chars++;
            }
        }
        if (remaining_chars > 0) {
            ESP_LOGE(TAG, "Data too long: %d bytes parsed, ~%d more bytes in input",
                     dlen, remaining_chars / 2);
            ESP_LOGE(TAG, "Maximum data length: %d bytes (%s mode)",
                     max_data_len,
#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
                     (is_fd_frame) ? "TWAI-FD" : "Classic"
#else
                     "Classic"
#endif
                    );
            return false;
        }
    }

    /* Handle DLC override for classic CAN (8 bytes with _DLC suffix) */
    if ((max_data_len == TWAI_FRAME_MAX_LEN) && (dlen == TWAI_FRAME_MAX_LEN) && (idx < len) && (frame_str[idx] == '_')) {
        idx++; /* Skip underscore */
        if (idx < len && (tmp = asc2nibble(frame_str[idx])) <= TWAIFD_FRAME_MAX_LEN) {
            if (tmp > TWAI_FRAME_MAX_LEN && tmp <= TWAIFD_FRAME_MAX_LEN) {
                /* Extended DLC for classic CAN - store in a custom field if needed */
                frame->header.dlc = TWAI_FRAME_MAX_LEN;
            } else {
                frame->header.dlc = dlen;
            }
        } else {
            frame->header.dlc = dlen;
        }
    } else {
        frame->header.dlc = dlen;
    }

#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
    /* For TWAI-FD, convert data length to DLC if needed */
    // Implement this if you have a len2dlc conversion function
    // if (is_fd_frame) {
    //     frame->header.dlc = twaifd_len2dlc(dlen);
    // }
#endif

    /* Set buffer length */
    frame->buffer_len = dlen;

    return true;
}

/**
 * @brief Send a TWAI frame with the provided parameters
 *
 * @param controller Pointer to the TWAI controller context
 * @param frame Pointer to the TWAI frame to send
 * @param timeout_ms Timeout in milliseconds to wait for TX completion (default: 1000ms)
 * @return esp_err_t ESP_OK on success, or an error code
 */
static esp_err_t send_frame_sync(twai_controller_ctx_t *controller, const twai_frame_t *frame, uint32_t timeout_ms)
{
    if (!controller) {
        ESP_LOGE(TAG, "Invalid controller pointer");
        return ESP_ERR_INVALID_ARG;
    }

    int controller_id = controller - &s_twai_controller_ctx[0];

    twai_core_ctx_t *ctx = &controller->core_ctx;

    // Check if TWAI driver is running
    if (!atomic_load(&ctx->is_initialized)) {
        ESP_LOGE(TAG, "TWAI%d not initialized", controller_id);
        return ESP_ERR_INVALID_STATE;
    }

    // Mark TX as pending
    atomic_store(&controller->send_ctx.is_tx_pending, true);

    // Transmit the frame
    esp_err_t ret = twai_node_transmit(ctx->driver_handle, frame, timeout_ms / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue TX frame: %s", esp_err_to_name(ret));
        atomic_store(&controller->send_ctx.is_tx_pending, false);
        return ret;
    }

    // Wait for TX completion or timeout
    if (xSemaphoreTake(controller->send_ctx.tx_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "TX timed out after %"PRIu32" ms", timeout_ms);
        atomic_store(&controller->send_ctx.is_tx_pending, false);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

/* =============================================================================
 * `twai-send` command
 * =============================================================================*/
static int twai_send_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&twai_send_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, twai_send_args.end, argv[0]);
        return ESP_FAIL;
    }

    // Check for mandatory arguments
    if (twai_send_args.controller->count == 0) {
        ESP_LOGE(TAG, "Controller ID is required");
        return ESP_ERR_INVALID_ARG;
    }

    int controller_id = twai_send_args.controller->ival[0];
    twai_controller_ctx_t *controller = get_controller_by_id(controller_id);
    if (!controller) {
        ESP_LOGE(TAG, "Invalid controller ID: %d", controller_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare frame
    twai_frame_t frame = {0};
    uint8_t data_buffer[TWAI_FRAME_BUFFER_SIZE] = {0};
    frame.buffer = data_buffer;

    // Get the frame string argument (no longer supporting legacy mode)
    const char *frame_str = NULL;

    // Check if frame string is provided as a positional argument (after controller_id)
    if (argc > 2) {
        frame_str = argv[2];
    }
    // If not, check if it's provided with -i or -d options
    else if (twai_send_args.id->count > 0 && strchr(twai_send_args.id->sval[0], '#')) {
        frame_str = twai_send_args.id->sval[0];
    } else if (twai_send_args.data->count > 0 && strchr(twai_send_args.data->sval[0], '#')) {
        frame_str = twai_send_args.data->sval[0];
    }

    if (!frame_str || !strchr(frame_str, '#')) {
        ESP_LOGE(TAG, "Frame string is required (format: 123#AABBCC or 12345678#AABBCC)");
        return ESP_ERR_INVALID_ARG;
    }

    // Parse frame string
    if (!parse_frame(frame_str, &frame)) {
        ESP_LOGE(TAG, "Failed to parse frame string: %s", frame_str);
        return ESP_ERR_INVALID_ARG;
    }

    // Log frame information
    ESP_LOGI(TAG, "Sending frame: ID=0x%"PRIx32" (%s), %s%s%s, DLC=%d",
             frame.header.id,
             frame.header.ide ? "EXT" : "STD",
             frame.header.rtr ? "RTR" : "Data",
             frame.header.fdf ? ", FD" : "",
             frame.header.brs ? ", BRS" : "",
             frame.header.dlc);

    if (!frame.header.rtr && frame.buffer_len > 0) {
        ESP_LOGI(TAG, "Data: ");
        for (int i = 0; i < frame.buffer_len; i++) {
            printf("%02X ", frame.buffer[i]);
        }
        printf("\n");
    }

    // Send frame with 1 second timeout
    esp_err_t ret = send_frame_sync(controller, &frame, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send frame: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    printf("Frame sent successfully\n");
    return ESP_OK;
}

void register_twai_send_commands(void)
{
    // Initialize send context for all controllers
    for (int i = 0; i < SOC_TWAI_CONTROLLER_NUM; i++) {
        twai_controller_ctx_t *controller = &s_twai_controller_ctx[i];
        esp_err_t ret = twai_send_init_controller(controller);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize send module for TWAI%d: %s",
                     i, esp_err_to_name(ret));
        }
    }

    // Register command arguments
    twai_send_args.controller = arg_int1(NULL, NULL, "<controller_id>", "TWAI controller ID (0 or 1)");

    // Allow providing frame string either positionally or via options
    twai_send_args.id = arg_str0("i", "id", "<frame_str>",
                                 "Frame string in format 123#AABBCC (standard) or 12345678#AABBCC (extended)");
    twai_send_args.data = arg_str0("d", "data", "<frame_str>",
                                   "Alternative way to provide frame string");

    // Keep legacy option declarations for backward compatibility with existing code
    // but they are no longer used in the command handler
    twai_send_args.rtr = arg_lit0(NULL, NULL, NULL);
    twai_send_args.ext = arg_lit0(NULL, NULL, NULL);
    twai_send_args.fd = arg_lit0(NULL, NULL, NULL);
    twai_send_args.brs = arg_lit0(NULL, NULL, NULL);
    twai_send_args.end = arg_end(20);

    // Register command
    const esp_console_cmd_t twai_send_cmd = {
        .command = "twai-send",
        .help = "Send a TWAI frame using string format",
        .hint = "<controller_id> <frame_str>",
        .func = &twai_send_handler,
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&twai_send_cmd));

    // Log usage examples
    ESP_LOGI(TAG, "Command usage examples:");
    ESP_LOGI(TAG, "  Standard frame:    twai-send 0 123#AABBCC");
    ESP_LOGI(TAG, "  Extended frame:    twai-send 0 12345678#AABBCC");
    ESP_LOGI(TAG, "  RTR frame:         twai-send 0 123#R");
    ESP_LOGI(TAG, "  FD frame:          twai-send 0 123##1AABBCCDD");
}
