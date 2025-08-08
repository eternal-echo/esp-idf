/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <stdatomic.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "argtable3/argtable3.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "cmd_twai_internal.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "twai_utils_parser.h"

#define DUMP_OUTPUT_LINE_SIZE 512

/** @brief Structure for queuing received frames with embedded buffer */
typedef struct {
    twai_frame_header_t header;
    uint8_t data[TWAI_FRAME_BUFFER_SIZE];
    size_t data_len;
    int64_t timestamp_us;
} rx_queue_item_t;

/** @brief Command line arguments structure */
static struct {
    struct arg_str *controller_filter;  /**< Format: <controller>[,<id>:<mask>[,<id>:<mask>...]] */
    struct arg_lit *stop;               /**< Stop option: --stop */
    struct arg_str *timestamp;          /**< Timestamp mode: -t <mode> */
    struct arg_end *end;
} twai_dump_args;

static const char *TAG = "cmd_twai_dump";

/**
 * @brief Format timestamp string based on the specified mode
 *
 * @param[in] dump_ctx Dump context with timestamp configuration
 * @param[in] frame_timestamp Frame timestamp in microseconds
 * @param[out] timestamp_str Buffer to store formatted timestamp string
 * @param[in] max_len Maximum length of timestamp string buffer
 */
static inline void format_timestamp(twai_dump_ctx_t *dump_ctx, int64_t frame_timestamp, char *timestamp_str, size_t max_len)
{
    if (dump_ctx->timestamp_mode == TIMESTAMP_MODE_NONE) {
        timestamp_str[0] = '\0';
        return;
    }

    int64_t timestamp_us;

    switch (dump_ctx->timestamp_mode) {
    case TIMESTAMP_MODE_ABSOLUTE:
        timestamp_us = frame_timestamp;
        break;
    case TIMESTAMP_MODE_DELTA:
        timestamp_us = frame_timestamp - dump_ctx->last_frame_time_us;
        dump_ctx->last_frame_time_us = frame_timestamp;
        break;
    case TIMESTAMP_MODE_ZERO:
        timestamp_us = frame_timestamp - dump_ctx->start_time_us;
        break;
    default:
        timestamp_str[0] = '\0';
        return;
    }

    /* Format output: (seconds.microseconds) */
    snprintf(timestamp_str, max_len, "(%lld.%06lld) ", timestamp_us / 1000000, timestamp_us % 1000000);
}

/**
 * @brief Format TWAI frame in twai_dump format
 *
 * @param[in] dump_ctx Dump context with timestamp configuration
 * @param[in] item Received frame item
 * @param[in] controller_id Controller ID for interface name
 * @param[out] output_line Buffer to store formatted output line
 * @param[in] max_len Maximum length of output line buffer
 */
static void format_twaidump_frame(twai_dump_ctx_t *dump_ctx, const rx_queue_item_t *item,
                                  int controller_id, char *output_line, size_t max_len)
{
    char timestamp_str[64] = {0};
    int pos = 0;

    /* Format timestamp */
    format_timestamp(dump_ctx, item->timestamp_us, timestamp_str, sizeof(timestamp_str));

    /* Add timestamp if enabled */
    if (strlen(timestamp_str) > 0) {
        pos += snprintf(output_line + pos, max_len - pos, "%s", timestamp_str);
    }

    /* Add interface name (e.g. use twai0, twai1) */
    pos += snprintf(output_line + pos, max_len - pos, "twai%d  ", controller_id);

    /* Format TWAI ID (formatted as: 3 digits for SFF, 8 digits for EFF) */
    if (item->header.ide) {
        /* Extended frame: 8 hex digits */
        pos += snprintf(output_line + pos, max_len - pos, "%08" PRIX32 "  ", item->header.id);
    } else {
        /* Standard frame: 3 hex digits (or less if ID is smaller) */
        pos += snprintf(output_line + pos, max_len - pos, "%03" PRIX32 "  ", item->header.id);
    }

    if (item->header.rtr) {
        /* RTR frame: add [R] and DLC */
        pos += snprintf(output_line + pos, max_len - pos, "[R%d]", item->header.dlc);
    } else {
        /* Data frame: add DLC and data bytes with spaces */
#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
        int actual_len = item->header.fdf ? twaifd_dlc2len(item->header.dlc) : item->header.dlc;
#else
        int actual_len = item->header.dlc;
#endif
        pos += snprintf(output_line + pos, max_len - pos, "[%d]", actual_len);
        for (int i = 0; i < actual_len && pos < max_len - 4; i++) {
            pos += snprintf(output_line + pos, max_len - pos, "  %02X", item->data[i]);
        }
    }

    /* Add newline */
    if (pos < max_len - 1) {
        pos += snprintf(output_line + pos, max_len - pos, "\n");
    }
}

/**
 * @brief TWAI receive done callback for dump functionality
 *
 * @param[in] handle TWAI node handle
 * @param[in] event_data Receive event data
 * @param[in] user_ctx Controller context pointer
 *
 * @return @c true if higher priority task woken, @c false otherwise
 */
static IRAM_ATTR bool twai_dump_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *event_data, void *user_ctx)
{
    ESP_UNUSED(handle);
    ESP_UNUSED(event_data);
    twai_controller_ctx_t *controller = (twai_controller_ctx_t *)user_ctx;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (!atomic_load(&controller->dump_ctx.is_running)) {
        return false;
    }

    rx_queue_item_t item;
    twai_frame_t frame = {
        .buffer = item.data,
        .buffer_len = sizeof(item.data),
    };

    if (ESP_OK == twai_node_receive_from_isr(handle, &frame)) {
        item.timestamp_us = esp_timer_get_time();
        item.header = frame.header;
        item.data_len = frame.buffer_len;

        /* Non-blocking queue send with explicit error handling */
        if (xQueueSendFromISR(controller->dump_ctx.rx_queue, &item, &higher_priority_task_woken) != pdTRUE) {
            /* Queue full - frame dropped silently to maintain ISR performance */
        }
    }

    return (higher_priority_task_woken == pdTRUE);
}

/**
 * @brief Frame reception task for dump functionality
 *
 * @param[in] parameter Controller context pointer
 */
static void dump_task(void *parameter)
{
    twai_controller_ctx_t *controller = (twai_controller_ctx_t *)parameter;
    twai_dump_ctx_t *dump_ctx = &(controller->dump_ctx);
    int controller_id = controller - s_twai_controller_ctx;

    ESP_LOGD(TAG, "Dump task started for controller %d", controller_id);

    while (atomic_load(&dump_ctx->is_running)) {
        rx_queue_item_t item;
        if (xQueueReceive(dump_ctx->rx_queue, &item, pdMS_TO_TICKS(CONFIG_EXAMPLE_DUMP_TASK_TIMEOUT_MS)) == pdPASS) {
            static char output_line[DUMP_OUTPUT_LINE_SIZE];

            format_twaidump_frame(dump_ctx, &item, controller_id, output_line, sizeof(output_line));
            printf("%s", output_line);
        }
    }

    /* Clean up our own resources before exit */
    if (dump_ctx->rx_queue) {
        vQueueDelete(dump_ctx->rx_queue);
        dump_ctx->rx_queue = NULL;
    }

    dump_ctx->dump_task_handle = NULL;
    /* Delete self */
    vTaskDelete(NULL);
}

/**
 * @brief Initialize TWAI dump module for a controller
 *
 * @param[in] controller Controller context to initialize
 *
 * @return @c ESP_OK on success, error code on failure
 */
static esp_err_t twai_dump_init_controller(twai_controller_ctx_t *controller)
{
    /* Just register the callback, resources will be created when dump starts */
    controller->core_ctx.driver_cbs.on_rx_done = twai_dump_rx_done_cb;

    /* Initialize atomic flags */
    atomic_store(&controller->dump_ctx.is_running, false);
    controller->dump_ctx.rx_queue = NULL;
    controller->dump_ctx.dump_task_handle = NULL;

    return ESP_OK;
}

/**
 * @brief Start dump for a controller - create resources and task
 *
 * @param[in] controller Controller context to start dump for
 *
 * @return @c ESP_OK on success, error code on failure
 */
static esp_err_t twai_dump_start_controller(twai_controller_ctx_t *controller)
{
    int controller_id = controller - s_twai_controller_ctx;
    twai_dump_ctx_t *dump_ctx = &controller->dump_ctx;

    /* Check if already running */
    if (atomic_load(&dump_ctx->is_running)) {
        ESP_LOGW(TAG, "Dump already running for controller %d", controller_id);
        return ESP_OK;
    }

    /* Create frame queue */
    dump_ctx->rx_queue = xQueueCreate(CONFIG_EXAMPLE_DUMP_QUEUE_SIZE, sizeof(rx_queue_item_t));
    if (!dump_ctx->rx_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue for controller %d", controller_id);
        return ESP_ERR_NO_MEM;
    }

    /* Set running flag before creating task */
    atomic_store(&dump_ctx->is_running, true);

    /* Create dump task */
    BaseType_t task_ret = xTaskCreate(
                              dump_task,
                              "twai_dump_task",
                              CONFIG_EXAMPLE_DUMP_TASK_STACK_SIZE,
                              controller,  /* Pass controller as user data */
                              CONFIG_EXAMPLE_DUMP_TASK_PRIORITY,
                              &dump_ctx->dump_task_handle);
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(task_ret == pdPASS, ESP_ERR_NO_MEM, err, TAG, "Failed to create dump task for controller %d", controller_id);

    return ESP_OK;

err:
    atomic_store(&dump_ctx->is_running, false);
    vQueueDelete(dump_ctx->rx_queue);
    dump_ctx->rx_queue = NULL;
    return ret;
}

/**
 * @brief Deinitialize TWAI dump module for a controller
 *
 * @param[in] controller Controller context to deinitialize
 */
static void twai_dump_deinit_controller(twai_controller_ctx_t *controller)
{
    int controller_id = controller - s_twai_controller_ctx;
    twai_dump_stop_internal(controller_id);

    /* Clear callback */
    controller->core_ctx.driver_cbs.on_rx_done = NULL;

    ESP_LOGD(TAG, "Dump module deinitialized for controller %d", controller_id);
}

/**
 * @brief Command handler for twai_dump command
 *
 * @param[in] argc Argument count
 * @param[in] argv Argument vector
 *
 * @return @c ESP_OK on success, error code on failure
 */
static int twai_dump_handler(int argc, char **argv)
{
    esp_err_t ret = ESP_OK;
    int nerrors = arg_parse(argc, argv, (void **)&twai_dump_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, twai_dump_args.end, argv[0]);
        return ESP_ERR_INVALID_ARG;
    }

    /* Stop dump */
    if (twai_dump_args.stop->count > 0) {
        /* For --stop option, controller ID is in the controller_filter argument */
        const char *controller_str = twai_dump_args.controller_filter->sval[0];
        int controller_id = parse_controller_string(controller_str);
        ESP_RETURN_ON_FALSE(controller_id >= 0, ESP_ERR_INVALID_ARG, TAG, "Invalid controller ID: %s", controller_str);
        twai_controller_ctx_t *controller = get_controller_by_id(controller_id);
        ESP_RETURN_ON_FALSE(controller != NULL, ESP_ERR_INVALID_ARG, TAG, "Failed to get controller for ID: %d", controller_id);

        ret = twai_dump_stop_internal(controller_id);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Failed to stop dump on controller %d", controller_id);

        return ESP_OK;
    }

    /* Start dump */
    const char *controller_str = twai_dump_args.controller_filter->sval[0];

    /* Parse controller ID, e.g. "twai0" -> 0 */
    int controller_id = -1;
    const char *filter_str = NULL;
    filter_str = parse_controller_id(controller_str, &controller_id);
    ESP_RETURN_ON_FALSE(controller_id >= 0, ESP_ERR_INVALID_ARG, TAG, "Failed to parse controller ID");
    twai_controller_ctx_t *controller = get_controller_by_id(controller_id);
    ESP_RETURN_ON_FALSE(controller != NULL, ESP_ERR_INVALID_ARG, TAG, "Failed to get controller for ID: %d", controller_id);
    /* Parse filter string, e.g. ",123:7FF" -> "123:7FF" */
    int mask_count = 0;
#if SOC_TWAI_SUPPORT_FD
    int range_count = 0;
#endif
    /* Clear filter configs first */
    memset(controller->dump_ctx.mask_filter_configs, 0, sizeof(controller->dump_ctx.mask_filter_configs));
#if SOC_TWAI_SUPPORT_FD
    memset(controller->dump_ctx.range_filter_configs, 0, sizeof(controller->dump_ctx.range_filter_configs));
#endif
    /* Parse twai's filters */
    ret = parse_filters(filter_str, controller->dump_ctx.mask_filter_configs, &mask_count
#if SOC_TWAI_SUPPORT_FD
                        , controller->dump_ctx.range_filter_configs, &range_count
#endif
                       );
    ESP_RETURN_ON_FALSE(ret == PARSE_OK, ret, TAG, "Failed(%d) to parse filter string", ret);

    /* Check if controller is initialized */
    if (!atomic_load(&controller->core_ctx.is_initialized)) {
        ESP_LOGE(TAG, "TWAI%d not initialized", (controller - s_twai_controller_ctx));
        return ESP_ERR_INVALID_STATE;
    }

    /* Configure filters */
    if (mask_count > 0 || range_count > 0) {
        /* Always disable and reconfigure to apply new filter settings */
        esp_err_t disable_ret = twai_node_disable(controller->node_handle);
        ESP_RETURN_ON_ERROR(disable_ret, TAG, "Failed to disable TWAI node%d for filter configuration: %s", controller_id, esp_err_to_name(disable_ret));

        for (int i = 0; i < mask_count; i++) {
            ret = twai_node_config_mask_filter(controller->node_handle, i,
                                               &controller->dump_ctx.mask_filter_configs[i]);
            ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Failed to configure mask filter %d", i);
            ESP_LOGD(TAG, "Configured mask filter %d: %08X : %08X", i,
                     controller->dump_ctx.mask_filter_configs[i].id,
                     controller->dump_ctx.mask_filter_configs[i].mask);
        }
#if SOC_TWAI_SUPPORT_FD
        for (int i = 0; i < range_count; i++) {
            ret = twai_node_config_range_filter(controller->node_handle, i,
                                                &controller->dump_ctx.range_filter_configs[i]);
            ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Failed to configure range filter %d", i);

            /* If no mask filter is configured, disable mask filter 0 which enabled by default */
            if (mask_count == 0) {
                twai_mask_filter_config_t mfilter_cfg = {
                    .id = 0xFFFFFFFF,
                    .mask = 0xFFFFFFFF,
                };
                esp_err_t mask_ret = twai_node_config_mask_filter(controller->node_handle, 0, &mfilter_cfg);
                ESP_RETURN_ON_ERROR(mask_ret, TAG, "Failed to configure node%d default mask filter: %s", controller_id, esp_err_to_name(mask_ret));
            }
            ESP_LOGD(TAG, "Configured range filter %d: %08X - %08X", i,
                     controller->dump_ctx.range_filter_configs[i].range_low,
                     controller->dump_ctx.range_filter_configs[i].range_high);
        }
#endif
        esp_err_t enable_ret = twai_node_enable(controller->node_handle);
        ESP_RETURN_ON_ERROR(enable_ret, TAG, "Failed to enable TWAI node%d after filter configuration: %s", controller_id, esp_err_to_name(enable_ret));
    }

    /* Parse timestamp mode */
    controller->dump_ctx.timestamp_mode = TIMESTAMP_MODE_NONE;
    if (twai_dump_args.timestamp->count > 0) {
        char mode = twai_dump_args.timestamp->sval[0][0];
        switch (mode) {
        case 'a': case 'd': case 'z': case 'n':
            controller->dump_ctx.timestamp_mode = (timestamp_mode_t)mode;
            break;
        default:
            ESP_LOGE(TAG, "Invalid timestamp mode: %c (use a/d/z/n)", mode);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* Initialize timestamp base time */
    int64_t current_time = esp_timer_get_time();
    controller->dump_ctx.start_time_us = current_time;
    controller->dump_ctx.last_frame_time_us = current_time;

    /* Start dump task and create resources */
    ret = twai_dump_start_controller(controller);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Failed to start dump task");

    return ESP_OK;
}

/**
 * @brief Stop dump and wait for task to exit naturally
 *
 * @param[in] controller_id Controller ID to stop dump for
 *
 * @return @c ESP_OK on success, error code on failure
 */
esp_err_t twai_dump_stop_internal(int controller_id)
{
    if (controller_id < 0 || controller_id >= SOC_TWAI_CONTROLLER_NUM) {
        ESP_LOGE(TAG, "Invalid controller ID: %d", controller_id);
        return ESP_ERR_INVALID_ARG;
    }

    twai_controller_ctx_t *controller = get_controller_by_id(controller_id);
    ESP_RETURN_ON_FALSE(controller != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid controller ID: %d", controller_id);
    twai_dump_ctx_t *dump_ctx = &controller->dump_ctx;

    if (!atomic_load(&dump_ctx->is_running)) {
        ESP_LOGD(TAG, "Dump not running for controller %d", controller_id);
        return ESP_OK;
    }

    /* Signal task to stop */
    atomic_store(&dump_ctx->is_running, false);
    ESP_LOGD(TAG, "Signaled dump task to stop for controller %d", controller_id);

    /* Wait for dump task to finish */
    int timeout_ms = CONFIG_EXAMPLE_DUMP_TASK_TIMEOUT_MS * 2;
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    ESP_RETURN_ON_FALSE(dump_ctx->dump_task_handle == NULL, ESP_ERR_TIMEOUT, TAG,
                        "Dump task did not exit naturally, timeout after %d ms", timeout_ms);

    return ESP_OK;
}

/**
 * @brief Register TWAI dump commands with console
 */
void register_twai_dump_commands(void)
{
    /* Initialize all controller dump modules */
    for (int i = 0; i < SOC_TWAI_CONTROLLER_NUM; i++) {
        twai_controller_ctx_t *controller = &s_twai_controller_ctx[i];
        esp_err_t ret = twai_dump_init_controller(controller);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize dump module for TWAI%d: %s", i, esp_err_to_name(ret));
        }
    }

    /* Register command */
    twai_dump_args.controller_filter = arg_str1(NULL, NULL, "<controller>[,filter]",
                                                "Controller ID and optional filters");
    twai_dump_args.stop = arg_lit0(NULL, "stop",
                                   "Stop monitoring the specified controller");
    twai_dump_args.timestamp = arg_str0("t", "timestamp", "<mode>",
                                        "Timestamp mode: a=absolute, d=delta, z=zero, n=none (default: n)");
    twai_dump_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "twai_dump",
        .help = "Monitor TWAI bus messages with timestamps\n"
        "Usage:\n"
        "  twai_dump [-t <mode>] <controller>[,filter...]\n"
        "  twai_dump <controller> --stop\n"
        "\n"
        "Options:\n"
        "  -t <mode>     Timestamp mode: a=absolute, d=delta, z=zero, n=none (default: n)\n"
        "  --stop        Stop monitoring the specified controller\n"
        "\n"
        "Filter formats:\n"
        "  id:mask       Mask filter (e.g., 123:7FF)\n"
#if SOC_TWAI_SUPPORT_FD
        "  low-high      Range filter (e.g., a-15)\n"
#endif
        "\n"
        "Examples:\n"
        "  twai_dump twai0                 # Monitor without timestamps (default)\n"
        "  twai_dump -t a twai0            # Monitor with absolute timestamps\n"
        "  twai_dump -t d twai0            # Monitor with delta timestamps\n"
        "  twai_dump -t n twai0,123:7FF    # Monitor ID 0x123 without timestamps\n"
#if SOC_TWAI_SUPPORT_FD
        "  twai_dump twai0,a-15            # Monitor range: [0xa, 0x15]\n"
        "  twai_dump twai0,123:7FF,a-15    # Mix mask and range filters\n"
        "  twai_dump twai0,000-666         # Monitor range: [0x000, 0x666]\n"
#endif
        "  twai_dump twai0 --stop          # Stop monitoring TWAI0\n"
        ,
        .hint = NULL,
        .func = &twai_dump_handler,
        .argtable = &twai_dump_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Unregister dump commands and cleanup resources
 */
void unregister_twai_dump_commands(void)
{
    /* Cleanup all controller dump modules */
    for (int i = 0; i < SOC_TWAI_CONTROLLER_NUM; i++) {
        twai_controller_ctx_t *controller = &s_twai_controller_ctx[i];
        twai_dump_deinit_controller(controller);
    }

    ESP_LOGI(TAG, "TWAI dump commands unregistered and resources cleaned up");
}
