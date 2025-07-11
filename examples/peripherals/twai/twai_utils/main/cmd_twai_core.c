/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <inttypes.h>
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

/* Default configuration values */
#define DEFAULT_BITRATE             500000              // Safe default: 500 kbps

/* Default Mode Configuration */
#define DEFAULT_ENABLE_LOOPBACK         0       // Default: Normal mode
#define DEFAULT_ENABLE_SELF_TEST        0       // Default: Disabled
#define DEFAULT_ENABLE_LISTEN_ONLY      0       // Default: Disabled

static const char *TAG = "cmd_twai_core";

static struct {
    struct arg_int *controller;
    struct arg_int *rate;
    struct arg_str *loopback;
    struct arg_str *listen;
    struct arg_int *fd_rate;
    struct arg_end *end;
} twai_init_args;

static struct {
    struct arg_int *controller;
    struct arg_end *end;
} twai_deinit_args;

static struct {
    struct arg_int *controller;
    struct arg_end *end;
} twai_info_args;

static struct {
    struct arg_int *controller;
    struct arg_end *end;
} twai_reset_args;

static bool parse_bool(const char* arg)
{
    return strcasecmp(arg, "true") == 0 || strcmp(arg, "1") == 0;
}

/* Core TWAI operations */
static twai_node_handle_t twai_start(twai_core_ctx_t *ctx)
{
    twai_node_handle_t res = NULL;
    esp_err_t ret = ESP_OK;

    if (atomic_load(&ctx->is_initialized)) {
        ESP_LOGD(TAG, "TWAI driver is already running. Please stop it first.");
        return ctx->driver_handle;
    }

    if (ctx->driver_config.io_cfg.tx == GPIO_NUM_NC || ctx->driver_config.io_cfg.rx == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "TWAI TX or RX GPIO is not configured");
        return NULL;
    }

    // Create a new TWAI node with the current configuration
#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
    if (ctx->driver_config.data_timing.bitrate > 0) {
        if (ctx->driver_config.data_timing.bitrate < ctx->driver_config.bit_timing.bitrate) {
            ESP_LOGW(TAG, "TWAI-FD disabled: data bitrate (%" PRIu32 ") must be higher than arbitration bitrate (%" PRIu32 ")",
                     ctx->driver_config.data_timing.bitrate, ctx->driver_config.bit_timing.bitrate);
            ctx->driver_config.data_timing.bitrate = 0;  // Disable FD
        } else {
            ESP_LOGD(TAG, "TWAI-FD enabled: Arbitration=%" PRIu32 " bps, Data=%" PRIu32 " bps",
                     ctx->driver_config.bit_timing.bitrate, ctx->driver_config.data_timing.bitrate);
        }
    }
#endif

    ret = twai_new_node_onchip(&(ctx->driver_config), &(ctx->driver_handle));
    res = ctx->driver_handle;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = twai_node_register_event_callbacks(ctx->driver_handle, &(ctx->driver_cbs), NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register event callbacks: %s", esp_err_to_name(ret));
        goto cleanup_node;
    }

    ret = twai_node_enable(ctx->driver_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TWAI node: %s", esp_err_to_name(ret));
        goto cleanup_node;
    }

    atomic_store(&ctx->is_initialized, true);
    return res;

cleanup_node:
    if (ctx->driver_handle) {
        twai_node_delete(ctx->driver_handle);
        ctx->driver_handle = NULL;
    }
cleanup:
    return res;
}

static esp_err_t twai_stop(twai_core_ctx_t *ctx)
{

    if (!atomic_load(&ctx->is_initialized)) {
        ESP_LOGI(TAG, "TWAI not running");
        return ESP_OK;
    }

    // Disable and delete TWAI node first to stop callbacks
    if (ctx->driver_handle) {
        twai_node_disable(ctx->driver_handle);
        twai_node_delete(ctx->driver_handle);
        ctx->driver_handle = NULL;
    }

    atomic_store(&ctx->is_initialized, false);
    return ESP_OK;
}

/* =============================================================================
 * `twai-init` command
 * =============================================================================*/
static int twai_init_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&twai_init_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, twai_init_args.end, argv[0]);
        return ESP_FAIL;
    }

    if (twai_init_args.controller->count == 0) {
        ESP_LOGE(TAG, "Controller ID is required");
        return ESP_ERR_INVALID_ARG;
    }

    int controller_id = twai_init_args.controller->ival[0];

    twai_controller_ctx_t* controller = get_controller_by_id(controller_id);
    if (!controller) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_core_ctx_t *ctx = &controller->core_ctx;

    if (atomic_load(&ctx->is_initialized)) {
        ESP_LOGI(TAG, "TWAI driver is already running. Please stop it first.");
        return ESP_OK;
    }

    // Update configuration based on arguments
    if (twai_init_args.rate->count > 0) {
        ctx->driver_config.bit_timing.bitrate = twai_init_args.rate->ival[0];
        ESP_LOGI(TAG, "Set bitrate to %" PRIu32 " bps", ctx->driver_config.bit_timing.bitrate);
    }

    if (twai_init_args.loopback->count > 0) {
        ctx->driver_config.flags.enable_loopback = parse_bool(twai_init_args.loopback->sval[0]);
        ctx->driver_config.flags.enable_self_test = ctx->driver_config.flags.enable_loopback; // Self-test enabled in loopback mode
        ESP_LOGI(TAG, "Loopback mode: %s", ctx->driver_config.flags.enable_loopback ? "enabled" : "disabled");
    }

    if (twai_init_args.listen->count > 0) {
        ctx->driver_config.flags.enable_listen_only = parse_bool(twai_init_args.listen->sval[0]);
        ESP_LOGI(TAG, "Listen-only mode: %s", ctx->driver_config.flags.enable_listen_only ? "enabled" : "disabled");
    }

#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
    if (twai_init_args.fd_rate->count > 0) {
        ctx->driver_config.data_timing.bitrate = twai_init_args.fd_rate->ival[0];
    } else {
        // Use a default FD bitrate if not specified, but FD is enabled
        ctx->driver_config.data_timing.bitrate = DEFAULT_FD_DATA_BITRATE;
    }
#else
    ctx->driver_config.data_timing.bitrate = 0;  // Disable FD
#endif

    controller->node_handle = twai_start(ctx);
    if (controller->node_handle == NULL) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* =============================================================================
 * `twai-deinit` command
 * =============================================================================*/
static int twai_deinit_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&twai_deinit_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, twai_deinit_args.end, argv[0]);
        return ESP_FAIL;
    }

    if (twai_deinit_args.controller->count == 0) {
        ESP_LOGE(TAG, "Controller ID is required");
        return ESP_ERR_INVALID_ARG;
    }

    int controller_id = twai_deinit_args.controller->ival[0];

    twai_controller_ctx_t* controller = get_controller_by_id(controller_id);
    if (!controller) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_core_ctx_t *ctx = &controller->core_ctx;

    if (!atomic_load(&ctx->is_initialized)) {
        ESP_LOGI(TAG, "TWAI%d not running", controller_id);
        return ESP_OK;
    }

    esp_err_t ret = twai_stop(ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI%d: %s", controller_id, esp_err_to_name(ret));
        return ret;
    }

    controller->node_handle = NULL;
    return ESP_OK;
}

/* =============================================================================
 * `twai-info` command
 * =============================================================================*/
static int twai_info_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&twai_info_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, twai_info_args.end, argv[0]);
        return ESP_FAIL;
    }

    if (twai_info_args.controller->count == 0) {
        ESP_LOGE(TAG, "Controller ID is required");
        return ESP_ERR_INVALID_ARG;
    }

    int controller_id = twai_info_args.controller->ival[0];

    twai_controller_ctx_t* controller = get_controller_by_id(controller_id);
    if (!controller) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_core_ctx_t *ctx = &controller->core_ctx;

    printf("========== TWAI%d Information ==========\n", controller_id);
    printf("Status: %s\n", atomic_load(&ctx->is_initialized) ? "Running" : "Stopped");
    printf("Driver Handle: %p\n", controller->node_handle);

    printf("\n--- GPIO Configuration ---\n");
    printf("TX GPIO: %d\n", ctx->driver_config.io_cfg.tx);
    printf("RX GPIO: %d\n", ctx->driver_config.io_cfg.rx);
    printf("Clock Output GPIO: %d\n", ctx->driver_config.io_cfg.quanta_clk_out);
    printf("Bus Off Indicator GPIO: %d\n", ctx->driver_config.io_cfg.bus_off_indicator);

    printf("\n--- Timing Configuration ---\n");
    printf("Arbitration Bitrate: %" PRIu32 " bps\n", ctx->driver_config.bit_timing.bitrate);
    printf("Sample Point: %" PRIu16 " per mille\n", ctx->driver_config.bit_timing.sp_permill);
    printf("Secondary Sample Point: %" PRIu16 " per mille\n", ctx->driver_config.bit_timing.ssp_permill);

#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
    if (ctx->driver_config.data_timing.bitrate > 0) {
        printf("\n--- TWAI-FD Configuration ---\n");
        printf("Data Bitrate: %" PRIu32 " bps\n", ctx->driver_config.data_timing.bitrate);
        printf("Data Sample Point: %" PRIu16 " per mille\n", ctx->driver_config.data_timing.sp_permill);
        printf("Data Secondary Sample Point: %" PRIu16 " per mille\n", ctx->driver_config.data_timing.ssp_permill);
    }
#endif

    printf("\n--- Driver Configuration ---\n");
    printf("Clock Source: %d\n", ctx->driver_config.clk_src);
    printf("Fail Retry Count: %d\n", ctx->driver_config.fail_retry_cnt);
    printf("TX Queue Depth: %" PRIu32 "\n", ctx->driver_config.tx_queue_depth);
    printf("Interrupt Priority: %d\n", ctx->driver_config.intr_priority);

    printf("\n--- Mode Flags ---\n");
    printf("Self Test: %s\n", ctx->driver_config.flags.enable_self_test ? "Enabled" : "Disabled");
    printf("Loopback: %s\n", ctx->driver_config.flags.enable_loopback ? "Enabled" : "Disabled");
    printf("Listen Only: %s\n", ctx->driver_config.flags.enable_listen_only ? "Enabled" : "Disabled");
    printf("No Receive RTR: %s\n", ctx->driver_config.flags.no_receive_rtr ? "Enabled" : "Disabled");

    printf("=====================================\n");

    return ESP_OK;
}

/* =============================================================================
 * `twai-reset` command
 * =============================================================================*/
static int twai_reset_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&twai_reset_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, twai_reset_args.end, argv[0]);
        return ESP_FAIL;
    }

    if (twai_reset_args.controller->count == 0) {
        ESP_LOGE(TAG, "Controller ID is required");
        return ESP_ERR_INVALID_ARG;
    }

    int controller_id = twai_reset_args.controller->ival[0];

    twai_controller_ctx_t* controller = get_controller_by_id(controller_id);
    if (!controller) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_core_ctx_t *ctx = &controller->core_ctx;

    // Stop the driver if it's running
    bool was_running = atomic_load(&ctx->is_initialized);
    if (was_running) {
        ESP_LOGI(TAG, "Stopping TWAI%d for reset", controller_id);
        esp_err_t ret = twai_stop(ctx);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop TWAI%d: %s", controller_id, esp_err_to_name(ret));
            return ret;
        }
        controller->node_handle = NULL;
    }

    // Reset configuration to default
    ESP_LOGI(TAG, "Resetting TWAI%d configuration to default", controller_id);
    ctx->driver_config = ctx->default_config;

    // If it was running, restart it with default configuration
    if (was_running) {
        ESP_LOGI(TAG, "Restarting TWAI%d with default configuration", controller_id);
        controller->node_handle = twai_start(ctx);
        if (controller->node_handle == NULL) {
            ESP_LOGE(TAG, "Failed to restart TWAI%d after reset", controller_id);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "TWAI%d reset and restarted successfully", controller_id);
    } else {
        ESP_LOGI(TAG, "TWAI%d reset to default configuration (not running)", controller_id);
    }

    return ESP_OK;
}

/* =============================================================================
 * COMMAND REGISTRATION
 * =============================================================================*/
void register_twai_core_commands(void)
{
    // Initialize all controllers
    for (int i = 0; i < SOC_TWAI_CONTROLLER_NUM; i++) {
        twai_controller_ctx_t* controller = &s_twai_controller_ctx[i];
        twai_core_ctx_t *ctx = &controller->core_ctx;

        ctx->driver_config = (twai_onchip_node_config_t) {
            .io_cfg = {
                .tx = GPIO_NUM_NC,
                .rx = GPIO_NUM_NC,
                .quanta_clk_out = -1,
                .bus_off_indicator = -1,
            },
            .clk_src = 0,
            .bit_timing = {
                .bitrate = DEFAULT_BITRATE,
                .sp_permill = 0,
                .ssp_permill = 0,
            },
            .data_timing = {
#if CONFIG_EXAMPLE_ENABLE_TWAI_FD
                .bitrate = DEFAULT_FD_DATA_BITRATE,
                .sp_permill = 0,
                .ssp_permill = 700,
#else
                .bitrate = 0,
                .sp_permill = 0,
                .ssp_permill = 0,
#endif
            },
            .fail_retry_cnt = -1,
            .tx_queue_depth = CONFIG_EXAMPLE_TX_QUEUE_LEN,
            .intr_priority = 0,
            .flags = {
                .enable_self_test = DEFAULT_ENABLE_SELF_TEST,
                .enable_loopback = DEFAULT_ENABLE_LOOPBACK,
                .enable_listen_only = DEFAULT_ENABLE_LISTEN_ONLY,
                .no_receive_rtr = 0,
            },
        };

        // Only the first controller's gpio is defaultly configured
        if (i == 0) {
            ctx->driver_config.io_cfg.tx = CONFIG_EXAMPLE_TX_GPIO_NUM;
            ctx->driver_config.io_cfg.rx = CONFIG_EXAMPLE_RX_GPIO_NUM;
        }

        // Store as default config for reset functionality
        ctx->default_config = ctx->driver_config;

        // Initialize atomic flag
        atomic_store(&ctx->is_initialized, false);

        ESP_LOGI(TAG, "Initialized TWAI%d with TX=%d, RX=%d",
                 i, ctx->driver_config.io_cfg.tx, ctx->driver_config.io_cfg.rx);
    }

    // Register command arguments
    twai_init_args.controller = arg_int1(NULL, NULL, "<controller_id>", "TWAI controller ID (0 or 1)");
    twai_init_args.rate = arg_int0("r", "rate", "<bitrate>", "Set arbitration bitrate (bps)");
    twai_init_args.loopback = arg_str0("l", "loopback", "<true|false>", "Enable loopback mode");
    twai_init_args.listen = arg_str0("L", "listen", "<true|false>", "Enable listen-only mode");
    twai_init_args.fd_rate = arg_int0("f", "fd-rate", "<bitrate>", "Set data bitrate for TWAI-FD (bps)");
    twai_init_args.end = arg_end(20);

    twai_deinit_args.controller = arg_int1(NULL, NULL, "<controller_id>", "TWAI controller ID (0 or 1)");
    twai_deinit_args.end = arg_end(20);

    twai_info_args.controller = arg_int1(NULL, NULL, "<controller_id>", "TWAI controller ID (0 or 1)");
    twai_info_args.end = arg_end(20);

    twai_reset_args.controller = arg_int1(NULL, NULL, "<controller_id>", "TWAI controller ID (0 or 1)");
    twai_reset_args.end = arg_end(20);

    /* Register commands */
    const esp_console_cmd_t twai_init_cmd = {
        .command = "twai_init",
        .help = "Initialize and start the TWAI driver",
        .hint = "<controller_id> [-r bitrate] [-l loopback] [-L listen] [-f fd-rate]",
        .func = &twai_init_handler,
        .argtable = &twai_init_args
    };

    const esp_console_cmd_t twai_deinit_cmd = {
        .command = "twai_deinit",
        .help = "Stop and de-initialize the TWAI driver",
        .hint = "<controller_id>",
        .func = &twai_deinit_handler,
        .argtable = &twai_deinit_args
    };

    const esp_console_cmd_t twai_info_cmd = {
        .command = "twai_info",
        .help = "Display TWAI controller information and status",
        .hint = "<controller_id>",
        .func = &twai_info_handler,
        .argtable = &twai_info_args
    };

    const esp_console_cmd_t twai_reset_cmd = {
        .command = "twai_reset",
        .help = "Reset the TWAI driver to default configuration",
        .hint = "<controller_id>",
        .func = &twai_reset_handler,
        .argtable = &twai_reset_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&twai_init_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&twai_deinit_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&twai_info_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&twai_reset_cmd));
}
