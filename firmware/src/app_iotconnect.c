/*******************************************************************************
  IoTConnect application task

  File Name:
    app_iotconnect.c

  Summary:
    FreeRTOS task that runs iotconnect_sdk(): HTTPS discovery, HTTPS identity,
    MQTT connect and periodic sensor telemetry.

  Description:
    The task waits for Wi-Fi, an IP address and NTP time, works out the device
    unique ID, then hands over to iotconnect_sdk().  That call only returns
    when the connection is lost or a publish fails, so the task simply retries
    after a delay -- the whole discovery/identity/connect sequence is repeated,
    which also picks up a broker change made on the IoTConnect side.

    The telemetry set matches the one APP_AWS_Tasks() publishes in app_aws.c.
*******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "definitions.h"
#include "FreeRTOS.h"
#include "task.h"

#include "iotc_config.h"
#include "app_common.h"
#include "app.h"
#include "app_aws.h"
#include "ws2812_spi.h"
#include "app_device.h"
#include "app_usb_msd.h"

#include "iot_init.h"
#include "iot_mqtt.h"

#include "iotcl.h"
#include "iotcl_telemetry.h"
#include "iotcl_c2d.h"
#include "sensors.h"

#include "iotc_https.h"
#include "iotconnect.h"
#include "app_iotconnect.h"

// *****************************************************************************

extern APP_DEVICE_DATA app_deviceData;
/* Defined in app_aws.c, filled from the USB MSD cloud config file. */
extern char g_Aws_ClientID[CLIENT_IDENTIFIER_MAX_LENGTH];

/** Device unique ID handed to IoTConnect; resolved once at startup. */
static char s_duid[CLIENT_IDENTIFIER_MAX_LENGTH];

// *****************************************************************************
// Section: IoTConnect callbacks
// *****************************************************************************

/* Fills every telemetry message.  Same value set as APP_AWS_Tasks(). */
static void app_iotc_telemetry_cb(IotclMessageHandle msg)
{
    /* app_ps.c drives the power-save decisions off this flag, and app_aws.c
     * is no longer running to maintain it.  A publish about to go out is the
     * one point where the connection is known to be up. */
    MQTT_CONNECTED;

    //iotcl_telemetry_set_number(msg, "WFI32IoT_button1", app_deviceData.switch1Status);
    iotcl_telemetry_set_number(msg, "WFI32IoT_button1_count", app_deviceData.switch1Cnt);
    //iotcl_telemetry_set_number(msg, "WFI32IoT_button2", app_deviceData.switch2Status);
    iotcl_telemetry_set_number(msg, "WFI32IoT_button2_count", app_deviceData.switch2Cnt);
    iotcl_telemetry_set_number(msg, "Onboard_Light_Lux", APP_readLight());
    iotcl_telemetry_set_number(msg, "Onboard_Temp_DegC", APP_readTemp());
    iotcl_telemetry_set_string(msg, "LED_Blue",  app_deviceData.LED_Blue  ? "On" : "Off");
    iotcl_telemetry_set_string(msg, "LED_Green", app_deviceData.LED_Green ? "On" : "Off");
    iotcl_telemetry_set_string(msg, "LED_Red",   app_deviceData.LED_Red   ? "On" : "Off");

    /* Values of the mikroBUS click boards, read by APP_DEVICE_Tasks(). */
    add_sensor_data_to_telemetry(msg);
}

/* Handles C2D commands.  Same command set as IoTC_CMD() in app_aws.c. */
static void app_iotc_command_cb(IotclC2dEventData data)
{
    const char *ack_id  = iotcl_c2d_get_ack_id(data);
    const char *command = iotcl_c2d_get_command(data);
    bool        handled = true;

    if (command == NULL) {
        return;
    }

    if (0 == strcmp(command, "led-green on")) {
        APP_IOTC_PRNT("LED GREEN: ON\r\n");
        app_deviceData.LED_Green = true;
    } else if (0 == strcmp(command, "led-green off")) {
        APP_IOTC_PRNT("LED GREEN: OFF\r\n");
        app_deviceData.LED_Green = false;
    } else if (0 == strcmp(command, "led-blue on")) {
        APP_IOTC_PRNT("LED BLUE: ON\r\n");
        app_deviceData.LED_Blue = true;
    } else if (0 == strcmp(command, "led-blue off")) {
        APP_IOTC_PRNT("LED BLUE: OFF\r\n");
        app_deviceData.LED_Blue = false;
    } else if (0 == strcmp(command, "led-red on")) {
        APP_IOTC_PRNT("LED RED: ON\r\n");
        app_deviceData.LED_Red = true;
    } else if (0 == strcmp(command, "led-red off")) {
        APP_IOTC_PRNT("LED RED: OFF\r\n");
        app_deviceData.LED_Red = false;
    } else if (WS2812_HandleCommand(command, &handled)) {
        /* "strip fill/pixel/brightness/off/run/stop/cycle ..." -> WS2812B strip (ws2812_spi.h) */
        APP_IOTC_PRNT("LED STRIP: %s (%s)\r\n", command, handled ? "ok" : "failed");
    } else {
        APP_IOTC_PRNT("Unknown COMMAND: %s\r\n", command);
        handled = false;
    }

    if (ack_id != NULL) {
        (void)iotcl_mqtt_send_cmd_ack(ack_id,
                                      handled ? IOTCL_C2D_EVT_CMD_SUCCESS_WITH_ACK
                                              : IOTCL_C2D_EVT_CMD_FAILED,
                                      handled ? NULL : "unknown command");
    }

    if (handled) {
        /* Report the new LED state right away instead of at the next tick. */
        iotc_request_publish();
    }
}

/* Inbound MQTT message on the C2D topic. */
static void app_iotc_mqtt_cb(void *pContext, IotMqttCallbackParam_t *const pPublish)
{
    (void)pContext;

    if (pPublish == NULL || pPublish->u.message.info.pPayload == NULL) {
        return;
    }

    /* The payload is not NUL-terminated, so pass it with its length. */
    (void)iotcl_mqtt_receive_c2d_with_length(
              (const uint8_t *)pPublish->u.message.info.pPayload,
              pPublish->u.message.info.payloadLength);
}

// *****************************************************************************
// Section: Helpers
// *****************************************************************************

/**
 * Work out the device unique ID, in order of preference:
 *   1. APP_IOTC_DUID, when defined at build time.
 *   2. The ClientID from the cloud config file on the USB MSD volume.
 *   3. The ECC608 serial number, which is what the Trust&Go certificate is
 *      issued to.
 *
 * @return true when an ID could be determined.
 */
static bool app_iotc_resolve_duid(void)
{
#ifdef APP_IOTC_DUID
    snprintf(s_duid, sizeof(s_duid), "%s", APP_IOTC_DUID);
#else
    if (g_Aws_ClientID[0] != '\0') {
        snprintf(s_duid, sizeof(s_duid), "%s", g_Aws_ClientID);
    } else if (appUSBMSDData.ecc608SerialNum[0] != '\0') {
        snprintf(s_duid, sizeof(s_duid), "%s", appUSBMSDData.ecc608SerialNum);
    } else {
        s_duid[0] = '\0';
    }
#endif

    if (s_duid[0] == '\0') {
        APP_IOTC_DBG(SYS_ERROR_ERROR, "no device ID available -- define "
                     "APP_IOTC_DUID or provide the cloud config file\r\n");
        return false;
    }

    APP_IOTC_PRNT("device ID: %s\r\n", s_duid);
    return true;
}

/** Block until the Wi-Fi link, the IP address and the clock are all ready. */
static void app_iotc_wait_for_network(void)
{
    bool reported = false;

    while (!(WIFI_IS_CONNECTED && IP_ADDR_IS_OBTAINED && NTP_IS_DONE)) {
        if (!reported) {
            APP_IOTC_PRNT("waiting for Wi-Fi, IP address and NTP time...\r\n");
            reported = true;
        }
        vTaskDelay(500U / portTICK_PERIOD_MS);
    }
}

#if APP_IOTC_PARK_APP_AWS
/**
 * APP_AWS_Tasks() and this task must not both hold an MQTT connection: they
 * would share one broker endpoint global (g_Cloud_Endpoint, which the wolfSSL
 * glue uses for SNI) and register two clients with the same client ID.
 *
 * APP_AWS_Tasks() also owns the one-time IotSdk_Init() / IotMqtt_Init(), so
 * let it finish that first, then park it in its idle state.
 */
static void app_iotc_park_app_aws(void)
{
    uint32_t waited_ms = 0U;

    while (appAwsData.awsCloudTaskState == APP_AWS_CLOUD_SDK_INIT) {
        vTaskDelay(10U / portTICK_PERIOD_MS);
        waited_ms += 10U;
        if (waited_ms >= 5000U) {
            APP_IOTC_DBG(SYS_ERROR_ERROR, "APP_AWS_Tasks did not finish its "
                         "SDK init\r\n");
            return;
        }
    }

    appAwsData.awsCloudTaskState = APP_AWS_CLOUD_IDLE;
    APP_IOTC_PRNT("APP_AWS_Tasks parked; IoTConnect owns the MQTT "
                  "connection\r\n");
}
#endif /* APP_IOTC_PARK_APP_AWS */

// *****************************************************************************
// Section: Task
// *****************************************************************************

/**
 * One-time initialisation of the AWS SDK and its MQTT library.  This used to
 * be done by APP_AWS_Tasks() in its APP_AWS_CLOUD_SDK_INIT state; that task
 * has been replaced by this one in tasks.c, so it happens here instead.
 *
 * @return true on success.
 */
static bool app_iotc_sdk_init(void)
{
    IotMqttError_t mqttStatus;

    if (!IotSdk_Init()) {
        APP_IOTC_DBG(SYS_ERROR_ERROR, "IotSdk_Init failed\r\n");
        return false;
    }

    mqttStatus = IotMqtt_Init();
    if (mqttStatus != IOT_MQTT_SUCCESS) {
        APP_IOTC_DBG(SYS_ERROR_ERROR, "IotMqtt_Init failed (%s)\r\n",
                     IotMqtt_strerror(mqttStatus));
        return false;
    }

    return true;
}

void APP_IOTC_Tasks(void)
{
    IotConnectClientConfig config;
    int status;

#if APP_IOTC_PARK_APP_AWS
    app_iotc_park_app_aws();
#endif

    if (!app_iotc_sdk_init()) {
        vTaskSuspend(NULL);
    }

    app_iotc_wait_for_network();

    if (!app_iotc_resolve_duid()) {
        /* Nothing to connect as; stop here rather than retry forever. */
        vTaskSuspend(NULL);
    }

    status = iotc_http_client_init();
    if (status != 0) {
        APP_IOTC_DBG(SYS_ERROR_ERROR, "HTTP client init failed (%d)\r\n", status);
        vTaskSuspend(NULL);
    }

    memset(&config, 0, sizeof(config));
    config.env                = APP_IOTC_ENV;
    config.cpid               = APP_IOTC_CPID;
    config.duid               = s_duid;
    config.connection_type    = IOTC_CT_AWS;
    config.event_callback     = app_iotc_mqtt_cb;
    config.cmd_cb             = app_iotc_command_cb;
    config.telemetry_cb       = app_iotc_telemetry_cb;
    config.telemetry_count    = IOTC_TELEMETRY_UNLIMITED;
    config.telemetry_delay_ms = APP_IOTC_PUBLISH_PERIOD_MS;

    for (;;) {
        /* The link may have dropped while the previous attempt was running. */
        app_iotc_wait_for_network();

        APP_IOTC_PRNT("starting IoTConnect (env=%s, cpid=%s)\r\n",
                      config.env, config.cpid);

        /* Runs discovery, identity, MQTT connect and the publish loop.  It
         * only returns when the connection is lost or a step fails. */
        status = iotconnect_sdk(&config);
        MQTT_DISCONNECTED;

        APP_IOTC_DBG(SYS_ERROR_ERROR, "IoTConnect stopped (%d); retrying in "
                     "%lu ms\r\n", status, (unsigned long)APP_IOTC_RETRY_DELAY_MS);

        vTaskDelay(APP_IOTC_RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
}

// *****************************************************************************

void APP_IOTC_Initialize(void)
{
    memset(s_duid, 0, sizeof(s_duid));
    MQTT_DISCONNECTED;
}

// *****************************************************************************

bool APP_IOTC_IsConnected(void)
{
    return iotc_mqtt_is_connected();
}

// *****************************************************************************

void APP_IOTC_RequestPublish(void)
{
    iotc_request_publish();
}

/*******************************************************************************
 End of File
 */
