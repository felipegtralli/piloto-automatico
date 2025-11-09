#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi_types_generic.h"
#include "esp_mac.h"
#include "wifi.h"

extern TaskHandle_t udp_task_handle;

static const char TAG[] = "wifi-ap";

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    wifi_ap_event_handler_ctx* ctx = arg;

    if(event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;

        ctx->station_connected = true;
        ESP_LOGI(TAG, "Station connected: MAC: "MACSTR", AID: %d", MAC2STR(event->mac), event->aid);

        vTaskResume(udp_task_handle);
    }

    if(event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;

        ctx->station_connected = false;
        ESP_LOGI(TAG, "Station disconnected: MAC: "MACSTR", AID: %d, REASON: %d", MAC2STR(event->mac), event->aid, event->reason);
    }
}
