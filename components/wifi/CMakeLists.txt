idf_component_register(SRCS "src/wifi_ap_handler.c" "src/wifi_udp_comm.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi error-handling)
