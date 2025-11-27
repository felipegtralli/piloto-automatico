idf_component_register(SRCS src/control_task.c src/udp_comm_task.c src/button_task.c src/udp_private/udp_priv.c
                    INCLUDE_DIRS "include"
                    REQUIRES esp-pid-control low-pass-filter motor wifi nvs-helper button error-handling driver)
