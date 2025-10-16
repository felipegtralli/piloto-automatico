idf_component_register(SRCS src/control_task.c
                    INCLUDE_DIRS "include"
                    REQUIRES esp-pid-control low-pass-filter driver)
