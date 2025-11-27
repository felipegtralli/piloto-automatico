idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp-pid-control low-pass-filter tasks driver nvs-helper motor button nvs_flash wifi lrgb esp_wifi)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wformat
    -Wformat-security
    -Wdouble-promotion
    -Wstrict-prototypes
    -Wmissing-prototypes
)