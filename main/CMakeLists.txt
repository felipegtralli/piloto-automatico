idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

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