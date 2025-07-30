set(srcs "sf8xxx_nm_stub.cpp")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver esp_timer)
