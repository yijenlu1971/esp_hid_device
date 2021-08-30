idf_component_register(SRCS "ble_hidd_demo_main.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    INCLUDE_DIRS "${IDF_PATH}/components/bt/include"
                                "${IDF_PATH}/components/bt/host/bluedroid/api/include/api"
                                "${IDF_PATH}/components/nvs_flash/include"
                                "${IDF_PATH}/components/spi_flash/include"
                    REQUIRES esp_adc_cal)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
