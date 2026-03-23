if(CONFIG_ZMK_INPUT_AMS_AS5600)
    zephyr_library()
    zephyr_include_directories(./include)

    # External modules do not automatically get ZMK app headers.
    # Pull in app/include so zmk/*.h APIs are available to this driver.
    if(DEFINED APPLICATION_SOURCE_DIR AND EXISTS ${APPLICATION_SOURCE_DIR}/include/zmk/endpoints.h)
        zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
    else()
        message(WARNING "zmk-driver-ams-as5600: ZMK app headers not found under APPLICATION_SOURCE_DIR/include")
    endif()

    zephyr_library_sources(./src/zmk_input_ams_as5600.c)
endif()
