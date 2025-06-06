idf_component_register(SRCS "sensor_iface.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)

target_link_libraries(${COMPONENT_LIB} INTERFACE m) 