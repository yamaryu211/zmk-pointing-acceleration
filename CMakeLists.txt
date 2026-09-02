if(CONFIG_ZMK_INPUT_PROCESSOR_ACCELERATION)
  zephyr_library_sources(src/input_processor_accel.c)
  zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dts/bindings)
endif()
