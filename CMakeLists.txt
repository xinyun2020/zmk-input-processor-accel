if(CONFIG_ZMK_INPUT_PROCESSOR_SIGMOID_ACCEL)
  target_sources(app PRIVATE src/input_processors/sigmoid_accel.c)
endif()
