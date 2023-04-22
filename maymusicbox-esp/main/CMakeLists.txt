idf_component_register(SRCS "maymusicbox-esp.cc" "buttons.cc" "logging.cc" "audio_player.cc" "led.cc" "led_downmix.cc"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c++2a -Wno-missing-field-initializers)

