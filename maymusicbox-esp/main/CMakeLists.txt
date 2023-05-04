idf_component_register(SRCS "maymusicbox-esp.cc" "buttons.cc" "logging.cc" "audio_player.cc" "led.cc" "led_downmix.cc" "wake.cc"
                    INCLUDE_DIRS ".")

set(ulp_app_name ulp_button_wake)
set(ulp_s_sources ulp/button_wake.S)
set(ulp_exp_dep_srcs "buttons.cc" "wake.cc")
ulp_embed_binary(ulp_button_wake "${ulp_s_sources}" "${ulp_exp_dep_srcs}")


target_compile_options(${COMPONENT_LIB} PRIVATE -std=c++2a -Wno-missing-field-initializers)
