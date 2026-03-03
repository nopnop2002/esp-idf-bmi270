set(srcs "bmi270.c" "bmi270_dsd.c" "bmi270_context.c" "bmi270_maximum_fifo.c" "bmi270_legacy.c" "bmi2_ois.c" "bmi2.c" "bmi2_driver.c")

idf_component_register(
	SRCS "${srcs}"
	PRIV_REQUIRES driver esp_driver_i2c esp_timer
	INCLUDE_DIRS "."
)
