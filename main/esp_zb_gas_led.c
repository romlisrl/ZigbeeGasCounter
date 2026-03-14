/*
 * Zigbee Gas Meter - An open-source Zigbee gas meter project.
 * Copyright (c) 2025 Ignacio Hernández-Ros.
 *
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. To view a copy of this license, visit
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * You may use, modify, and share this work for personal and non-commercial purposes, as long
 * as you credit the original author(s) and share any derivatives under the same license.
 */
#include "esp_check.h"
#include "driver/gpio.h"
#include "led_strip.h"

#include "esp_zb_gas_meter.h"
#include "esp_zb_gas_led.h"

static led_strip_handle_t led_strip;

typedef enum LedState_e {
	ON,
	OFF
} led_state_t;

led_state_t led_state;

/**
 * @brief Turn led on
 *
 */
void led_on()
{
	led_state = ON;
	if (led_strip) {
		led_strip_set_pixel(led_strip, 0, 50, 50, 50);
		led_strip_refresh(led_strip);
	}
}

/**
 * @brief Turn led off
 *
 */
void led_off()
{
	led_state = OFF;
	if (led_strip) {
		led_strip_clear(led_strip);
	}
}

/**
 * @brief true if the led is on
 *
 * @return true
 * @return false
 */
bool led_is_on() {
	return led_state == ON;
}

/**
 * @brief Configure the led pin
 *
 */
esp_err_t config_led()
{
	//uint64_t led_switch_pin = 1ULL << LED_PIN;
	led_strip_config_t strip_config = {
		.strip_gpio_num = LED_PIN,
		.max_leds = 1,
		//.led_model = LED_MODEL_WS2812, // Led model
		//.led_pixel_format = LED_PIXEL_FORMAT_GRB,
		//.color_order = LED_PIXEL_FORMAT_GRB, // color order
	};

	led_strip_rmt_config_t rmt_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 10 * 1000 * 1000, // 10MHz
		.flags = {
			.with_dma = false,
		}
	};

	// ESP_RETURN_ON_ERROR(gpio_config(&led_conf), TAG, "Failed to configure LED pin");
	// ESP_RETURN_ON_ERROR(gpio_set_level(LED_PIN, 1), TAG, "Failed to turn off led");

	return led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}
