/**************************************************

Author: Ashwin Vinoo
Date: 8/27/2019
Project: Robot Mediated Driving Practice
Description: This code is developed to control the reward base and all its peripherals

Note: Work performed under the guidance of Prof. Naomi Fitter at Oregon State University

**************************************************/

//--------------- The libraries used within the program ---------------
#include <FastLED.h>
#include <NewPing.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Open_Sans_Light_13.h>
#include <Fonts/Roboto_Condensed_Light_13.h>
#include <Fonts/Open_Sans_Light_15.h>

//--------------- Hyperparameters ---------------

// The number of pixels in the 8x32 led panel
const word led_panel_pixel_count = 256;
// The number of pixel columns in the 8x32 led panel
const byte led_panel_pixel_columns = 32;
// The number of pixel columns in the 8x32 led panel
const byte led_panel_pixel_rows = 8;
// The maximum brightness of the LED panel
const byte led_panel_max_brightness = 100;
// The maximum number of modes for the light reward
const byte light_max_modes = 6;
// The maximum duration of the light reward
const word light_max_duration = 60000;
// The minimum duration of the light reward
const word light_min_duration = 500;
// The maximum frequency of the light reward
const byte light_max_frequency = 200;
// The minimum frequency of the light reward
const byte light_min_frequency = 1;
// The width of the led panel beacon mode
const byte led_beacon_width = 5;
// The time period after which a finished light reward should be restarted during configuration
const word light_revival_time = 2000;
// The maximum number of times the sound reward can be repeated
const byte sound_max_repeats = 10;
// The minimum time in milliseconds before the sound gets clipped
const word sound_min_clip_time = 500;
// The maximum time in milliseconds before the sound gets clipped
const word sound_max_clip_time = 10000;
// The gap between subsequent sound reward calls in milliseconds
const byte sound_maintenance_gap = 25;
// The time period after which a finished sound reward should be restarted during configuration
const word sound_revival_time = 2000;
// The maximum duration for the bubble reward
const word bubble_max_duration = 60000;
// The minimum duration for the bubble reward
const word bubble_min_duration = 1000;
// The minimum analog motor control value
const byte bubble_min_motor_control = 100;
// The time in seconds between bubble reward refreshes (milliseconds)
const unsigned long bubble_refresh_interval = 30000;
// The duration for which the bubble refresh process is conducted (milliseconds)
const word bubble_refresh_duration = 7000;
// The radial fan speed control
const byte bubble_fan_control = 255;
// The time period after which a finished bubble reward should be restarted during configuration
const word bubble_revival_time = 2000;
// The maximum distance that we plan to detect using the HCSR-04 Sensor (Rated Maximum is 400cm to 500cm)
const word ultrasonic_max_distance = 300;
// The sampling interval for the 8 ultrasonic sensors (milliseconds)
const byte ultrasonic_sampling_interval = 500;
// The KY-040 clock threshold time in milliseconds
const byte ky040_switch_threshold_time = 5;
// The KY-040 clock threshold time in microseconds
const word ky040_clock_threshold_time = 300;
// This variable hold the duration of a single flash cycle
const word reward_activation_flash_duration = 600;

//--------------- The pins used to control other electronics are specified ---------------

// The data pin from where program the LED panel
const byte led_panel_data_in = 26;

// The shutdown pins of the two mono audio amplifiers
const byte left_audio_amplifier_shutdown = 24;
const byte right_audio_amplifier_shutdown = 25;

// The enbale pins of the l298N motor driver
const byte motor_driver_enable_A = 22;
const byte motor_driver_enable_B = 23;

// The 4 input pins of the l298N based motor driver board
const byte motor_driver_input_1 = 12;
const byte motor_driver_input_2 = 13;
const byte motor_driver_input_3 = 10;
const byte motor_driver_input_4 = 11;

// The analog read pin of the add on reward
const byte add_on_analog_code = A14;

// The trigger and echo pins of the ultrasonic sensors used
const byte ultrasonic_1_trigger = 42;
const byte ultrasonic_1_echo = 43;
const byte ultrasonic_2_trigger = 40;
const byte ultrasonic_2_echo = 41;
const byte ultrasonic_3_trigger = 38;
const byte ultrasonic_3_echo = 39;
const byte ultrasonic_4_trigger = 36;
const byte ultrasonic_4_echo = 37;
const byte ultrasonic_5_trigger = 34;
const byte ultrasonic_5_echo = 35;
const byte ultrasonic_6_trigger = 32;
const byte ultrasonic_6_echo = 33;
const byte ultrasonic_7_trigger = 30;
const byte ultrasonic_7_echo = 31;
const byte ultrasonic_8_trigger = 28;
const byte ultrasonic_8_echo = 29;

// The data pins of the IR sensor modules
const byte infrared_1_data_in = 2;
const byte infrared_2_data_in = 3;

// The KY-040 rotary encoder pins are specified
const byte rotary_encoder_switch = 18;
const byte rotary_encoder_clock = 19;
const byte rotary_encoder_data = 15;

//--------------- Splash Screen Bitmap Data ---------------

// Stores the splash screen bitmap image to display on turning on the device
const static PROGMEM unsigned char  splash_screen_image [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xF8, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x3E, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x0E, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC2, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00,
0x00, 0x3F, 0x11, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x70, 0x00, 0x00,
0x00, 0x70, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xE0, 0x70, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00,
0x01, 0xE0, 0x30, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00,
0x01, 0xE0, 0x30, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xC0, 0x00, 0x00,
0x01, 0xF0, 0x10, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xC0, 0x00, 0x00,
0x01, 0xF8, 0x10, 0xF3, 0xC0, 0x3F, 0x07, 0xCE, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00,
0x00, 0xFE, 0x00, 0xF7, 0xE0, 0xC3, 0x83, 0xDF, 0x39, 0xC0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00,
0x00, 0xFF, 0x00, 0xF8, 0xF1, 0xE3, 0xC3, 0xEF, 0x70, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
0x00, 0x7F, 0xC0, 0xF0, 0xF1, 0xE3, 0xC3, 0xE6, 0x70, 0xE0, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
0x00, 0x1F, 0xE0, 0xF0, 0xF0, 0xE3, 0xC3, 0xC0, 0xF0, 0xE0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x07, 0xF0, 0xF0, 0xF0, 0x07, 0xC3, 0xC0, 0xF0, 0xE0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x03, 0xF8, 0xF0, 0xF0, 0x1B, 0xC3, 0xC0, 0xFF, 0xE0, 0x03, 0x06, 0x00, 0x01, 0x83, 0x00,
0x01, 0x00, 0xF8, 0xF0, 0xF0, 0x73, 0xC3, 0xC0, 0xF0, 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x00,
0x01, 0x00, 0x78, 0xF0, 0xF0, 0xE3, 0xC3, 0xC0, 0xF0, 0x00, 0x03, 0x33, 0xFF, 0xFF, 0x33, 0x00,
0x01, 0x80, 0x78, 0xF0, 0xF1, 0xE3, 0xC3, 0xC0, 0xF0, 0x00, 0x03, 0x30, 0xFF, 0xFC, 0x33, 0x00,
0x01, 0x80, 0x70, 0xF0, 0xF1, 0xE3, 0xC3, 0xC0, 0x78, 0x20, 0x0F, 0x00, 0x00, 0x00, 0x03, 0xC0,
0x01, 0xC0, 0x70, 0xF0, 0xF1, 0xE7, 0xC3, 0xC0, 0x7C, 0x40, 0x3F, 0x00, 0x00, 0x00, 0x03, 0xF0,
0x01, 0xF0, 0xE0, 0xF0, 0xF1, 0xFB, 0xF3, 0xC0, 0x3F, 0x80, 0x73, 0x07, 0x80, 0x07, 0x83, 0x38,
0x01, 0x1F, 0x81, 0xF9, 0xF8, 0xF1, 0xC7, 0xE0, 0x1F, 0x00, 0x63, 0x0F, 0xC0, 0x0F, 0xC3, 0x18,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0x1C, 0xE0, 0x1C, 0xE3, 0x0C,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0x18, 0x60, 0x18, 0x63, 0x0C,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0x18, 0x60, 0x18, 0x63, 0x0C,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0x1C, 0xE0, 0x1C, 0xE3, 0x0C,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x0F, 0xFF, 0xFF, 0xC3, 0x18,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x07, 0xBF, 0xF7, 0x83, 0x38,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x30, 0x30, 0x03, 0xF0,
0x00, 0x01, 0xFF, 0x80, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x30, 0x30, 0x1F, 0xC0,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x03, 0xF8, 0x30, 0x30, 0x7F, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x03, 0x18, 0x18, 0x60, 0x63, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x03, 0x0C, 0x0F, 0xC0, 0xC3, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x03, 0x0C, 0x07, 0x80, 0xC3, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x03, 0xF0, 0x1E, 0x78, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x0C, 0x38, 0x1F, 0xFC, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01, 0x80, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x1E, 0x3C, 0x1F, 0x1E, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01, 0x80, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x1E, 0x3C, 0x1E, 0x0E, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
0x00, 0x00, 0x3C, 0x00, 0x0E, 0x3C, 0x1E, 0x0F, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x7C, 0x1E, 0x0F, 0x00, 0x00, 0x66, 0x60, 0x00, 0x00, 0x19, 0x98,
0x00, 0x00, 0x3C, 0x00, 0x01, 0xBC, 0x1E, 0x0F, 0x00, 0x00, 0x66, 0x60, 0x00, 0x00, 0x19, 0x98,
0x00, 0x00, 0x3C, 0x00, 0x87, 0x3C, 0x1E, 0x0F, 0x00, 0x00, 0x66, 0x60, 0x33, 0x30, 0x19, 0x98,
0x00, 0x00, 0x3C, 0x00, 0x8E, 0x3C, 0x1E, 0x0F, 0x00, 0x00, 0x66, 0x60, 0x33, 0x30, 0x19, 0x98,
0x00, 0x00, 0x3C, 0x01, 0x9E, 0x3C, 0x1E, 0x0F, 0x00, 0x00, 0x66, 0x60, 0x00, 0x00, 0x19, 0x98,
0x00, 0x00, 0x3C, 0x01, 0x9E, 0x3C, 0x1E, 0x0E, 0x00, 0x00, 0x66, 0x60, 0x00, 0x00, 0x19, 0x98,
0x00, 0x00, 0x3C, 0x03, 0x1E, 0x7C, 0x1E, 0x1C, 0x00, 0x00, 0x7F, 0xE7, 0xFF, 0xFF, 0x9F, 0xF8,
0x00, 0x00, 0x3C, 0x07, 0x1F, 0xBF, 0x1F, 0x1C, 0x00, 0x00, 0x7F, 0xE7, 0xFF, 0xFF, 0x9F, 0xF8,
0x00, 0x01, 0xFF, 0xFF, 0x0F, 0x1C, 0x19, 0xF0, 0x00, 0x00, 0x66, 0x66, 0x00, 0x01, 0x99, 0x98,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x00, 0x01, 0x99, 0x98,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//--------------- Sound reward track and album information ---------------

// The number of sound albums
const byte sound_album_count = 4;
// The number of sound tracks in each album
const byte sound_track_count[] = {30, 30, 125, 25};

//--------------- Reading previously stored settings from the EEPROM ---------------

// ----- Reward activation settings -----

// Each bit can be toggled to activate a particular reward type
byte reward_activation_status = EEPROM.read(0);

// ----- Light reward settings ----- 

// The mode used by the light reward (Still - 0, Pulse - 1, Fade - 2, Beacon - 3, Wipe - 4, Cross - 5)
byte light_mode = EEPROM.read(1);
// Frequency at which light reward pulses or fades (light_min_frequency to light_max_frequency)
byte light_frequency = EEPROM.read(2);
// The light color as specified by the R, G and B channels (0 - 255)
byte light_r_channel = EEPROM.read(3);
byte light_g_channel = EEPROM.read(4);
byte light_b_channel = EEPROM.read(5);
// The maximum brightness of the led panel (0 - led_panel_max_brightness)
byte light_brightness = EEPROM.read(6);
// The duration for which this reward is provided (light_min_duration - light_max_duration) (milliseconds)
word light_duration = 256*EEPROM.read(8) + EEPROM.read(7);

// ----- Sound reward settings ----- 

// The album choice for the sound reward (Animals - 1, Daily - 2, Musical - 3, Baby - 4)
byte sound_album = EEPROM.read(9);
// The track choice for the sound reward (0 - variable)
byte sound_track = EEPROM.read(10);
// The loudness of the sound reward (0 - 30)
byte sound_loudness = EEPROM.read(11);
// The number of times the sound is played (1 - sound_max_repeats)
byte sound_repetition = EEPROM.read(12);
// The time after which the sound is clipped (sound_min_clip_time - sound_max_clip_time) (milliseconds)
word sound_clip_time = 100 * EEPROM.read(13);

// ----- Bubble reward settings -----

// The duration for which this reward is provided (bubble_min_duration - bubble_max_duration) (milliseconds)
word bubble_duration = 256*EEPROM.read(15) + EEPROM.read(14);
// The geared motor speed control (bubble_min_motor_control - 255)
byte bubble_motor_control = EEPROM.read(16);

// ----- Generic Settings -----

// Indicates whether the rewards should be displayed during configuration (0 - don't show, 1 - show)
byte reward_configuration_display = EEPROM.read(17);

//--------------- Variables ---------------

//----- Light reward -----

// Indicates the light reward state
byte light_state = 4;
// This flag is raised if we want to update the static light variables in its function
bool light_update = true;
// Creating an array to store the RGB colors of each pixel in the led panel
CRGB led_panel[led_panel_pixel_count];
// The current preset color option
byte light_preset_color_option = 0;
// This flag indicates whether we are dealing with a preset color (true) or a custom color (false)
bool light_preset_color = false;

//----- Sound reward -----

// Indicates the sound reward state
byte sound_state = 4;
// This variable retains memory of sound tracks selected for each album
byte sound_track_memory[sound_album_count] = {0, 0, 0, 0};

//----- Add on reward -----

// Inidcates the type of add on module currently installed
byte add_on_type = 0;
// Indicates the add on state
byte add_on_state = 4;

//----- Common reward -----

// This flag indicates whether a reward is being configured or not
bool reward_configuration = false;
// This variable indicates whether its the first time that we saw an inactive state in the configured reward
bool reward_configuration_end_unnoticed = true;
// This flag helps control the reward activation flash on the main screen
bool reward_activation_flash_flag = false;
// This variable holds the timestamp that can toggle the reward activation flashes
unsigned long reward_activation_flash_refresh_time = 0;
// We use this flag to help regulate flashing of the reward activation icons
bool reward_activation_flash_extra_flag = false;

//----- OLED Display -----

// The screen width of the OLED display in pixels
const byte oled_screen_width = 128;
// The screen height of the OLED display in pixels
const byte oled_screen_height = 64;
// The width of the splash screen image on the OLED display
const byte oled_splash_screen_width = 128;
// The height of the splash screen image on the OLED display
const byte oled_splash_screen_height = 64;

// Used to store the action to take on the OLED screen (1 - move up, 2 - move down, 3 - switch press)
volatile byte oled_action = 0;
// Used to store the state of the screen
byte oled_screen_state = 0;
// Used to store the state of the cursor on the screen
byte oled_cursor_state = 0;
// Used to indicate whether the value pointed by the cursor is highlighted
bool oled_cursor_toggle = false;

//----- Ultrasonic sensors -----

// The last time the 8 ultrasonic sensors were sampled (milliseconds)
unsigned long ultrasonic_refresh_time = 0;
// This variable is used to store the the ultrasonic sensor values
word ultrasonic_distances[8];

//----- NEC remote -----

// Used to store the NEC remote data
volatile unsigned long nec_remote_data = 0;
// Indicates if NEC remote data is available
volatile bool nec_data_available = false;
// Inidcates whether sensor one is active
volatile bool nec_sensor_1_active = false;
// Inidcates whether sensor two is active
volatile bool nec_sensor_2_active = false;

//----- KY-040 rotary encoder -----

// This variable holds the timestamp of the previous ky-040 function call
unsigned long ky040_last_call_time = 0;
// The variable indicates if the KY-040 knob was in a stable position while turning
bool ky040_clock_release = true;
// This variable indicates the the clock filter is reset
bool ky040_clock_filter_reset = true;

//--------------- Initializations ---------------

// Initializing up the Adafruit display object
Adafruit_SSD1306 display(oled_screen_width, oled_screen_height, &Wire, false);

// Setting up the HCSR-04 ultrasonic sensors
NewPing sonar[8] = {
NewPing(ultrasonic_1_trigger, ultrasonic_1_echo, ultrasonic_max_distance),
NewPing(ultrasonic_2_trigger, ultrasonic_2_echo, ultrasonic_max_distance), 
NewPing(ultrasonic_3_trigger, ultrasonic_3_echo, ultrasonic_max_distance), 
NewPing(ultrasonic_4_trigger, ultrasonic_4_echo, ultrasonic_max_distance), 
NewPing(ultrasonic_5_trigger, ultrasonic_5_echo, ultrasonic_max_distance), 
NewPing(ultrasonic_6_trigger, ultrasonic_6_echo, ultrasonic_max_distance), 
NewPing(ultrasonic_7_trigger, ultrasonic_7_echo, ultrasonic_max_distance), 
NewPing(ultrasonic_8_trigger, ultrasonic_8_echo, ultrasonic_max_distance)
};

//--------------- Function Prototypes ---------------

// Sets the EEPROM data as per factory ettings
void eeprom_factory_reset();
// This function updates the oled screen
void oled_updater();

// The light reward maintenance function is called
void light_reward_maintain();
// The sound reward maintenance function is called
void sound_reward_maintain();
// The add on reward maintenance function is called
void add_on_reward_maintain();

// This function helps to set the sound volume
void sound_volume_set(byte sound_level);
// This function helps to play a track in a specified album
void sound_play_track(byte album, byte track);
// This function helps to stop the track being played
void sound_stop_track();

// Helps restart rewards during settings configuration
void reward_configuration_restarter();
// This function decides if we can activate the different rewards based on their status and configuration override
void reward_activation(bool light_status, bool sound_status, bool add_on_status, bool configuration_override = false);

// This function responds to the interrupt on the first infrared sensor
void infrared_sensor_filter_one();
// This function responds to the interrupt on the second infrared sensor
void infrared_sensor_filter_two();

// This function is called to take action if the knob of the KY-040 rotary encoder is pressed
void ky040_switch_filter();
// This function is called when the KY-040 rotary encoder knob is turned
void ky040_clock_filter();

//--------------- Setting up before executing the main loop ---------------

void setup() 
{ 
  
  // ----- EEPROM Factory reset -----
  
  // EEPROM factory reset function is called
  //eeprom_factory_reset();
  
  // ----- Serial to PC/Raspberry Pi -----

  // Setting up serial communication with the Raspberry Pi/PC
  Serial.begin(9600);

  // ----- Light Reward -----

  // Setting up the led panel driver
  FastLED.addLeds<WS2812B, led_panel_data_in, RGB>(led_panel, led_panel_pixel_count);

  // ----- Sound Reward -----
  
   // Setting up serial communication with the mp3 serial module
  Serial2.begin(9600);
  // We specify the pins connected to the audio amplifier shutdowns as output
  pinMode(left_audio_amplifier_shutdown, OUTPUT);
  pinMode(right_audio_amplifier_shutdown, OUTPUT);
  // We set the two audio amplifiers to shutdown when not in use
  digitalWrite(left_audio_amplifier_shutdown, LOW);
  digitalWrite(right_audio_amplifier_shutdown, LOW);
  // Set the sound volume level initially
  sound_volume_set(sound_loudness);
  // We update the sound track memory with the current album and track
  sound_track_memory[int(sound_album)] = sound_track;

  // ----- Add On Reward -----

  // We specify the pins connected to the motor driver enables as output
  pinMode(motor_driver_enable_A, OUTPUT);
  pinMode(motor_driver_enable_B, OUTPUT);
  // We specify the pins connected to the motor driver inputs as output
  pinMode(motor_driver_input_1, OUTPUT);
  pinMode(motor_driver_input_2, OUTPUT);
  pinMode(motor_driver_input_3, OUTPUT);
  pinMode(motor_driver_input_4, OUTPUT);
  // We disable the motor driver enables
  digitalWrite(motor_driver_enable_A, LOW);
  digitalWrite(motor_driver_enable_B, LOW);
  // We make the analog code pin input pullup
  pinMode(add_on_analog_code, INPUT_PULLUP);

  // ----- OLED -----

  // Setting up to communicate with OLED display at I2C address specified
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // Clears the OLED display
  display.clearDisplay();
  // Draws the splash screen bitmap
  display.drawBitmap((display.width()-oled_splash_screen_width)/2, (display.height()-oled_splash_screen_height)/2, splash_screen_image, oled_splash_screen_width, oled_splash_screen_height, WHITE);      
  // Refreshes the display
  display.display();
  // Waits for two seconds so that the user can view the splash screen
  delay(2500);

  // ----- Infrared Sensor -----

  // We attach interrupts on the falling edge for the first infrared sensor
  attachInterrupt(digitalPinToInterrupt(infrared_1_data_in), infrared_sensor_filter_one, FALLING);
  // We attach interrupts on the falling edge for the second infrared sensor
  attachInterrupt(digitalPinToInterrupt(infrared_2_data_in), infrared_sensor_filter_two, FALLING);

  // ----- KY-040 Rotary Encoder -----

  // We specify that the rotary encoder switch pin is an input. Pullup to prevent floating
  pinMode(rotary_encoder_switch, INPUT_PULLUP);
  // We specify that the rotary encoder clock pin is an input
  pinMode(rotary_encoder_clock, INPUT);
  // We specify that the rotary encoder data pin is an input
  pinMode(rotary_encoder_data, INPUT);
  
}

//--------------- Running the main loop ---------------

void loop() 
{
  
  // ----- Sampling from ultrasonic sensors -----
  
  // We ensure that ultrasound samples are taken at the specified sampling frequency
  if(8 * abs(millis() - ultrasonic_refresh_time) > ultrasonic_sampling_interval)
  {
    // We create a static variable to shift between the eight ultrasonic sensors
    static byte sensor_to_activate = 0;
    // We obtain the distances detected by all the ultrasonic sesnors
    ultrasonic_distances[sensor_to_activate] = sonar[sensor_to_activate].ping_cm();
    // We update the ultrasound sample time
    ultrasonic_refresh_time = millis();
    // we check if we have already obtained data from all sensors
    if(sensor_to_activate == 7)
    {
      // We update the OLED display
      oled_updater();
    }
    // We update the the sensor to activate
    sensor_to_activate = (sensor_to_activate + 1) % 8;
  }
  
  // ----- Responding to NEC remote requests -----
  
  // We check if NEC remote data is available
  if(nec_data_available)
  {
    // We switch between the different NEC code options
    switch(nec_remote_data) 
    {
      // Button 1 is pressed
      case 3125149440:
        // We set the light reward to be activated
        reward_activation(true, false, false);
        // We break out of the switch body
        break;
      // Button 2 is pressed
      case 3108437760:
        // We set the sound reward to be activated
        reward_activation(false, true, false);
        // We break out of the switch body
        break;
      // Button 3 is pressed
      case 3091726080:
        // We set the add on reward to be activated
        reward_activation(false, false, true);
        // We break out of the switch body
        break;
      // Button 4 is pressed
      case 3141861120:
        // We set the light and sound reward to be activated
        reward_activation(true, true, false);
        // We break out of the switch body
        break;
      // Button 5 is pressed
      case 3208707840:
        // We set the light and add on reward to be activated
        reward_activation(true, false, true);
        // We break out of the switch body
        break;
      // Button 6 is pressed
      case 3158572800:
        // We set the sound and add on reward to be activated
        reward_activation(false, true, true);
        // We break out of the switch body
        break;
      // Button 8 is pressed
      case 3927310080:
        // We set the light, sound and add on reward to be activated
        reward_activation(true, true, true);
        // We break out of the switch body
        break;
      // Button * is pressed
      case 3910598400:
         // We set the light, sound and add on reward to be deactivated
        reward_activation(false, false, false);
        // We break out of the switch body
        break;
      // Button 0 is pressed
      case 3860463360:
        // We set the light, sound and add on reward to be activated based on their activation status
        reward_activation(bitRead(reward_activation_status,0), bitRead(reward_activation_status,1), bitRead(reward_activation_status,2));
        // We break out of the switch body
        break;
      // Button up is pressed
      case 3877175040:
        // We specify that the rotary encoder knob has been turned clockwise
        oled_action = 1;
        // We break out of the switch body
        break;
      // Button ok is pressed
      case 3810328320:
        // We specify that the rotary encoder knob has been pressed and released
        oled_action = 3;
        // We break out of the switch body
        break;
      // Button down is pressed
      case 2907897600:
        // We specify that the rotary encoder knob has been turned anti-clockwise
        oled_action = 2;
        // We break out of the switch body
        break;
    }
    // We mark that we have read the existing NEC remote data
    nec_data_available = false;
  }
  
  // ----- Responding to serial requests -----
  
  // We check if there is any serial data sent from the raspberry pi
  if(Serial.available())
  {
    // We read the byte sent
    byte serial_data = Serial.read();
    // We switch between the different serial instructions
    switch(serial_data) 
    {
      // 48 - Instruction code asking to toggle activated rewards
      case 48:
        // We set the light, sound and add on reward to be activated based on their activation status
        reward_activation(bitRead(reward_activation_status,0), bitRead(reward_activation_status,1), bitRead(reward_activation_status,2));
        // We break out of the switch body
        break;
      // 49 - Instruction code asking to activate the light reward
      case 49:
        // We set the light reward to be activated
        reward_activation(true, false, false);
        // We break out of the switch body
        break;
      // 50 - Instruction code asking to activate the sound reward
      case 50:
        // We set the sound reward to be activated
        reward_activation(false, true, false);
        // We break out of the switch body
        break;
      // 51 - Instruction code asking to activate the add on reward
      case 51:
        // We set the add on reward to be activated
        reward_activation(false, false, true);
        // We break out of the switch body
        break;
      // 52 - Instruction code asking to activate the light and sound reward
      case 52:
        // We set the light and sound reward to be activated
        reward_activation(true, true, false);
        // We break out of the switch body
        break;
      // 53 - Instruction code asking to activate the light and add on reward
      case 53:
        // We set the light and add on reward to be activated
        reward_activation(true, false, true);
        // We break out of the switch body
        break;
      // 54 - Instruction code asking to activate the sound and add on reward
      case 54:
        // We set the sound and add on reward to be activated
        reward_activation(false, true, true);
        // We break out of the switch body
        break;
      // 55 - Instruction code asking to stop all the rewards
      case 55:
         // We set the light, sound and add on reward to be deactivated
        reward_activation(false, false, false);
        // We break out of the switch body
        break;
      // 56 - Instruction code asking to activate all the rewards
      case 56:
        // We set the light, sound and add on reward to be activated
        reward_activation(true, true, true);
        // We break out of the switch body
        break;
      // 57 - Instruction code asking to send ultrasonic data
      case 57:
        // We iterate through the eight ultrasonic sensors
        for(byte i=0; i<8; i++)
        {
          // We split the 2 bytes of word "word" object into separate bytes
          byte ultrasonic_distance_msb = highByte(ultrasonic_distances[i]);
          byte ultrasonic_distance_lsb = lowByte(ultrasonic_distances[i]);
          // We send the msb followed by the lsb
          Serial.write(ultrasonic_distance_msb);
          Serial.write(ultrasonic_distance_lsb);
        }
        // We break out of the switch body
        break;
      // 58 - Instruction code asking to emulate rotary knob clockwise turn
      case 58:
        // We specify that the rotary encoder knob has been turned clockwise
        oled_action = 1;
        // We break out of the switch body
        break;
      // 59 - Instruction code asking to emulate rotary knob press and release
      case 59:
        // We specify that the rotary encoder knob has been pressed and released
        oled_action = 3;
        // We break out of the switch body
        break;
      // 60 -  Instruction code asking to emulate rotary knob anticlockwise turn
      case 60:
        // We specify that the rotary encoder knob has been turned anti-clockwise
        oled_action = 2;
        // We break out of the switch body
        break;
      // 61 - Instruction code asking to display shutdown message
      case 61:
        // Stores the text x and y positions
        int16_t text_x, text_y;
        // Stores the text width and the text height
        uint16_t text_width, text_height;
        // We set the light, sound and add on reward to be deactivated and use override
        reward_activation(false, false, false, true);
        // The light reward maintenance function is called
        light_reward_maintain();
        // The sound reward maintenance function is called
        sound_reward_maintain();
        // The add on reward maintenance function is called
        add_on_reward_maintain();
        // We clear the previous display
        display.clearDisplay();
        // We set the required font - size 15
        display.setFont(&Open_Sans_Light_15);
        // We set the text color to white
        display.setTextColor(WHITE);
        // We obtain the the bounding box around the text that we are to print
        display.getTextBounds("Shutting down", 0, 27, &text_x, &text_y, &text_width, &text_height);
        // We set the cursor correspondingly
        display.setCursor(floor(64 - text_width/2.0), 27);
        // We print the required text
        display.println("Shutting down");
        // We obtain the the bounding box around the text that we are to print
        display.getTextBounds("Please Wait...", 0, 27, &text_x, &text_y, &text_width, &text_height);
        // We set the cursor correspondingly
        display.setCursor(floor(64 - text_width/2.0), 46);
        // We print the required text
        display.println("Please Wait...");
        // We update the display
        display.display();
        // We wait for twenty seconds
        delay(20000);
        // We clear the previous display
        display.clearDisplay();
        // We obtain the the bounding box around the text that we are to print
        display.getTextBounds("Turn the power", 0, 27, &text_x, &text_y, &text_width, &text_height);
        // We set the cursor correspondingly
        display.setCursor(floor(64 - text_width/2.0), 27);
        // We print the required text
        display.println("Turn the power");
        // We obtain the the bounding box around the text that we are to print
        display.getTextBounds("off now !", 0, 46, &text_x, &text_y, &text_width, &text_height);
        // We set the cursor correspondingly
        display.setCursor(floor(64 - text_width/2.0), 46);
        // We print the required text
        display.println("off now !");
        // We update the display
        display.display();
        // We wait indefinitely
        while(true) {};
    }
  }
  
  // ----- KY-040 rotary encoder polling One -----

  // We check if no previous oled action is pending
  if(!oled_action)
  {
    // We call the rotary encoder switch filter to check if the knob has been pressed
    ky040_switch_filter();
    // We call the rotary encoder clock filter to check if the knob has been turned
    ky040_clock_filter();
  }
  
  // ----- OLED action processing -----
  
  // We check if any oled action is pending
  if(oled_action)
  {
    // Used to indicate whether the oled display needs to be updated
    bool oled_display_update = true;
  
    // - Main screen -
    
    // We check if we are on the main screen
    if(oled_screen_state == 0)
    {
      // In case we pressed the knob or clicked ok
      if(oled_action == 3)
      {
        // We specify that we are moving to the settings screen
        oled_screen_state = 1;
        // We specify that the cursor state is on top
        oled_cursor_state = 0;
      }
      // We turned the rotary encoder knob
      else
      {
        // We specify that we don't need to update the oled screen
        oled_display_update = false;
      }
    }
  
    // - Settings screen -
    
    // We check if we are on the settings screen
    else if(oled_screen_state == 1)
    {
      // In case we clicked up arrow or moved the knob clockwise
      if(oled_action == 1)
      {
        // We check if the cursor value is already at the highest menu option
        if(oled_cursor_state == 0)
        {
          // We specify that we don't need to update the oled screen
          oled_display_update = false;
        }
        // the cursor value is not already at the highest menu option
        else
        {
          // We move the cursor state to a higher position
          oled_cursor_state--;
        }
      }
      // In case we clicked down arrow or moved the knob anticlockwise
      else if(oled_action == 2)
      {
        // The lowest cursor position in the menu
        byte oled_cursor_lowest_position = 5 + (add_on_type > 0);
        // We check if the cursor value crosses the lowest possible one
        if(oled_cursor_state == oled_cursor_lowest_position)
        {
          // We specify that we don't need to update the oled screen
          oled_display_update = false;
        }
        else
        {
          // We move the cursor state to a lower position
          oled_cursor_state++;
        }
      }
      // In case we clicked on the ok button or pressed the knob
      else
      {
        // We check if we are hovering over the light settings while pressing the knob or clicking ok
        if(oled_cursor_state == 0)
        {
          // We specify that we are moving to the light reward configuration screen
          oled_screen_state = 2;
        }
        // We check if we are hovering over the sound settings while pressing the knob or clicking ok
        else if(oled_cursor_state == 1)
        {
          // We specify that we are moving to the sound reward configuration screen
          oled_screen_state = 3;
          // We set the oled cursor back to zero
          oled_cursor_state = 0;
        }
        // We check if we are hovering over the bubble settings while pressing the knob or clicking ok
        else if(oled_cursor_state == 2 && add_on_type == 1)
        {
          // We specify that we are moving to the bubble reward configuration screen
          oled_screen_state = 4;
          // We set the oled cursor back to zero
          oled_cursor_state = 0;
        }
        // We check if we are hovering over the generic settings while pressing the knob or clicking ok
        else if(oled_cursor_state == 2 && add_on_type == 0 || oled_cursor_state == 3 && add_on_type >0)
        {
          // We specify that we are moving to the generic settings screen
          oled_screen_state = 5;
          // We set the oled cursor back to zero
          oled_cursor_state = 0;
        }
        // We check if we are hovering over the Test Reward(s) while pressing the knob or clicking ok
        else if(oled_cursor_state == 3 && add_on_type == 0 || oled_cursor_state == 4 && add_on_type >0)
        {
          // We set the light, sound and add on reward to be activated based on their activation status
          reward_activation(bitRead(reward_activation_status,0), bitRead(reward_activation_status,1), bitRead(reward_activation_status,2));
        }
        // We check if we are hovering over the Remote Keypad option while pressing the knob or clicking ok
        else if(oled_cursor_state == 4 && add_on_type == 0 || oled_cursor_state == 5 && add_on_type >0)
        {
          // We specify that we are moving to the remote keypad information screen
          oled_screen_state = 6;
          // We set the oled cursor back to zero
          oled_cursor_state = 0;
        }
        // We check if we are hovering over the exit option while pressing the knob or clicking ok
        else if(oled_cursor_state == 5 && add_on_type == 0 || oled_cursor_state == 6 && add_on_type >0)
        {
          // We specify that we are moving to the main screen
          oled_screen_state = 0;
          // We set the oled cursor back to zero
          oled_cursor_state = 0;
        }
      }
    }
  
    // - Light reward configuration screen -
    
    // We check if we are on the light reward configuration screen
    else if(oled_screen_state == 2)
    {
      // In case we clicked up arrow or moved the knob clockwise
      if(oled_action == 1)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We switch the cursor state
          switch(oled_cursor_state)
          {
            // We check if we are hovering over the "Status" option
            case 0:
              // We invert the current light status
              bitWrite(reward_activation_status, 0, (bitRead(reward_activation_status, 0) + 1) % 2);
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Mode" option
            case 1:
              // We increase the mode (make it zero if it crosses over)
              light_mode = (light_mode + 1) % light_max_modes;
              // We check if the reward configuration display is on
              if(reward_configuration_display == 1)
              {
                // We reactivate the light reward and use configuration override
                reward_activation(true, false, false, true);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Frequency" option
            case 2:
              // We check if the light frequency is at the the upper limit
              if(light_frequency == light_max_frequency)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // the light frequency is not at the the upper limit
              else
              {
                // We increment the light frequency
                light_frequency++;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }      
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Color" option
            case 3:
              // We increase the light preset by one
              light_preset_color_option = (light_preset_color_option + 1) % 25;
              // We specify that a light preset color is being used
              light_preset_color = true;
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Channel-R" option
            case 4:
              // We check if the light r channel value is at the upper limit
              if(light_r_channel == 255)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light r channel value is not at the upper limit
              else
              {
                // We increase the light r channel value
                light_r_channel++;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Channel-G" option
            case 5:
              // We check if the light g channel value is at the upper limit
              if(light_g_channel == 255)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light g channel value is not at the upper limit
              else
              {
                // We increase the light g channel value
                light_g_channel++;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Channel-B" option
            case 6:
              // We check if the light b channel value is at the upper limit
              if(light_b_channel == 255)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light b channel value is not at the upper limit
              else
              {
                // We increase the light b channel value
                light_b_channel++;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Brightness" option
            case 7:
              // We check if the light brightness value is at the upper limit
              if(light_brightness == led_panel_max_brightness)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The brightness value is not at the upper limit
              else
              {
                // We increase the light brightness value
                light_brightness++;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Duration" option
            case 8:
              // We check if the light duration is at the maximum duration
              if(light_duration == light_max_duration)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light duration is at the maximum duration
              else
              {
                // We increase the light duration by 100 milliseconds
                light_duration = light_duration + 100;
              }
              // We break out of the switch case
              break;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the first menu option
          if(oled_cursor_state == 0)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the first menu option
          else
          {
            // We move the cursor state to a higher position
            oled_cursor_state--;
          }
        }
      }
      // In case we clicked down arrow or moved the knob anticlockwise
      else if(oled_action == 2)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We switch the cursor state
          switch(oled_cursor_state)
          {
            // We check if we are hovering over the "Status" option
            case 0:
              // We invert the current light status
              bitWrite(reward_activation_status, 0, (bitRead(reward_activation_status, 0) + 1) % 2);
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Mode" option
            case 1:
              // We check if the mode is zero
              if(light_mode == 0)
              {
                // We make it the maximum value
                light_mode = light_max_modes - 1;
              }
              // we are not hovering over the "Mode" option
              else
              {
                // We decrease the mode
                light_mode = (light_mode - 1) % light_max_modes;
              }
              // We check if the reward configuration display is on
              if(reward_configuration_display == 1)
              {
                // We reactivate the light reward and use configuration override
                reward_activation(true, false, false, true);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Frequency" option
            case 2:
              // We check if the light frequency is at the the lower limit
              if(light_frequency == light_min_frequency)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // the light frequency is not at the the lower limit
              else
              {
                // We decrement the light frequency
                light_frequency--;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }        
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Color" option
            case 3:
              // We check if the light preset color option is zero
              if(light_preset_color_option == 0)
              {
                // We set it to the highest preset value
                light_preset_color_option = 24;
              }
              // If its not zero we can simply decrease the light preset color option
              else
              {
                // We decrease the light preset by one
                light_preset_color_option = light_preset_color_option - 1;
              }
              // We specify that a light preset color is being used
              light_preset_color = true;
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Channel-R" option
            case 4:
              // We check if the light r channel value is at the lower limit
              if(light_r_channel == 0 || (light_r_channel == 1 && light_g_channel == 0 && light_b_channel == 0))
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light r channel value is not at the lower limit
              else
              {
                // We decrease the light r channel value
                light_r_channel--;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Channel-G" option
            case 5:
              // We check if the light g channel value is at the lower limit
              if(light_g_channel == 0 || (light_g_channel == 1 && light_r_channel == 0 && light_b_channel == 0))
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light g channel value is not at the lower limit
              else
              {
                // We decrease the light g channel value
                light_g_channel--;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Channel-B" option
            case 6:
              // We check if the light b channel value is at the lower limit
              if(light_b_channel == 0 || (light_b_channel == 1 && light_r_channel == 0 && light_g_channel == 0))
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light b channel value is not at the lower limit
              else
              {
                // We decrease the light b channel value
                light_b_channel--;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Brightness" option
            case 7:
              // We check if the light brightness value is at the lower limit
              if(light_brightness == 1)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The brightness value is not at the lower limit
              else
              {
                // We decrease the light brightness value
                light_brightness--;
                // We specify that we need to update the static variables of the light maintenance function
                light_update = true;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Duration" option
            case 8:
              // We check if the light duration is at the minimum duration
              if(light_duration == light_min_duration)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The light duration is not at the minimum duration
              else
              {
                // We decrease the light duration by 100 milliseconds
                light_duration = light_duration - 100;
              }
              // We break out of the switch case
              break;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the last menu option
          if(oled_cursor_state == 9)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the last menu option
          else
          {
            // We move the cursor state to a lower position
            oled_cursor_state++;
          }
        }
      }
      // In case we pressed the rotary encoder knob
      else if(oled_action == 3)
      {
        // We check if we have already toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We toggle the cursor to false
          oled_cursor_toggle = false;
          // We check if we are hovering over the "Status" option
          if(oled_cursor_state == 0)
          {
            // We write the light reward status to EEPROM memory
            EEPROM.write(0, reward_activation_status);
          }
          else
          {
            // We switch between the oled cursor state
            switch(oled_cursor_state)
            {
              // We check if we are hovering over the "Mode" option
              case 1:
                // We write the light mode to EEPROM memory
                EEPROM.write(1, light_mode);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Frequency" option
              case 2:
                // We write the light frequency to EEPROM memory
                EEPROM.write(2, light_frequency);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Color" option
              case 3:
                // We write the light r channel value to EEPROM memory
                EEPROM.write(3, light_r_channel);
                // We write the light g channel value to EEPROM memory
                EEPROM.write(4, light_g_channel);
                // We write the light b channel value to EEPROM memory
                EEPROM.write(5, light_b_channel);
                // We break out of the switch case
                break;                
              // We check if we are hovering over the "Channel-R" option
              case 4:
                // We write the light r channel value to EEPROM memory
                EEPROM.write(3, light_r_channel);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Channel-G" option
              case 5:
                // We write the light g channel value to EEPROM memory
                EEPROM.write(4, light_g_channel);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Channel-B" option
              case 6:
                // We write the light b channel value to EEPROM memory
                EEPROM.write(5, light_b_channel);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Brightness" option
              case 7:
                // We write the light brightness value to EEPROM memory
                EEPROM.write(6, light_brightness);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Duration" option
              case 8:
                // We write the light duration value LSB to EEPROM memory
                EEPROM.write(7, lowByte(light_duration));
                // We write the light duration value MSB to EEPROM memory
                EEPROM.write(8, highByte(light_duration));
                // We break out of the switch case
                break;
            }
            // We check if the reward configuration display is on
            if(reward_configuration_display == 1)
            {
              // We specify that we are moving out of the configuration mode
              reward_configuration = false;
              // We deactivate the light reward
              reward_activation(false, false, false);
            }
          }
        }
        // we have not already toggled a configuration to change its value
        else
        {
          // We check if we are hovering over the "Exit" option
          if(oled_cursor_state == 9)
          {
            // We specify that we are moving to the settings screen
            oled_screen_state = 1;
            // We specify that the cursor state is at the light settings
            oled_cursor_state = 0;
          }
          // We check if we are not hovering over the light reward activation option
          else if(oled_cursor_state != 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true;
            // We check if the reward configuration display is on
            if(reward_configuration_display == 1)
            {
              // We specify that we are moving to the configuration mode
              reward_configuration = true;
              // We activate the light reward and use configuration override
              reward_activation(true, false, false, true);
            }
          }
          // We check if we are hovering over the light reward activation option
          else if(oled_cursor_state == 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true; 
          }
        }
      }
    }  
  
    // - sound reward configuration screen -
    
    // We check if we are on the sound reward configuration screen
    else if(oled_screen_state == 3)
    {
      // In case we clicked up arrow or moved the knob clockwise
      if(oled_action == 1)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We switch the cursor state
          switch(oled_cursor_state)
          {
            // We check if we are hovering over the "Status" option
            case 0:
              // We invert the current sound status
              bitWrite(reward_activation_status, 1, (bitRead(reward_activation_status, 1) + 1) % 2);
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Album" option
            case 1:
              // We increase the album number (make it zero if it crosses over)
              sound_album = (sound_album + 1) % sound_album_count;
              // The sound track is restored from memory
              sound_track = sound_track_memory[sound_album];
              // We check if the reward configuration display is on
              if(reward_configuration_display == 1)
              {
                // We reactivate the sound reward and use configuration override
                reward_activation(false, true, false, true);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Track" option
            case 2:
              // We increase the album number (make it zero if it crosses over)
              sound_track = (sound_track + 1) % sound_track_count[sound_album];
              // The sound track is stored in memory as well
              sound_track_memory[sound_album] = sound_track;
              // We check if the reward configuration display is on
              if(reward_configuration_display == 1)
              {
                // We reactivate the sound reward and use configuration override
                reward_activation(false, true, false, true);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Loudness" option
            case 3:
              // We check if the sound loudness value is at the upper limit
              if(sound_loudness == 30)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The sound loudness value is not at the upper limit
              else
              {
                // We increase the sound loudness value
                sound_loudness++;
                // We update teh sound loudness
                sound_volume_set(sound_loudness);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Sound Repetition" option
            case 4:
              // We check if the sound repetition value is at the upper limit
              if(sound_repetition == sound_max_repeats)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The sound repetition value is not at the upper limit
              else
              {
                // We increase the sound repetition value
                sound_repetition++;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Clip Time" option
            case 5:
              // We check if the sound clip time value is at the upper limit
              if(sound_clip_time == sound_max_clip_time)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The sound clip time value is not at the upper limit
              else
              {
                // We increase the sound clip time value
                sound_clip_time = sound_clip_time + 100;
              }
              // We break out of the switch case
              break;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the first menu option
          if(oled_cursor_state == 0)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the first menu option
          else
          {
            // We move the cursor state to a higher position
            oled_cursor_state--;
          }
        }
      }
      // In case we clicked down arrow or moved the knob anticlockwise
      else if(oled_action == 2)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We switch the cursor state
          switch(oled_cursor_state)
          {
            // We check if we are hovering over the "Status" option
            case 0:
              // We invert the current sound status
              bitWrite(reward_activation_status, 1, (bitRead(reward_activation_status, 1) + 1) % 2);
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Album" option
            case 1:
              // We check if the sound album is zero
              if(sound_album == 0)
              {
                // We make it the maximum value
                sound_album = sound_album_count - 1;
              }
              else
              {
                // We decrease the album number (make it maxmium value if it crosses under)
                sound_album = (sound_album - 1) % sound_album_count;
                // The sound track is restored from memory
                sound_track = sound_track_memory[sound_album];
              }
              // We check if the reward configuration display is on
              if(reward_configuration_display == 1)
              {
                // We reactivate the sound reward and use configuration override
                reward_activation(false, true, false, true);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Track" option
            case 2:
  
              // We check if the sound track is zero
              if(sound_track == 0)
              {
                // We make it the maximum value
                sound_track = sound_track_count[sound_album] - 1;
              }
              else
              {
              // We decrease the track number (make it maxmium value if it crosses under)
              sound_track = (sound_track - 1) % sound_track_count[sound_album];
              }
              // The sound track is stored in memory as well
              sound_track_memory[sound_album] = sound_track;
              // We check if the reward configuration display is on
              if(reward_configuration_display == 1)
              {
                // We reactivate the sound reward and use configuration override
                reward_activation(false, true, false, true);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Loudness" option
            case 3:
              // We check if the sound loudness value is at the lower limit
              if(sound_loudness == 0)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The sound loudness value is not at the lower limit
              else
              {
                // We decrease the sound loudness value
                sound_loudness--;
                // We update teh sound loudness
                sound_volume_set(sound_loudness);
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Sound Repetition" option
            case 4:
              // We check if the sound repetition value is at the lower limit
              if(sound_repetition == 1)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The sound repetition value is not at the lower limit
              else
              {
                // We decrease the sound repetition value
                sound_repetition--;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Clip Time" option
            case 5:
              // We check if the sound clip time value is at the lower limit
              if(sound_clip_time == sound_min_clip_time)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The sound clip time value is not at the lower limit
              else
              {
                // We decrease the sound clip time value
                sound_clip_time = sound_clip_time - 100;
              }
              // We break out of the switch case
              break;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the last menu option
          if(oled_cursor_state == 6)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the last menu option
          else
          {
            // We move the cursor state to a lower position
            oled_cursor_state++;
          }
        }
      }    
      // In case we pressed the rotary encoder knob
      else if(oled_action == 3)
      {
        // We check if we have already toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We toggle the cursor to false
          oled_cursor_toggle = false;
          // We check if we are hovering over the "Status" option
          if(oled_cursor_state == 0)
          {
            // We write the sound reward status to EEPROM memory
            EEPROM.write(0, reward_activation_status);
          }
          else
          {
            // We switch between the oled cursor state
            switch(oled_cursor_state)
            {
              // We check if we are hovering over the "Album" option
              case 1:
                // We write the sound album number to EEPROM memory
                EEPROM.write(9, sound_album);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Track" option
              case 2:
                // We write the sound track number to EEPROM memory
                EEPROM.write(10, sound_track);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Loudness" option
              case 3:
                // We write the sound loudness value to EEPROM memory
                EEPROM.write(11, sound_loudness);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Repetition" option
              case 4:
                // We write the sound repetition value to EEPROM memory
                EEPROM.write(12, sound_repetition);
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Clip Time" option
              case 5:
                // We write the sound clip time to EEPROM memory
                EEPROM.write(13, sound_clip_time/100);
                // We break out of the switch case
                break;
            }
            // We check if the reward configuration display is on
            if(reward_configuration_display == 1)
            {
              // We specify that we are moving out of the configuration mode
              reward_configuration = false;
              // We deactivate the light reward
              reward_activation(false, false, false);
            }
          }
        }
        // we have not already toggled a configuration to change its value
        else
        { 
          // We check if we are hovering over the "Exit" option
          if(oled_cursor_state == 6)
          {
            // We specify that we are moving to the settings screen
            oled_screen_state = 1;
            // We specify that the cursor state is at the sound settings
            oled_cursor_state = 1;
          }
          // We check if we are not hovering over the sound status option
          else if(oled_cursor_state != 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true;
            // We check if the reward configuration display is on
            if(reward_configuration_display == 1)
            {
              // We specify that we are moving to the configuration mode
              reward_configuration = true;
              // We activate the sound reward and use configuration override
              reward_activation(false, true, false, true);
            }
          }
          // We check if we are hovering over the sound reward activation option
          else if(oled_cursor_state == 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true; 
          }
        }
      }
    }
  
    // - bubble reward configuration screen -
    
    // We check if we are on the bubble reward configuration screen
    else if(oled_screen_state == 4)
    {
       // In case we clicked up arrow or moved the knob clockwise
      if(oled_action == 1)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We switch the cursor state
          switch(oled_cursor_state)
          {
            // We check if we are hovering over the "Status" option
            case 0:
              // We invert the current sound status
              bitWrite(reward_activation_status, 2, (bitRead(reward_activation_status, 2) + 1) % 2);
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Duration" option
            case 1:
              // We check if the bubble duration is at the upper limit
              if(bubble_duration == bubble_max_duration)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The bubble duration value is not at the upper limit
              else
              {
                // We increase the bubble duration value
                bubble_duration = bubble_duration + 100;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Density" option
            case 2:
              // We check if the bubble density is at the upper limit
              if(bubble_motor_control == 255)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The bubble density value is not at the upper limit
              else
              {
                // We increase the bubble density value
                bubble_motor_control++;
              }
              // We break out of the switch case
              break;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the first menu option
          if(oled_cursor_state == 0)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the first menu option
          else
          {
            // We move the cursor state to a higher position
            oled_cursor_state--;
          }
        }
      }
      // In case we clicked down arrow or moved the knob anticlockwise
      else if(oled_action == 2)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We switch the cursor state
          switch(oled_cursor_state)
          {
            // We check if we are hovering over the "Status" option
            case 0:
              // We invert the current sound status
              bitWrite(reward_activation_status, 2, (bitRead(reward_activation_status, 2) + 1) % 2);
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Duration" option
            case 1:
              // We check if the bubble duration is at the lower limit
              if(bubble_duration == bubble_min_duration)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The bubble duration value is not at the lower limit
              else
              {
                // We decrease the bubble duration value
                bubble_duration = bubble_duration - 100;
              }
              // We break out of the switch case
              break;
            // We check if we are hovering over the "Density" option
            case 2:
              // We check if the bubble density is at the lower limit
              if(bubble_motor_control == bubble_min_motor_control)
              {
                // We specify that we don't need to update the oled screen
                oled_display_update = false;
              }
              // The bubble density value is not at the lower limit
              else
              {
                // We decrease the bubble density value
                bubble_motor_control--;
              }
              // We break out of the switch case
              break;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the last menu option
          if(oled_cursor_state == 3)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the last menu option
          else
          {
            // We move the cursor state to a lower position
            oled_cursor_state++;
          }
        }
      }  
      // In case we pressed the rotary encoder knob
      else if(oled_action == 3)
      {
        // We check if we have already toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We toggle the cursor to false
          oled_cursor_toggle = false;
          // We check if we are hovering over the "Status" option
          if(oled_cursor_state == 0)
          {
            // We write the sound reward status to EEPROM memory
            EEPROM.write(0, reward_activation_status);
          }
          else
          {
            // We switch between the oled cursor state
            switch(oled_cursor_state)
            {
              // We check if we are hovering over the "Duration" option
              case 1:
                // We write the bubble duration value LSB to EEPROM memory
                EEPROM.write(14, lowByte(bubble_duration));
                // We write the bubble duration value MSB to EEPROM memory
                EEPROM.write(15, highByte(bubble_duration));
                // We break out of the switch case
                break;
              // We check if we are hovering over the "Density" option
              case 2:
                // We write the geared motor speed control to EEPROM memory
                EEPROM.write(16, bubble_motor_control);
                // We break out of the switch case
                break;
            }
            // We check if the reward configuration display is on
            if(reward_configuration_display == 1)
            {
              // We specify that we are moving out of the configuration mode
              reward_configuration = false;
              // We deactivate the light reward
              reward_activation(false, false, false);
            }
          }
        }
        // we have not already toggled a configuration to change its value
        else
        {
          // We check if we are hovering over the "Exit" option
          if(oled_cursor_state == 3)
          {
            // We specify that we are moving to the settings screen
            oled_screen_state = 1;
            // We specify that the cursor state is at the bubble settings
            oled_cursor_state = 2;
          }
          // We check if we are not hovering over the bubble status option
          else if(oled_cursor_state != 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true; 
            // We check if the reward configuration display is on
            if(reward_configuration_display == 1)
            {
              // We specify that we are moving to the configuration mode
              reward_configuration = true;
              // We activate the bubble reward and use configuration override
              reward_activation(false, false, true, true);
            }
          }
          // We check if we are hovering over the bubble reward activation option
          else if(oled_cursor_state == 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true; 
          }
        }
      }
    }

    // - Generic Settings screen -

    // We check if we are on the generic settings screen
    else if(oled_screen_state == 5)
    {
      // In case we clicked up arrow or moved the knob clockwise
      if(oled_action == 1)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We check if we are changing reward configuration display status
          if(oled_cursor_state == 0)
          {
            // We toggle the reward configuration display status
            reward_configuration_display = (reward_configuration_display + 1) % 2;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the first menu option
          if(oled_cursor_state == 0)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the last menu option
          else
          {
            // We move the cursor state to a higher position
            oled_cursor_state--;
          }
        }
      }
      // In case we clicked down arrow or moved the knob anticlockwise
      else if(oled_action == 2)
      {
        // We check if we have toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We check if we are changing reward configuration display status
          if(oled_cursor_state == 0)
          {
            // We toggle the reward configuration display status
            reward_configuration_display = (reward_configuration_display + 1) % 2;
          }
        }
        // We check if we have not toggled a configuration to change its value
        else
        {
          // We check if the cursor is at the last menu option
          if(oled_cursor_state == 1)
          {
            // We specify that we don't need to update the oled screen
            oled_display_update = false;
          }
          // The cursor is not at the last menu option
          else
          {
            // We move the cursor state to a lower position
            oled_cursor_state++;
          }
        }
      }
      // In case we pressed the rotary encoder knob
      else if(oled_action == 3)
      {
        // We check if we have already toggled a configuration to change its value
        if(oled_cursor_toggle)
        {
          // We toggle the cursor to false
          oled_cursor_toggle = false;
          // We check if we are hovering over the "Configuration Display" option
          if(oled_cursor_state == 0)
          {
            // We write the reward configuration display status to EEPROM memory
            EEPROM.write(17, reward_configuration_display);
          }
        }
        // we have not already toggled a configuration to change its value
        else
        {
          // We check if we are hovering over the "Configuration Display" option
          if(oled_cursor_state == 0)
          {
            // We toggle the cursor to true
            oled_cursor_toggle = true; 
          }
          // We check if we are hovering over the "Exit" option
          else if(oled_cursor_state == 1)
          {
            // We specify that we are moving to the settings screen
            oled_screen_state = 1;
            // We specify that the cursor state is at the generic settings
            oled_cursor_state = (add_on_type == 0 ? 2 : 3);
          }
        }
      }
    }
  
    // - Remote keypad screen -
    
    // We check if we are on the remote keypad information screen
    else if(oled_screen_state == 6)
    {
      // In case we pressed the rotary encoder knob
      if(oled_action == 3)
      {
        // We specify that we are moving to the settings screen
        oled_screen_state = 1;
        // We specify that the cursor state is at the remote keypad option
        oled_cursor_state = 4 + (add_on_type > 0);
      }
    }
  
    // - OLED screen update -

    // We check if we need to update the OLED screen
    if(oled_display_update)
    {
      // We update the OLED screen
      oled_updater();
    }
  }

  // ----- KY-040 rotary encoder polling Two -----
  
  // We call the rotary encoder switch filter to check if the knob has been pressed
  ky040_switch_filter();
  // We call the rotary encoder clock filter to check if the knob has been turned
  ky040_clock_filter();
  
  // ----- Add on validity check -----
  
  // We check the add on rewards analog code
  word add_on_analog_value = analogRead(add_on_analog_code);
  // We check if it falls within any add on group
  if(add_on_analog_value > 300 and add_on_analog_value < 700)
  {
    // We check if the there was no add on previously
    if(add_on_type == 0)
    {
      // We specify that the add on type is the bubble reward
      add_on_type = 1;
      // We check if we are on the main screen or the settings screen
      if(oled_screen_state < 2)
      {
        // Updates the OLED display with the main screen or the settings screen
        oled_updater();
      }
    }
  }
  // We check if an add on was installed earlier
  else if(add_on_type > 0)
  {
    // We check if the add on was the bubble reward
    if(add_on_type == 1)
    {
      // We specify that the add on state for the bubble reward is emergency stop
      add_on_state = 4;
      // The add on reward maintenance function is called
      add_on_reward_maintain();
    }
    // We specify that no add on is present
    add_on_type = 0;
    // We check if the current screen was the bubble configuration screen
    if(oled_screen_state == 4)
    {
      // We pull back to the settings screen
      oled_screen_state = 1;
      // We make the cursor return to the top most element
      oled_cursor_state = 0;
    }
    if(oled_screen_state < 2)
    {
      // Updates the OLED display with the main screen or the settings screen
      oled_updater();
    }
  }

  // ----- Main Screen Reward activation flashing -----

  // We check if any reward is being activated
  if(light_state > 0 || sound_state > 0 || add_on_state == 1 || add_on_state == 2)
  {
    // We are at the main scrren
    if(oled_screen_state == 0)
    {
      // We check if it is time to flash the reward activation icons
      if(2 * abs(millis() - reward_activation_flash_refresh_time) > reward_activation_flash_duration)
      {
        // We toggle the reward activation icon flag
        reward_activation_flash_flag = !reward_activation_flash_flag;
        // We update the reward activation flash refresh time
        reward_activation_flash_refresh_time = millis();
        // We update the extra activation flag
        reward_activation_flash_extra_flag = true;
        // We update the OLED screen
        oled_updater();
      } 
    }
  }
  // If no reward is activated we check if should run onle last time
  else if(reward_activation_flash_extra_flag)
  {
    // We update the extra activation flag
    reward_activation_flash_extra_flag = false;
    // We turn the reward icon to flash all the time
    reward_activation_flash_flag = true;
    // We update the OLED screen
    oled_updater();
  }
  
  // ----- Reward maintenance functions called -----
  
  // The light reward maintenance function is called
  light_reward_maintain();
  // The sound reward maintenance function is called
  sound_reward_maintain();
  // The add on reward maintenance function is called
  add_on_reward_maintain();
  // Helps restart rewards during settings configuration
  reward_configuration_restarter();
}
// ---------------- Function Definitions ----------------

// ----- EEPROM set function -----

// Sets the EEPROM data as per factory ettings
void eeprom_factory_reset()
{
  // Reward activation status - Each bit can be toggled to activate a particular reward type
  EEPROM.write(0, B111);
  
  // Light reward (Still - 0, Pulse - 1, Fade - 2, Beacon - 3, Wipe - 4)
  EEPROM.write(1,1);
  // Frequency at which light reward pulses or fades
  EEPROM.write(2, 20);
  // The light color as specified by the R, G and B channels
  EEPROM.write(3, 0);
  EEPROM.write(4, 0);
  EEPROM.write(5, 255);
  // The maximum brightness of the led panel
  EEPROM.write(6, 100);
  // The duration for which light reward is provided -> 256 * (8) + (7)
  EEPROM.write(7, 16);
  EEPROM.write(8, 39);
  
  // The album choice for the sound reward
  EEPROM.write(9, 1);
  // The track choice for the sound reward
  EEPROM.write(10, 3);
  // The loudness of the sound reward
  EEPROM.write(11, 30);
  // The number of times the sound is played 
  EEPROM.write(12, 3);
  // The time after which the sound is clipped
  EEPROM.write(13, 30);
  
  // The duration for which bubble reward is provided -> 256 * (8) + (7)
  EEPROM.write(14, 16);
  EEPROM.write(15, 39);

  // The geared motor speed control
  EEPROM.write(16, 255);

  // The reward configuration display status (0 - don't show, 1 - show)
  EEPROM.write(17, 1);
}

// ----- OLED updater function -----

// This function updates the oled screen
void oled_updater()
{
  // The vertical offset from which which we start the oled display
  static int oled_cursor_offset = 0;
  // The vertical offset from which which we start the oled display for settings page only
  static int oled_cursor_offset_settings = 0;
  // The oled screen that was previously displayed
  static byte oled_previous_screen_state = 0;
  // The vertical gap in pixels between two oled texts of size 15
  const static byte oled_size_15_vertical_gap = 8;
  // The vertical size in pixels of oled text of size 15
  const static byte oled_size_15_vertical_size = 11;
  // The vertical gap in pixels between two oled texts of size 12
  const static byte oled_size_13_vertical_gap = 5;
  // The vertical size in pixels of oled text of size 15
  const static byte oled_size_13_vertical_size = 10;  

  // The light activation icon
  static unsigned char light_activation_icon [] = {
  0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x10, 0x08, 0x08, 0x08, 0x00, 0x10, 0x04,
  0x00, 0x20, 0x00, 0x7E, 0x00, 0x00, 0xE7, 0x00, 0x01, 0x8F, 0x80, 0x03, 0x3F, 0xC0, 0x03, 0x7F,
  0xC0, 0xF3, 0x7F, 0xCF, 0x03, 0xFF, 0xC0, 0x03, 0xFF, 0xC0, 0x01, 0xFF, 0x80, 0x00, 0xFF, 0x00,
  0x00, 0xFF, 0x00, 0x04, 0x7E, 0x20, 0x08, 0x7E, 0x10, 0x10, 0x00, 0x08, 0x00, 0x7E, 0x00, 0x00,
  0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x3C, 0x00,
  };
  
  // The sound activation icon
  static unsigned char sound_activation_icon [] = {
  0x00, 0x0C, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x7C, 0x60, 0x00, 0xFC, 0x10, 0x01,
  0xFC, 0x08, 0x07, 0xFD, 0x84, 0x7F, 0xFC, 0x42, 0xFF, 0xFC, 0x22, 0xFF, 0xFC, 0x21, 0xFF, 0xFC,
  0x11, 0xFF, 0xFC, 0x11, 0xFF, 0xFC, 0x11, 0xFF, 0xFC, 0x11, 0xFF, 0xFC, 0x21, 0xFF, 0xFC, 0x22,
  0x7F, 0xFC, 0x42, 0x07, 0xFD, 0x84, 0x01, 0xFC, 0x08, 0x00, 0xFC, 0x10, 0x00, 0x7C, 0x60, 0x00,
  0x3C, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x0C, 0x00, 
  };
  
  // The bubble activation icon
  static unsigned char bubble_activation_icon [] = {
  0x0F, 0xC0, 0x1C, 0x30, 0x30, 0x22, 0x40, 0x08, 0x41, 0x4C, 0x04, 0x41, 0x90, 0x04, 0x41, 0x90,
  0x02, 0x22, 0x80, 0x02, 0x1C, 0x80, 0x02, 0x00, 0x80, 0x02, 0x00, 0x40, 0x04, 0x00, 0x40, 0x04,
  0x00, 0x20, 0x08, 0xF8, 0x18, 0x31, 0x04, 0x07, 0xC2, 0x02, 0x40, 0x04, 0x01, 0xA0, 0x04, 0x01,
  0x40, 0x04, 0x01, 0x03, 0x84, 0x01, 0x04, 0x44, 0x01, 0x08, 0x22, 0x02, 0x08, 0x21, 0x04, 0x08,
  0x20, 0xF8, 0x04, 0x40, 0x00, 0x03, 0x80, 0x00, 
  };
 
  // We clear the previous display
  display.clearDisplay(); 
  // We switch between the oled screen states
  switch(oled_screen_state)
  {

    //- Main Page -
    
    // We check if we are on the main page
    case 0:
    {
      // We draw the 8 triangles of the ultrasonic sensor panel
      display.drawTriangle(95, 32, 95, 63, 117, 54, WHITE);
      display.drawTriangle(95, 32, 117, 54, 126, 32, WHITE);
      display.drawTriangle(95, 32, 126, 32, 117, 10, WHITE);
      display.drawTriangle(95, 32, 117, 10, 95, 1, WHITE);
      display.drawTriangle(95, 32, 95, 1, 73, 10, WHITE);
      display.drawTriangle(95, 32, 73, 10, 64, 32, WHITE);
      display.drawTriangle(95, 32, 64, 32, 73, 54, WHITE);
      display.drawTriangle(95, 32, 73, 54, 95, 63, WHITE);
      
      // This variable denotes the percentage covered by each ultrasonic sensor
      double ultrasonic_percentage_coverage = double(ultrasonic_distances[0]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 1
      display.fillTriangle(95, 32, 
                           95, 32 + floor(31 * ultrasonic_percentage_coverage), 
                           95 + floor(22 * ultrasonic_percentage_coverage), 
                           32 + floor(22 * ultrasonic_percentage_coverage), WHITE);

      // We update the value for ultrasonic sensor 2
      ultrasonic_percentage_coverage = double(ultrasonic_distances[1]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 2
      display.fillTriangle(95, 32, 
                           95 + floor(22 * ultrasonic_percentage_coverage), 
                           32 + floor(22 * ultrasonic_percentage_coverage), 
                           95 + floor(31 * ultrasonic_percentage_coverage),
                           32, WHITE);
                           
      // We update the value for ultrasonic sensor 3
      ultrasonic_percentage_coverage = double(ultrasonic_distances[2]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 3
      display.fillTriangle(95, 32,
                           95 + floor(31 * ultrasonic_percentage_coverage), 
                           32,
                           95 + floor(22 * ultrasonic_percentage_coverage), 
                           32 - floor(22 * ultrasonic_percentage_coverage), WHITE);

      // We update the value for ultrasonic sensor 4
      ultrasonic_percentage_coverage = double(ultrasonic_distances[3]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 4
      display.fillTriangle(95, 32,
                           95 + floor(22 * ultrasonic_percentage_coverage), 
                           32 - floor(22 * ultrasonic_percentage_coverage), 
                           95, 32 - floor(31 * ultrasonic_percentage_coverage), WHITE);

      // We update the value for ultrasonic sensor 5
      ultrasonic_percentage_coverage = double(ultrasonic_distances[4]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 5
      display.fillTriangle(95, 32,
                           95, 32 - floor(31 * ultrasonic_percentage_coverage), 
                           95 - floor(22 * ultrasonic_percentage_coverage), 
                           32 - floor(22 * ultrasonic_percentage_coverage), WHITE);

      // We update the value for ultrasonic sensor 6
      ultrasonic_percentage_coverage = double(ultrasonic_distances[5]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 6
      display.fillTriangle(95, 32,
                           95 - floor(22 * ultrasonic_percentage_coverage), 
                           32 - floor(22 * ultrasonic_percentage_coverage), 
                           95 - floor(31 * ultrasonic_percentage_coverage), 
                           32, WHITE);

      // We update the value for ultrasonic sensor 7
      ultrasonic_percentage_coverage = double(ultrasonic_distances[6]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 6
      display.fillTriangle(95, 32,
                           95 - floor(31 * ultrasonic_percentage_coverage), 32, 
                           95 - floor(22 * ultrasonic_percentage_coverage), 
                           32 + floor(22 * ultrasonic_percentage_coverage), WHITE);

      // We update the value for ultrasonic sensor 8
      ultrasonic_percentage_coverage = double(ultrasonic_distances[7]) / ultrasonic_max_distance;
      // We update ultrasonic sensor 7
      display.fillTriangle(95, 32,
                           95 - floor(22 * ultrasonic_percentage_coverage), 
                           32 + floor(22 * ultrasonic_percentage_coverage), 
                           95, 32 + floor(31 * ultrasonic_percentage_coverage), WHITE);
    
      // We check if the light reward is activated
      if(bitRead(reward_activation_status, 0))
      {
        // We check if the reward symbol is to be flashed on during its activation
        if(reward_activation_flash_flag || light_state == 0)
        {
          // Draws the light activation bitmap
          display.drawBitmap(32, 0, light_activation_icon, 24, 24, WHITE);
        }
      }
      // We check if the light reward is currently active
      else if(light_state > 0)
      {
        // We check if the reward symbol is to be flashed on during its activation
        if(reward_activation_flash_flag)
        {
          // Draws the light activation bitmap
          display.drawBitmap(32, 0, light_activation_icon, 24, 24, WHITE);
        }
      }
      // We check if the sound reward is activated
      if(bitRead(reward_activation_status, 1))
      {
        // We check if the reward symbol is to be flashed on during its activation
        if(reward_activation_flash_flag || sound_state == 0)
        {
          // Draws the sound activation bitmap      
          display.drawBitmap(32, 40, sound_activation_icon, 24, 24, WHITE);
        }
      }
      // We check if the sound state is greater than zero
      else if(sound_state > 0)
      {
        // We check if the reward symbol is to be flashed on during its activation
        if(reward_activation_flash_flag)
        {
          // Draws the sound activation bitmap      
          display.drawBitmap(32, 40, sound_activation_icon, 24, 24, WHITE);
        }
      }
      // We check if the bubble reward is activated
      if(bitRead(reward_activation_status, 2) && add_on_type == 1)
      {
        // We check if the reward symbol is to be flashed on during its activation
        if(reward_activation_flash_flag || add_on_state == 0)
        {
          // Draws the bubble activation bitmap
          display.drawBitmap(0, 20, bubble_activation_icon, 24, 24, WHITE);
        }
      }
      // We check if the bubble reward is currently active
      else if(add_on_state == 1 || add_on_state == 2)
      {
        // We check if the reward symbol is to be flashed on during its activation
        if(reward_activation_flash_flag)
        {
          // Draws the bubble activation bitmap
          display.drawBitmap(0, 20, bubble_activation_icon, 24, 24, WHITE);
        }
      }
      // We break out of the switch body
      break;
    }

    //- Settings Page -
    
    // We check if we are on the settings page
    case 1:
    {
      // The list of strings that make up the settings menu (excluding add on rewards)
      const PROGMEM char * const oled_add_on_list[] = {"Bubble Settings"};  
      // The list of strings that make up the settings menu (excluding add on rewards)
      const PROGMEM char * const oled_settings_page_list[] = {"Light Settings", "Sound Settings", "Generic Settings", "Test Rewards", "Remote Keypad", "Exit >"};
      // Stores the text x and y positions
      int16_t text_x, text_y;
      // Stores the text width and the text height
      uint16_t text_width, text_height;
  
      // We set the required font - size 15
      display.setFont(&Open_Sans_Light_15);
      // We check if the previous oled page was from the main page
      if(oled_previous_screen_state == 0)
      {
        // We update the oled cursor offset back to zero
        oled_cursor_offset_settings = 0;
      }
      
      // The top pixel row number of the current text pointed to by the cursor
      int oled_cursor_top = oled_cursor_offset_settings + oled_cursor_state * (oled_size_15_vertical_size + oled_size_15_vertical_gap);
      // The bottom pixel row number of the current text pointed to by the cursor
      int oled_cursor_bottom = oled_cursor_top + oled_size_15_vertical_size;
      // We check if the text has gone above the top row of pixels in the oled screen
      if(oled_cursor_top < 0)
      {
        // We update the oled cursor offset
        oled_cursor_offset_settings = oled_cursor_offset_settings - oled_cursor_top;
      }
      // We check if the text has gone below the bottom row of pixels in the oled screen
      else if(oled_cursor_bottom > 63)
      {
        // We update the oled cursor offset
        oled_cursor_offset_settings = oled_cursor_offset_settings - oled_cursor_bottom + 63;
      }
      // Used to store an offset for the add on modules
      byte oled_add_on_offset = 0;
      // We iterate through the possible menu options
      for(byte i=0; i < 6 + (add_on_type>0); i++)
      {
        // We obtain the cursor position needed to print the text
        oled_cursor_bottom = oled_cursor_offset_settings + i * (oled_size_15_vertical_size + oled_size_15_vertical_gap) + oled_size_15_vertical_size;
        // We set the cursor correspondingly
        display.setCursor(0, oled_cursor_bottom);
        // We check if the oled cursor state matches the current menu option
        if(oled_cursor_state == i)
        {
          // We check if we are on the add on reward option
          if(add_on_type > 0 && i == 2)
          {
            // We increment the oled add on offset to one
            oled_add_on_offset = 1;
            // We obtain the the bounding box around the text that we are to print
            display.getTextBounds(oled_add_on_list[add_on_type - 1], 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
            // We fill the bounding rectangle with white color
            display.fillRect(text_x, text_y, text_width, text_height, WHITE);
            // We set the text color to black
            display.setTextColor(BLACK);
            // We print the required text
            display.println(oled_add_on_list[add_on_type - 1]);
          }
          // we are not on the add on reward option
          else
          {
            // We obtain the the bounding box around the text that we are to print
            display.getTextBounds(oled_settings_page_list[i - oled_add_on_offset], 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
            // We fill the bounding rectangle with white color
            display.fillRect(text_x, text_y, text_width, text_height, WHITE);
            // We set the text color to black
            display.setTextColor(BLACK);
            // We print the required text
            display.println(oled_settings_page_list[i - oled_add_on_offset]);
          }
          // We set the text color back to white
          display.setTextColor(WHITE);
        }
        // we are not on a highlighted menu option
        else
        {
          // We check if we are on the add on reward option
          if(add_on_type > 0 && i == 2)
          {
            // We increment the oled add on offset to one
            oled_add_on_offset = 1;
            // We print the required add on reward text
            display.println(oled_add_on_list[add_on_type - 1]);
          }
          // we are not on the add on reward option
          else
          {
            // We print the required text
            display.println(oled_settings_page_list[i - oled_add_on_offset]);
          }
        }
      }
      // We break out of the switch body
      break;
    }

    //- Light Configuration Page -
    
    // We check if we are on the light reward configuration page
    case 2:
    {
      // The list of strings that make up the light configuration menu
      const PROGMEM char * const oled_light_page_list[] = {"Status", "Mode", "Frequency", "Color", "Component-R", "Component-G", "Component-B", "Brightness", "Duration", "Exit >"};
      // The list of strings that make up the light modes
      const PROGMEM char * const oled_light_modes[] = {"Still", "Pulse", "Fade", "Beacon", "Wipe", "Cross"};

      // The list of strings that make up the standard light color options
      const PROGMEM char * const light_preset_colors[] = {"Red", "Orange", "Turmeric", "Cheese", "Yellow", "Gold", 
                                                            "Chartreuse", "Pea", "Green", "Chayote", "Chrysolite", "Sea",
                                                            "Cyan", "Turquoise", "Azure", "Delphinium", "Blue", "Purple", 
                                                            "Violet", "Rose", "Magenta", "Purple", "Pink", "Strawberry",
                                                            "White"};

      // The corresponding RGB values
      const PROGMEM byte light_preset_colors_rgb[25][3] = {{255, 0, 0}, {255, 128, 0}, {255, 190, 0}, {255, 212, 0}, {255, 255, 0}, {222, 255, 0},
                                                           {188, 255, 0}, {158, 255, 0}, {0,255, 0}, {0, 255, 115}, {0, 255, 169}, {0, 255, 195},
                                                           {0, 255, 255}, {0, 195, 255}, {0, 170, 255}, {0, 111, 255}, {0, 0, 255}, {93, 0, 255},
                                                           {150, 0, 255}, {184, 0, 255}, {255, 0, 255}, {255, 0, 170}, {255, 0, 150}, {255, 0, 106},
                                                           {255, 255, 255}};

      // Stores the text x and y positions
      int16_t text_x, text_y;
      // Stores the text width and the text height
      uint16_t text_width, text_height;
      
      // We check if the previous oled page was from the settings menu
      if(oled_previous_screen_state == 1)
      {
        // We update the oled cursor offset back to zero
        oled_cursor_offset = 0;
      }
      // The top pixel row number of the current text pointed to by the cursor
      int oled_cursor_top = oled_cursor_offset + oled_cursor_state * (oled_size_13_vertical_size + oled_size_13_vertical_gap);
      // The bottom pixel row number of the current text pointed to by the cursor
      int oled_cursor_bottom = oled_cursor_top + oled_size_13_vertical_size;
      // We check if the text has gone above the top row of pixels in the oled screen
      if(oled_cursor_top < 0)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_top;
      }
      // We check if the text has gone below the bottom row of pixels in the oled screen
      else if(oled_cursor_bottom > 63)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_bottom + 63;
      }

      // This string will hold the name of the light preset selected
      String light_preset_selected;
      // We check if the light preset color is selected
      if(light_preset_color)
      {
        // We deselect the light preset color
        light_preset_color = false;
        // We update the light r channel
        light_r_channel = light_preset_colors_rgb[light_preset_color_option][0];
        // We update the light g channel
        light_g_channel = light_preset_colors_rgb[light_preset_color_option][1];
        // We update the light b channel
        light_b_channel = light_preset_colors_rgb[light_preset_color_option][2];
        // We ask to update the light static variables to reflect the RGB changes
        light_update = true;
        // The name of the color preset selected
        light_preset_selected = light_preset_colors[light_preset_color_option];
      }
      else
      {
        // We iterate through the 25 different preset color options
        for(byte i=0; i<25; i++)
        {
          // We check if the currrent RGB profile matches a light color preset
          if(light_preset_colors_rgb[i][0] == light_r_channel && light_preset_colors_rgb[i][1] == light_g_channel && light_preset_colors_rgb[i][2] == light_b_channel)
          {
            // We update the preset color option
            light_preset_color_option = i;
            // We indicate that we are using a preset 
            light_preset_color = true;
            // We break the execution of the for loop
            break;
          }
        }
        // We check if the light preset color is selected
        if(light_preset_color)
        {
          // We deselect the light preset color
          light_preset_color = false; 
          // The name of the color preset selected
          light_preset_selected = light_preset_colors[light_preset_color_option];
        }
        // The current color is a custom one
        else
        {
          // The name of the color preset selected
          light_preset_selected = "Custom";
        }
      }
      
      // The list of strings that make up the light configuration values
      String oled_light_configuration_values[] = {(bitRead(reward_activation_status,0) == 1)? "On":"Off", oled_light_modes[light_mode], 
                                                  String(light_frequency/10.0, 1), light_preset_selected, String(light_r_channel), String(light_g_channel), 
                                                  String(light_b_channel), String(light_brightness), String(light_duration/1000.0, 1), ""};
                                                  
      // We iterate through the possible configuration options
      for(byte i=0; i < 10; i++)
      {
        // We use this string to store the value that we are currently configuring
        String oled_configuration_value = oled_light_configuration_values[i];
        // We obtain the cursor position needed to print the text
        oled_cursor_bottom = oled_cursor_offset + i * (oled_size_13_vertical_size + oled_size_13_vertical_gap) + oled_size_13_vertical_size;
        // We set the required font - size 13
        display.setFont(&Open_Sans_Light_13);
        // We set the cursor correspondingly
        display.setCursor(0, oled_cursor_bottom);
        // We check if the oled cursor state matches the current menu option
        if(oled_cursor_state == i && !oled_cursor_toggle)
        {
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_light_page_list[i], 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // We fill the bounding rectangle with white color
          display.fillRect(text_x, text_y, text_width, text_height, WHITE);
          // We set the text color to black
          display.setTextColor(BLACK);
          // We print the required text
          display.println(oled_light_page_list[i]);
          // We set the text color back to white
          display.setTextColor(WHITE);
        }
        // we are not on a highlighted menu option
        else
        {
          // We print the required text
          display.println(oled_light_page_list[i]);
        }
        // We dont print anything on the right side for "Exit"
        if(i != 9)
        {
          // We set the required font - size 13 (Condensed)
          display.setFont(&Roboto_Condensed_Light_13);
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_configuration_value, 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // This is the new x location from where we will print the configuration text
          unsigned int oled_configuration_text_x = 127-text_width;
          // We set the cursor correspondingly
          display.setCursor(oled_configuration_text_x, oled_cursor_bottom);
          // We check if have toggled the configurations value and we are on the selected configuration
          if(oled_cursor_state == i && oled_cursor_toggle)
          {
            // We obtain the the bounding box around the text that we are to print
            display.getTextBounds(oled_configuration_value, oled_configuration_text_x, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
            // We fill the bounding rectangle with white color
            display.fillRect(text_x, text_y, text_width, text_height, WHITE);
            // We set the text color to black
            display.setTextColor(BLACK);
            // We print the required text
            display.println(oled_configuration_value);
            // We set the text color back to white
            display.setTextColor(WHITE);
          }
          // We have not toggled the configurations value
          else
          {
            // We print the required configuration text
            display.println(oled_configuration_value);
          }
        }
      }
      // We break out of the switch body
      break;
    }

    //- Sound Configuration Page -
    
    // We check if we are on the sound reward configuration page
    case 3:
    {
      // The list of strings that make up the sound configuration menu
      const PROGMEM char * const oled_sound_page_list[] = {"Status", "Album", "Track", "Loudness", "Repetition", "Clip Time", "Exit >"};
      // The list of sound albums
      const PROGMEM char * const sound_albums[] = {"Animal", "Household", "Musical", "Baby"};
      // Stores the text x and y positions
      int16_t text_x, text_y;
      // Stores the text width and the text height
      uint16_t text_width, text_height;
  
      // The list of sound tracks in the "Animal" folder
      const PROGMEM char * const sound_tracks_animal[] = {"Duck", "Cow", "Rooster", "Sheep", "Pig", "Goat",
      "Horse", "Cat", "Dog", "Owl", "Swallow", "Chick", "Lion", "Turkey", "Chimp", "Frog", "Elephant", 
      "Seagull", "Hawk", "Bee", "Cricket", "Bear", "Raven", "Hyena", "Snake", "Dolphin", "Whale", 
      "Bat", "Wolf", "Dove"};
      
      // The list of sound tracks in the "Household" folder
      const PROGMEM char * const sound_tracks_household[] = {"Bubbles", "vacuum", "Bath", "Dripping", "Toy Windup",
      "Toy Bricks", "Flush", "Water Drop", "Rain", "Balloon", "Toy Train", "Toy Car", "Toy Dog", 
      "Alarm Clock", "Horn", "Siren", "Toy Rattle", "Doorbell", "Toy Wheel", "Phone Ring", "Marbles", 
      "Toy Ratchet", "Bed", "Motorcycle", "Rubber Duck", "Bicycle Bell", "Coin", "Static", "Cuckoo Clock", 
      "Table Fan"};
      
      // The list of sound tracks in the "Musical" folder
      const PROGMEM char * const sound_tracks_musical[] = {"Alto Sax A3", "Alto Sax A4", "Alto Sax A5", "Bass Clarinet A2", 
      "Bass Clarinet A3", "Bass Clarinet A4", "Bass Clarinet A5", "Flute A4", "Flute A5", "Flute A6", 
      "Alto Flute A3", "Alto Flute A4", "Alto Flute A5", "Bass Flute A3", "Bass Flute A4", "Bass Flute A5",
      "Oboe A4", "Oboe A5", "Eb Clarinet A3", "Eb Clarinet A4", "Eb Clarinet A5", "Eb Clarinet A6", 
      "Bb Clarinet A3", "Bb Clarinet A4", "Bb Clarinet A5", "Bb Clarinet A6", "Bassoon A2", "Bassoon A3", 
      "Bassoon A4", "Sop Sax A3", "Sop Sax A4", "Sop Sax A5", "Alto Sax A3", "Alto Sax A4", "Horn A2",
      "Horn A3", "Horn A4", "Trumpet A3", "Trumpet A4", "Trumpet A5", "Trem Trombone A2", "Trem Trombone A3",
      "Trem Trombone A4", "Bass Trombone A1", "Bass Trombone A2", "Tuba A1", "Tuba A2", "Tuba A3",
      "Violin Arco A4", "Violin Arco A5", "Violin Pizz A4", "Violin Pizz A5", "Viola Arco A4",
      "Viola Arco A5", "Viola Arco A6", "Viola Pizz A4", "Viola Pizz A5", "Viola Pizz A6", "Cello Arco A3",
      "Cello Arco A4", "Cello Arco A5", "Cello Pizz A3", "Cello Pizz A4", "Cello Pizz A5", "Music Box", 
      "Buzzer", "Bass Pizz A1", "Bass Pizz A2", "Bass Pizz A3", "Marimba A2", "Marimba A3", "Marimba A4",
      "Marimba A5", "Marimba A6", "Xylophone A4", "Xylophone A5", "Xylophone A6", "Xylophone A7", 
      "Vibraphone A3", "Vibraphone A4", "Vibraphone A5", "Bells A5", "Bells A6", "Bells A7", "Crotale A6", 
      "Crotale A7", "Cymbals Orch", "Cymbals Choke", "Cymbals Hihat", "Cymbals Splash", "Wind Gong 20",
      "Tamtam 22", "Tamtam 28", "Tamtam 40", "Thai Gong A4", "Thai Gong B4", "Thai Gong C4", "Thai Gong E4", 
      "Thai Gong F4", "Thai Gong G4", "Whistle", "Guiro", "Triangle 6", "Triangle 8", "Tambourine 1", 
      "Tambourine 2", "Tambourine 3", "Balloon Pop", "Harp", "Church Bell", "Guitar A2B2", "Guitar C3B3", 
      "Guitar D2B3", "Guitar E4B4", "Guitar C2B5", "Bass Arco A1", "Bass Arco A2", "Bass Arco A3", "Piano A1",
      "Piano A2", "Piano A3", "Piano A4", "Piano A5", "Piano A6", "Piano A7"};
      
      // The list of sound tracks in the "Baby" folder
      const PROGMEM char * const sound_tracks_baby[] = {"Laugh", "Cheer", "Goodbye", "Hello", "Bye Bye", "Snoring",
      "Sneeze", "Scream", "Giggles", "Yummy", "Annoyed", "Cry", "Shout", "Happy", "Hiccups", "Lip Flicking",
      "Cough", "Disgruntled", "Mommy", "Daddy", "Babbling", "Cooing", "Murmur", "Call Over", "Yay"};
   
      // We check if the previous oled page was from the settings menu
      if(oled_previous_screen_state == 1)
      {
        // We update the oled cursor offset back to zero
        oled_cursor_offset = 0;
      }
      // The top pixel row number of the current text pointed to by the cursor
      int oled_cursor_top = oled_cursor_offset + oled_cursor_state * (oled_size_13_vertical_size + oled_size_13_vertical_gap);
      // The bottom pixel row number of the current text pointed to by the cursor
      int oled_cursor_bottom = oled_cursor_top + oled_size_13_vertical_size;
      // We check if the text has gone above the top row of pixels in the oled screen
      if(oled_cursor_top < 0)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_top;
      }
      // We check if the text has gone below the bottom row of pixels in the oled screen
      else if(oled_cursor_bottom > 63)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_bottom + 63;
      }

      // This variable is used to store the OLED sound track
      String oled_sound_track;
      // We switch amongst the sound albums
      switch(sound_album)
      {
        // We are on the animals album
        case 0:
          // We obtain the corresponding sound track
          oled_sound_track = sound_tracks_animal[sound_track];
          // We break the switch body
          break;
        // We are on the household album
        case 1:
          // We obtain the corresponding sound track
          oled_sound_track = sound_tracks_household[sound_track];
          // We break the switch body
          break;
        // We are on the musical album
        case 2:
          // We obtain the corresponding sound track
          oled_sound_track = sound_tracks_musical[sound_track];
          // We break the switch body
          break;
        // We are on the baby album
        case 3:
          // We obtain the corresponding sound track
          oled_sound_track = sound_tracks_baby[sound_track];
          // We break the switch body
          break;
      }
      
      // The list of strings that make up the sound configuration values
      String oled_sound_configuration_values[] = {(bitRead(reward_activation_status,1) == 1)? "On":"Off", sound_albums[sound_album], 
                                                  oled_sound_track, String(sound_loudness), String(sound_repetition), 
                                                  String(sound_clip_time/1000.0, 1), ""};
                                                  
      // We iterate through the possible configuration options
      for(byte i=0; i < 7; i++)
      {
        // We use this string to store the value that we are currently configuring
        String oled_configuration_value = oled_sound_configuration_values[i];
        // We obtain the cursor position needed to print the text
        oled_cursor_bottom = oled_cursor_offset + i * (oled_size_13_vertical_size + oled_size_13_vertical_gap) + oled_size_13_vertical_size;
        // We set the required font - size 13
        display.setFont(&Open_Sans_Light_13);
        // We set the cursor correspondingly
        display.setCursor(0, oled_cursor_bottom);
        // We check if the oled cursor state matches the current menu option
        if(oled_cursor_state == i && !oled_cursor_toggle)
        {
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_sound_page_list[i], 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // We fill the bounding rectangle with white color
          display.fillRect(text_x, text_y, text_width, text_height, WHITE);
          // We set the text color to black
          display.setTextColor(BLACK);
          // We print the required text
          display.println(oled_sound_page_list[i]);
          // We set the text color back to white
          display.setTextColor(WHITE);
        }
        // we are not on a highlightd menu option
        else
        {
          // We print the required text
          display.println(oled_sound_page_list[i]);
        }
        // We dont print anything on the right side for "Exit"
        if(i != 6)
        {
          // We set the required font - size 13 (Condensed)
          display.setFont(&Roboto_Condensed_Light_13);
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_configuration_value, 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // This is the new x location from where we will print the configuration text
          unsigned int oled_configuration_text_x = 127-text_width;
          // We set the cursor correspondingly
          display.setCursor(oled_configuration_text_x, oled_cursor_bottom);
          // We check if have toggled the configurations value and we are on the selected configuration
          if(oled_cursor_state == i && oled_cursor_toggle)
          {
            // We obtain the the bounding box around the text that we are to print
            display.getTextBounds(oled_configuration_value, oled_configuration_text_x, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
            // We fill the bounding rectangle with white color
            display.fillRect(text_x, text_y, text_width, text_height, WHITE);
            // We set the text color to black
            display.setTextColor(BLACK);
            // We print the required text
            display.println(oled_configuration_value);
            // We set the text color back to white
            display.setTextColor(WHITE);
          }
          // We have not toggled the configurations value
          else
          {
            // We print the required configuration text
            display.println(oled_configuration_value);
          }
        }
      }
      // We break out of the switch body
      break;
    }

    //- Bubble Configuration Page -
    
    // We check if we are on the sound reward configuration page
    case 4:
    {
      // The list of strings that make up the bubble configuration menu
      const PROGMEM char * const oled_bubble_page_list[] = {"Status", "Duration", "Density", "Exit >"};
      // Stores the text x and y positions
      int16_t text_x, text_y;
      // Stores the text width and the text height
      uint16_t text_width, text_height;
      
      // We check if the previous oled page was from the settings menu
      if(oled_previous_screen_state == 1)
      {
        // We update the oled cursor offset back to zero
        oled_cursor_offset = 0;
      }
      // The top pixel row number of the current text pointed to by the cursor
      int oled_cursor_top = oled_cursor_offset + oled_cursor_state * (oled_size_13_vertical_size + oled_size_13_vertical_gap);
      // The bottom pixel row number of the current text pointed to by the cursor
      int oled_cursor_bottom = oled_cursor_top + oled_size_13_vertical_size;
      // We check if the text has gone above the top row of pixels in the oled screen
      if(oled_cursor_top < 0)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_top;
      }
      // We check if the text has gone below the bottom row of pixels in the oled screen
      else if(oled_cursor_bottom > 63)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_bottom + 63;
      }
      
      // The list of strings that make up the sound configuration values
      String oled_bubble_configuration_values[] = {(bitRead(reward_activation_status,2) == 1)? "On":"Off", String(bubble_duration/1000.0,1), String(bubble_motor_control) ,""};
                                                  
      // We iterate through the possible configuration options
      for(byte i=0; i < 4; i++)
      {
        // We use this string to store the value that we are currently configuring
        String oled_configuration_value = oled_bubble_configuration_values[i];
        // We obtain the cursor position needed to print the text
        oled_cursor_bottom = oled_cursor_offset + i * (oled_size_13_vertical_size + oled_size_13_vertical_gap) + oled_size_13_vertical_size;
        // We set the required font - size 13
        display.setFont(&Open_Sans_Light_13);
        // We set the cursor correspondingly
        display.setCursor(0, oled_cursor_bottom);
        // We check if the oled cursor state matches the current menu option
        if(oled_cursor_state == i && !oled_cursor_toggle)
        {
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_bubble_page_list[i], 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // We fill the bounding rectangle with white color
          display.fillRect(text_x, text_y, text_width, text_height, WHITE);
          // We set the text color to black
          display.setTextColor(BLACK);
          // We print the required text
          display.println(oled_bubble_page_list[i]);
          // We set the text color back to white
          display.setTextColor(WHITE);
        }
        // we are not on a highlightd menu option
        else
        {
          // We print the required text
          display.println(oled_bubble_page_list[i]);
        }
        // We dont print anything on the right side for "Exit"
        if(i != 3)
        {
          // We set the required font - size 13 (Condensed)
          display.setFont(&Roboto_Condensed_Light_13);
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_configuration_value, 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // This is the new x location from where we will print the configuration text
          unsigned int oled_configuration_text_x = 127-text_width;
          // We set the cursor correspondingly
          display.setCursor(oled_configuration_text_x, oled_cursor_bottom);
          // We check if have toggled the configurations value and we are on the selected configuration
          if(oled_cursor_state == i && oled_cursor_toggle)
          {
            // We obtain the the bounding box around the text that we are to print
            display.getTextBounds(oled_configuration_value, oled_configuration_text_x, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
            // We fill the bounding rectangle with white color
            display.fillRect(text_x, text_y, text_width, text_height, WHITE);
            // We set the text color to black
            display.setTextColor(BLACK);
            // We print the required text
            display.println(oled_configuration_value);
            // We set the text color back to white
            display.setTextColor(WHITE);
          }
          // We have not toggled the configurations value
          else
          {
            // We print the required configuration text
            display.println(oled_configuration_value);
          }
        }
      }
      // We break out of the switch body
      break;
    }

    //- Generic Settings Page -

    // We check if we are on the generic settings page
    case 5:
    {
      // The list of strings that make up the generic settings menu
      const PROGMEM char * const oled_generic_settings_page_list[] = {"Interact Mode", "Exit >"};
      // Stores the text x and y positions
      int16_t text_x, text_y;
      // Stores the text width and the text height
      uint16_t text_width, text_height;
      
      // We check if the previous oled page was from the settings menu
      if(oled_previous_screen_state == 1)
      {
        // We update the oled cursor offset back to zero
        oled_cursor_offset = 0;
      }
      // The top pixel row number of the current text pointed to by the cursor
      int oled_cursor_top = oled_cursor_offset + oled_cursor_state * (oled_size_13_vertical_size + oled_size_13_vertical_gap);
      // The bottom pixel row number of the current text pointed to by the cursor
      int oled_cursor_bottom = oled_cursor_top + oled_size_13_vertical_size;
      // We check if the text has gone above the top row of pixels in the oled screen
      if(oled_cursor_top < 0)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_top;
      }
      // We check if the text has gone below the bottom row of pixels in the oled screen
      else if(oled_cursor_bottom > 63)
      {
        // We update the oled cursor offset
        oled_cursor_offset = oled_cursor_offset - oled_cursor_bottom + 63;
      }
      
      // The list of strings that make up the sound configuration values
      String oled_generic_settings_configuration_values[] = {(reward_configuration_display == 1)? "On":"Off", ""};
                                                  
      // We iterate through the possible configuration options
      for(byte i=0; i < 2; i++)
      {
        // We use this string to store the value that we are currently configuring
        String oled_configuration_value = oled_generic_settings_configuration_values[i];
        // We obtain the cursor position needed to print the text
        oled_cursor_bottom = oled_cursor_offset + i * (oled_size_13_vertical_size + oled_size_13_vertical_gap) + oled_size_13_vertical_size;
        // We set the required font - size 13
        display.setFont(&Open_Sans_Light_13);
        // We set the cursor correspondingly
        display.setCursor(0, oled_cursor_bottom);
        // We check if the oled cursor state matches the current menu option
        if(oled_cursor_state == i && !oled_cursor_toggle)
        {
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_generic_settings_page_list[i], 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // We fill the bounding rectangle with white color
          display.fillRect(text_x, text_y, text_width, text_height, WHITE);
          // We set the text color to black
          display.setTextColor(BLACK);
          // We print the required text
          display.println(oled_generic_settings_page_list[i]);
          // We set the text color back to white
          display.setTextColor(WHITE);
        }
        // we are not on a highlighted menu option
        else
        {
          // We print the required text
          display.println(oled_generic_settings_page_list[i]);
        }
        // We dont print anything on the right side for "Exit"
        if(i != 1)
        {
          // We set the required font - size 13 (Condensed)
          display.setFont(&Roboto_Condensed_Light_13);
          // We obtain the the bounding box around the text that we are to print
          display.getTextBounds(oled_configuration_value, 0, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
          // This is the new x location from where we will print the configuration text
          unsigned int oled_configuration_text_x = 127-text_width;
          // We set the cursor correspondingly
          display.setCursor(oled_configuration_text_x, oled_cursor_bottom);
          // We check if have toggled the configurations value and we are on the selected configuration
          if(oled_cursor_state == i && oled_cursor_toggle)
          {
            // We obtain the the bounding box around the text that we are to print
            display.getTextBounds(oled_configuration_value, oled_configuration_text_x, oled_cursor_bottom, &text_x, &text_y, &text_width, &text_height);
            // We fill the bounding rectangle with white color
            display.fillRect(text_x, text_y, text_width, text_height, WHITE);
            // We set the text color to black
            display.setTextColor(BLACK);
            // We print the required text
            display.println(oled_configuration_value);
            // We set the text color back to white
            display.setTextColor(WHITE);
          }
          // We have not toggled the configurations value
          else
          {
            // We print the required configuration text
            display.println(oled_configuration_value);
          }
        }
      }
      // We break out of the switch body
      break;
    }

    //- Remote Keypad Info Page -
    
    // The remote keypad info is presented
    case 6:
      // We use the default font
      display.setFont();
      // We set the cursor back at (0, 0)
      display.setCursor(0, 0);
      // We print out all the remote keypad information
      display.println("1 - Light");
      display.println("2 - Sound");
      display.println("3 - Add On");
      display.println("4 - Light and Sound");
      display.println("5 - Sound and Add On");
      display.println("6 - Light and Add On");
      display.println("8 - All Rewards");
      display.println("0 - Activated Rewards");
    // We break out of the switch body
    break;
  }
  // We display the current OLED screen configuration
  display.display();
  // We update the previous OLED screen state
  oled_previous_screen_state = oled_screen_state;
  // We reset the oled action to zero
  oled_action = 0;
}

// ----- Reward restarter function -----

// This function helps restart rewards during the reward configuration modes
void reward_configuration_restarter()
{
  // This variable holds the time in which the configured reward was halted
  static unsigned long  reward_end_time;
  // We check if any reward is being maintained
  if(reward_configuration)
  {
    // We check if we are on the light configuration page and it is inactive
    if(oled_screen_state == 2 && light_state == 0)
    {
      // We check if we have noticed the end of the reward already
      if(reward_configuration_end_unnoticed)
      {
        // We noticed that the reward was stopped during confiuration
        reward_configuration_end_unnoticed = false;
        // We set the time at which we noticed the reward to have ended
        reward_end_time = millis();
      }
      // We check if we have passed enough time since the end of the reward
      else if(abs(millis() - reward_end_time) > light_revival_time)
      {
        // We did not notice that the reward was stopped during confiuration
        reward_configuration_end_unnoticed = true;
        // We activate the light reward
        light_state = 1;
      }
    }
    // We check if we are on the sound configuration page and it is inactive
    else if(oled_screen_state == 3 && sound_state == 0)
    {
      // We check if we have noticed the end of the reward already
      if(reward_configuration_end_unnoticed)
      {
        // We noticed that the reward was stopped during confiuration
        reward_configuration_end_unnoticed = false;
        // We set the time at which we noticed the reward to have ended
        reward_end_time = millis();
      }
      // We check if we have passed enough time since the end of the reward
      else if(abs(millis() - reward_end_time) > sound_revival_time)
      {
        // We did not notice that the reward was stopped during confiuration
        reward_configuration_end_unnoticed = true;
        // We activate the sound reward
        sound_state = 1;
      }
    }
    // We check if we are on the bubble configuration page and it is inactive
    else if(oled_screen_state == 4 && add_on_state == 0)
    {
      // We check if we have noticed the end of the reward already
      if(reward_configuration_end_unnoticed)
      {
        // We noticed that the reward was stopped during confiuration
        reward_configuration_end_unnoticed = false;
        // We set the time at which we noticed the reward to have ended
        reward_end_time = millis();
      }
      // We check if we have passed enough time since the end of the reward
      else if(abs(millis() - reward_end_time) > bubble_revival_time)
      {
        // We did not notice that the reward was stopped during confiuration
        reward_configuration_end_unnoticed = true;
        // We activate the bubble reward
        add_on_state = 1;
      }
    }
  }
}

// ----- Reward activation function -----

// This function decides if we can activate the different rewards based on their status and configuration override
void reward_activation(bool light_status, bool sound_status, bool add_on_status, bool configuration_override = false)
{
  // We check if we are not configuring any reward
  if(!reward_configuration || configuration_override)
  {
    // We check if the light reward is to be activated
    if(light_status)
    {
      // We activate the light reward
      light_state = 1;
    }
    // The light reward is to be deactivated
    else if(light_state != 0)
    {
      // We deactivate the light reward
      light_state = 3;
    }
    // We check if the sound reward is to be activated
    if(sound_status)
    {
      // We activate the sound reward
      sound_state = 1;
    }
    // The sound reward is to be deactivated
    else if(sound_state != 0)
    {
      // We deactivate the sound reward
      sound_state = 4;
    }
    // We check if the add on reward is to be activated and we have a valid add on module
    if(add_on_status && add_on_type > 0)
    {
      // We activate the add on reward
      add_on_state = 1;
    }
    // The add on reward is to be deactivated
    else if(add_on_state != 0 && !(add_on_type == 1 && add_on_state == 3))
    {
      // We deactivate the add on reward
      add_on_state = 4;
    }
  }
}

// ----- Light reward functions -----

// Maintains the ongoing light reward
void light_reward_maintain()
{
  // Creating an array to store the previous RGB colors of each pixel in the led panel
  static CRGB led_panel_previous[led_panel_pixel_count];
  // Used to store the exact time in which the light reward was activated
  static unsigned long light_refresh_time = 0;
  // Static variables to hold the total of the RGB components and the reward time period
  static word light_rgb_total, light_time_period;
  // Static variables to hold the normalized R, G and B components
  static byte light_norm_r, light_norm_g, light_norm_b;

  // We check if we have to update the static variables
  if(light_update)
  {
    // The total of the R, G and B components
    light_rgb_total = light_r_channel + light_g_channel + light_b_channel;
    // We obtain the normalized R component
    light_norm_r = double(light_r_channel) / light_rgb_total * light_brightness;
    // We obtain the normalized G component
    light_norm_g = double(light_g_channel) / light_rgb_total * light_brightness;
    // We obtain the normalized B component
    light_norm_b = double(light_b_channel) / light_rgb_total * light_brightness;
    // We obtain the waveform time period in milliseconds
    light_time_period = 10000.0/light_frequency;
    // We set the light update to false
    light_update = false;
  }

  // state 1 - light timer is initialized
  if(light_state == 1)
  {
    // The light start time is updated
    light_refresh_time = millis();
    // We did not notice that the reward was stopped during confiuration
    reward_configuration_end_unnoticed = true;
    // We update the state to the next one
    light_state = 2;
  }
  // state 2 - light pattern decided based on time passed
  else if(light_state == 2)
  {
    // We initialize the CRGB light color object
    CRGB light_color;
    // We obtain the time that has passed in current cycle of the waveform
    word light_cycle_time = abs(millis() - light_refresh_time) % light_time_period;
    // We check if the light duration has expired and are not configuring a reward
    if(abs(millis()-light_refresh_time) > light_duration)
    {
      light_color = CRGB(0,0,0);
      // We iterate through all the pixels of the LED display
      for(int i=0; i<led_panel_pixel_count; i++)
      {
        // We turn the led panel pixel off
        led_panel[i] = light_color;
      }
      // We shift the light state back to zero
      light_state = 0;
    }
    // We check if light mode is "Still", "Pulse" or "Fade"
    else if(light_mode < 3)
    {
      // We check if the light mode is "Still"
      if(light_mode == 0)
      {
          // The selected RGB color is directly used as it is "Still"
          light_color = CRGB(light_norm_g, light_norm_r, light_norm_b);  
      }
      // We check if the light mode is "Pulse"
      else if(light_mode == 1)
      {
          // We check if we are at the first half of the pulse waveform
          if (2 * light_cycle_time < light_time_period)
          {
            // The selected RGB color is directly used
            light_color = CRGB(light_norm_g, light_norm_r, light_norm_b);  
          }
          else
          {
            // The RGB color is set to zeros (no lighting)
            light_color = CRGB(0, 0, 0); 
          }
      }
      // light mode is "Fade"
      else
      {
        // We check if we are at the first half of the "Fade" waveform
        if (2 * light_cycle_time < light_time_period)
        {
          // We calculate the brightness factor
          double brightness_factor = 2 * double(light_cycle_time) / light_time_period;
          // The selected RGB color is directly used
          light_color = CRGB(byte(light_norm_g * brightness_factor), byte(light_norm_r * brightness_factor), byte(light_norm_b * brightness_factor));  
        }
        // We check if we are at the second half of the "Fade" waveform
        else
        {
          // We calculate the brightness factor
          double brightness_factor = 2 * double(light_time_period - light_cycle_time) / light_time_period;
          // The selected RGB color is directly used
          light_color = CRGB(byte(light_norm_g * brightness_factor), byte(light_norm_r * brightness_factor), byte(light_norm_b * brightness_factor)); 
        }
      }
      // We iterate through all the pixels of the LED display
      for(word i=0; i<led_panel_pixel_count; i++)
      {
        // We turn the led panel pixel to the required RGB settings
        led_panel[i] = light_color;  
      }
    }
    // We check if light mode is "Beacon"
    else if(light_mode==3)
    {
      // We create an array to store the illuminated columns of the led panel
      int beacon_columns[led_beacon_width];
      // We calculate the first column that is illuminated
      beacon_columns[0] = floor(led_panel_pixel_columns * double(light_cycle_time) / light_time_period);
      // We iterate through the other 4 columns
      for(byte i=1; i<led_beacon_width; i++)
      {
        // We check if the current column is not exceeding the maximum number of columns in the led panel
        if(beacon_columns[0] + i < led_panel_pixel_columns)
        {
          // The next beacon column is identified
          beacon_columns[i] = beacon_columns[0] + i;
        }
        // Current column is exceeding the maximum number of columns in the led panel
        else
        {
          // The next beacon column is identified
          beacon_columns[i] = beacon_columns[0] + i - led_panel_pixel_columns;
        }
      }
      // We iterate through all the columns of the LED panel
      for(int i=0; i<led_panel_pixel_columns; i++)
      {
        // This flag checks if the current column is a beacon column
        bool beacon_column_flag = false;
        // We iterate through the beacon columns
        for(byte j=0; j<led_beacon_width; j++)
        {
          // We check if the current column is stored in the beacon column array
          if(beacon_columns[j]==i)
          {
            // We indicate that we are dealing with a beacon column
            beacon_column_flag = true;
            // We break out of the for loop
            break;
          }
        }
        // We check if we are dealing with a beacon column
        if(beacon_column_flag)
        {
          // We check if the current column is at the beacon extremes
          if(i == beacon_columns[0] || i == beacon_columns[led_beacon_width-1])
          {
            // We iterate through all the pixels of the current column
            for(int j=i*led_panel_pixel_rows+1; j<(i+1)*led_panel_pixel_rows-1; j++)
            {
              // We turn the led panel pixel to zeros
              led_panel[j] = CRGB(light_norm_g, light_norm_r, light_norm_b);  
            }
            // We make the pixels at the two extremes of the column zero
            led_panel[i*led_panel_pixel_rows] = CRGB(0, 0, 0);
            led_panel[(i+1)*led_panel_pixel_rows-1] = CRGB(0, 0, 0); 
          }
          // The current column is in the middle of the beacon display
          else
          {
            // We iterate through all the pixels of the current column
            for(int j=i*led_panel_pixel_rows; j<(i+1)*led_panel_pixel_rows; j++)
            {
              // We turn the led panel pixel to zeros
              led_panel[j] = CRGB(light_norm_g, light_norm_r, light_norm_b);  
            }
          }
        }
        // We are not dealing with a beacon column
        else
        {
          // We iterate through all the pixels of the current column
          for(int j=i*led_panel_pixel_rows; j<(i+1)*led_panel_pixel_rows; j++)
          {
            // We turn the led panel pixel to zeros
            led_panel[j] = CRGB(0, 0, 0);  
          }
        }
      }
    }
    // We check if light mode is "Wipe" or "Cross"
    else if(light_mode>3)
    {
      // We obtain the height to which we must display the pixels
      byte wipe_height = floor(9 * double(light_cycle_time) / light_time_period);
      // check if light mode is "Wipe"
      if(light_mode==4)
      {
        // We iterate through all the pixels of the LED display
        for(word i=0; i<led_panel_pixel_count; i++)
        {
          if(i%16 < 8 && (7-i%8) < wipe_height || i%16 >= 8 && i%8 < wipe_height)
          {
            // We turn the led panel pixel to the required RGB settings
            led_panel[i] = CRGB(light_norm_g, light_norm_r, light_norm_b);
          }
          else
          {
            // We turn the led panel pixel to all zeros
            led_panel[i] = CRGB(0, 0, 0);
          }
        }  
      } 
      // check if light mode is "Cross"
      else if(light_mode==5)
      {
        // We iterate through all the pixels of the LED display
        for(word i=0; i<led_panel_pixel_count; i++)
        {
          // We check if the pixel is to be activated
          if(i%8 < wipe_height)
          {
            // We turn the led panel pixel to the required RGB settings
            led_panel[i] = CRGB(light_norm_g, light_norm_r, light_norm_b);
          }
          else
          {
            // We turn the led panel pixel to all zeros
            led_panel[i] = CRGB(0, 0, 0);
          }
        }  
      } 
    }
    // This variable checks if the LED panel needs to be updated
    bool panel_update = false;
    // We iterate through all the pixels
    for(word i=0; i<led_panel_pixel_count; i++)
    {
      // We check if the current pixel is different from that used previously
      if(led_panel[i] != led_panel_previous[i])
      {
        // We indicate that the panel requires to be updated
        panel_update = true;
        // We break the for loop
        break;
      }
    }
    // We check if the LED panel is to be updated
    if(panel_update)
    {
      // We update the previous record of the led panel
      for(int i=0; i<256; i++)
      {
        // We update the current value of the led panel in the previous one
        led_panel_previous[i] = led_panel[i];
      }
      // Update the LED display
      FastLED.show();
    }
  }
  // state 3 - emergency stop
  else if(light_state == 3)
  {
    // We initialize the CRGB light color object
    CRGB light_color = CRGB(0,0,0);
    // This variable checks if the LED panel needs to be updated
    bool panel_update = false;
    // We iterate through all the pixels
    for(word i=0; i<led_panel_pixel_count; i++)
    {
      // We check if the current pixel is different from that used previously
      if(led_panel[i] != light_color)
      {
        // We indicate that the panel requires to be updated
        panel_update = true;
        // We store zeros in those positions of the led panel
        led_panel[i] = light_color;
      }
    }
    // We check if the LED panel is to be updated
    if(panel_update)
    {
      // We update the previous record of the led panel
      for(int i=0; i<256; i++)
      {
        // We update the current value of the led panel in the previous one
        led_panel_previous[i] = led_panel[i];
      }
      // Update the LED display
      FastLED.show();
    }
    // We set the light state back to zero
    light_state = 0;
  }
  // state 4 - emergency stop without check
  else if(light_state == 4)
  {
    // We initialize the CRGB light color object
    CRGB light_color = CRGB(0,0,0);
    // We iterate through all the pixels
    for(word i=0; i<led_panel_pixel_count; i++)
    {
      // We store zeros in those positions of the led panel
      led_panel[i] = light_color;
    }
    // We update the previous record of the led panel
    for(int i=0; i<256; i++)
    {
      // We update the current value of the led panel in the previous one
      led_panel_previous[i] = led_panel[i];
    }
    // Update the LED display
    FastLED.show();
    // We set the light state back to zero
    light_state = 0;
  }
}

// ----- Sound reward functions -----

// Maintains the ongoing sound reward
void sound_reward_maintain()
{
  // Initialized to no sound is playing
  static bool sound_playing = false;
  // Used to store the exact time in which the sound reward modes are switched
  static unsigned long sound_refresh_time = 0;
  // Inicates the current sound sound cycle if repeating
  static byte sound_current_cycle = 0;
  // This timestamp helps regulate subsequent sound reward maintenance calls
  static unsigned long sound_maintenance_time = 0;
  // We check if we havent maintained the sound in more time than the specified gap
  if(abs(millis() - sound_maintenance_time) >= sound_maintenance_gap)
  {
    // We update the sound maintenance time
    sound_maintenance_time = millis();
    // state 1 - the sound reward is to be activated
    if(sound_state == 1)
    {
      // We check if a sound is already playing
      if(sound_playing)
      {
        // We stop the current track being played
        sound_stop_track();
        // We specify that the next state is to play the sound
        sound_state = 3;
      }
      else
      {
        // We play the sound album and track
        sound_play_track(sound_album+1, sound_track+1);
        // We mark that the sound reward is activated
        sound_state = 2;
      }
      // We did not notice that the reward was stopped during confiuration
      reward_configuration_end_unnoticed = true;
      // We record the sound start time in milliseconds
      sound_refresh_time = millis();
      // We specify that the current number of sound cycles completed is zero
      sound_current_cycle = 0;
    }
    // state 2 - the sound reward is to be stopped
    else if(sound_state == 2)
    {
      // We check if a sound is playing and needs to be clipped
      if(abs(millis() - sound_refresh_time) >= sound_clip_time)
      {
        // We increment the sound cycle by one
        sound_current_cycle++;
        // We deactivate the sound reward if we have completed the required number of cycles and are not configuring a reward
        if(sound_current_cycle == sound_repetition)
        {
          // We mark that the sound reward is to be terminated
          sound_state = 0;
        }
        else
        {
          // We shift the state to 3 - check when to restart
          sound_state = 3;
        }
        // We stop the current track being played
        sound_stop_track();
        // We mark the time that the sound has been stopped
        sound_refresh_time = millis();
        // We specify that we have stopped running the current sound
        sound_playing =  false;
      }
    }
    // state 3 - the sound reward is to be restarted
    else if(sound_state == 3)
    {
      // We check if the sound has been stopped for atleast 25ms before starting the next cycle
      if(abs(millis()-sound_refresh_time) > 25)
      {
        // We play the sound album and track
        sound_play_track(sound_album+1, sound_track+1);
        // We mark the time that the sound has been started
        sound_refresh_time = millis();
        // We shift the state to 2 - check when to stop the sound
        sound_state = 2;
        // We mark that a sound is playing
        sound_playing = true;
      }
    }
    // state 4 - emergency stop
    else if(sound_state == 4)
    {
      // We check if sound is playing
      if(sound_playing)
      {
        // We stop the current track being played
        sound_stop_track();
      }
      // We reset the sound state back to zero
      sound_state=0;
    }
  }
}

// This function helps to set the sound volume
void sound_volume_set(byte sound_level)
{
  // We maintain the sound level between 0-30
  sound_level=max(0, min(sound_level, 30));
  // Sends serial commands to set the sound level
  Serial2.write(0x7E);
  Serial2.write(0xFF);
  Serial2.write(0x06);
  Serial2.write(0x06);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(sound_level);
  Serial2.write(0xEF);
}

// This function helps to play a track in a specified album
void sound_play_track(byte album, byte track)
{
  // We set the two audio amplifiers to come out of shutdown
  digitalWrite(left_audio_amplifier_shutdown, HIGH);
  digitalWrite(right_audio_amplifier_shutdown, HIGH);
  // Sends serial commands to set the sound level
  Serial2.write(0x7E);
  Serial2.write(0xFF);
  Serial2.write(0x06);
  Serial2.write(0x0F);
  Serial2.write(0x00);
  Serial2.write(album);
  Serial2.write(track);
  Serial2.write(0xEF);
}

// This function helps to stop the track being played
void sound_stop_track()
{
  // We set the two audio amplifiers to shutdown when not in use
  digitalWrite(left_audio_amplifier_shutdown, LOW);
  digitalWrite(right_audio_amplifier_shutdown, LOW);
  // Sends serial commands to set the sound level
  Serial2.write(0x7E);
  Serial2.write(0xFF);
  Serial2.write(0x06);
  Serial2.write(0x16);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0xEF);
}

// ----- Add on reward function -----

// This function should be called if the bubble module is inserted. Maintains bubble reward
void add_on_reward_maintain()
{
  // Used to store the exact time in which the add on reward was activated
  static unsigned long add_on_refresh_time = 0;
  
  // We check if the add on type is the bubble module
  if(add_on_type == 1)
  {
    // state 0 - waiting for a bubble refresh opening
    if(add_on_state == 0)
    {
      // In case the reward base just got activated or we havent refreshed in some time
      if(add_on_refresh_time == 0 ||  abs(millis() - add_on_refresh_time) > bubble_refresh_interval)
      {
        // We specify that we have started a bubble refresh opening
        add_on_state = 3;
        // We update the bubble refresh time
        add_on_refresh_time = millis();
        // We enable the motor driver enable pin to spin the bubble reward motor
        digitalWrite(motor_driver_enable_B, HIGH);
        // We switch on the geared motor that dips loops into the bubble solution
        analogWrite(motor_driver_input_4, 255);
      }
    }
    // state 1 - the bubble reward is to be activated
    else if(add_on_state == 1)
    {
      // We specify that the add on module is now activated
      add_on_state = 2;
      // We store the time at which the add on was activated
      add_on_refresh_time = millis();
      // We did not notice that the reward was stopped during confiuration
      reward_configuration_end_unnoticed = true;
      // We enables the motor driver enable pins to use the bubble reward
      digitalWrite(motor_driver_enable_B, HIGH);
      // We switch on the radial fan that blows bubbles upwards
      analogWrite(motor_driver_input_3, bubble_fan_control);
      // We switch on the geared motor that dips loops into the bubble solution
      analogWrite(motor_driver_input_4, bubble_motor_control);
    }
    // state 2 - the bubble reward is already activated and is to be deactivated
    else if(add_on_state == 2)
    {
      // We check whether we have provided bubbles for the desired duration and are not configuring a reward
      if(abs(millis() - add_on_refresh_time) > bubble_duration)
      {
        // We specify that we are waiting for the next bubble refresh openings
        add_on_state = 0;
        // We record this point of time as the latest bubble refresh point
        add_on_refresh_time = millis();
        // We disable the motor driver enable pins to use the bubble reward
        digitalWrite(motor_driver_enable_B, LOW);
        // We switch off the radial fan that blows bubbles upwards
        analogWrite(motor_driver_input_3, 0);
        // We switch off the geared motor that dips loops into the bubble solution
        analogWrite(motor_driver_input_4, 0);
      }
    }
    // state 3 - waiting to terminate a bubble refresh
    else if(add_on_state == 3)
    {
      // If we have refreshed for enough time to soak all the rings in solution
      if(abs(millis() - add_on_refresh_time) > bubble_refresh_duration)
      {
          // We specify that we are waiting for the next bubble refresh openings
          add_on_state = 0;
          // We record this point of time as the latest bubble refresh point
          add_on_refresh_time = millis();
          // We disable the motor driver enable pin to spin the bubble reward motor
          digitalWrite(motor_driver_enable_B, LOW);
          // We switch off the geared motor that dips loops into the bubble solution
          analogWrite(motor_driver_input_4, 0);
      }
    }
    // state 4 - emergency stop
    else if(add_on_state == 4)
    {
      // We specify that we are waiting for the next bubble refresh openings
      add_on_state = 0;
      // We record this point of time as the latest bubble refresh point
      add_on_refresh_time = millis();
      // We disable the motor driver enable pin to spin the bubble reward motor
      digitalWrite(motor_driver_enable_B, LOW);
      // We switch on the geared motor that dips loops into the bubble solution
      analogWrite(motor_driver_input_3, 0);
      // We switch off the geared motor that dips loops into the bubble solution
      analogWrite(motor_driver_input_4, 0);
    }
  }
}

// ----- Infrared sensor functions -----

// This function responds to the interrupt on the first infrared sensor
void infrared_sensor_filter_one()
{
  // We check that infrared sensor one is not activated yet
  if(!nec_sensor_2_active)
  {
    // We indicate that the NEC sensor 1 is currently active
    nec_sensor_1_active = true;
    // We call the infrared sensor scan function to read the NEC code
    infrared_sensor_scan();
  }
}

// This function responds to the interrupt on the second infrared sensor
void infrared_sensor_filter_two()
{
  // We check that infrared sensor two is not activated yet
  if(!nec_sensor_1_active)
  {
    // We indicate that the NEC sensor 1 is currently active
    nec_sensor_2_active = true;
    // We call the infrared sensor scan function to read the NEC code
    infrared_sensor_scan();
  }
}

// This function reads the bits from the NEC remote signal
void infrared_sensor_scan()
{
  // We use this variable to store the bit number
  static volatile byte ir_bit_number = 0;
  // We use this variable to store the timestamp in which the last falling pulse was transmitted
  static volatile unsigned long ir_pulse_timestamp = 0;
  // We use this variable to store the 32 bits of data in the NEC code
  static volatile unsigned long ir_bit_data = 0;
  
  // We check if the time since last falling pulse to ensure it isnt the NEC starting pulse
  if(abs(micros()-ir_pulse_timestamp)<5000)
  {
    // We write the data recieved into the unsigned long variable
    bitWrite(ir_bit_data,ir_bit_number++,(micros()-ir_pulse_timestamp>1700));
    // We reocrd the timestamp of the last falling pulse
    ir_pulse_timestamp = micros();
    // We check if we have obtained all the 32 required bits of the NEC format
    if(ir_bit_number==32)
    {
      // We store the NEC remote data for external access
      nec_remote_data = ir_bit_data;
      // We indicate that NEC remote data is available
      nec_data_available = true;
      // We mark both the infrared sensors as inactive
      nec_sensor_1_active = false;
      nec_sensor_2_active = false;
    }
  }
  // We are dealing with the NEC starting pulse 
  else 
  {
    // We reset the timestamp to match the NEC starting pulses falling edge
    ir_pulse_timestamp = micros();
    // We reset the pulse number back to zero
    ir_bit_number = 0;
  }
}

// ----- KY-040 sensor functions -----

// This function is called to take action if the knob of the KY-040 rotary encoder is pressed
void ky040_switch_filter()
{
  // The variable indicates if the KY-040 knob was released
  static bool ky040_switch_release = true;
  // This variable indicates the the switch filter is reset
  static bool ky040_switch_filter_reset = true;
  // This variable helps ensure oled action is taken only upon release
  static bool ky040_switch_action = false;
  // This variable holds the initial timestamp post reset
  static unsigned long ky040_initial_switch_time;
  // This variable holds the latest timestamp
  static unsigned long ky040_latest_switch_time;
  // We check if the rotary encoder switch was previously released
  if(ky040_switch_release)
  {
    //  We check if the rotary encoder switch is pressed down
    if( digitalRead(rotary_encoder_switch) == LOW)
    {
      // We check if the switch filter is reset
      if(ky040_switch_filter_reset)
      {
        // This variable holds the initial timestamp
        ky040_initial_switch_time = millis();
        // We state that the switch filter is not reset
        ky040_switch_filter_reset = false;
      }
      // the switch filter is not reset
      else
      {
        // This variable holds the latest timestamp
        ky040_latest_switch_time = millis();
        // We check if we have experienced the switch being pressed down for sufficient lengths of time
        if(abs(ky040_latest_switch_time - ky040_initial_switch_time) > ky040_switch_threshold_time)
        {
          // We specify that the knob has been released and oled action must be taken
          ky040_switch_action = true;
          // We indicate that the knob was pressed down for a sufficient period of time
          ky040_switch_release = false;
        }
      }
    }
  }
  // We check if the rotary encoder knob is released
  else if(digitalRead(rotary_encoder_switch) == HIGH)
  {
    // We check if the knob has been released and oled action must be taken
    if(ky040_switch_action)
    {
      // We specify that the rotary encoder knob has been pressed
      oled_action = 3;
      // We disable the knob release
      ky040_switch_action = false;
    }
    // We indiate that the rotary encoder knob was released
    ky040_switch_release = true;
    // We indicate that the switch filter is reset
    ky040_switch_filter_reset = true;
  }
}

// This function is called when the KY-040 rotary encoder knob is turned
void ky040_clock_filter()
{
  // The variable indicates if the KY-040 knob is in a stable position while turning
  static bool ky040_clock_release = true;
  // This variable indicates the the clock filter is reset
  static bool ky040_clock_filter_reset = true;
  // This variable indicates the rotary encoder data value we were observing
  static bool ky040_data_value = false;
  // This variable holds the initial timestamp post reset
  static unsigned long ky040_initial_clock_time;
  // This variable holds the latest timestamp
  static unsigned long ky040_latest_clock_time;
  // We check if the rotary encoder clock was previously released
  if(ky040_clock_release)
  {
    //  We check if the rotary encoder knob is at an unstable point of its turn
    if(digitalRead(rotary_encoder_clock) == LOW)
    {
      // We check if the clock filter is reset
      if(ky040_clock_filter_reset)
      {
        // This variable holds the initial timestamp
        ky040_initial_clock_time = micros();
        // We state that the clock filter is not reset
        ky040_clock_filter_reset = false;
        // We read the rotary encoder data value
        ky040_data_value = digitalRead(rotary_encoder_data);
      }
      // the clock filter is not reset and we are getting the same data value
      else if(digitalRead(rotary_encoder_data) == ky040_data_value)
      {
        // This variable holds the latest timestamp
        ky040_latest_clock_time = micros();
        // We check if we have experienced the clock being active for sufficient lengths of time
        if(abs(ky040_latest_clock_time - ky040_initial_clock_time) > ky040_clock_threshold_time)
        {
          // We turned the knob clockwise
          if(ky040_data_value)
          {
            // We specify that the rotary encoder knob has been turned clockwise
            oled_action = 1;
          }
          // We turned the knob anti-clockwise
          else
          {
            // We specify that the rotary encoder knob has been turned anti-clockwise
            oled_action = 2;   
          }
          // We indicate that the rotary encoder clock has been released
          ky040_clock_release = false;
        }
      }
      // We are not getting the same data value as we did initially
      else
      {
        // We indicate that the clock filter is reset
        ky040_clock_filter_reset = true;
        // The rotary encoder clock was not at a stable point and has to be reset
        ky040_clock_release = false;
      }
    }
    else
    {
      // We indicate that the clock filter is reset
      ky040_clock_filter_reset = true;
    }
  }
  // We check if the rotary encoder knob is in a stable position while turning
  else if(digitalRead(rotary_encoder_clock) == HIGH)
  {
    // We indiate that the rotary encoder knob was rotated to a stable position
    ky040_clock_release = true;
    // We indicate that the clock filter is reset
    ky040_clock_filter_reset = true;
  }
}

// ---------------- End of Program ----------------

