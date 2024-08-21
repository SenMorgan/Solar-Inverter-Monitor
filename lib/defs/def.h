/**
 * @file def.h
 * @author SenMorgan https://github.com/SenMorgan
 * @date 2023-05-23
 *
 * @copyright Copyright (c) 2023 Sen Morgan
 *
 */

#ifndef _DEF_H_
#define _DEF_H_

// INA226 settings
#define SHUNT_VALUE            0.01F
#define MAX_CURRENT_EXCEPTED_A 6.0F

// MQTT definitions
#define DEFAULT_TOPIC                   "/solar-inverter-monitor/"
#define MQTT_WILL_TOPIC                 DEFAULT_TOPIC "availability"
#define MQTT_QOS                        1
#define MQTT_RETAIN                     0
#define MQTT_WILL_MESSAGE               "offline"
#define MQTT_AVAILABILITY_MESSAGE       "online"
#define MQTT_SUBSCRIBE_TOPIC            DEFAULT_TOPIC "set/#"
#define MQTT_CMD_TOPIC_RESET            DEFAULT_TOPIC "set/reset"
#define MQTT_CMD_TOPIC_WH_RESET         DEFAULT_TOPIC "set/wh-reset"
#define MQTT_CMD_TOPIC_SET_WH           DEFAULT_TOPIC "set/wh"
#define MQTT_STATE_TOPIC_VOLT           DEFAULT_TOPIC "state/volt"
#define MQTT_STATE_TOPIC_AMP            DEFAULT_TOPIC "state/amp"
#define MQTT_STATE_TOPIC_POWER          DEFAULT_TOPIC "state/watt"
#define MQTT_STATE_TOPIC_WH             DEFAULT_TOPIC "state/wh"
#define MQTT_STATE_TOPIC_TEMP           DEFAULT_TOPIC "state/temp"
#define MQTT_STATE_TOPIC_MPPT_STATE     DEFAULT_TOPIC "state/mppt"
#define MQTT_STATE_TOPIC_INVERTER_ERROR DEFAULT_TOPIC "state/error"
#define MQTT_STATE_TOPIC_SIG            DEFAULT_TOPIC "state/signal-quality"
#define MQTT_STATE_TOPIC_UPTIME         DEFAULT_TOPIC "state/uptime"
#define MQTT_CMD_ON                     "1"
#define MQTT_CMD_OFF                    "0"

// Inverter constants
#define CONSIDER_MPPT_LOCKED_MS         5000
#define CONSIDER_INVERTER_ERROR_MS      5000
#define INVERTER_ERROR_ANALOG_THRESHOLD 400
#define INVERTER_ADJ_PERIOD_LO_MS       500
#define INVERTER_ADJ_PERIOD_HI_MS       1500

#define STATUS_LED_IDLE_BRIGHTNESS_INV 240
// Interval between publishing data
#define PUBLISH_INTERVAL_FAST_MS       1000
// Some delay to process MQTT messages before going to sleep
#define DELAY_AFTER_PUBLISH_MS         500
/* Interval between reattempting connection to the WiFi
    after unsuccessful reconnection during MAX_WIFI_RECONN_TIME_MS period*/
#define CONN_FAILED_TIMEOUT_MS         10 * 60 * 1000 // 10 minutes
// Period between MQTT tries to reconnect to the broker
#define MQTT_RECONN_PERIOD_MS          2000 // 2 seconds
// Save data to EEPROM after this number of cycles
#define SAVE_WH_AFTER_CNT              10

// IO pins
#define SDA_PIN        4
#define SCL_PIN        5
#define INVERTER_RUN   14
#define INVERTER_ERROR A0
#define STATUS_LED     2
#define ONE_WIRE_BUS   12   // DS18B20 on pin D6

#endif // _DEF_H_