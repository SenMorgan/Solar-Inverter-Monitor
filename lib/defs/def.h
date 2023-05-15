/**
 * @file def.h
 * @author SenMorgan https://github.com/SenMorgan
 * @date 2022-06-18
 *
 * @copyright Copyright (c) 2023 Sen Morgan
 *
 */

#ifndef _DEF_H_
#define _DEF_H_

// INA226 settings
#define SHUNT_VALUE            0.01F
#define MAX_CURRENT_EXCEPTED_A 4.0F

// MQTT definitions
#define DEFAULT_TOPIC             "/solar-inverter-monitor/"
#define MQTT_WILL_TOPIC           DEFAULT_TOPIC "availability"
#define MQTT_QOS                  1
#define MQTT_RETAIN               0
#define MQTT_WILL_MESSAGE         "offline"
#define MQTT_AVAILABILITY_MESSAGE "online"
#define MQTT_SUBSCRIBE_TOPIC      DEFAULT_TOPIC "set/#"
#define MQTT_CMD_TOPIC_RESET      DEFAULT_TOPIC "set/reset"
#define MQTT_CMD_TOPIC_WH_RESET   DEFAULT_TOPIC "set/wh-reset"
#define MQTT_CMD_TOPIC_SET_WH     DEFAULT_TOPIC "set/wh"
#define MQTT_STATE_TOPIC_VOLT     DEFAULT_TOPIC "state/volt"
#define MQTT_STATE_TOPIC_AMP      DEFAULT_TOPIC "state/amp"
#define MQTT_STATE_TOPIC_POWER    DEFAULT_TOPIC "state/watt"
#define MQTT_STATE_TOPIC_WH       DEFAULT_TOPIC "state/wh"
#define MQTT_STATE_TOPIC_SIG      DEFAULT_TOPIC "state/signal-quality"
#define MQTT_STATE_TOPIC_UPTIME   DEFAULT_TOPIC "state/uptime"
#define MQTT_CMD_ON               "1"
#define MQTT_CMD_OFF              "0"

// Interval between publishing data
#define PUBLISH_INTERVAL_FAST_MS 1000
// Some delay to process MQTT messages before going to sleep
#define DELAY_AFTER_PUBLISH_MS   500
/* Interval between reattempting connection to the WiFi
    after unsuccessful reconnection during MAX_WIFI_RECONN_TIME_MS period*/
#define CONN_FAILED_TIMEOUT_MS   10 * 60 * 1000 // 10 minutes
// Period between MQTT tries to reconnect to the broker
#define MQTT_RECONN_PERIOD_MS    2000 // 2 seconds
// Save data to EEPROM after this number of cycles
#define SAVE_WH_AFTER_CNT        10

// IO pins
#define SDA_PIN          0
#define SCL_PIN          2
#define INA226_ALERT_PIN 3
/** WARN: ESP-01 built-in LED can't be used if I2C is enabled on GPIO 2
 * so we are using GPIO 1 (TX pin) and external LED
 */
#define STATUS_LED       1

#endif // _DEF_H_