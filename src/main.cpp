#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <INA226.h>
#include <EEPROM.h>

#include "credentials.h"
#include "def.h"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

INA226 ina226(Wire);

enum
{
    WIFI_DISABLED,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    CONNECTED_TO_BROKER,
} stage;

// EEPROM variables
float generated_wh;
// Global variables
float measured_V, measured_A, measured_P, saved_V, saved_A, saved_P;
uint8_t mqtt_conn;
uint32_t timestamp_on_wifi_begin, timestamp_last_published, timestamp_last_mqtt_reconn;
uint32_t timestamp_on_mqtt_begin, timestamp_conn_failed, timestamp_pub_started;
int8_t signal_quality;

void read_ina226_values(void)
{
    static uint32_t last_meas_timestamp;
    static int eeprom_cnt;

    measured_V = ina226.readBusVoltage();
    measured_A = ina226.readShuntCurrent();
    measured_P = ina226.readBusPower();

    uint32_t time_now = millis();

    if (last_meas_timestamp > 0)
    {
        generated_wh += (float(time_now - last_meas_timestamp) * measured_P) / 3600000.0f;
    }
    last_meas_timestamp = time_now;

    // Save values to EEPROM every 10 measurements
    eeprom_cnt++;
    if (eeprom_cnt > SAVE_WH_AFTER_CNT)
    {
        eeprom_cnt = 0;
        bool eeprom_write_success = false;
        while (!eeprom_write_success)
        {
            eeprom_write_success = EEPROM.put(0, generated_wh);
            if (eeprom_write_success)
            {
                EEPROM.commit();
            }
            else
            {
                // If EEPROM write failed, wait 100ms and reset board
                delay(100);
                ESP.restart();
            }
        }
    }
}

int8_t read_signal_quality()
{
    int ss = WiFi.RSSI();
    ss = isnan(ss) ? -100 : ss;
    ss = min(max(2 * (ss + 100), 0), 100);

    return (int8_t)ss;
}

/**
 * @brief Callback function for MQTT client
 */
void callback(String topic, byte *payload, unsigned int length)
{
    static uint8_t reset_toggled;
    String msgString = "";
    for (uint16_t i = 0; i < length; i++)
        msgString += (char)payload[i];

    // Reset board when received "reset" message
    if (topic == (MQTT_CMD_TOPIC_RESET))
    {
        // We need to add reset_toggled flag, to prevent resetting the board twice
        if (msgString == MQTT_CMD_ON)
        {
            if (!reset_toggled)
            {
                mqttClient.publish(MQTT_CMD_TOPIC_RESET, MQTT_CMD_OFF, true);
            }
            else
            {
                mqttClient.publish(MQTT_CMD_TOPIC_RESET, MQTT_CMD_OFF, true);
                delay(DELAY_AFTER_PUBLISH_MS);
                espClient.flush(DELAY_AFTER_PUBLISH_MS);
                ESP.restart();
            }
        }
        else if (msgString == MQTT_CMD_OFF && !reset_toggled)
        {
            reset_toggled = 1;
        }
    }
    else if (topic == (MQTT_CMD_TOPIC_WH_RESET))
    {
        if (msgString == MQTT_CMD_ON)
        {
            mqttClient.publish(MQTT_CMD_TOPIC_WH_RESET, MQTT_CMD_OFF, true);
            generated_wh = 0;
            EEPROM.put(0, generated_wh);
            EEPROM.commit();
        }
    }
    else if (topic == MQTT_CMD_TOPIC_SET_WH)
    {
        generated_wh = msgString.toFloat();
        EEPROM.put(0, generated_wh);
        EEPROM.commit();
    }
}

/**
 * @brief Establishing connection or reconnecting to MQTT server
 */
void reconnect(void)
{
    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
    }
}

/**
 * @brief Publish data to broker
 */
void publish_data()
{
    static char buff[20];

    mqttClient.publish(MQTT_WILL_TOPIC, MQTT_AVAILABILITY_MESSAGE);

    sprintf(buff, "%0.3f", measured_V);
    mqttClient.publish(MQTT_STATE_TOPIC_VOLT, buff);
    sprintf(buff, "%0.3f", measured_A);
    mqttClient.publish(MQTT_STATE_TOPIC_AMP, buff);
    sprintf(buff, "%0.3f", measured_P);
    mqttClient.publish(MQTT_STATE_TOPIC_POWER, buff);
    sprintf(buff, "%0.3f", generated_wh);
    mqttClient.publish(MQTT_STATE_TOPIC_WH, buff);

    mqttClient.publish(MQTT_STATE_TOPIC_SIG, String(signal_quality).c_str());
    mqttClient.publish(MQTT_STATE_TOPIC_UPTIME, String(millis() / 1000).c_str());
}

void state_machine()
{
    switch (stage)
    {
        case WIFI_DISABLED:
            uint8_t begin_wifi;
            begin_wifi = 0;

            if (!timestamp_conn_failed || millis() - timestamp_conn_failed > CONN_FAILED_TIMEOUT_MS)
            {
                begin_wifi = 1;
            }

            if (begin_wifi)
            {
                // Setup WiFi connection
                WiFi.mode(WIFI_STA);
                WiFi.begin(WIFI_SSID, WIFI_PASSWD);
                timestamp_on_wifi_begin = millis();
                stage = WIFI_CONNECTING;
            }
            break;

        case WIFI_CONNECTING:
            // Blink with LED while connecting
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(100); // Delay for yield

            // If lost WIFi connection
            if (WiFi.status() == WL_CONNECTED)
            {
                timestamp_last_mqtt_reconn = 0;
                timestamp_on_mqtt_begin = millis();
                stage = WIFI_CONNECTED;
            }
            break;

        case WIFI_CONNECTED:
            // Blink with LED while connecting
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(50); // Delay for yield

            ArduinoOTA.handle();

            // If lost WIFi connection
            if (WiFi.status() != WL_CONNECTED)
            {
                timestamp_on_wifi_begin = millis();
                stage = WIFI_CONNECTING;
            }
            else
            {
                // If connected to MQTT broker
                if (mqttClient.loop())
                {
                    timestamp_on_mqtt_begin = 0;
                    timestamp_pub_started = millis();
                    analogWrite(STATUS_LED, 10);
                    stage = CONNECTED_TO_BROKER;
                }
                else if (!timestamp_last_mqtt_reconn || millis() - timestamp_last_mqtt_reconn > MQTT_RECONN_PERIOD_MS)
                {
                    reconnect();
                    timestamp_last_mqtt_reconn = millis();
                }
            }
            break;

        case CONNECTED_TO_BROKER:
            ArduinoOTA.handle();
            mqtt_conn = mqttClient.loop();
            signal_quality = read_signal_quality();

            // If lost WIFi connection
            if (WiFi.status() != WL_CONNECTED)
            {
                timestamp_on_wifi_begin = millis();
                stage = WIFI_CONNECTING;
            }
            else
            {
                if (!mqtt_conn)
                {
                    timestamp_on_mqtt_begin = millis();
                    stage = WIFI_CONNECTED;
                }
                else if (millis() - timestamp_last_published > PUBLISH_INTERVAL_FAST_MS)
                {
                    publish_data();
                    timestamp_last_published = millis();
                }
            }
            break;
    }
}

void setup()
{
    EEPROM.begin(4);
    // EEPROM.put(0, generated_wh); // erase EEPROM
    EEPROM.get(0, generated_wh);

    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, 1);

    WiFi.mode(WIFI_OFF);
    WiFi.hostname(HOSTNAME);

    // MQTT initializing
    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);

    // Arduino OTA initializing
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();

    ina226.begin(INA226_ADDRESS);
    // When set INA226_AVERAGES_64 and INA226_SHUNT_CONV_TIME_8244US, the period is about 1 second
    ina226.configure(INA226_AVERAGES_512, INA226_BUS_CONV_TIME_1100US,
                     INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    ina226.calibrate(SHUNT_VALUE, MAX_CURRENT_EXCEPTED_A);
    ina226.enableConversionReadyAlert();
}

void loop()
{
    read_ina226_values();

    state_machine();

    yield();
}