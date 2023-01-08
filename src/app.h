#ifndef __APP_H
#define __APP_H

#include <Arduino.h>

#define LCD_BACKLIGHT_TIMEOUT_MILLIS  15000
#define LCD_PROGRESS_BAR_CHAR         0xff
#define LCD_PROGRESS_BAR              "" LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR

#define MOTION_SENSOR_PIN             39

#ifndef INT_LED_PIN
#define INT_LED_PIN                   2
#endif

#define BLINDS_ZERO_PIN               26

#define APP_INIT_DELAY_MILLIS         2500

#define WIFI_RECONNECT_MILLIS         10000
#define WIFI_WATCHDOG_MILLIS          60000

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME                 "multiswitch-01"
#endif

#define MQTT_SERVER_NAME              "10.9.9.96"
#define MQTT_SERVER_PORT              1883
#define MQTT_USERNAME                 NULL
#define MQTT_PASSWORD                 NULL
#define MQTT_RECONNECT_MILLIS         5000

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                WIFI_HOSTNAME
#endif

#define MQTT_STATUS_TOPIC             MQTT_CLIENT_ID "/status"
#define MQTT_VERSION_TOPIC            MQTT_CLIENT_ID "/version"
#define MQTT_RESTART_CONTROL_TOPIC    MQTT_CLIENT_ID "/restart"
#define MQTT_BACKLIGHT_TOPIC          MQTT_CLIENT_ID "/backlight"
#define MQTT_CONFIG_TOPIC             MQTT_CLIENT_ID "/config"
#define MQTT_CURRENT_MEETING_TOPIC    "ay/calendar/events/current/title"

#define MQTT_BLINDS_ZERO_TOPIC        MQTT_CLIENT_ID "/blinds-zero"

#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

#define SW_RESET_REASON_FILENAME      "/swresr.bin"

#define GMT_OFFSET_SEC                2*60*60
#define DAYLIGHT_OFFSET_SEC           1*60*60

#define NTP_SERVER                    "ua.pool.ntp.org"
#define NTP_UPDATE_MS                 1*60000                                  // every 1 minute

#define OTA_UPDATE_TIMEOUT_MILLIS     5*60000

#define WDT_TIMEOUT_SEC               20
#define UI_FORCED_REDRAW_MS           15*1000

typedef std::function<void(void)> ActionCallback;

extern bool getSpiffsEnabled();
extern bool parseBooleanMessage(byte* payload, unsigned int length, boolean defaultValue = false);

#endif