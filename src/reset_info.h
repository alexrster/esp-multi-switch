#ifndef __RESET_INFO_H
#define __RESET_INFO_H

#include <Arduino.h>
#include <rom/rtc.h>

#define RESET_NO_ERROR                0
#define RESET_ON_WIFI_WD_TIMEOUT      1
#define RESET_ON_CONFIG_UPDATE        2
#define RESET_ON_MQTT_RESET_TOPIC     3
#define RESET_ON_OTA_SUCCESS          4
#define RESET_ON_OTA_FAIL             5
#define RESET_ON_OTA_TIMEOUT          6

String get_sw_reset_reason_info(char code)
{
  switch (code)
  {
    case RESET_NO_ERROR:            return String("{code:0,descr:\"NO_ERROR\"}");
    case RESET_ON_WIFI_WD_TIMEOUT:  return String("{code:1,descr:\"WIFI_SW_WATCHDOG_TIMEOUT\"}");
    case RESET_ON_CONFIG_UPDATE:    return String("{code:2,descr:\"CONFIGURATION_UPDATED\"}");
    case RESET_ON_MQTT_RESET_TOPIC: return String("{code:3,descr:\"MQTT_RESET_REQUEST\"}");
    case RESET_ON_OTA_SUCCESS:      return String("{code:4,descr:\"OTA_SUCCESS\"}");
    case RESET_ON_OTA_FAIL:         return String("{code:5,descr:\"OTA_FAIL\"}");
    case RESET_ON_OTA_TIMEOUT:      return String("{code:6,descr:\"OTA_TIMEOUT\"}");
    default:                        
      auto result = String("{code:");
      result.concat(code);
      result.concat(",descr:\"UNKNOWN\"}");
      return result;
  }
}

String get_reset_reason_info(RESET_REASON reason)
{
  switch (reason)
  {
    case 0:  return String("NO_ERROR");
    case 1:  return String("POWERON_RESET");          /**<1, Vbat power on reset*/
    case 3:  return String("SW_RESET");               /**<3, Software reset digital core*/
    case 4:  return String("OWDT_RESET");             /**<4, Legacy watch dog reset digital core*/
    case 5:  return String("DEEPSLEEP_RESET");        /**<5, Deep Sleep reset digital core*/
    case 6:  return String("SDIO_RESET");             /**<6, Reset by SLC module, reset digital core*/
    case 7:  return String("TG0WDT_SYS_RESET");       /**<7, Timer Group0 Watch dog reset digital core*/
    case 8:  return String("TG1WDT_SYS_RESET");       /**<8, Timer Group1 Watch dog reset digital core*/
    case 9:  return String("RTCWDT_SYS_RESET");       /**<9, RTC Watch dog Reset digital core*/
    case 10: return String("INTRUSION_RESET");        /**<10, Instrusion tested to reset CPU*/
    case 11: return String("TGWDT_CPU_RESET");        /**<11, Time Group reset CPU*/
    case 12: return String("SW_CPU_RESET");           /**<12, Software reset CPU*/
    case 13: return String("RTCWDT_CPU_RESET");       /**<13, RTC Watch dog Reset CPU*/
    case 14: return String("EXT_CPU_RESET");          /**<14, for APP CPU, reseted by PRO CPU*/
    case 15: return String("RTCWDT_BROWN_OUT_RESET"); /**<15, Reset when the vdd voltage is not stable*/
    case 16: return String("RTCWDT_RTC_RESET");       /**<16, RTC Watch dog reset digital core and rtc module*/
    default: return String("UNKNOWN");
  }
}

#endif