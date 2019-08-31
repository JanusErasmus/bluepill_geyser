#include <stdio.h>

#include "Utils/cli.h"
#include "Utils/commands.h"

void rtc_debug(uint8_t argc, char **argv);
const sTermEntry_t rtcEntry =
{ "date", "RTC date", rtc_debug };

void mqtt(uint8_t argc, char **argv);
const sTermEntry_t mqttEntry =
{ "m", "MQTT report", mqtt };

void reset_sonoff(uint8_t argc, char **argv);
const sTermEntry_t espEntry =
{ "esp", "Reset ESP32", reset_sonoff };


void exit_py(uint8_t argc, char **argv);
const sTermEntry_t exitEntry =
{ "epy", "Exit python script", exit_py };

void adc(uint8_t argc, char **argv);
const sTermEntry_t adcEntry =
{ "a", "ADC sample", adc };


const sTermEntry_t *cli_entries[] =
{
      &hEntry,
	  &exitEntry,
	  &espEntry,
      &helpEntry,
      &rebootEntry,
      &bootEntry,
	  &rtcEntry,
	  &mqttEntry,
	  &adcEntry,
      0
};
