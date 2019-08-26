#include <stdio.h>

#include "Utils/cli.h"
#include "Utils/commands.h"

void rtc_debug(uint8_t argc, char **argv);
const sTermEntry_t rtcEntry =
{ "date", "RTC date", rtc_debug };

void mqtt(uint8_t argc, char **argv);
const sTermEntry_t mqttEntry =
{ "m", "MQTT report", mqtt };

void reset_esp(uint8_t argc, char **argv);
const sTermEntry_t espEntry =
{ "esp", "Reset ESP32", reset_esp };


void exit_py(uint8_t argc, char **argv);
const sTermEntry_t exitEntry =
{ "epy", "Exit python script", exit_py };

void adc(uint8_t argc, char **argv);
const sTermEntry_t adcEntry =
{ "a", "ADC sample", adc };

void water(uint8_t argc, char **argv);
const sTermEntry_t waterEntry =
{ "w", "Toggle water", water };

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
	  &waterEntry,
      0
};
