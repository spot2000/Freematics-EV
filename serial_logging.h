#ifndef SERIAL_LOGGING_H
#define SERIAL_LOGGING_H

#include "config.h"
#include <Arduino.h>
#include <stdarg.h>

enum SerialLogLevel : uint8_t {
  LOG_ERROR = 0,
  LOG_WARN = 1,
  LOG_INFO = 2,
  LOG_DEBUG = 3
};

inline bool serial_log_is_enabled(SerialLogLevel level)
{
  return ACTIVE_LOGS && level <= LOG_LEVEL;
}

inline const char* serial_log_level_name(SerialLogLevel level)
{
  switch (level) {
    case LOG_ERROR:
      return "ERROR";
    case LOG_WARN:
      return "WARN";
    case LOG_INFO:
      return "INFO";
    case LOG_DEBUG:
      return "DEBUG";
    default:
      return "LOG";
  }
}

inline void serial_log_prefix(SerialLogLevel level)
{
  Serial.print('[');
  Serial.print(serial_log_level_name(level));
  Serial.print("] ");
}

inline void serial_log_print(SerialLogLevel level)
{
  if (!serial_log_is_enabled(level)) return;
  serial_log_prefix(level);
  Serial.println();
}

template <typename T>
inline void serial_log_print(SerialLogLevel level, const T& message)
{
  if (!serial_log_is_enabled(level)) return;
  serial_log_prefix(level);
  Serial.println(message);
}

inline void serial_log_vprintf(SerialLogLevel level, const char* format, va_list args)
{
  if (!serial_log_is_enabled(level)) return;

  char stackBuffer[256];
  va_list argsCopy;
  va_copy(argsCopy, args);
  int needed = vsnprintf(stackBuffer, sizeof(stackBuffer), format, argsCopy);
  va_end(argsCopy);

  serial_log_prefix(level);
  if (needed < 0) {
    Serial.println("log formatting failed");
    return;
  }

  if (needed < (int)sizeof(stackBuffer)) {
    Serial.println(stackBuffer);
    return;
  }

  size_t heapSize = (size_t)needed + 1;
  char* heapBuffer = (char*)malloc(heapSize);
  if (!heapBuffer) {
    Serial.println("log allocation failed");
    return;
  }

  vsnprintf(heapBuffer, heapSize, format, args);
  Serial.println(heapBuffer);
  free(heapBuffer);
}

inline void serial_log_printf(SerialLogLevel level, const char* format, ...)
{
  va_list args;
  va_start(args, format);
  serial_log_vprintf(level, format, args);
  va_end(args);
}

#endif
