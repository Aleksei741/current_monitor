//******************************************************************************
//include
//******************************************************************************
#include "serial_protocol.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CurrentMonitor.h"
//******************************************************************************
// Секция определения констант
//******************************************************************************
#define RX_LINE_MAX 128
//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------
static char msg[128];

// Буфер для накопления строки
static char rx_line[RX_LINE_MAX];
static uint16_t rx_index = 0;

static SerialProtocol_UpdateCallback UpdateParamModule = NULL;
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
void ProcessLine(const char* line);
//******************************************************************************
// Секция описания функций
//******************************************************************************
void InitSerialProtocol(SerialProtocol_UpdateCallback cb)
{
  UpdateParamModule = cb;
}
//------------------------------------------------------------------------------
void TransmiteCurrent(uint32_t* currents)
{
  float t = HAL_GetTick() / 1000.0f;
  snprintf(msg, sizeof(msg),
           "%.3f %d %d %d %d\n",
           (double)t, 
           currents[0], 
           currents[1], 
           currents[2], 
           currents[3]);
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}
//------------------------------------------------------------------------------
void ReceiveCDCAccumulate(uint8_t* Buf, uint32_t *Len)
{
  for (uint32_t i = 0; i < *Len; i++)
  {
    char c = (char)Buf[i];
    
    // Конец строки: LF или CR
    if (c == '\n' || c == '\r')
    {
      if (rx_index > 0)
      {
        rx_line[rx_index] = '\0'; // Нуль-терминатор
        ProcessLine(rx_line); // Отправка строки в обработчик
        rx_index = 0; // Сброс буфера
      }
    }
    else
    {
      if (rx_index < RX_LINE_MAX - 1)
      {
        rx_line[rx_index++] = c; // Добавляем символ в буфер
      }
      else
      {
        // Переполнение буфера — сброс
        rx_index = 0;
      }
    }
  }
}
//------------------------------------------------------------------------------
void ProcessLine(const char* line)
{
  // Пример строки: "address 65 68 69 70"
  char keyword[16];
  float in_values[INA226_COUNT];
  
  int n = sscanf(line, "%15s %f %f %f %f", keyword, &in_values[0], &in_values[1], &in_values[2], &in_values[3]);
  //Запрос параметров
  if(n == 1)
  {
    
    if(strcmp(keyword, "address") == 0)
    {
      uint8_t value[INA226_COUNT];
      GetParameters(value, TYPE_ADDRESS);
      snprintf(msg, sizeof(msg),
               "address %u %u %u %u\n",
               value[0], value[1], value[2], value[3]);
    }
    else if(strcmp(keyword, "rshunt") == 0)
    {
      float value[INA226_COUNT];
      GetParameters(value, TYPE_RSHUNT);
      snprintf(msg, sizeof(msg),
               "rshunt %.6f %.6f %.6f %.6f\n",
               value[0], value[1], value[2], value[3]);
    }
    else if(strcmp(keyword, "current_lsb") == 0)
    {
      float value[INA226_COUNT];
      GetParameters(value, TYPE_CURRENT_LSB);
      snprintf(msg, sizeof(msg),
               "current_lsb %.6f %.6f %.6f %.6f\n",
               value[0], value[1], value[2], value[3]);
    }
    else if(strcmp(keyword, "out_didvider") == 0)
    {
      uint32_t value[INA226_COUNT];
      GetParameters(value, TYPE_OUT_DIVIDER);
      snprintf(msg, sizeof(msg),
               "out_didvider %d %d %d %d\n",
               value[0], value[1], value[2], value[3]);
    }
    else if(strcmp(keyword, "downsample") == 0)
    {
      uint32_t value;
      GetParameters(&value, TYPE_DOWNSAMPLE);
      snprintf(msg, sizeof(msg), "downsample %d\n", value);
    }
    else if(strcmp(keyword, "avg_wnd") == 0)
    {
      uint8_t value[INA226_COUNT];
      GetParameters(value, TYPE_AVG_WINDOW_SIZE);
      snprintf(msg, sizeof(msg),
               "avg_wnd %u %u %u %u\n",
               value[0], value[1], value[2], value[3]);
    }
    else
    {
      snprintf(msg, sizeof(msg), "ERROR\n");
    }
    
    // Отправка через CDC
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  }    
  //Установка параметров извне
  else if (n == 2) 
  {
    if(strcmp(keyword, "downsample") == 0)
    {
      UpdateParamModule(in_values, TYPE_DOWNSAMPLE);
    }
    else
    {
      snprintf(msg, sizeof(msg), "ERROR\n");
      CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }    
  }
  else if (n == 5) 
  {
    
    if (strcmp(keyword, "address") == 0)
    {
      uint8_t data[INA226_COUNT];
      for(size_t i = 0; i < INA226_COUNT; i++)
        data[i] = (uint8_t)in_values[i];
      
      UpdateParamModule(data, TYPE_ADDRESS);
    }
    else if(strcmp(keyword, "rshunt") == 0)
    {
      UpdateParamModule(in_values, TYPE_RSHUNT);
    }
    else if(strcmp(keyword, "current_lsb") == 0)
    {
      UpdateParamModule(in_values, TYPE_CURRENT_LSB);
    }
    else if(strcmp(keyword, "out_didvider") == 0)
    {
      uint32_t data[INA226_COUNT];
      for(size_t i = 0; i < INA226_COUNT; i++)
        data[i] = (uint8_t)in_values[i];
      UpdateParamModule(data, TYPE_OUT_DIVIDER);
    }
    else if(strcmp(keyword, "avg_wnd") == 0)
    {
      uint8_t data[INA226_COUNT];
      for(size_t i = 0; i < INA226_COUNT; i++)
        data[i] = (uint8_t)in_values[i];
      UpdateParamModule(data, TYPE_AVG_WINDOW_SIZE);
    }
  }
  else
  {
    // Некорректная команда
    snprintf(msg, sizeof(msg), "%s\n", "ERROR");
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
  }
}