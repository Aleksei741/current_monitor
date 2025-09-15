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

// Ключевые команды протокола
#define CMD_ADDRESS        "address"
#define CMD_RSHUNT         "rshunt"
#define CMD_CURRENT_LSB    "current_lsb"
#define CMD_OUT_DIVIDER    "out_didvider"
#define CMD_DOWNSAMPLE     "downsample"
#define CMD_AVG_WND        "avg_wnd"

// Форматы ответа протокола с использованием define команд
#define FMT_ADDRESS       CMD_ADDRESS " %u %u %u %u\n"
#define FMT_RSHUNT        CMD_RSHUNT " %.6f %.6f %.6f %.6f\n"
#define FMT_CURRENT_LSB   CMD_CURRENT_LSB " %.6f %.6f %.6f %.6f\n"
#define FMT_OUT_DIVIDER   CMD_OUT_DIVIDER " %d %d %d %d\n"
#define FMT_DOWNSAMPLE    CMD_DOWNSAMPLE " %d\n"
#define FMT_AVG_WND       CMD_AVG_WND " %u %u %u %u\n"
#define FMT_ERROR         "ERROR\n"

//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
static char msg[170];
static char rx_msg[RX_LINE_MAX];
static char rx_line[RX_LINE_MAX];
static uint16_t rx_index = 0;
static SerialProtocol_UpdateCallback UpdateParamModule = NULL;
// флаги для каждого параметра
static uint8_t flagRX = 0;
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
void SerialProcess(void)
{
  if(flagRX)
  {
    ProcessLine(rx_msg);
    memset(rx_msg, 0, sizeof(rx_msg));
  }
  flagRX = 0;  
}
//------------------------------------------------------------------------------
void TransmiteCurrent_double(double* currents)
{
  float t = HAL_GetTick() / 1000.0f;
  snprintf(msg, sizeof(msg),
           "%.3f %4.9f %4.9f %4.9f %4.9f\n",
           (double)t, 
           (double)currents[0], 
           (double)currents[1], 
           (double)currents[2], 
           (double)currents[3]);
  
  while(CDC_Transmit_FS((uint8_t*)msg, strlen(msg)) != USBD_OK)
  {
    HAL_Delay(1);
  }
}
//------------------------------------------------------------------------------
void TransmiteCurrent_uint(uint32_t* currents)
{
  float t = HAL_GetTick() / 1000.0f;
  snprintf(msg, sizeof(msg),
           "%.3f %d %d %d %d\n",
           (double)t, 
           currents[0], 
           currents[1], 
           currents[2], 
           currents[3]);
  
  while(CDC_Transmit_FS((uint8_t*)msg, strlen(msg)) != USBD_OK)
  {
    HAL_Delay(1);
  }
}
//------------------------------------------------------------------------------
void ReceiveCDCAccumulate(uint8_t* Buf, uint32_t *Len)
{
    for (uint32_t i = 0; i < *Len; i++)
    {
        char c = (char)Buf[i];

        if (c == '\n' || c == '\r')
        {
            if (rx_index > 0)
            {
                rx_line[rx_index] = '\0';
                memcpy(rx_msg, rx_line, strlen(rx_line));
                flagRX = 1;
                //ProcessLine(rx_line);
                rx_index = 0;
            }
        }
        else
        {
            if (rx_index < RX_LINE_MAX - 1)
                rx_line[rx_index++] = c;
            else
                rx_index = 0;
        }
    }
}
//------------------------------------------------------------------------------
void ProcessLine(const char* line)
{
    char keyword[16];
    float in_values[INA226_COUNT];

    int n = sscanf(line, "%15s %f %f %f %f", keyword, &in_values[0], &in_values[1], &in_values[2], &in_values[3]);

    if(n == 1)
    {
        if(strcmp(keyword, CMD_ADDRESS) == 0)
        {
            uint8_t value[INA226_COUNT];
            GetParameters(value, TYPE_ADDRESS);
            snprintf(msg, sizeof(msg), FMT_ADDRESS,
                     value[0], value[1], value[2], value[3]);
        }
        else if(strcmp(keyword, CMD_RSHUNT) == 0)
        {
            float value[INA226_COUNT];
            GetParameters(value, TYPE_RSHUNT);
            snprintf(msg, sizeof(msg), FMT_RSHUNT,
                     (double)value[0], (double)value[1], 
                     (double)value[2], (double)value[3]);
        }
        else if(strcmp(keyword, CMD_CURRENT_LSB) == 0)
        {
            float value[INA226_COUNT];
            GetParameters(value, TYPE_CURRENT_LSB);
            snprintf(msg, sizeof(msg), FMT_CURRENT_LSB,
                     (double)value[0], (double)value[1], 
                     (double)value[2], (double)value[3]);
        }
        else if(strcmp(keyword, CMD_OUT_DIVIDER) == 0)
        {
            uint32_t value[INA226_COUNT];
            GetParameters(value, TYPE_OUT_DIVIDER);
            snprintf(msg, sizeof(msg), FMT_OUT_DIVIDER,
                     value[0], value[1], value[2], value[3]);
        }
        else if(strcmp(keyword, CMD_DOWNSAMPLE) == 0)
        {
            uint32_t value;
            GetParameters(&value, TYPE_DOWNSAMPLE);
            snprintf(msg, sizeof(msg), FMT_DOWNSAMPLE, value);
        }
        else if(strcmp(keyword, CMD_AVG_WND) == 0)
        {
            uint8_t value[INA226_COUNT];
            GetParameters(value, TYPE_AVG_WINDOW_SIZE);
            snprintf(msg, sizeof(msg), FMT_AVG_WND,
                     value[0], value[1], value[2], value[3]);
        }
        else
        {
            snprintf(msg, sizeof(msg), FMT_ERROR);
        }

        //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        while(CDC_Transmit_FS((uint8_t*)msg, strlen(msg)) != USBD_OK)
        {
          HAL_Delay(1);
        }
    }
    else if(n == 2)
    {
        if(strcmp(keyword, CMD_DOWNSAMPLE) == 0)
        {
          uint32_t value = (uint32_t)in_values[0];
          UpdateParamModule(&value, TYPE_DOWNSAMPLE);
        }
        else
        {
            snprintf(msg, sizeof(msg), FMT_ERROR);
            //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            while(CDC_Transmit_FS((uint8_t*)msg, strlen(msg)) != USBD_OK)
            {
              HAL_Delay(1);
            }
        }
    }
    else if(n == 5)
    {
        if(strcmp(keyword, CMD_ADDRESS) == 0)
        {
            uint8_t data[INA226_COUNT];
            for(size_t i = 0; i < INA226_COUNT; i++)
                data[i] = (uint8_t)in_values[i];
            UpdateParamModule(data, TYPE_ADDRESS);
        }
        else if(strcmp(keyword, CMD_RSHUNT) == 0)
            UpdateParamModule(in_values, TYPE_RSHUNT);
        else if(strcmp(keyword, CMD_CURRENT_LSB) == 0)
            UpdateParamModule(in_values, TYPE_CURRENT_LSB);
        else if(strcmp(keyword, CMD_OUT_DIVIDER) == 0)
        {
            uint32_t data[INA226_COUNT];
            for(size_t i = 0; i < INA226_COUNT; i++)
                data[i] = (uint32_t)in_values[i];
            UpdateParamModule(data, TYPE_OUT_DIVIDER);
        }
        else if(strcmp(keyword, CMD_AVG_WND) == 0)
        {
            uint8_t data[INA226_COUNT];
            for(size_t i = 0; i < INA226_COUNT; i++)
                data[i] = (uint8_t)in_values[i];
            UpdateParamModule(data, TYPE_AVG_WINDOW_SIZE);
        }
    }
    else
    {
        snprintf(msg, sizeof(msg), FMT_ERROR);
        //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        while(CDC_Transmit_FS((uint8_t*)msg, strlen(msg)) != USBD_OK)
        {
          HAL_Delay(1);
        }
    }
}
