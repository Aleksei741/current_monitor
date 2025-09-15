
//******************************************************************************
//include
//******************************************************************************
#include "main.h"
#include "CurrentMonitor.h"
#include "flash_store.h"
#include "ina226.h"
#include "serial_protocol.h"
#include "MovingAverage.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
//******************************************************************************
// Секция определения констант
//******************************************************************************
#define DELAY_SAVE_PARAM        10000
//#define DEBUG_OUT

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0f)
#endif
//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------
uint8_t flagData = 0;
uint8_t flagSaveParams = 0;
uint8_t flagUpdateINA226 = 0;
uint32_t timeDelaySaveParam = 0;

Parameters_t parameters = {
    .INA226_Addr      = {0x40, 0x41, 0x44, 0x45},
    .Rshunt_INA226    = {0.150f, 0.040f, 2.500f, 2.500f},
    .Current_LSB_INA226 = {0.00025f, 0.00025f, 0.00025f, 0.00025f},
    .OutDivider = {1, 1, 1, 1},
    .dataDownsampleFactor = 1,
    .avgWindowSize = {1, 1, 1, 1}
};

float currents[INA226_COUNT];
//double outCurrents[INA226_COUNT];
uint32_t outCurrents[INA226_COUNT];
uint32_t cntDownsample = 0;

static Parameters_t lastSavedParams;
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
void Load_Parameters(void);
void Save_Parameters(void);
void UpdateParameters(void* param, ParamType_t type);
//******************************************************************************
// Секция описания функций
//******************************************************************************
void CurrentMonitorInit(void)
{
  flagData = 0;
  FlashStore_Init(); // Инициализация работы с Flash  
  
  
  Load_Parameters(); // Загрузка параметров микросхем из Flash
  memcpy(&lastSavedParams, &parameters, sizeof(Parameters_t)); // обновляем копию
  INA226_Init(); // Инициализация INA226
  INA226_SetCalibration(parameters.INA226_Addr, // Установка калибровки
                        parameters.Rshunt_INA226, parameters.
                          Current_LSB_INA226); 
  INA226_start_reade(); // Начать первый опрос
  InitSerialProtocol(UpdateParameters);
}
//------------------------------------------------------------------------------
void CurrentMonitorProcess(void)
{
  if(INA226_get_status() != INA226_OK) 
  {
    ina226_status_t status = INA226_get_status();
    if(status == INA226_CPLT) 
    {
      // Опрос завершен, можно читать данные
      INA226_ReadCurrents(currents); //мА
      flagData = 1;
    }
    else if(status == INA226_TIMEOUT || status == INA226_ERR)
    {
#ifdef DEBUG_OUT
    uint32_t tick = HAL_GetTick(); // время в мс
    float t = tick / 1000.0f;      // в секундах
    const float freq = 0.5f;       // частота сигнала, Гц
    const float amplitude = 1.0f;  // амплитуда тока, А

    for(size_t i = 0; i < INA226_COUNT; i++)
    {
        float phase = i * (M_PI_2); // 0, pi/2, pi, 3pi/2
        currents[i] = amplitude * sinf(2.0f * M_PI * freq * t + phase);
    }
#else
      for(size_t i = 0; i < INA226_COUNT; i++)    
        currents[i] = 0;
#endif
      flagData = 1;
    }
    
    if(flagUpdateINA226)
    {
      flagUpdateINA226 = 0;
      __disable_irq(); // Отключить все прерывания
      INA226_SetCalibration(parameters.INA226_Addr, 
                            parameters.Rshunt_INA226, parameters.
                              Current_LSB_INA226);
      __enable_irq(); // Включить снова
    }
    
    INA226_start_reade(); // Начать новый опрос
  }
  
  if(flagData) 
  {
    flagData = 0;  
    
    for(size_t i = 0; i < INA226_COUNT; i++)    //Заносим в буфер скользящего окна
      MovingAverage_AddSample(i, currents[i]);
    
    if(++cntDownsample >= parameters.dataDownsampleFactor)
    {
      cntDownsample = 0;
      // Отправка данных по USB, если опрос завершен
      for(size_t i = 0; i < INA226_COUNT; i++)
      {
        //outCurrents[i] = (double)MovingAverage_Calc(i, parameters.avgWindowSize[i]) * 1000.0 * parameters.OutDivider[i]; 
        outCurrents[i] = (uint32_t)roundf((float)MovingAverage_Calc(i, parameters.avgWindowSize[i]) * 1000000.0f /*мА в нА*/
                                    / (float)parameters.OutDivider[i]);
      }
      //TransmiteCurrent_double(outCurrents);     
      TransmiteCurrent_uint(outCurrents); 
    }
  }
  
  SerialProcess();
  
  Save_Parameters();  
  
  //HAL_Delay(1000);
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
//------------------------------------------------------------------------------
void Load_Parameters(void)
{
  FlashStore_Init();
  FlashStore_ReadParams(&parameters, sizeof(Parameters_t));
}
//------------------------------------------------------------------------------
void Save_Parameters(void)
{
    // сравнение параметров с предыдущими сохранёнными
    if (memcmp(&parameters, &lastSavedParams, sizeof(Parameters_t)) != 0 && timeDelaySaveParam < HAL_GetTick())
    {
        if (FlashStore_WriteParams(&parameters, sizeof(Parameters_t)) == HAL_OK)
        {
            memcpy(&lastSavedParams, &parameters, sizeof(Parameters_t)); // обновляем копию
        }
    }
}
//------------------------------------------------------------------------------
// Функция колбэк
void UpdateParameters(void* param, ParamType_t type)
{
  if(param == NULL) return;
  
  switch(type)
  {
  case TYPE_RSHUNT:
    {
      float* vals = (float*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        parameters.Rshunt_INA226[i] = vals[i];      
      flagUpdateINA226 = 1;
      flagSaveParams = 1;
      timeDelaySaveParam = HAL_GetTick() + DELAY_SAVE_PARAM;
      break;
    }
    
  case TYPE_CURRENT_LSB:
    {
      float* vals = (float*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        parameters.Current_LSB_INA226[i] = vals[i];      
      flagUpdateINA226 = 1;
      flagSaveParams = 1;
      timeDelaySaveParam = HAL_GetTick() + DELAY_SAVE_PARAM;
      break;
    }
    
  case TYPE_ADDRESS:
    {
      uint8_t* vals = (uint8_t*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        parameters.INA226_Addr[i] = vals[i];      
      flagUpdateINA226 = 1;
      flagSaveParams = 1;
      timeDelaySaveParam = HAL_GetTick() + DELAY_SAVE_PARAM;
      break;
    }
    
  case TYPE_OUT_DIVIDER:
    {
      uint32_t* vals = (uint32_t*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
      {
        if(vals[i] == 0)
          parameters.OutDivider[i] = 1; 
        else
          parameters.OutDivider[i] = vals[i];
      }
	  flagSaveParams = 1;
	  timeDelaySaveParam = HAL_GetTick() + DELAY_SAVE_PARAM;
	  break;
    }
    
  case TYPE_DOWNSAMPLE:
    {
      parameters.dataDownsampleFactor = *(uint32_t*)param;       
      flagSaveParams = 1;
      timeDelaySaveParam = HAL_GetTick() + DELAY_SAVE_PARAM;
      break;
    }
    
  case TYPE_AVG_WINDOW_SIZE:
    {
      uint8_t* vals = (uint8_t*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        parameters.avgWindowSize[i] = vals[i];
      flagSaveParams = 1;
      timeDelaySaveParam = HAL_GetTick() + DELAY_SAVE_PARAM;
      break;
    }
    
  default:
    return;
    break;
  }
}
//------------------------------------------------------------------------------
// Функция получения текущих параметров
void GetParameters(void* param, ParamType_t type)
{
  if(param == NULL) return;
  
  switch(type)
  {
  case TYPE_RSHUNT:
    {
      float* vals = (float*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        vals[i] = parameters.Rshunt_INA226[i];
      break;
    }
    
  case TYPE_CURRENT_LSB:
    {
      float* vals = (float*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        vals[i] = parameters.Current_LSB_INA226[i];
      break;
    }
    
  case TYPE_ADDRESS:
    {
      uint8_t* vals = (uint8_t*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        vals[i] = parameters.INA226_Addr[i];
      break;
    }
    
  case TYPE_OUT_DIVIDER:
    {
      uint32_t* vals = (uint32_t*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        vals[i] = parameters.OutDivider[i];
      break;
    }
    
   case TYPE_DOWNSAMPLE:
    {
      uint32_t* vals = (uint32_t*)param;
      *vals = parameters.dataDownsampleFactor;
      break;
    }
    
   case TYPE_AVG_WINDOW_SIZE:
    {
      uint8_t* vals = (uint8_t*)param;
      for(size_t i = 0; i < INA226_COUNT; i++)
        vals[i] = parameters.avgWindowSize[i];
      break;
    }
    
  default:
    break;
  }
}
//------------------------------------------------------------------------------
