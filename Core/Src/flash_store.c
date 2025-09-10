//******************************************************************************
//include
//******************************************************************************
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "flash_store.h"
//******************************************************************************
// Секция определения констант
//******************************************************************************
#define BLOCK_SIZE       128U
#define WORDS_IN_BLOCK   (BLOCK_SIZE / 4U)
#define FERST_START_VALUE   0xA5A5A5A5
//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------
static uint32_t last_page_addr   = 0;
static uint32_t page_size        = 0;
static uint32_t last_page_index  = 0;
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
static void Flash_Unlock(void);
static void Flash_Lock(void);
static void Flash_EraseLastPage(void);
//******************************************************************************
// Секция описания функций
//******************************************************************************
static void Flash_Unlock(void) 
{ 
    HAL_FLASH_Unlock(); 
}
//------------------------------------------------------------------------------
static void Flash_Lock(void)   
{ 
    HAL_FLASH_Lock();   
}
//------------------------------------------------------------------------------
static void Flash_EraseLastPage(void) 
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;
    eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector       = last_page_index;
    eraseInit.NbSectors    = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&eraseInit, &pageError);
}
//------------------------------------------------------------------------------
void FlashStore_Init(void) 
{
#if defined(STM32F401xC)
    // STM32F401RCT6 (256 KB flash)
    last_page_index = 5;          // последний сектор
    last_page_addr  = 0x08020000; // начало сектора 5
    //last_page_addr  = 0x08060000; // начало сектора 7
    page_size       = 128 * 1024; // размер сектора 5
#else
    // Другие STM32F4 (например F407/F411) – нужно настраивать вручную
    last_page_index = 11;         // последний сектор
    last_page_addr  = 0x080E0000; // пример адреса последнего сектора
    page_size       = 128 * 1024;
#endif
}
//------------------------------------------------------------------------------
uint32_t FlashStore_GetPageSize(void)      
{ 
    return page_size; 
}
//------------------------------------------------------------------------------
uint32_t FlashStore_GetLastPageAddr(void)  
{ 
    return last_page_addr; 
}
//------------------------------------------------------------------------------
uint32_t FlashStore_GetLastPageIndex(void) 
{ 
    return last_page_index; 
}
//------------------------------------------------------------------------------
HAL_StatusTypeDef FlashStore_WriteParams(void* param, size_t size)
{
  if(param == NULL) return HAL_ERROR;
  if(size+1 > BLOCK_SIZE) return HAL_ERROR; // Ограничение блока 128 байт
    
  uint32_t buffer[WORDS_IN_BLOCK];
  memset(buffer, 0xFF, sizeof(buffer));
  memcpy(buffer, param, size); // копируем данные структуры в буфер
  
  uint32_t* flag_ptr = &buffer[size / 4]; // следующее слово после структуры
  *flag_ptr = FERST_START_VALUE;          // записываем флаг
  
  __disable_irq();
  Flash_Unlock();
  Flash_EraseLastPage();
  
  HAL_StatusTypeDef status = HAL_OK;
  for(uint32_t i = 0; i < WORDS_IN_BLOCK; i++)
  {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               last_page_addr + i*4,
                               buffer[i]);
    if(status != HAL_OK) break;
  }
  
  Flash_Lock();
  __enable_irq();
  
  return status;
}
//------------------------------------------------------------------------------
uint32_t flagFirstStart;
void FlashStore_ReadParams(void* param, size_t size)
{
  if(param == NULL) return;
  if(size+1 > BLOCK_SIZE) size = BLOCK_SIZE;
  
  flagFirstStart = *((uint32_t*)(last_page_addr + size)); // читаем слово после структуры
  if(flagFirstStart != FERST_START_VALUE)
    return; // flash пустая, не читаем
  
  memcpy(param, (void*)last_page_addr, size);
}
//------------------------------------------------------------------------------
void FlashStore_Clear(void) 
{
    Flash_Unlock();
    Flash_EraseLastPage();
    Flash_Lock();
}
//------------------------------------------------------------------------------
