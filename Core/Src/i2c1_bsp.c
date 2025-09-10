//******************************************************************************
//include
//******************************************************************************
#include "i2c1_bsp.h"
#include <stm32f4xx_hal.h>
//******************************************************************************
// Секция определения переменных, используемых в модуле
//******************************************************************************
//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
static i2c_callback_t i2c_cb = NULL;
//******************************************************************************
// Секция прототипов локальных функций
//******************************************************************************
static int hi2c_cb_valid(I2C_HandleTypeDef *hi2c);
//******************************************************************************
// Секция описания функций
//******************************************************************************
void I2C1_Init(void)
{
  
    //Инициализация проведена в 
    //main.c -> static void MX_I2C1_Init(void)
    //stm32f4xx_hal_msp.c -> void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
    /*GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // I2C1 SCL (PB6), SDA (PB7)
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;     //400кГц Fast-mode
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&hi2c1) != HAL_OK) 
    {
        while(1);
    }*/

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}
//------------------------------------------------------------------------------
i2c1_ret_t I2C1_Write(uint8_t addr7bit, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if(!data || len == 0) return I2C1_ERR;
    return (HAL_I2C_Master_Transmit(&hi2c1, addr7bit << 1, (uint8_t*)data, len, timeout_ms) == HAL_OK) ? I2C1_OK : I2C1_ERR;
}
//------------------------------------------------------------------------------
i2c1_ret_t I2C1_Read(uint8_t addr7bit, uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if(!data || len == 0) return I2C1_ERR;
    return (HAL_I2C_Master_Receive(&hi2c1, addr7bit << 1, data, len, timeout_ms) == HAL_OK) ? I2C1_OK : I2C1_ERR;
}
//------------------------------------------------------------------------------
i2c1_ret_t I2C1_Write_IT(uint8_t addr7bit, const uint8_t *data, size_t len)
{
    if(!data || len == 0) return I2C1_ERR;
    return (HAL_I2C_Master_Transmit_IT(&hi2c1, addr7bit << 1, (uint8_t*)data, len) == HAL_OK) ? I2C1_OK : I2C1_ERR;
}
//------------------------------------------------------------------------------
i2c1_ret_t I2C1_Read_IT(uint8_t addr7bit, uint8_t *data, size_t len)
{
    if(!data || len == 0) return I2C1_ERR;
    return (HAL_I2C_Master_Receive_IT(&hi2c1, addr7bit << 1, data, len) == HAL_OK) ? I2C1_OK : I2C1_ERR;
}
//------------------------------------------------------------------------------
void I2C1_SetCallback(i2c_callback_t cb)
{
    i2c_cb = cb;
}
//------------------------------------------------------------------------------
// Колбэки HAL
//------------------------------------------------------------------------------
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c_cb_valid(hi2c) && i2c_cb) i2c_cb(I2C1_TX_CPLT);
}
//------------------------------------------------------------------------------
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c_cb_valid(hi2c) && i2c_cb) i2c_cb(I2C1_RX_CPLT);
}
//------------------------------------------------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c_cb_valid(hi2c) && i2c_cb) i2c_cb(I2C1_ERR);
}
//------------------------------------------------------------------------------
/* Вспомогательная функция для проверки, что это I2C1 */
static int hi2c_cb_valid(I2C_HandleTypeDef *hi2c)
{
    return (hi2c == &hi2c1);
}
//------------------------------------------------------------------------------
// Interrupt Handlers
//------------------------------------------------------------------------------
void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}