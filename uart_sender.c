/* uart_sender.c
 * This module is used to send characters onto the serial port
 *
 */

#include "stm32f4xx_hal.h"

#include "cmsis_os2.h"              	// ::CMSIS:RTOS2


extern UART_HandleTypeDef huart1; ;

uint8_t DataToSend[16] ;
uint16_t DataToSendSize=0 ;

void prepare_cmd(uint8_t *DataPwm, uint8_t *DataPwmSize);


void CopyDataToSend(uint8_t const *_Data, uint16_t const _DataSize)
{
    for (int i=0 ; i<_DataSize ;i++)
   	 DataToSend[i]=_Data[i] ;

    DataToSendSize=_DataSize ;
}

void StartSendUart(void *argument)
{
  uint8_t DataPwm[9] = "";
  uint8_t DataPwmSize = 0;

  CopyDataToSend(DataPwm, DataPwmSize) ;

  for(;;)
  {
	osDelay(100);
	huart1.Lock=HAL_UNLOCKED;
	HAL_UART_Transmit(&huart1, DataToSend, DataToSendSize, 100) ;

	prepare_cmd(DataPwm, &DataPwmSize);

	CopyDataToSend(DataPwm, DataPwmSize) ;
  }

}



