
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include <stdbool.h>

extern void DoSendBalloon(void) ;


static unsigned int BalloonNeeded=0 ;
static bool finished_updating = false;

void UpdateBalloonNeeded(uint8_t const _BalloonNeeded)
{
   	 BalloonNeeded =_BalloonNeeded ;

}


/**
 * This task is used to send as many balloons as needed, according
 * to what is asked in UpdateBalloonNeeded
 */
void SendBalloon(void *argument)
{
    while(1)
    {
   	 if (BalloonNeeded>0)
   	 {
   		 DoSendBalloon();
   		 BalloonNeeded-- ;
   	 }
   	 osDelay(10);

    }
}


