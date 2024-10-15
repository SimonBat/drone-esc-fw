#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "bsp.h"

#define SYSTEM_LED_ENABLE_TMO		25U /* ms */
#define SYSTEM_LED_DISABLE_TMO		500U /* ms */

typedef enum{
	SYSTEM_LED_STATE_DISABLED,
	SYSTEM_LED_STATE_ENABLED
}system_led_state_te;

typedef struct{
	__IO uint16_t ledTmo;
	uint8_t ledState;
}system_ts;

/* Global functions declarations */
void SYSTEM_Update_TMO(void);

#endif
