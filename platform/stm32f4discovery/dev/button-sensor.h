#include <contiki.h>
#include <lib/sensors.h>

#define	BUTTON_RELEASED 0
#define BUTTON_SHORT_PRESS 1
#define BUTTON_LONG_PRESS 2

extern const struct sensors_sensor button_sensor;
