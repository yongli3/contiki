#include <lib/sensors.h>
#include <button-sensor.h>

const struct sensors_sensor *sensors[] =
{
	&button_sensor,
	0
};

unsigned char sensors_flags[(sizeof(sensors) / sizeof(struct sensors_sensor *))];
