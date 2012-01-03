#ifndef _DESKTOP_H
#define _DESKTOP_H

#include <unistd.h>
#include <sys/time.h>

struct desktop_info {
	bool slider; // slider switch state, True means CLI mode
	struct timeval sketch_start_time;
	bool quadcopter; // use quadcopter outputs
	unsigned framerate;
	float initial_height;
};

extern struct desktop_info desktop_state;

void desktop_serial_select_setup(fd_set *fds, int *fd_high);
void sitl_input(void);
void sitl_setup(void);
int sitl_gps_pipe(void);
ssize_t sitl_gps_read(int fd, void *buf, size_t count);
void sitl_update_compass(float heading, float roll, float pitch, float yaw);
void sitl_update_gps(float latitude, float longitude, float altitude,
		     float speedN, float speedE, bool have_lock);
void sitl_update_adc(float roll, float pitch, float yaw,
		     float rollRate, float pitchRate, float yawRate,
		     float xAccel, float yAccel, float zAccel,
		     float airspeed);
void sitl_setup_adc(void);
void sitl_update_barometer(float altitude);

#endif
