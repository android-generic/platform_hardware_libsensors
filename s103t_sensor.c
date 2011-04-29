/*
 * s103t_sensor.c
 *
 *      Created on: 19.04.2011
 *      Author: Oliver Dill (oliver@ratio-informatik.de)
 *      Licensed under GPLv2 or later
 */

#define LOG_TAG "S103TSensors"

#include <linux/types.h>
#include <linux/input.h>
#include <fcntl.h>
#include <cutils/sockets.h>
#include <cutils/log.h>
#include <cutils/native_handle.h>
#include <dirent.h>
#include <math.h>
#include <hardware/sensors.h>

#define DRIVER_DESC 			"Lenovo front-screen buttons driver"
#define SKEY_ROTATE_MAPPING		KEY_F12
#define ID_ACCELERATION  		(SENSORS_HANDLE_BASE + 0)

typedef struct SensorData {
	struct sensors_data_device_t device;
	int events_fd;
	sensors_data_t orientation;
} SensorData;

typedef struct SensorControl {
	struct sensors_control_device_t device;
	int fd;
	uint32_t active_sensors;
} SensorControl;

/* this must return a file descriptor that will be used to read
 * the sensors data (it is passed to data__data_open() below
 */
static native_handle_t*
control__open_data_source(struct sensors_control_device_t *dev) {
	SensorControl* ctl = (void*) dev;
	native_handle_t *handle;
	handle = native_handle_create(1, 1);
	ctl->fd = -1;
	const char *dirname = "/dev/input";
	DIR *dir = opendir(dirname);
	if (dir != NULL) {
		struct dirent *de;
		// loop over all "eventXX" in /dev/input and look for our driver
		LOGD("%s: looping over all eventXX...",__FUNCTION__);
		do {
			de = readdir(dir);
			if (de->d_name[0] != 'e') // eventX
				continue;
			char name[PATH_MAX];
			snprintf(name, PATH_MAX, "%s/%s", dirname, de->d_name);
			LOGD("%s: open device %s",__FUNCTION__, name);
			ctl->fd = open(name, O_RDWR);
			if (ctl->fd < 0) {
				LOGE("could not open %s, %s", name, strerror(errno));
				continue;
			}
			name[sizeof(name) - 1] = '\0';
			if (ioctl(ctl->fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
				LOGE("could not get device name for %s, %s\n", name, strerror(errno));
				name[0] = '\0';
			}

			if (!strcmp(name, DRIVER_DESC)) {
				// ok, it's our driver, stop the loop ...
				LOGI("found device %s", name);
				break;
			}
			close(ctl->fd);
		} while (de != NULL);
		LOGD("%s: stop loop and closing directory",__FUNCTION__);
		closedir(dir);
	}
	// and save the handle.
	handle->data[0] = ctl->fd;
	LOGD("%s: dev=%p handle=%p data[0]=%d", __FUNCTION__, dev, handle, handle->data[0]);
	return handle;
}

static int control__activate(struct sensors_control_device_t *dev, int handle,
		int enabled) {
	LOGD("%s: called",__FUNCTION__);
	return 0;
}

static int control__set_delay(struct sensors_control_device_t *dev, int32_t ms) {
	LOGD("%s: called",__FUNCTION__);
	return 0;
}

static int control__wake(struct sensors_control_device_t *dev) {
	LOGD("%s: called",__FUNCTION__);
	return 0;
}

static int control__close(struct hw_device_t *dev) {
	LOGD("%s: called",__FUNCTION__);
	return 0;
}

static int data__data_open(struct sensors_data_device_t *dev,
		native_handle_t* handle) {
	LOGD("%s: called",__FUNCTION__);
	SensorData* data = (void*) dev;
	LOGD("%s: dev=%p fd=%d", __FUNCTION__, dev, handle->data[0]);
	data->events_fd = dup(handle->data[0]);
	native_handle_close(handle);
	native_handle_delete(handle);
	return 0;
}

static int data__data_close(struct sensors_data_device_t *dev) {
	SensorData* data = (void*) dev;
	LOGD("%s: dev=%p", __FUNCTION__, dev);
	if (data->events_fd > 0) {
		LOGD("%s: closing device",__FUNCTION__);
		close(data->events_fd);
		data->events_fd = -1;
	}
	return 0;
}

static int data__poll(struct sensors_data_device_t *dev, sensors_data_t* values) {
	SensorData* data = (void*) dev;
	LOGD("%s: dev=%p", __FUNCTION__, dev);
	while (1) {
		struct input_event iev;
		size_t res = read(data->events_fd, &iev, sizeof(iev));
		if (res == sizeof(iev)) {
			if (iev.type == EV_KEY) {
				LOGD("type=%d scancode=%d value=%d from fd=%d", iev.type, iev.code, iev.value, data->events_fd);
				if (iev.code == SKEY_ROTATE_MAPPING && iev.value == 1) {
					if (data->orientation.acceleration.x != 0.0) {
						data->orientation.acceleration.x = 0.00;
						data->orientation.acceleration.y = 9.77;
						data->orientation.acceleration.z = 0.81;
					} else {
						data->orientation.acceleration.x = 9.77;
						data->orientation.acceleration.y = 0.00;
						data->orientation.acceleration.z = 0.81;
					}
					*values = data->orientation;
					LOGI("orientation changed");
					return 0;
				}
			}

			if (iev.type == EV_SW) {
				LOGD("%s: switching to/from Table Mode type=%d scancode=%d value=%d", __FUNCTION__,iev.type, iev.code, iev.value);
				if (iev.value == 0) {
					data->orientation.acceleration.x = 0.00;
					data->orientation.acceleration.y = 9.77;
					data->orientation.acceleration.z = 0.81;
				} else {
					data->orientation.acceleration.x = 9.77;
					data->orientation.acceleration.y = 0.00;
					data->orientation.acceleration.z = 0.81;
				}
				*values = data->orientation;
				LOGI("orientation changed");
				return 0;
			}
		}
	}
}

static int data__close(struct hw_device_t *dev) {
	LOGD("%s: data__close",__FUNCTION__);
	return 0;
}

static const struct sensor_t sSensorListInit[] = {
	{ .name =
		"S103T Orientation sensor",
		.vendor = "Oliver Dill",
		.version = 1,
		.handle = ID_ACCELERATION,
		.type = SENSOR_TYPE_ACCELEROMETER,
		.maxRange = 2.8f,
		.resolution = 1.0f/4032.0f,
		.power = 3.0f,
		.reserved = { }
	},
};

static uint32_t sensors__get_sensors_list(struct sensors_module_t* module,
		struct sensor_t const** list) {
	LOGD("%s: sensors__get_sensors_list called",__FUNCTION__);
	// there is exactly one sensor available, the accelerometer sensor
	*list = sSensorListInit;
	return 1;
}

static int open_sensors(const struct hw_module_t* module, const char* name,
		struct hw_device_t* *device) {
	int status = -EINVAL;

	LOGD("%s: name=%s", __FUNCTION__, name);

	if (!strcmp(name, SENSORS_HARDWARE_CONTROL)) {
		SensorControl *dev = malloc(sizeof(*dev));
		LOGD("%s: init sensors control device" , __FUNCTION__);
		memset(dev, 0, sizeof(*dev));

		dev->device.common.tag = HARDWARE_DEVICE_TAG;
		dev->device.common.version = 0;
		dev->device.common.module = (struct hw_module_t*) module;
		dev->device.common.close = control__close;
		dev->device.open_data_source = control__open_data_source;
		dev->device.activate = control__activate;
		dev->device.set_delay = control__set_delay;
		dev->device.wake = control__wake;
		dev->fd = -1;

		*device = &dev->device.common;
		status = 0;
	} else if (!strcmp(name, SENSORS_HARDWARE_DATA)) {

		SensorData *dev = malloc(sizeof(*dev));
		LOGD("%s: init sensors_data_device" , __FUNCTION__);
		memset(dev, 0, sizeof(*dev));
		dev->orientation.sensor = ID_ACCELERATION;
		dev->orientation.acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
		dev->device.common.tag = HARDWARE_DEVICE_TAG;
		dev->device.common.version = 0;
		dev->device.common.module = (struct hw_module_t*) module;
		dev->device.common.close = data__close;
		dev->device.data_open = data__data_open;
		dev->device.data_close = data__data_close;
		dev->device.poll = data__poll;
		dev->events_fd = -1;
		*device = &dev->device.common;
		status = 0;
	}
	return status;
}

static struct hw_module_methods_t sensors_module_methods = { .open =
		open_sensors };

const struct sensors_module_t HAL_MODULE_INFO_SYM = {
		.common = {
				.tag = HARDWARE_MODULE_TAG,
				.version_major = 1,
				.version_minor = 0,
				.id = SENSORS_HARDWARE_MODULE_ID,
				.name = "s103t SENSORS Module",
				.author = "Oliver Dill",
				.methods = &sensors_module_methods,
		},
		.get_sensors_list = sensors__get_sensors_list
};

