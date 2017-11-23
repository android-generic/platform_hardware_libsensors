/**
 *
 * IIO style sensor
 *
 * Copyright (C) 2015 The Android-x86 Open Source Project
 *
 * by Chih-Wei Huang <cwhuang@linux.org.tw>
 *
 * Licensed under GPLv2 or later
 *
 **/

#define LOG_TAG "iio-sensors"

#include <new>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <cinttypes>
#include <fcntl.h>
#include <dirent.h>
#include <cutils/log.h>
#include <hardware/sensors.h>
#include <cutils/properties.h>

static const char *IIO_DIR        = "/sys/bus/iio/devices/";

enum {
	ID_ACCELERATION           = SENSORS_HANDLE_BASE,
	ID_MAGNETIC_FIELD,
	ID_ORIENTATION,
	ID_LIGHT,
	ID_PROXIMITY,
	ID_GYROSCOPE,
	ID_PRESSURE,
	ID_TEMPERATURE,
	ID_ROT_VECTOR,
	ID_SYNCOMPASS,
	MAX_SENSORS
};


// 720 LSG = 1G
#define LSG                         (1024.0f)
#define NUMOFACCDATA                (8.0f)

// conversion of acceleration data to SI units (m/s^2)
#define RANGE_A                     (2*GRAVITY_EARTH)
#define RESOLUTION_A                (RANGE_A/(256*NUMOFACCDATA))

// conversion of magnetic data to uT units
#define RANGE_M                     (2048.0f)
#define RESOLUTION_M                (0.01)

/* conversion of orientation data to degree units */
#define CONVERT_O                   (1.0f/64.0f)
#define CONVERT_O_A                 (CONVERT_O)
#define CONVERT_O_P                 (CONVERT_O)
#define CONVERT_O_R                 (-CONVERT_O)

// conversion of gyro data to SI units (radian/sec)
#define RANGE_G                     (2000.0f*(float)M_PI/180.0f)
#define RESOLUTION_G                (RANGE_G/(2000*NUMOFACCDATA))

// conversion of pressure and temperature data
#define CONVERT_PRESSURE            (1.0f/100.0f)
#define CONVERT_TEMPERATURE         (1.0f/100.0f)

#define SENSOR_STATE_MASK           (0x7FFF)

// proximity threshold
#define PROXIMITY_THRESHOLD_GP2A    (5.0f)

// used in timespec_to_ns calculations
const long NSEC_PER_SEC           = 1000000000L;

#define BIT(x) (1 << (x))

struct SensorEvent : public sensors_event_t {
	SensorEvent(int32_t id, int32_t t);
};

SensorEvent::SensorEvent(int32_t id, int32_t t)
{
	version = sizeof(sensors_event_t);
	sensor = id;
	type = t;

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	timestamp = int64_t(ts.tv_sec) * NSEC_PER_SEC + ts.tv_nsec;
}

class sensor_base : public sensor_t {
  protected:
	sensor_base() { memset(this, 0, sizeof(*this)); }

	bool enabled;
	char *path;
	const char ***nodes;
	struct timespec delay;
};

class SensorBase : public sensor_base {
  public:
	SensorBase();
	virtual ~SensorBase();

	operator bool() const { return enabled; }
	int setDelay(int64_t ns);
	bool scan(const char *d);
	virtual int activate(bool);
	virtual int readEvents(sensors_event_t *data, int);

  protected:
	int read_sysfs_str(const char *file, char *buf);
	int read_sysfs_int(const char *file);
	float read_sysfs_float(const char *file);
};

SensorBase::SensorBase()
{
	vendor = "Android-x86 Open Source Project";
	version = 1;

	delay.tv_nsec = 200000000L;
}

SensorBase::~SensorBase()
{
	free(path);
}

int SensorBase::setDelay(int64_t ns)
{
	delay.tv_sec = ns / NSEC_PER_SEC;
	delay.tv_nsec = ns % NSEC_PER_SEC;
	return 0;
}

bool SensorBase::scan(const char *p)
{
	int i;
	char node[PATH_MAX];
	while (const char **ns = *nodes) {
		for (i = 0; ns[i]; ++i) {
			snprintf(node, PATH_MAX, "%s/%s", p, ns[i]);
			if (access(node, F_OK))
				break;
		}
		if (!ns[i])
			break;
		nodes++;
	}
	if (*nodes) {
		path = strdup(p);
		node[0] = '\0';
		for (i = 0; (*nodes)[i]; ++i)
			strncat(strncat(node, (*nodes)[i], 1024), " ", 1024);
		ALOGD("found node %s: %s", path, node);
	}
	return (path != 0);
}

int SensorBase::activate(bool e)
{
	enabled = e;
	return 0;
}

int SensorBase::readEvents(sensors_event_t *data, int)
{
	nanosleep(&delay, 0);
	SensorEvent *e = new (data) SensorEvent(handle, type);
	return 1;
}

int SensorBase::read_sysfs_str(const char *file, char *buf)
{
	int res = 0;
	char filename[PATH_MAX];
	snprintf(filename, PATH_MAX, "%s/%s", path, file);
	int fd = open(filename, O_RDONLY);
	if (fd >= 0) {
		ssize_t sz = read(fd, buf, 4096);
		if (sz < 0) {
			ALOGE("failed to read from %s: %s", filename, strerror(errno));
			res = -errno;
		}
		close(fd);
	}
	return res;
}

int SensorBase::read_sysfs_int(const char *file)
{
	char buf[4096];
	return read_sysfs_str(file, buf) ? 0 : atoi(buf);
}

float SensorBase::read_sysfs_float(const char *file)
{
	char buf[4096];
	return read_sysfs_str(file, buf) ? 0 : atof(buf);
}

template <int H> class Sensor : SensorBase {
  public:
	Sensor();
	virtual int readEvents(sensors_event_t *data, int);

	static SensorBase *probe(const char *d);
};

template<int H>
SensorBase *Sensor<H>::probe(const char *p)
{
	Sensor<H> *s = new Sensor<H>;
	s->handle = H;
	if (!s->scan(p)) {
		delete s;
		s = 0;
	}
	return s;
}

template<> Sensor<ID_ACCELERATION>::Sensor()
{
	static const char *ns0[] = { "in_accel_scale", 0 };
	static const char **ns[] = { ns0, 0 };
	nodes = ns;

	name = "IIO Accelerometer Sensor";
	type = SENSOR_TYPE_ACCELEROMETER;
	maxRange = RANGE_A;
	resolution = RESOLUTION_A;
	power = 0.23f;
	minDelay = 10000;
}

template<> int Sensor<ID_ACCELERATION>::readEvents(sensors_event_t *data, int cnt)
{
	static float scale = read_sysfs_float((*nodes)[0]);
	int ret = SensorBase::readEvents(data, cnt);
	char cm[PROPERTY_VALUE_MAX];
	float m[9];
	int v[3];

	property_get("hal.sensors.iio.accel.matrix", cm, "-1,0,0,0,1,0,0,0,-1" );
	sscanf(cm, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]);

	for (int i = 0; i < ret; ++i) {
		v[0] = read_sysfs_int("in_accel_x_raw");
		v[1] = read_sysfs_int("in_accel_y_raw");
		v[2] = read_sysfs_int("in_accel_z_raw");
		// create matrix * vector product
		data[i].acceleration.x = scale * (m[0] * v[0] + m[1] * v[1] + m[2] * v[2]);
		data[i].acceleration.y = scale * (m[3] * v[0] + m[4] * v[1] + m[5] * v[2]);
		data[i].acceleration.z = scale * (m[6] * v[0] + m[7] * v[1] + m[8] * v[2]);
		data[i].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
	}
	return ret;
}

template<> Sensor<ID_MAGNETIC_FIELD>::Sensor()
{
	static const char *ns0[] = { "in_magn_scale", 0, 0 };
	static const char *ns1[] = { "in_magn_x_scale", "in_magn_y_scale", "in_magn_z_scale", 0 };
	static const char **ns[] = { ns0, ns1, 0 };
	nodes = ns;

	name = "IIO Magnetic Sensor";
	type = SENSOR_TYPE_MAGNETIC_FIELD;
	maxRange = RANGE_M;
	resolution = RESOLUTION_M;
	power = 0.1f;
	minDelay = 0;
}

template<> int Sensor<ID_MAGNETIC_FIELD>::readEvents(sensors_event_t *data, int cnt)
{
	static float scale_x = read_sysfs_float((*nodes)[0]);
	static float scale_y = (*nodes)[1] ? read_sysfs_float((*nodes)[1]) : scale_x;
	static float scale_z = (*nodes)[2] ? read_sysfs_float((*nodes)[2]) : scale_x;
	int ret = SensorBase::readEvents(data, cnt);
	char cm[PROPERTY_VALUE_MAX];
	float m[9];
	int v[3];

	property_get("hal.sensors.iio.magn.matrix", cm, "1,0,0,0,1,0,0,0,1" );
	sscanf(cm, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]);

	for (int i = 0; i < ret; ++i) {
		v[0] = read_sysfs_int("in_magn_x_raw");
		v[1] = read_sysfs_int("in_magn_y_raw");
		v[2] = read_sysfs_int("in_magn_z_raw");
		// create matrix * vector product
		data[i].magnetic.x = scale_x * (m[0] * v[0] + m[1] * v[1] + m[2] * v[2]);
		data[i].magnetic.y = scale_y * (m[3] * v[0] + m[4] * v[1] + m[5] * v[2]);
		data[i].magnetic.z = scale_z * (m[6] * v[0] + m[7] * v[1] + m[8] * v[2]);
		data[i].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;
	}
	return ret;
}

template<> Sensor<ID_LIGHT>::Sensor()
{
	static const char *ns0[] = { "in_illuminance_scale", 0 };
	static const char *ns1[] = { "in_illuminance_calibscale", 0 };
	static const char **ns[] = { ns0, ns1, 0 };
	nodes = ns;

	name = "IIO Ambient Light Sensor";
	type = SENSOR_TYPE_LIGHT;
	maxRange = 50000.0f;
	resolution = 1.0f;
	power = 0.75f;
	minDelay = 0;
}

template<> int Sensor<ID_LIGHT>::readEvents(sensors_event_t *data, int cnt)
{
	static float scale = read_sysfs_float((*nodes)[0]);
	int ret = SensorBase::readEvents(data, cnt);
	for (int i = 0; i < ret; ++i) {
		data[i].light = scale * read_sysfs_int("in_illuminance_input");
	}
	return ret;
}

template<> Sensor<ID_GYROSCOPE>::Sensor()
{
	static const char *ns0[] = { "in_anglvel_scale", 0 };
	static const char **ns[] = { ns0, 0 };
	nodes = ns;

	name = "IIO Gyro 3D Sensor";
	type = SENSOR_TYPE_GYROSCOPE;
	maxRange = RANGE_G;
	resolution = RESOLUTION_G;
	power = 6.10f;
	minDelay = 0;
}

template<> int Sensor<ID_GYROSCOPE>::readEvents(sensors_event_t *data, int cnt)
{
	static float scale = read_sysfs_float((*nodes)[0]);
	int ret = SensorBase::readEvents(data, cnt);
	char cm[PROPERTY_VALUE_MAX];
	float m[9];
	int v[3];

	property_get("hal.sensors.iio.anglvel.matrix", cm, "1,0,0,0,1,0,0,0,1" );
	sscanf(cm, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7], &m[8]);

	for (int i = 0; i < ret; ++i) {
		v[0] = read_sysfs_int("in_anglvel_x_raw");
		v[1] = read_sysfs_int("in_anglvel_y_raw");
		v[2] = read_sysfs_int("in_anglvel_z_raw");
		// create matrix * vector product
		data[i].gyro.x = scale * (m[0] * v[0] + m[1] * v[1] + m[2] * v[2]);
		data[i].gyro.y = scale * (m[3] * v[0] + m[4] * v[1] + m[5] * v[2]);
		data[i].gyro.z = scale * (m[6] * v[0] + m[7] * v[1] + m[8] * v[2]);
		data[i].gyro.status = SENSOR_STATUS_ACCURACY_HIGH;
	}
	return ret;
}

static SensorBase *(*probeSensors[])(const char *) = {
	Sensor<ID_ACCELERATION>::probe,
	Sensor<ID_MAGNETIC_FIELD>::probe,
	Sensor<ID_LIGHT>::probe,
	Sensor<ID_GYROSCOPE>::probe,
};

class SensorPollContext : sensors_poll_device_1 {
  public:
	SensorPollContext(const struct hw_module_t *module, struct hw_device_t **device);
	~SensorPollContext();

	bool isValid() const;
	int getSensor(struct sensor_t const **list) const;

  private:
	static int poll_close(struct hw_device_t *dev);
	static int poll_activate(struct sensors_poll_device_t *dev, int handle, int enabled);
	static int poll_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns);
	static int poll_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count);
	static int poll_batch(struct sensors_poll_device_1* dev, int sensor_handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns);
	static int poll_flush(struct sensors_poll_device_1* dev, int sensor_handle);

	int doPoll(sensors_event_t *data, int count);

	sensor_t sensors_list[MAX_SENSORS];
	SensorBase *sensors[MAX_SENSORS];
	int count;
};

static SensorPollContext *sctx = 0;

SensorPollContext::SensorPollContext(const struct hw_module_t *module, struct hw_device_t **device)
{
	memset(this, 0, sizeof(*this));

	common.tag     = HARDWARE_DEVICE_TAG;
	common.version = SENSORS_DEVICE_API_VERSION_1_3;
	common.module  = const_cast<struct hw_module_t *>(module);
	common.close   = poll_close;
	activate       = poll_activate;
	setDelay       = poll_setDelay;
	poll           = poll_poll;
	batch          = poll_batch;
	flush          = poll_flush;
	*device        = &common;

	char path[PATH_MAX];
	strcpy(path, IIO_DIR);
	int len = strlen(path);
	if (DIR *dir = opendir(path)) {
		while (struct dirent *de = readdir(dir)) {
			if (!strncmp(de->d_name, "iio:device", 10)) {
				strcpy(path + len, de->d_name);
				for (size_t i = 0; i < (sizeof(probeSensors) / sizeof(*probeSensors)); ++i) {
					if (SensorBase *s = probeSensors[i](path)) {
						sensors[i] = s;
						sensors_list[count++] = *s;
						ALOGD("found %s", s->name);
					}
				}
			}
		}
		closedir(dir);
	}
	ALOGD("%s: module=%p sensors: %d", __FUNCTION__, module, count);
}

SensorPollContext::~SensorPollContext()
{
	for (int i = 0; i < MAX_SENSORS; ++i) {
		delete sensors[i];
	}
}

bool SensorPollContext::isValid() const
{
	return count > 0;
}

int SensorPollContext::getSensor(struct sensor_t const **list) const
{
	*list = sensors_list;
	return count;
}

int SensorPollContext::poll_close(struct hw_device_t *dev)
{
	ALOGD("%s: dev=%p", __FUNCTION__, dev);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	delete ctx;
	if (sctx == ctx) {
		sctx = 0;
	} else {
		ALOGW("close a ctx(%p) rather than sctx(%p)", ctx, sctx);
	}
	return 0;
}

int SensorPollContext::poll_activate(struct sensors_poll_device_t *dev, int handle, int enabled)
{
	ALOGD("%s: dev=%p handle=%d enabled=%d", __FUNCTION__, dev, handle, enabled);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	if (handle >= 0 && handle < MAX_SENSORS && ctx->sensors[handle])
		return ctx->sensors[handle]->activate(enabled);
	else
		return -EINVAL;
}

int SensorPollContext::poll_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	if (handle >= 0 && handle < MAX_SENSORS && ctx->sensors[handle])
		return ctx->sensors[handle]->setDelay(ns);
	else
		return -EINVAL;
}

int SensorPollContext::poll_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count)
{
	ALOGV("%s: dev=%p data=%p count=%d", __FUNCTION__, dev, data, count);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	return ctx->doPoll(data, count);
}

int SensorPollContext::poll_batch(struct sensors_poll_device_1* dev, int sensor_handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{
	ALOGD("%s: dev=%p sensor_handle=%d flags=%d sampling_period_ns=%" PRId64 " max_report_latency_ns=%" PRId64,
			__FUNCTION__, dev, sensor_handle, flags, sampling_period_ns, max_report_latency_ns);
	return poll_setDelay(&dev->v0, sensor_handle, sampling_period_ns);
}

int SensorPollContext::poll_flush(struct sensors_poll_device_1* dev, int sensor_handle)
{
	ALOGD("%s: dev=%p sensor_handle=%d", __FUNCTION__, dev, sensor_handle);
	return EXIT_SUCCESS;
}

int SensorPollContext::doPoll(sensors_event_t *data, int cnt)
{
	if (!isValid())
		return 0;

	int events = 0;
	for (int i = 0; cnt > 0 && i < MAX_SENSORS; ++i) {
		if (sensors[i] && *sensors[i]) {
			int nb = sensors[i]->readEvents(data, cnt);
			cnt -= nb;
			data += nb;
			events += nb;
		}
	}

	return events;
}

static int open_iio_sensors(const struct hw_module_t *module, const char *id, struct hw_device_t **device)
{
	ALOGD("%s: id=%s", __FUNCTION__, id);
	if (!sctx) {
		sctx = new SensorPollContext(module, device);
	}
	return (sctx && sctx->isValid()) ? 0 : -EINVAL;
}

static int sensors_get_sensors_list(struct sensors_module_t *, struct sensor_t const **list)
{
	ALOGD("enter %s", __FUNCTION__);
	return sctx ? sctx->getSensor(list) : 0;
}

static struct hw_module_methods_t sensors_methods = {
	.open = open_iio_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.module_api_version = 1,
		.hal_api_version = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "IIO Sensors",
		.author = "Chih-Wei Huang",
		.methods = &sensors_methods,
		.dso = 0,
		.reserved = { }
	},
	.get_sensors_list = sensors_get_sensors_list,
	.set_operation_mode = 0
};
