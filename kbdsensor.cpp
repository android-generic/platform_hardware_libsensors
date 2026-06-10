/**
 *
 * Atkbd style sensor
 *
 * Copyright (C) 2011-2013 The Android-x86 Open Source Project
 * Copyright (C) 2026 BlissLabs
 *
 * by Chih-Wei Huang <cwhuang@linux.org.tw>
 * modified by HMTheBoy154 <hmtheboy154@blisslabs.org>
 *
 * Licensed under GPLv2 or later
 *
 **/

#define LOG_TAG "KbdSensor"

#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <cinttypes>
#include <sys/stat.h>
#include <sys/inotify.h>
#include <poll.h>
#include <fcntl.h>
#include <dirent.h>
#include <cutils/log.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <hardware/sensors.h>
#include <cutils/properties.h>
#include <tinyxml2.h>
#include <vector>
#include <bitset>
#include <string>

using namespace tinyxml2;

struct KbdActionConfig {
    int key = -1;
    int mod1 = -1;
    int mod2 = -1;
    bool isMatch(const std::bitset<KEY_MAX>& state, int code) const {
        if (key != code) return false;
        if (mod1 != -1 && !state.test(mod1)) return false;
        if (mod2 != -1 && !state.test(mod2)) return false;
        return true;
    }
};

struct KbdInputConfig {
    std::string name;
    KbdActionConfig rot0;
    KbdActionConfig rot90;
    KbdActionConfig rot180;
    KbdActionConfig rot270;
};

struct KbdSensorKeys {
	char name[64];
	int keys[8];
} KeysType[] = {
	{ "", { } },
	{ "AT Translated Set 2 keyboard", { EV_KEY, KEY_UP, KEY_RIGHT, KEY_DOWN, KEY_LEFT, KEY_LEFTALT, KEY_LEFTCTRL, 1 } },
	{ "AT Translated Set 2 keyboard", { EV_MSC, 91, 115, 123, 109, KEY_LEFTALT, KEY_LEFTCTRL, 3 } },
	{ "AT Translated Set 2 keyboard", { EV_KEY, KEY_F5, KEY_F8, KEY_F6, KEY_F7, KEY_LEFTALT, KEY_LEFTCTRL, 1 } },
	{ "AT Translated Set 2 keyboard", { EV_KEY, KEY_F9, KEY_F12, KEY_F10, KEY_F11, KEY_LEFTALT, KEY_LEFTCTRL, 1 } },
	{ "Asus Laptop extra buttons", { EV_KEY, KEY_F9, KEY_F12, KEY_F10, KEY_F11, KEY_LEFTALT, KEY_LEFTCTRL, 2 } },
	{ "HP WMI hotkeys", { -1, KEY_DIRECTION, 0, 0, 0, 0, 0, 3 } },
};

const int ID_ACCELERATION = (SENSORS_HANDLE_BASE + 0);

template <typename T> struct SensorFd : T {
	SensorFd(const struct hw_module_t *module);
};

template <typename T> SensorFd<T>::SensorFd(const struct hw_module_t *module)
{
	memset(this, 0, sizeof(*this));
	this->common.tag     = HARDWARE_DEVICE_TAG;
	this->common.version = SENSORS_DEVICE_API_VERSION_1_3;
	this->common.module  = const_cast<struct hw_module_t *>(module);
}

struct SensorPollContext : SensorFd<sensors_poll_device_1> {
  public:
	SensorPollContext(const struct hw_module_t *module, struct hw_device_t **device);
	~SensorPollContext();
	bool isValid() const { return inotify_fd >= 0; }

  private:
	static int poll_close(struct hw_device_t *dev);
	static int poll_activate(struct sensors_poll_device_t *dev, int handle, int enabled);
	static int poll_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns);
	static int poll_poll(struct sensors_poll_device_t *dev, sensors_event_t *data, int count);
	static int poll_batch(struct sensors_poll_device_1* dev, int sensor_handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns);
	static int poll_flush(struct sensors_poll_device_1* dev, int sensor_handle);

	int doPoll(sensors_event_t *data, int count);
	std::vector<KbdInputConfig> parseXmlConfig();

	void openDevice(const char* path);
	void removeDevice(size_t index);
	void scanDevices();

	enum {
		ROT_0,
		ROT_90,
		ROT_180,
		ROT_270
	};

	struct TrackedDevice {
		int fd;
		std::string path;
		bool is_xml;
		KbdInputConfig xml_config;
		KbdSensorKeys* fallback_config;
		std::bitset<KEY_MAX> key_state;
	};

	bool enabled;
	int rotation;
	int64_t sampling_period_ns;
	int inotify_fd;
	int watch_input;
	int watch_config;
	std::vector<struct pollfd> pfds;
	std::vector<TrackedDevice> devices;
	std::vector<KbdInputConfig> parsed_configs;
	KbdSensorKeys* fallback_ktype;
	sensors_event_t orients[4];
};

void parse_kbd_keys_from_prop(char *prop, KbdSensorKeys *ktype)
{
	strlcpy(ktype->name, strsep(&prop, ","), sizeof(ktype->name));
	sscanf(prop, "%d,%d,%d,%d,%d,%d,%d,%d", ktype->keys,
			ktype->keys + 1, ktype->keys + 2, ktype->keys + 3, ktype->keys + 4, ktype->keys + 5, ktype->keys + 6, ktype->keys + 7);
	ALOGD("[%s]: %d,%d,%d,...", ktype->name, ktype->keys[0], ktype->keys[1], ktype->keys[2]);
}

SensorPollContext::SensorPollContext(const struct hw_module_t *module, struct hw_device_t **device)
      : SensorFd<sensors_poll_device_1>(module), enabled(false), rotation(ROT_0), inotify_fd(-1), watch_input(-1), watch_config(-1)
{
	common.close = poll_close;
	activate     = poll_activate;
	setDelay     = poll_setDelay;
	poll         = poll_poll;
	batch        = poll_batch;
	flush        = poll_flush;

	inotify_fd = inotify_init1(IN_NONBLOCK);
	if (inotify_fd >= 0) {
		watch_input = inotify_add_watch(inotify_fd, "/dev/input", IN_CREATE | IN_DELETE);
		watch_config = inotify_add_watch(inotify_fd, "/data/system", IN_CLOSE_WRITE | IN_MOVED_TO | IN_DELETE | IN_CREATE);
		struct pollfd p;
		p.fd = inotify_fd;
		p.events = POLLIN;
		p.revents = 0;
		pfds.push_back(p);
	}

	char prop[PROPERTY_VALUE_MAX];
	fallback_ktype = KeysType;
	if (property_get("vendor.hal.sensors.kbd.keys", prop, 0))
		parse_kbd_keys_from_prop(prop, fallback_ktype);
	else if (property_get("vendor.hal.sensors.kbd.type", prop, 0))
		fallback_ktype = &KeysType[atoi(prop)];
	else
		fallback_ktype = 0;

	parsed_configs = parseXmlConfig();

	scanDevices();

	if (devices.empty() && inotify_fd < 0) {
		ALOGW("could not find any kbdsensor device and inotify failed");
		return;
	}
	*device = &common;

	orients[ROT_0].version = sizeof(sensors_event_t);
	orients[ROT_0].sensor = ID_ACCELERATION;
	orients[ROT_0].type = SENSOR_TYPE_ACCELEROMETER;
	orients[ROT_0].acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;
	orients[ROT_270] = orients[ROT_180] = orients[ROT_90] = orients[ROT_0];
	const double angle = 20.0;
	const double cos_angle = GRAVITY_EARTH * cos(angle / M_PI);
	const double sin_angle = GRAVITY_EARTH * sin(angle / M_PI);
	orients[ROT_0].acceleration.x   = 0.0;
	orients[ROT_0].acceleration.y   = cos_angle;
	orients[ROT_0].acceleration.z   = sin_angle;
	orients[ROT_90].acceleration.x  = cos_angle;
	orients[ROT_90].acceleration.y  = 0.0;
	orients[ROT_90].acceleration.z  = sin_angle;
	orients[ROT_180].acceleration.x = 0.0;
	orients[ROT_180].acceleration.y = -cos_angle;
	orients[ROT_180].acceleration.z = -sin_angle;
	orients[ROT_270].acceleration.x = -cos_angle;
	orients[ROT_270].acceleration.y = 0.0;
	orients[ROT_270].acceleration.z = -sin_angle;

	ALOGD("%s: module=%p dev=%p pfds_count=%zu", __FUNCTION__, module, this, pfds.size());
}

void SensorPollContext::openDevice(const char* path) {
	for (const auto& dev : devices) {
		if (dev.path == path) return;
	}
	int fd = open(path, O_RDWR);
	if (fd < 0) return;

	char name[PATH_MAX];
	name[0] = '\0';
	if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
		close(fd);
		return;
	}

	bool is_xml = false;
	KbdInputConfig xml_cfg;
	KbdSensorKeys* fb_cfg = nullptr;

	if (!parsed_configs.empty()) {
		for (const auto& config : parsed_configs) {
			if (config.name == name) {
				is_xml = true;
				xml_cfg = config;
				break;
			}
		}
	}

	if (!is_xml) {
		if (fallback_ktype) {
			if (!strcmp(name, fallback_ktype->name)) {
				fb_cfg = fallback_ktype;
			}
		}
		if (!fb_cfg) {
			KbdSensorKeys* temp_ktype = KeysType + (sizeof(KeysType) / sizeof(KeysType[0]));
			while (--temp_ktype != KeysType) {
				if (!strcmp(name, temp_ktype->name)) {
					fb_cfg = temp_ktype;
					break;
				}
			}
		}
	}

	if (is_xml || fb_cfg) {
		TrackedDevice dev;
		dev.fd = fd;
		dev.path = path;
		dev.is_xml = is_xml;
		dev.xml_config = xml_cfg;
		dev.fallback_config = fb_cfg;
		devices.push_back(dev);

		struct pollfd p;
		p.fd = fd;
		p.events = POLLIN;
		p.revents = 0;
		pfds.push_back(p);
		ALOGI("Hotplug matched and opened %s, fd=%d", name, fd);
	} else {
		close(fd);
	}
}

void SensorPollContext::removeDevice(size_t index) {
	if (index < devices.size()) {
		close(devices[index].fd);
		devices.erase(devices.begin() + index);
		pfds.erase(pfds.begin() + index + 1); // +1 because pfds[0] is inotify_fd
	}
}

void SensorPollContext::scanDevices() {
	const char *dirname = "/dev/input";
	if (DIR *dir = opendir(dirname)) {
		while (struct dirent *de = readdir(dir)) {
			if (de->d_name[0] != 'e') continue; // eventX
			char path[PATH_MAX];
			snprintf(path, PATH_MAX, "%s/%s", dirname, de->d_name);
			openDevice(path);
		}
		closedir(dir);
	}
}

std::vector<KbdInputConfig> SensorPollContext::parseXmlConfig() {
	std::vector<KbdInputConfig> configs;
	XMLDocument doc;
	if (doc.LoadFile("/data/system/kbd_config.xml") != XML_SUCCESS) {
		return configs;
	}

	XMLElement* root = doc.RootElement();
	if (!root || strcmp(root->Name(), "kbd_config") != 0) {
		ALOGE("Invalid root element in kbd_config.xml");
		return configs;
	}

	for (XMLElement* device = root->FirstChildElement("device"); device != nullptr; device = device->NextSiblingElement("device")) {
		const char* name = device->Attribute("name");
		if (!name) continue;

		KbdInputConfig config;
		config.name = name;

		auto parseRot = [](XMLElement* rotElem, KbdActionConfig& action) {
			if (!rotElem) return;
			action.key = rotElem->IntAttribute("key", -1);
			action.mod1 = rotElem->IntAttribute("mod1", -1);
			action.mod2 = rotElem->IntAttribute("mod2", -1);
		};

		parseRot(device->FirstChildElement("rot0"), config.rot0);
		parseRot(device->FirstChildElement("rot90"), config.rot90);
		parseRot(device->FirstChildElement("rot180"), config.rot180);
		parseRot(device->FirstChildElement("rot270"), config.rot270);

		configs.push_back(config);
	}
	return configs;
}

SensorPollContext::~SensorPollContext()
{
	for (auto& dev : devices) {
		close(dev.fd);
	}
	if (inotify_fd >= 0) {
		close(inotify_fd);
	}
}

int SensorPollContext::poll_close(struct hw_device_t *dev)
{
	ALOGD("%s: dev=%p", __FUNCTION__, dev);
	delete reinterpret_cast<SensorPollContext *>(dev);
	return 0;
}

int SensorPollContext::poll_activate(struct sensors_poll_device_t *dev, int handle, int enabled)
{
	ALOGD("%s: dev=%p handle=%d enabled=%d", __FUNCTION__, dev, handle, enabled);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	ctx->enabled = enabled;
	return 0;
}

int SensorPollContext::poll_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
	ALOGD("%s: dev=%p handle=%d ns=%" PRId64, __FUNCTION__, dev, handle, ns);
	SensorPollContext *ctx = reinterpret_cast<SensorPollContext *>(dev);
	ctx->sampling_period_ns = ns;
	return EXIT_SUCCESS;
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
	return -EINVAL;
}

int SensorPollContext::doPoll(sensors_event_t *data, int count)
{
	if (!isValid())
		return 0;

	while (int pollres = ::poll(pfds.data(), pfds.size(), -1)) {
		if (pollres < 0) {
			if (errno == EINTR) continue;
			ALOGE("%s: poll error: %s", __FUNCTION__, strerror(errno));
			break;
		}

		bool event_processed = false;

		if (inotify_fd >= 0 && (pfds[0].revents & POLLIN)) {
			char buffer[4096];
			int length = ::read(inotify_fd, buffer, sizeof(buffer));
			if (length > 0) {
				int i = 0;
				bool xml_changed = false;
				while (i < length) {
					struct inotify_event *event = (struct inotify_event *) &buffer[i];
					if (event->len) {
						if (event->wd == watch_input) {
							if (event->mask & IN_CREATE) {
								if (strncmp(event->name, "event", 5) == 0) {
									char path[PATH_MAX];
									snprintf(path, PATH_MAX, "/dev/input/%s", event->name);
									openDevice(path);
								}
							} else if (event->mask & IN_DELETE) {
								if (strncmp(event->name, "event", 5) == 0) {
									char path[PATH_MAX];
									snprintf(path, PATH_MAX, "/dev/input/%s", event->name);
									for (size_t d = 0; d < devices.size(); ++d) {
										if (devices[d].path == path) {
											removeDevice(d);
											break;
										}
									}
								}
							}
						} else if (event->wd == watch_config) {
							if (strcmp(event->name, "kbd_config.xml") == 0) {
								xml_changed = true;
							}
						}
					}
					i += sizeof(struct inotify_event) + event->len;
				}

				if (xml_changed) {
					ALOGI("kbd_config.xml changed, reloading configurations");
					parsed_configs = parseXmlConfig();

					for (size_t d = 0; d < devices.size(); ) {
						char name[PATH_MAX];
						name[0] = '\0';
						bool is_xml = false;
						KbdInputConfig xml_cfg;

						if (ioctl(devices[d].fd, EVIOCGNAME(sizeof(name) - 1), &name) >= 1) {
							for (const auto& config : parsed_configs) {
								if (config.name == name) {
									is_xml = true;
									xml_cfg = config;
									break;
								}
							}
						}

						if (is_xml || devices[d].fallback_config) {
							devices[d].is_xml = is_xml;
							if (is_xml) devices[d].xml_config = xml_cfg;
							++d;
						} else {
							ALOGI("Device %s no longer matches any config, closing", devices[d].path.c_str());
							removeDevice(d);
						}
					}
					scanDevices();
				}
			}
		}

		for (size_t i = 1; i < pfds.size(); ) {
			size_t d = i - 1;
			if (pfds[i].revents & (POLLERR | POLLHUP)) {
				ALOGW("Device disconnected: %s", devices[d].path.c_str());
				removeDevice(d);
				continue;
			} else if (pfds[i].revents & POLLIN) {
				struct input_event iev;
				size_t res = ::read(pfds[i].fd, &iev, sizeof(iev));
				if (res < sizeof(iev)) {
					ALOGW("insufficient input data on %s", devices[d].path.c_str());
					removeDevice(d);
					continue;
				}
				ALOGV("type=%d scancode=%d value=%d from fd=%d", iev.type, iev.code, iev.value, pfds[i].fd);

				int rot = -1;
				if (devices[d].is_xml) {
					if (iev.type == EV_KEY && iev.code < KEY_MAX) {
						devices[d].key_state.set(iev.code, iev.value != 0); // 1 for press, 0 for release, 2 for repeat
						if (iev.value) { // Trigger on press or repeat
							if (devices[d].xml_config.rot0.isMatch(devices[d].key_state, iev.code)) rot = ROT_0;
							else if (devices[d].xml_config.rot90.isMatch(devices[d].key_state, iev.code)) rot = ROT_90;
							else if (devices[d].xml_config.rot180.isMatch(devices[d].key_state, iev.code)) rot = ROT_180;
							else if (devices[d].xml_config.rot270.isMatch(devices[d].key_state, iev.code)) rot = ROT_270;
						}
					}
				} else {
					KbdSensorKeys* ktype = devices[d].fallback_config;
					int *keys = ktype->keys;
					if (iev.type == keys[0]) {
						int input = (keys[0] == EV_MSC) ? iev.value : iev.code;
						if (input == keys[1])
							rot = ROT_0;
						else if (input == keys[2])
							rot = ROT_90;
						else if (input == keys[3])
							rot = ROT_180;
						else if (input == keys[4])
							rot = ROT_270;
						else if (input == keys[5] || input == keys[6])
							rot = rotation;
						else
							rot = -1;
					} else if (iev.type == EV_KEY) {
						if (iev.code == keys[1] && iev.value) {
							if (rotation == ROT_270) rot = ROT_0;
							else rot = rotation + 1;
						}
						if (iev.code == keys[2] && iev.value) {
							if (rotation == ROT_0) rot = ROT_270;
							else rot = rotation - 1;
						}
					} else if (iev.type == EV_SW && iev.code == SW_TABLET_MODE) {
						if (!iev.value) rot = ROT_0;
						else if (rotation == ROT_0) rot = ROT_90;
					}
				}

				if (rot >= 0 && rot <= ROT_270) {
					if (rot != rotation) {
						ALOGI("orientation changed from %d to %d", rotation * 90, rot * 90);
						rotation = rot;
					}
					if (enabled && count > 0) {
						event_processed = true;
					}
				}
				++i;
			} else {
				++i;
			}
		}
		if (event_processed) break;
	}

	int cnt;
	struct timespec t = { 0, 0 };
	data[0] = orients[rotation];
	clock_gettime(CLOCK_MONOTONIC, &t);
	data[0].timestamp = int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
	struct timespec delay = { 0, static_cast<long>(sampling_period_ns) };
	
	int max_cnt = 1;
	for (const auto& dev : devices) {
		if (!dev.is_xml && dev.fallback_config) {
			max_cnt = dev.fallback_config->keys[7];
			break;
		}
	}

	for (cnt = 1; !nanosleep(&delay, 0) && cnt < max_cnt && cnt < count; ++cnt) {
		data[cnt] = data[cnt - 1];
		data[cnt].timestamp += sampling_period_ns;
	}
	ALOGV("%s: dev=%p rotation=%d cnt=%d", __FUNCTION__, this, rotation * 90, cnt);
	return cnt;
}

static int open_kbd_sensor(const struct hw_module_t *module, const char *id, struct hw_device_t **device)
{
	ALOGD("%s: id=%s", __FUNCTION__, id);
	SensorPollContext *ctx = new SensorPollContext(module, device);
	return (ctx && ctx->isValid()) ? 0 : -EINVAL;
}

static struct sensor_t sSensorListInit[] = {
	{
		.name = "Kbd Orientation Sensor",
		.vendor = "Android-x86 Open Source Project",
		.version = 2,
		.handle = ID_ACCELERATION,
		.type = SENSOR_TYPE_ACCELEROMETER,
		.maxRange = 2.8f,
		.resolution = 1.0f/4032.0f,
		.power = 3.0f,
		.minDelay = 0,
		.fifoReservedEventCount = 0,
		.fifoMaxEventCount = 0,
		.stringType = 0,
		.requiredPermission = 0,
		.maxDelay = 2000,
		.flags = SENSOR_FLAG_CONTINUOUS_MODE,
		.reserved = { }
	}
};

static int sensors_get_sensors_list(struct sensors_module_t *, struct sensor_t const **list)
{
	*list = sSensorListInit;
	return sizeof(sSensorListInit) / sizeof(struct sensor_t);
}

static struct hw_module_methods_t sensors_methods = {
	.open = open_kbd_sensor
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.module_api_version = 2,
		.hal_api_version = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "Kbd Orientation Sensor",
		.author = "Chih-Wei Huang",
		.methods = &sensors_methods,
		.dso = 0,
		.reserved = { }
	},
	.get_sensors_list = sensors_get_sensors_list
};
