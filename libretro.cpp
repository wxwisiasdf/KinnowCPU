// STL
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <exception>
// Verilator
#include <verilated.h>
#include <verilated_vpi.h>
#include <verilated.cpp>
// Project
#include "Vl2k_cpu.h"
#include "Vl2k_cpu__Syms.h"
#include "Vl2k_cpu.cpp"
#include "Vl2k_cpu__1.cpp"
#include "Vl2k_cpu__Slow.cpp"
#include "Vl2k_cpu__Syms.cpp"

// RetroArch
#if defined(_WIN32) && !defined(_XBOX)
#include <windows.h>
#endif
#include "libretro.h"

#define VIDEO_WIDTH 1024
#define VIDEO_HEIGHT 768

// System
static std::unique_ptr<Vl2k_cpu> l2k_cpu;
static uint32_t l2k_rom[32768];
static uint32_t l2k_nvram[16384];
static uint32_t l2k_ram[16384];

// Implementation
static uint8_t *framebuffer;
static struct retro_log_callback logging;
static retro_log_printf_t log_cb;
static bool use_audio_cb;
char retro_base_directory[4096];
char retro_game_path[4096];

static void fallback_log(enum retro_log_level level, const char *fmt, ...)
{
	(void)level;
	va_list va;
	va_start(va, fmt);
	vfprintf(stderr, fmt, va);
	va_end(va);
}

static retro_environment_t environ_cb;

void retro_init(void)
{
	framebuffer = (uint8_t *)malloc(VIDEO_WIDTH * VIDEO_HEIGHT * sizeof(uint32_t));
	const char *dir = NULL;
	if (environ_cb(RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY, &dir) && dir)
		snprintf(retro_base_directory, sizeof(retro_base_directory), "%s", dir);

	// Create the system
	l2k_cpu = std::unique_ptr<Vl2k_cpu>(new Vl2k_cpu());

	// Populate ROM
	FILE* rom_fp = fopen("boot.bin", "rb");
	if(rom_fp == nullptr)
	{
		log_cb(RETRO_LOG_INFO, "Unable to open boot.bin.\n");
	}
	else
	{
		fread(l2k_rom, 1, sizeof(l2k_rom), rom_fp);
		fclose(rom_fp);
	}

	retro_reset();
}

void retro_deinit(void)
{
	l2k_cpu->final();
	l2k_cpu.reset();

	free(framebuffer);
	framebuffer = NULL;
}

unsigned retro_api_version(void)
{
	return RETRO_API_VERSION;
}

void retro_set_controller_port_device(unsigned port, unsigned device)
{
	log_cb(RETRO_LOG_INFO, "Plugging device %u into port %u.\n", device, port);
}

void retro_get_system_info(struct retro_system_info *info)
{
	memset(info, 0, sizeof(*info));
	info->library_name = "KinnowCPU";
	info->library_version = "0.1";
	info->need_fullpath = false;
	info->valid_extensions = "bin|iso|dae";
}

static retro_video_refresh_t video_cb;
static retro_audio_sample_t audio_cb;
static retro_audio_sample_batch_t audio_batch_cb;
static retro_input_poll_t input_poll_cb;
static retro_input_state_t input_state_cb;

void retro_get_system_av_info(struct retro_system_av_info *info)
{
	info->timing.fps = 28.0f;
	info->timing.sample_rate = 44100;
	info->geometry.base_width = VIDEO_WIDTH;
	info->geometry.base_height = VIDEO_HEIGHT;
	info->geometry.max_width = VIDEO_WIDTH;
	info->geometry.max_height = VIDEO_HEIGHT;
	info->geometry.aspect_ratio = 0.f;
}

void retro_set_environment(retro_environment_t cb)
{
	environ_cb = cb;
	if (cb(RETRO_ENVIRONMENT_GET_LOG_INTERFACE, &logging))
		log_cb = logging.log;
	else
		log_cb = fallback_log;

	static const struct retro_controller_description controllers[] = {
		{"Joypad", RETRO_DEVICE_SUBCLASS(RETRO_DEVICE_JOYPAD, 0)},
	};
	static const struct retro_controller_info ports[] = {
		{controllers, 1},
		{NULL, 0},
	};
	cb(RETRO_ENVIRONMENT_SET_CONTROLLER_INFO, (void *)ports);
}

void retro_set_audio_sample(retro_audio_sample_t cb)
{
	audio_cb = cb;
}

void retro_set_audio_sample_batch(retro_audio_sample_batch_t cb)
{
	audio_batch_cb = cb;
}

void retro_set_input_poll(retro_input_poll_t cb)
{
	input_poll_cb = cb;
}

void retro_set_input_state(retro_input_state_t cb)
{
	input_state_cb = cb;
}

void retro_set_video_refresh(retro_video_refresh_t cb)
{
	video_cb = cb;
}

static unsigned phase;

void retro_reset(void)
{
	l2k_cpu->irq = 0;
	l2k_cpu->rst = 1;
	for(size_t i = 0; i < 10; i++) {
		l2k_cpu->clk = !l2k_cpu->clk;
		l2k_cpu->eval();
	}
	l2k_cpu->rst = 0;
}

static void update_input(void)
{
}

static void check_variables(void)
{
}

static void audio_callback(void)
{
	for (unsigned i = 0; i < 30000 / 60; i++, phase++)
	{
		int16_t val = 0x800 * sinf(2.0f * M_PI * phase * 300.0f / 30000.0f);
		audio_cb(val, val);
	}
	phase %= 100;
}

static void audio_set_state(bool enable)
{
	(void)enable;
}

void retro_run(void)
{
	update_input();

	bool updated = false;
	if (environ_cb(RETRO_ENVIRONMENT_GET_VARIABLE_UPDATE, &updated) && updated)
		check_variables();
	
	l2k_cpu->irq = 0;
	l2k_cpu->clk = !l2k_cpu->clk;
	if(!l2k_cpu->clk && l2k_cpu->ce) {
		uint16_t hi_addr = (l2k_cpu->addr >> 16) & 0xFFFF;
		uint16_t lo_addr = l2k_cpu->addr & 0xFFFF;
		// UART command
		if(l2k_cpu->addr == 0xF8000000) {
			l2k_cpu->data_in = 0;
			if(l2k_cpu->we) {
				log_cb(RETRO_LOG_INFO, "Write '%c'\n", (char)l2k_cpu->data_out);	
			}
		}
		// UART data
		if(l2k_cpu->addr == 0xF8000004) {
			l2k_cpu->data_in = 0xFFFF;
			if(l2k_cpu->we) {
				log_cb(RETRO_LOG_INFO, "Write '%c'\n", (char)l2k_cpu->data_out);	
			}
		}
		// ROM
		if(hi_addr == 0xFFFE) {
			l2k_cpu->data_in = l2k_rom[lo_addr / 4];
		}
		// NVRAM
		if(hi_addr == 0xF800) {
			l2k_cpu->data_in = l2k_nvram[lo_addr / 4];
		}
		// RAM
		if(hi_addr == 0x0000) {
			l2k_cpu->data_in = l2k_ram[lo_addr / 4];
			if(l2k_cpu->we) {
				l2k_ram[lo_addr / 4] = l2k_cpu->data_out;
			}
		}
		// EBUS
		if(hi_addr == 0xC000) {
			// Slot 1
			if(lo_addr == 0x0000) {
				l2k_cpu->data_in = 0x0C007CA1;
			} else if(lo_addr == 0x0004) {
				l2k_cpu->data_in = 0x4B494E36;
			}
		}
		l2k_cpu->rdy = 1;
	}
	l2k_cpu->eval();
	video_cb(framebuffer, VIDEO_WIDTH, VIDEO_HEIGHT, VIDEO_WIDTH * sizeof(short));
}

bool retro_load_game(const struct retro_game_info *info)
{
	struct retro_input_descriptor desc[] = {
		{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT, "Left"},
		{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP, "Up"},
		{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN, "Down"},
		{0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT, "Right"},
		{0},
	};
	environ_cb(RETRO_ENVIRONMENT_SET_INPUT_DESCRIPTORS, desc);

	enum retro_pixel_format fmt = RETRO_PIXEL_FORMAT_XRGB8888;
	if (!environ_cb(RETRO_ENVIRONMENT_SET_PIXEL_FORMAT, &fmt))
	{
		log_cb(RETRO_LOG_INFO, "XRGB8888 is not supported.\n");
		return false;
	}

	snprintf(retro_game_path, sizeof(retro_game_path), "%s", info->path);
	struct retro_audio_callback audio_cb = {audio_callback, audio_set_state};
	use_audio_cb = environ_cb(RETRO_ENVIRONMENT_SET_AUDIO_CALLBACK, &audio_cb);

	check_variables();

	(void)info;
	return true;
}

void retro_unload_game(void)
{
}

unsigned retro_get_region(void)
{
	return RETRO_REGION_PAL;
}

bool retro_load_game_special(unsigned type, const struct retro_game_info *info, size_t num)
{
	return false;
}

size_t retro_serialize_size(void)
{
	return sizeof(l2k_rom) + sizeof(l2k_ram) + sizeof(l2k_nvram);
}

bool retro_serialize(void *data_, size_t size)
{
	if(size < retro_serialize_size())
		return false;
	
	char *data = (char *)data_;
	memcpy(data, l2k_rom, sizeof(l2k_rom));
	data += sizeof(l2k_rom);
	memcpy(data, l2k_nvram, sizeof(l2k_nvram));
	data += sizeof(l2k_nvram);
	memcpy(data, l2k_ram, sizeof(l2k_ram));
	return true;
}

bool retro_unserialize(const void *data_, size_t size)
{
	if(size < retro_serialize_size())
		return false;
	
	const char *data = (const char *)data_;
	memcpy(l2k_rom, data, sizeof(l2k_rom));
	data += sizeof(l2k_rom);
	memcpy(l2k_nvram, data, sizeof(l2k_nvram));
	data += sizeof(l2k_nvram);
	memcpy(l2k_ram, data, sizeof(l2k_ram));
	return true;
}

void *retro_get_memory_data(unsigned id)
{
	(void)id;
	return NULL;
}

size_t retro_get_memory_size(unsigned id)
{
	(void)id;
	return 0;
}

void retro_cheat_reset(void)
{
}

void retro_cheat_set(unsigned index, bool enabled, const char *code)
{
	(void)index;
	(void)enabled;
	(void)code;
}
