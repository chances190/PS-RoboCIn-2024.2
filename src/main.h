#pragma once

#include <cstddef>
#include <cstdint>

struct gyro_raw {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct gyro_rad_s {
	float x;
	float y;
	float z;
};

struct gyro_angle {
	float x;
	float y;
	float z;
};

// I2C
int write_register(uint8_t reg, uint8_t value);
int read_data(uint8_t reg, char *data, size_t length);

// Calibração
int initialize();
bool self_test();
void read_g_test_data(uint8_t (&g_tests)[3], gyro_raw &gyro_data);
float calc_factory_trim(uint8_t g_test);
void calibrate(gyro_raw &offsets);

// Dados
void read_gyro_data(gyro_raw &out);
int readFIFO(gyro_raw *out);
void process_gyro_data(const gyro_raw &in, const int fs_range, const gyro_raw offsets, gyro_rad_s &out);
void integrate_gyro_data(const gyro_rad_s &in, const float dt, gyro_angle &out);