#include "mbed.h"
#include "register_map.h"

I2C i2c(PB_9, PB_8);  // SDA, SCL
InterruptIn fifo_interrupt_pin(D8);

int write_register(uint8_t reg, uint8_t value) {
	char data[2] = {static_cast<char>(reg), static_cast<char>(value)};

	int ret = i2c.write(MPU6050_I2C_ADDRESS, data, 2);
	if (ret != 0) {
		printf("Failed to write to register 0x%02X\n", reg);
		return ret;
	}

	return 0;
}

int read_data(uint8_t reg, char *data, size_t length) {
	char reg_addr = static_cast<char>(reg);

	int ack = i2c.write(MPU6050_I2C_ADDRESS, &reg_addr, 1, true);
	if (ack != 0) {
		printf("Failed to set register address 0x%02X\n", reg);
	}

	ack = i2c.read(MPU6050_I2C_ADDRESS, data, length);
	if (ack != 0) {
		printf("Failed to read from register 0x%02X\n", reg);
	}

	return 0;
}

struct gyro_raw {
	int16_t x;
	int16_t y;
	int16_t z;
};

// Read raw gyroscope data
void read_gyro_data(gyro_raw &out) {
	char data[6];  // 3x 16-bit signed values
	read_data(GYRO_XOUT_H_REG, data, sizeof(data));

	// Combine high and low bytes to form 16-bit values for X, Y, Z axes
	out.x = static_cast<int16_t>((data[0] << 8) | data[1]);
	out.y = static_cast<int16_t>((data[2] << 8) | data[3]);
	out.z = static_cast<int16_t>((data[4] << 8) | data[5]);
}

uint16_t read_fifo_count() {
	char fifo_count[2];
	read_data(FIFO_COUNTH_REG, fifo_count, 2);
	uint16_t fifo_count_h = static_cast<uint8_t>(fifo_count[0]);
	uint16_t fifo_count_l = static_cast<uint8_t>(fifo_count[1]);

	return (fifo_count_h << 8) | fifo_count_l;
}

int read_fifo(gyro_raw *out) {
	char buffer[1024];
	uint16_t count = read_fifo_count();
	int ret = read_data(FIFO_R_W_REG, buffer, count);
	if (ret != 0) {
		printf("Failed to read from FIFO\n");
		return -1;
	}

	int sample = 0;
	for (int i = 0; i < count; i += 6) {
		out[sample].x = static_cast<int16_t>((buffer[i] << 8) | buffer[i + 1]);
		out[sample].y = static_cast<int16_t>((buffer[i + 2] << 8) | buffer[i + 3]);
		out[sample].z = static_cast<int16_t>((buffer[i + 4] << 8) | buffer[i + 5]);
		sample++;
	}

	return count / sizeof(gyro_raw);
}

void read_g_test_data(uint8_t (&g_test)[3], gyro_raw &gyro_data) {
	write_register(GYRO_CONFIG_REG, G_ST_ON);  // Enable self-test
	thread_sleep_for(100);  // Allow sensor to stabilize in self-test mode
	char self_test[3];
	read_data(SELF_TEST_X_REG, self_test, sizeof(self_test));  // Read SELF_TEST_XYZ data

	// Bitmask SELF_TEST_XYZ[4:0] to get XYZG_TEST
	g_test[0] = static_cast<uint8_t>(self_test[0]) & 0b00011111;
	g_test[1] = static_cast<uint8_t>(self_test[1]) & 0b00011111;
	g_test[2] = static_cast<uint8_t>(self_test[2]) & 0b00011111;

	read_gyro_data(gyro_data);  // Read Gyroscope output data with Self-Test ON
	write_register(GYRO_CONFIG_REG, G_ST_OFF);  // Disable self-test
}

// Helper function to calculate factory trim
float calc_factory_trim(uint8_t g_test) {
	if (g_test == 0) {
		return 0.0f;
	}
	return 25.0f * 131.0f * pow(1.046f, (float) g_test - 1.0f);
}

// Perform self-test
bool self_test() {
	write_register(GYRO_CONFIG_REG, FS_SEL_250dps);  // Set sensitivity to base value

	// Collect data with Self-Test ON
	uint8_t g_test[3];
	gyro_raw data_st_on;
	read_g_test_data(g_test, data_st_on);

	// Collect data with Self-Test OFF
	gyro_raw data_st_off;
	read_gyro_data(data_st_off);

	// Calculate factory trims
	float x_ft = calc_factory_trim(g_test[0]);
	float y_ft = -calc_factory_trim(g_test[1]);
	float z_ft = calc_factory_trim(g_test[2]);

	// Calculate self-test response
	float x_str = (data_st_on.x - data_st_off.x);
	float y_str = (data_st_on.y - data_st_off.y);
	float z_str = (data_st_on.z - data_st_off.z);

	// Calculate percentage changes
	float x_change = (x_str - x_ft) / x_ft;
	float y_change = (y_str - y_ft) / y_ft;
	float z_change = (z_str - z_ft) / z_ft;

	// Check if within tolerance
	bool x_pass = fabs(x_change) <= 0.14f;
	bool y_pass = fabs(y_change) <= 0.14f;
	bool z_pass = fabs(z_change) <= 0.14f;

	// Print diagnostics
	printf("X-Axis: STR=%.2f, FT=%.2f, Change=%.2f%%, Pass=%s\n", x_str, x_ft, x_change * 100, x_pass ? "YES" : "NO");
	printf("Y-Axis: STR=%.2f, FT=%.2f, Change=%.2f%%, Pass=%s\n", y_str, y_ft, y_change * 100, y_pass ? "YES" : "NO");
	printf("Z-Axis: STR=%.2f, FT=%.2f, Change=%.2f%%, Pass=%s\n", z_str, z_ft, z_change * 100, z_pass ? "YES" : "NO");

	return x_pass && y_pass && z_pass;
}

// Initialize the MPU-6000 with ±250°/s fs range and 42hz DLPF
int initialize() {
	char address = '-';
	read_data(WHO_AM_I_REG, &address, 1);
	if (address != MPU6050_I2C_ADDRESS) {
		printf("Failed to connect to the sensor. Stopping program.\n");
		exit(1);
	}

	write_register(PWR_MGMT_1_REG, DEVICE_RESET);  // Reset sensor
	write_register(PWR_MGMT_1_REG, 0x00);  // Wake up from sleep
	thread_sleep_for(100);

	printf("Performing Self-Test...\n");
	if (self_test()) {
		printf("Self-Test passed!");
	} else {
		printf("MPU-6050 failed Self-Test. Stopping program.\n");
		exit(1);
	}

	write_register(PWR_MGMT_1_REG, CLKSEL_PLL_GYROX_REF);  // Sync clock with Gyroscope X-axis (for stability)
	write_register(GYRO_CONFIG_REG, FS_SEL_500dps);  // Configure gyroscope (±500°/s full scale range)
	write_register(SMPLRT_DIV_REG, 39);  // Set sample rate at 200Hz (8kHz/(1+39))
	write_register(CONFIG_REG, DLPF_CFG_44_42);  // Enable Digital Low-Pass Filter (DLPF)

	write_register(FIFO_EN, G_FIFO_EN);  // Enable FIFO buffer
	write_register(INT_ENABLE_REG, FIFO_OFLOW_EN);  // Enable interrupts for FIFO buffer overflow
	return 0;
}

struct gyro_rad_s {
	float x;
	float y;
	float z;
};

inline float deg_to_rad(float deg) {
	return deg * (M_PI / 180.0f);
}

// Process raw gyroscope data to convert to rad/s
void process_gyro_data(const gyro_raw &in, const int fs_range, const gyro_raw offsets, gyro_rad_s &out) {
	float lsb_sensitivity;
	switch (fs_range) {
		case 250:
			lsb_sensitivity = 131.0f;
			break;
		case 500:
			lsb_sensitivity = 65.5f;
			break;
		case 1000:
			lsb_sensitivity = 32.8f;
			break;
		case 2000:
			lsb_sensitivity = 16.4f;
			break;
		default:
			lsb_sensitivity = 0.0f;
			break;  // Invalid range will divide by 0
	}

	// Convert raw data to dps (LSB * 1/LSB/°/s = °/s)
	out.x = deg_to_rad((in.x - offsets.x) / lsb_sensitivity);
	out.y = deg_to_rad((in.y - offsets.y) / lsb_sensitivity);
	out.z = deg_to_rad((in.z - offsets.z) / lsb_sensitivity);
}

struct gyro_angle {
	float x;
	float y;
	float z;
};

// Integrate gyroscope data to find the angle
void integrate_gyro_data(const gyro_rad_s &gyro_data, const float dt, gyro_angle &angle) {
	angle.x += gyro_data.x * dt;
	angle.y += gyro_data.y * dt;
	angle.z += gyro_data.z * dt;
}

void calibrate(gyro_raw &offsets) {
	int32_t sum[3] = {0, 0, 0};

	const int sample_count = 1000;
	for (int i = 100; i < sample_count + 100; i++) {  // Discard initial values
		gyro_raw data;
		read_gyro_data(data);

		sum[0] += data.x;
		sum[1] += data.y;
		sum[2] += data.z;

		thread_sleep_for(10);  // Avoid repeated samples
	}

	// Take the average for offsets
	offsets.x = sum[0] / sample_count;
	offsets.y = sum[1] / sample_count;
	offsets.z = sum[2] / sample_count;
}

int main() {
	printf("Initializing MPU-6050...\n");
	initialize();
	printf("MPU-6050 initialized!\n");

	printf("Calibrating...\n");
	gyro_raw offsets;
	calibrate(offsets);
	printf("MPU-6050 calibrated!\nOffsets: x=%d, y=%d, z=%d\n", offsets.x, offsets.y, offsets.z);

	constexpr size_t buffer_size = 1024 / sizeof(gyro_raw);
	constexpr float sample_delay_s = 1 / 200.0f;
	gyro_raw raw[buffer_size];
	gyro_angle angle = {0.0f, 0.0f, 0.0f};

	gyro_rad_s data;
	while (true) {
		int samples = read_fifo(raw);
		for (int i = 0; i < samples; i++) {
			process_gyro_data(raw[i], 500, offsets, data);
			printf("Gyro (rad/s): x=%.2f dps, y=%.2f dps, z=%.2f\n", data.x, data.y, data.z);

			integrate_gyro_data(data, sample_delay_s, angle);
			printf("Angle (theta): x=%.2f rad, y=%.2f rad, z=%.2f rad\n", angle.x, angle.y, angle.z);
		}
	}
	// Wait for the FIFO to fill up again
	thread_sleep_for(buffer_size * static_cast<int>(sample_delay_s * 1000) - 100);
}
