#include "mbed.h"
#include "register_map.h"

// I2C pins (adjust according to your Nucleo board)
I2C i2c(PB_9, PB_8); // SDA, SCL


int writeRegister(uint8_t reg, uint8_t value) {
    char data[2] = {static_cast<char>(reg), static_cast<char>(value)};
    
    int ret = i2c.write(MPU6050_I2C_ADDRESS, data, 2);
    if (ret != 0) {
        printf("Failed to write to register 0x%02X\n", reg);
        return ret;
    }

    return 0;
}

int readData(uint8_t reg, char *data, size_t length) {
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

struct Gyro_raw {
    int16_t x;
    int16_t y;
    int16_t z;
};

// Read raw gyroscope data
void readGyroData(Gyro_raw &out) {
    char data[6]; // 3x 16-bit signed values
    readData(GYRO_XOUT_H_REG, data, sizeof(data)); 
    
    // Combine high and low bytes to form 16-bit values for X, Y, Z axes
    out.x = static_cast<int16_t>((data[0] << 8) | data[1]);
    out.y = static_cast<int16_t>((data[2] << 8) | data[3]);
    out.z = static_cast<int16_t>((data[4] << 8) | data[5]);
}

void readGTestData(uint8_t (&g_test)[3], Gyro_raw &gyro_data){
    writeRegister(GYRO_CONFIG_REG, G_ST_ON); // Enable self-test
    thread_sleep_for(100); // Allow sensor to stabilize in self-test mode
    char self_test[3];
    readData(SELF_TEST_X_REG, self_test, sizeof(self_test)); // Read SELF_TEST_XYZ data

    // Bitmask SELF_TEST_XYZ[4:0] to get XYZG_TEST
    g_test[0] = static_cast<uint8_t>(self_test[0]) & 0b00011111;
    g_test[1] = static_cast<uint8_t>(self_test[1]) & 0b00011111;
    g_test[2] = static_cast<uint8_t>(self_test[2]) & 0b00011111;
    
    readGyroData(gyro_data); // Read Gyroscope output data with Self-Test ON
    writeRegister(GYRO_CONFIG_REG, G_ST_OFF); // Disable self-test
}

// Helper function to calculate factory trim
float calcFactoryTrim(uint8_t g_test) {
    if (g_test == 0) {
        return 0.0f;
    }
    return 25.0f * 131.0f * pow(1.046f, (float) g_test - 1.0f);
}

// Perform self-test
bool selfTest() {
    writeRegister(GYRO_CONFIG_REG, G_ST_OFF); // Disable self-test
    uint8_t g_test[3];
    Gyro_raw data_st_on;
    readGTestData(g_test, data_st_on);

    Gyro_raw data_st_off;
    readGyroData(data_st_off);

    // Calculate factory trims
    float x_ft = calcFactoryTrim(g_test[0]);
    float y_ft = -calcFactoryTrim(g_test[1]);
    float z_ft = calcFactoryTrim(g_test[2]);

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
    readData(WHO_AM_I_REG, &address, 1);
    if (address != MPU6050_I2C_ADDRESS){
        printf("Failed to connect to the sensor. Stopping program.\n");
        exit(1);
    }

    writeRegister(PWR_MGMT_1_REG, DEVICE_RESET); // Reset sensor
    thread_sleep_for(100);
    writeRegister(PWR_MGMT_1_REG, CLKSEL_PLL_GYROX_REF); // Wake up from sleep mode and sync clock with Gyroscope X-axis (for stability) 

    printf("Performing Self-Test...\n");
    if (selfTest()) {
        printf("Self-Test passed!");
    } else {
        printf("MPU-6050 failed Self-Test. Stopping program.\n");
        exit(1);
    }

    writeRegister(GYRO_CONFIG_REG, FS_SEL_250dps); // Configure gyroscope (±250°/s full scale range)
    writeRegister(CONFIG_REG, DLPF_CFG_44_42); // Enable Digital Low-Pass Filter (DLPF)
    return 0;
}

struct Gyro_rad_s {
    float x;
    float y;
    float z;
};

inline float degToRad(float deg){
    return deg * (M_PI / 180.0f);
}

// Process raw gyroscope data to convert to rad/s
void processGyroData(const Gyro_raw &in, const int fs_range, const Gyro_raw offsets, Gyro_rad_s &out) {
    float lsb_sensitivity;
    switch (fs_range) {
        case 250:  lsb_sensitivity = 131.0f; break;
        case 500:  lsb_sensitivity = 65.5f;  break;
        case 1000: lsb_sensitivity = 32.8f;  break;
        case 2000: lsb_sensitivity = 16.4f;  break;
        default:   lsb_sensitivity = 0.0f;   break; // Invalid range will divide by 0
    }
    
    // Convert raw data to dps (LSB * 1/LSB/°/s = °/s)
    out.x = degToRad((in.x - offsets.x) / lsb_sensitivity);
    out.y = degToRad((in.y - offsets.y) / lsb_sensitivity);
    out.z = degToRad((in.z - offsets.z) / lsb_sensitivity);
}

void calibrate(Gyro_raw &offsets) {
    int32_t sum[3] = {0, 0, 0};

    const int sampleCount = 1000;
    for (int i = 100; i < sampleCount+100; i++) { // Discard initial values
        Gyro_raw data;
        readGyroData(data);

        sum[0] += data.x;
        sum[1] += data.y;
        sum[2] += data.z;

        thread_sleep_for(10); // Avoid repeated samples
    }

    // Take the average for offsets
    offsets.x = sum[0] / sampleCount;
    offsets.y = sum[1] / sampleCount;
    offsets.z = sum[2] / sampleCount;
}

int main() {
    printf("Initializing MPU-6050...\n");
    initialize();
    printf("MPU-6050 initialized!\n");

    printf("Calibrating...\n");
    Gyro_raw offsets;
    calibrate(offsets);
    printf("MPU-6050 calibrated!\nOffsets: x=%d, y=%d, z=%d\n", offsets.x, offsets.y, offsets.z);

    Gyro_raw raw;
    Gyro_rad_s data;
    while (true) {
        readGyroData(raw);
        processGyroData(raw, 250, offsets, data);

        printf("Gyro (rad/s): x=%.2f dps, y=%.2f dps, z=%.2f\n", data.x, data.y, data.z);
        thread_sleep_for(500); // Delay for readability
    }
}
