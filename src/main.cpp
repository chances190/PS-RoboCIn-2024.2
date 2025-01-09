#include "mbed.h"

// I2C pins (adjust according to your Nucleo board)
I2C i2c(PB_9, PB_8); // SDA, SCL

// MPU-6050 I2C address (AD0 <- 0)
#define MPU6050_ADDRESS 0b1101000 

// MPU-6050 Registers
#define PWR_MGMT_1 107
#define GYRO_CONFIG 27
#define GYRO_XOUT_H 67
#define GYRO_OUT_LEN 6

int writeRegister(uint8_t reg, uint8_t value) {
    char data[2] = {static_cast<char>(reg), static_cast<char>(value)};
    
    int ret = i2c.write(MPU6050_ADDRESS, data, 2);
    if (ret != 0) {
        printf("Failed to write to register 0x%02X\n", reg);
        return ret;
    }

    return 0;
}

int readRegister(uint8_t reg, char *data, int length) {
    char reg_addr = static_cast<char>(reg);

    int ret = i2c.write(MPU6050_ADDRESS, &reg_addr, 1, true);
    if (ret != 0) {
        printf("Failed to set register address 0x%02X\n", reg);
    }

    ret = i2c.read(MPU6050_ADDRESS, data, length);
    if (ret != 0) {
        printf("Failed to read from register 0x%02X\n", reg);
    }

    return 0;
}

// Initialize the MPU-6000
void initializeMPU6050() {
    // Reset sensor
    writeRegister(PWR_MGMT_1, 0b10000000);
    ThisThread::sleep_for(500ms);

    // Wake up from sleep mode
    writeRegister(PWR_MGMT_1, 0b00000000);

    // Configure gyroscope (±250°/s full scale range)
    writeRegister(GYRO_CONFIG, 0b00000000);
}

struct Accel_raw {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Accel_dps {
    float x;
    float y;
    float z;
};

// Read raw gyroscope data
void readGyroData(Accel_raw &out) {
    char readData[6];
    readRegister(GYRO_XOUT_H, readData, 6); // 3x 16-bit signed values
    
    // Combine high and low bytes to form 16-bit values for X, Y, Z axes
    out.x = (readData[0] << 8) | readData[1];
    out.y = (readData[2] << 8) | readData[3];
    out.z = (readData[4] << 8) | readData[5];
}

// Process raw gyroscope data to convert to degrees per second (dps)
void processGyroData(const Accel_raw &in, Accel_dps &out) {
    const float scale_factor = 131.0f;  // 131 LSB/°/s for ±250°/s range
    
    out.x = in.x / scale_factor;
    out.y = in.y / scale_factor;
    out.z = in.z / scale_factor;
}

int main() {
    printf("Initializing MPU-6050...\n");
    initializeMPU6050();
    printf("MPU-6050 initialized!\n");

    Accel_raw raw;
    Accel_dps accel;

    while (true) {
        readGyroData(raw);
        processGyroData(raw, accel);

        printf("Gyro X: %.2f dps, Gyro Y: %.2f dps, Gyro Z: %.2f dps\n", 
                accel.x, accel.y, accel.z);

        ThisThread::sleep_for(500ms);
    }
}
