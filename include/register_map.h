/**
 * @author Crispin Mukalay
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * MPU-6050 triple-axis MEMS gyroscope and triple-axis MEMS accelerometer.
 *
 * Datasheet:
 *
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */

#ifndef MPU6050_H
#define MPU6050_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define MPU6050_I2C_ADDRESS 0x68 // 7-bit I2C address of MPU6050 (AD0 <= 0).

//-----------
// Registers
//-----------
#define SELF_TEST_X_REG        0x0D
#define SELF_TEST_Y_REG        0x0E
#define SELF_TEST_Z_REG        0x0F
#define SELF_TEST_A_REG        0x10
#define SMPLRT_DIV_REG         0x19
#define CONFIG_REG             0x1A
#define GYRO_CONFIG_REG        0x1B
#define ACCEL_CONFIG_REG       0x1C
#define FIFO_EN_REG            0x23
#define I2C_MST_CTRL_REG       0x24
#define I2C_SLV0_ADDR_REG      0x25
#define I2C_SLV0_REG           0x26
#define I2C_SLV0_CTRL_REG      0x27
#define I2C_SLV1_ADDR_REG      0x28
#define I2C_SLV1_REG           0x29
#define I2C_SLV1_CTRL_REG      0x2A
#define I2C_SLV2_ADDR_REG      0x2B
#define I2C_SLV2_REG           0x2C
#define I2C_SLV2_CTRL_REG      0x2D
#define I2C_SLV3_ADDR_REG      0x2E
#define I2C_SLV3_REG           0x2F
#define I2C_SLV3_CTRL_REG      0x30
#define I2C_SLV4_ADDR_REG      0x31
#define I2C_SLV4_REG           0x32
#define I2C_SLV4_DO_REG        0x33
#define I2C_SLV4_CTRL_REG      0x34
#define I2C_SLV4_DI_REG        0x35
#define I2C_MST_STATUS_REG     0x36
#define INT_PIN_CFG_REG        0x37
#define INT_ENABLE_REG         0x38
#define INT_STATUS_REG         0x3A
#define ACCEL_XOUT_H_REG       0x3B
#define ACCEL_XOUT_L_REG       0x3C
#define ACCEL_YOUT_H_REG       0x3D
#define ACCEL_YOUT_L_REG       0x3E
#define ACCEL_ZOUT_H_REG       0x3F
#define ACCEL_ZOUT_L_REG       0x40
#define TEMP_OUT_H_REG         0x41
#define TEMP_OUT_L_REG         0x42
#define GYRO_XOUT_H_REG        0x43
#define GYRO_XOUT_L_REG        0x44
#define GYRO_YOUT_H_REG        0x45
#define GYRO_YOUT_L_REG        0x46
#define GYRO_ZOUT_H_REG        0x47
#define GYRO_ZOUT_L_REG        0x48
#define EXT_SENS_DATA_00_REG   0x49
#define EXT_SENS_DATA_01_REG   0x4A
#define EXT_SENS_DATA_02_REG   0x4B
#define EXT_SENS_DATA_03_REG   0x4C
#define EXT_SENS_DATA_04_REG   0x4D
#define EXT_SENS_DATA_05_REG   0x4E
#define EXT_SENS_DATA_06_REG   0x4F
#define EXT_SENS_DATA_07_REG   0x50
#define EXT_SENS_DATA_08_REG   0x51
#define EXT_SENS_DATA_09_REG   0x52
#define EXT_SENS_DATA_10_REG   0x53
#define EXT_SENS_DATA_11_REG   0x54
#define EXT_SENS_DATA_12_REG   0x55
#define EXT_SENS_DATA_13_REG   0x56
#define EXT_SENS_DATA_14_REG   0x57
#define EXT_SENS_DATA_15_REG   0x58
#define EXT_SENS_DATA_16_REG   0x59
#define EXT_SENS_DATA_17_REG   0x5A
#define EXT_SENS_DATA_18_REG   0x5B
#define EXT_SENS_DATA_19_REG   0x5C
#define EXT_SENS_DATA_20_REG   0x5D
#define EXT_SENS_DATA_21_REG   0x5E
#define EXT_SENS_DATA_22_REG   0x5F
#define EXT_SENS_DATA_23_REG   0x60
#define I2C_SLV0_DO_REG        0x63
#define I2C_SLV1_DO_REG        0x64
#define I2C_SLV2_DO_REG        0x65
#define I2C_SLV3_DO_REG        0x66
#define I2C_MST_DELAY_CTRL_REG 0x67
#define SIGNAL_PATH_RESET_REG  0x68
#define USER_CTRL_REG          0x6A
#define PWR_MGMT_1_REG         0x6B
#define PWR_MGMT_2_REG         0x6C
#define FIFO_COUNTH_REG        0x72
#define FIFO_COUNTL_REG        0x73
#define FIFO_R_W_REG           0x74
#define WHO_AM_I_REG           0x75

//----------------------------------------------
// External Frame Synchronization Configuration
//----------------------------------------------
#define FSYNC_DISABLE      0x00
#define FSYNC_TEMP_OUT_L   0x01
#define FSYNC_GYRO_XOUT_L  0x02
#define FSYNC_GYRO_YOUT_L  0x03
#define FSYNC_GYRO_ZOUT_L  0x04
#define FSYNC_ACCEL_XOUT_L 0x05
#define FSYNC_ACCEL_YOUT_L 0x06
#define FSYNC_ACCEL_ZOUT_L 0x07

//---------------------------------------
// Digital Low Pass Filter Configuration
//---------------------------------------
#define DLPF_CFG_260_256 0x00
#define DLPF_CFG_184_188 0x01
#define DLPF_CFG_94_98   0x02
#define DLPF_CFG_44_42   0x03
#define DLPF_CFG_21_20   0x04
#define DLPF_CFG_10_10   0x05
#define DLPF_CFG_5_5     0x06

//------------------------------------
// Gyroscope Configuration
//------------------------------------
#define FS_SEL_250dps  0x00
#define FS_SEL_500dps  0x08
#define FS_SEL_1000dps 0x10
#define FS_SEL_2000dps 0x18
#define G_ST_ON        0xE0
#define G_ST_OFF       0x00

//----------------------------------------------
// Sensor Measurements To Load In FIFO Register
//----------------------------------------------
#define TEMP_FIFO_EN  0x80
#define GYRO_FIFO_EN  0x70
#define ACCEL_FIFO_EN 0x08
#define SLV2_FIFO_EN  0x04
#define SLV1_FIFO_EN  0x02
#define SLV0_FIFO_EN  0x01

//---------------------------
// Signal Path Reset Values
//---------------------------
#define GYRO_RESET  0x04
#define ACCEL_RESET 0x02
#define TEMP_RESET  0x01
#define ALL_RESET   0x07

//-------------------------------
// User Control settings
//-------------------------------
#define FIFO_EN        0x40
#define FIFO_DIS       0x00
#define I2C_MST_EN     0x60
#define FIFO_RESET     0x04
#define I2C_MST_RESET  0x02
#define SIG_COND_RESET 0x41

//---------------------------------
// Power Management Setings One
//---------------------------------
#define DEVICE_RESET             0x80
#define SLEEP                    0x40
#define CYCLE                    0x20
#define TEMP_DIS                 0x08
#define CLKSEL_INT_8MHz_OSC      0x00
#define CLKSEL_PLL_GYROX_REF     0x01
#define CLKSEL_PLL_GYROY_REF     0x02
#define CLKSEL_PLL_GYROZ_REF     0x03
#define CLKSEL_PLL_EXT_32KHz_REF 0x04
#define CLKSEL_PLL_EXT_19KHz_REF 0x05

//---------------------------------
// Power Management Settings Two
//---------------------------------
#define LP_WAKE_CTRL_1Hz  0x00
#define LP_WAKE_CTRL_5Hz  0x40
#define LP_WAKE_CTRL_20Hz 0x80
#define LP_WAKE_CTRL_40Hz 0xC0
#define STBY_A            0x38
#define STBY_G            0x07

#endif