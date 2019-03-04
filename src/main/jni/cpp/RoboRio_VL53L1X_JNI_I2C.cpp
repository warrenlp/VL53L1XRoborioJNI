
#include "frc_robot_vl53l1x_VL53L1X_JNI_I2C.h"

#include "vl53l1x_class.h"
#include "frc/I2C.h"

#include <iostream>
#include <array>

// std::shared_ptr<frc::I2C> i2c_ptr = std::make_shared<frc::I2C>(frc::I2C::Port::kOnboard, VL53L1X_DEFAULT_DEVICE_ADDRESS);

#define NUM_SENSORS 1

// Split into array for multiple sensors
VL53L1X vl53l1x_list[] = {VL53L1X(std::make_shared<frc::I2C>(frc::I2C::Port::kOnboard, VL53L1X_DEFAULT_DEVICE_ADDRESS), 1)};

JNIEXPORT jbooleanArray JNICALL Java_frc_robot_vl53l1x_VL53L1X_1JNI_1I2C_sensorInit(JNIEnv *env, jobject thisObj)
{   
    std::cout << "INFO: C++ sensorInit:" << std::endl;

    uint8_t index = 0;
    jboolean *cResults = new jboolean[NUM_SENSORS];
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        index++;
        vl53l1x.VL53L1_On();
        int status = vl53l1x.VL53L1X_SetI2CAddress(VL53L1X_DEFAULT_DEVICE_ADDRESS + index);
        if (status == 0) {
            std::cout << "DEBUG: C++ SetI2CAddress successful:" << std::endl;
            // TODO: Actually initialize sensor
            // cResults[index - 1] = vl53l1x.VL53L1X_SensorInit();
            cResults[index - 1] = false;
        } else {
            std::cout << "DEBUG: C++ SetI2CAddress NOT successful:" << std::endl;
            cResults[index - 1] = true;
        }
    }
    
    jbooleanArray results = env->NewBooleanArray(NUM_SENSORS);
    env->SetBooleanArrayRegion(results, 0, NUM_SENSORS, cResults);

    delete cResults;
    return results;
}

JNIEXPORT jintArray JNICALL Java_frc_robot_vl53l1x_VL53L1X_1JNI_1I2C_getSensorId(JNIEnv *env, jobject thisObj)
{   
    uint16_t sensorID = 0;

    uint8_t index = 0;
    jint *cResults = new jint[NUM_SENSORS];
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_GetSensorId(&sensorID);
        cResults[index++] = sensorID;
    }

    jintArray results = env->NewIntArray(NUM_SENSORS);
    env->SetIntArrayRegion(results, 0, NUM_SENSORS, cResults);

    return results;
}

JNIEXPORT void JNICALL Java_frc_robot_vl53l1x_VL53L1X_1JNI_1I2C_vl53l2xI2CSetIntermeasurementPeriod(JNIEnv *, jobject, jint)
{

}