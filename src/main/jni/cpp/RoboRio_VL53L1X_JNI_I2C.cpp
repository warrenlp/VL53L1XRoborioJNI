
#include "com_github_warrenlp_VL53L1X_JNI_I2C.h"

#include "vl53l1x_class.h"
#include "frc/I2C.h"

#include <iostream>
#include <array>

// std::shared_ptr<frc::I2C> i2c_ptr = std::make_shared<frc::I2C>(frc::I2C::Port::kOnboard, VL53L1X_DEFAULT_DEVICE_ADDRESS);

#define NUM_SENSORS 1

// Split into array for multiple sensors
VL53L1X vl53l1x_list[] = {VL53L1X(std::make_shared<frc::I2C>(frc::I2C::Port::kOnboard, VL53L1X_DEFAULT_DEVICE_ADDRESS), 1)};

JNIEXPORT jbooleanArray JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_sensorInit(JNIEnv *env, jobject thisObj)
{   
    std::cout << "INFO: C++ sensorInit:" << std::endl;

    uint8_t index = 0;
    jboolean *cResults = new jboolean[NUM_SENSORS];
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        index++;
        vl53l1x.VL53L1_On();

        int status = vl53l1x.VerifySensor(VL53L1X_DEFAULT_DEVICE_ADDRESS);

        if (status == 0) {
            std::cout << "INFO: C++ VerifySensor 1 successful:" << std::endl;
            status = vl53l1x.VL53L1X_SetI2CAddress(VL53L1X_DEFAULT_DEVICE_ADDRESS + index);
            if (status == 0) {
                std::cout << "INFO: C++ SetI2CAddress successful:" << std::endl;
            } else {
                std::cout << "INFO: C++ SetI2CAddress NOT successful:" << std::endl;
            }
        } else {
            std::cout << "INFO: C++ VerifySensor 1 NOT successful:" << std::endl;
            status = vl53l1x.AddressChangeThenVerifySensor(VL53L1X_DEFAULT_DEVICE_ADDRESS + index);
            if (status == 0) {
                std::cout << "INFO: C++ VerifySensor 2 successful:" << std::endl;
            } else {
                std::cout << "INFO: C++ VerifySensor 2 NOT successful:" << std::endl;
                
            }
        }

        if (status == 0) {
            status = vl53l1x.VL53L1X_SensorInit();
            if (status == 0) {
                std::cout << "INFO: C++ SensorInit successful:" << std::endl;
                cResults[index - 1] = false;
            } else {
                std::cout << "INFO: C++ SensorInit NOT successful:" << std::endl;
                cResults[index - 1] = true;
            }
        } else {
            cResults[index - 1] = true;
        }
    }
    
    jbooleanArray results = env->NewBooleanArray(NUM_SENSORS);
    env->SetBooleanArrayRegion(results, 0, NUM_SENSORS, cResults);

    delete cResults;
    return results;
}

JNIEXPORT jintArray JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_getSensorId(JNIEnv *env, jobject thisObj)
{   
    uint16_t sensorID = 0;

    uint8_t index = 0;
    jint cResults[NUM_SENSORS];
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_GetSensorId(&sensorID);
        cResults[index++] = sensorID;
    }

    jintArray results = env->NewIntArray(NUM_SENSORS);
    env->SetIntArrayRegion(results, 0, NUM_SENSORS, cResults);

    return results;
}

JNIEXPORT void JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_startRanging(JNIEnv *env, jobject thisObj)
{
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_StartRanging();
    }
}

JNIEXPORT jintArray JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_getDistance(JNIEnv * env, jobject thisObj)
{
    uint16_t distance = 0;
    jint distances[NUM_SENSORS];
    int i = 0;
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_GetDistance(&distance);
        distances[i++] = distance;
    }

    jintArray results = env->NewIntArray(NUM_SENSORS);
    env->SetIntArrayRegion(results, 0, NUM_SENSORS, distances);

    return results;
}

JNIEXPORT void JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_stopRanging(JNIEnv * env, jobject thisObj)
{
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_StopRanging();
    }
}

JNIEXPORT jbyteArray JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_getRangeStatus(JNIEnv *env, jobject thisObj)
{
    uint8_t rangeStatus;
    jbyte rangeStatuses[NUM_SENSORS];
    int i =0;
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_GetRangeStatus(&rangeStatus);
        rangeStatuses[i++] = rangeStatus;
    }

    jbyteArray results = env->NewByteArray(NUM_SENSORS);
    env->SetByteArrayRegion(results, 0, NUM_SENSORS, rangeStatuses);

    return results;
}

JNIEXPORT void JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_setIntermeasurementPeriod(JNIEnv *env, jobject thisObj, jint intermeasurement)
{
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_SetInterMeasurementInMs(intermeasurement);
    }
}

JNIEXPORT void JNICALL Java_com_github_warrenlp_VL53L1X_1JNI_1I2C_setTimingBudgetInMs(JNIEnv *env, jobject thisObj, jint TB)
  {
    for (VL53L1X& vl53l1x : vl53l1x_list) {
        vl53l1x.VL53L1X_SetTimingBudgetInMs(TB);
    }
  }
