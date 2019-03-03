
#include "frc_robot_vl53l1x_VL53L1X_JNI_I2C.h"

#include "SparkFun_VL53L1X.h"
#include "frc/I2C.h"

#include <iostream>


SFEVL53L1X distanceSensor;

JNIEXPORT jboolean JNICALL Java_frc_robot_vl53l1x_VL53L1X_1JNI_1I2C_i2CBegin(JNIEnv *, jobject)
{   
    std::cout << "Inside of i2cBegin:" << std::endl;
    return distanceSensor.begin();
}

JNIEXPORT void JNICALL Java_frc_robot_vl53l1x_VL53L1X_1JNI_1I2C_vl53l2xI2CSetIntermeasurementPeriod(JNIEnv *, jobject, jint)
{

}