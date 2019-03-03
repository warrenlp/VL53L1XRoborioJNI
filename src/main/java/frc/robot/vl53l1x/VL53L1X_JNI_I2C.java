package frc.robot.vl53l1x;

import frc.robot.Robot;

public class VL53L1X_JNI_I2C implements Runnable {
    static {
       System.loadLibrary("vl53l1x_i2c");
    }

    private native boolean i2CBegin();

    private native void i2CSetIntermeasurementPeriod(int period);

    // private VL53L1X_JNI_I2C vl53l1x_jni_i2c;

    @Override
    public void run() {
        System.out.println("Inside of run:");
        // vl53l1x_jni_i2c = new VL53L1X_JNI_I2C();
        boolean result = Robot.vl53l1x_jni_i2c.i2CBegin();
        if (result) {
            System.out.println("Result was true");
        } else {
            System.out.println("Result was false");
        }
    }

}