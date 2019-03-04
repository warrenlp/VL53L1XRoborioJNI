package frc.robot.vl53l1x;

import frc.robot.Robot;

public class VL53L1X_JNI_I2C implements Runnable {
    static {
       System.loadLibrary("vl53l1x_i2c");
    }

    private native boolean[] sensorInit();

    private native int[] getSensorId();

    private native void i2CSetIntermeasurementPeriod(int period);

    // private VL53L1X_JNI_I2C vl53l1x_jni_i2c;

    @Override
    public void run() {
        System.out.println("Inside of run:");
        boolean[] sensorInitResults = Robot.vl53l1x_jni_i2c.sensorInit();

        for (boolean sensorInitResult: sensorInitResults) {
            String extra_success = sensorInitResult ? " NOT" : "";
            System.out.println(String.format("INFO: JAVA: Sensor Init was%s successful", extra_success));
        }

        int[] sensorIDs = Robot.vl53l1x_jni_i2c.getSensorId();
        for (int sensorID : sensorIDs) {
            System.out.println(String.format("INFO: JAVA: Sensor ID is: %#X", sensorID));
        }
    }
}