package frc.robot.vl53l1x;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class VL53L1X_JNI_I2C implements Runnable {
    static {
       System.loadLibrary("vl53l1x_i2c");
    }

    private native boolean[] sensorInit();

    private native int[] getSensorId();

    private native void startRanging();

    private native int[] getDistance();

    private native void stopRanging();

    private native void i2CSetIntermeasurementPeriod(int period);

    private double m_expirationTime;
    private final int m_notifier = NotifierJNI.initializeNotifier();
    private final double m_period = .5;

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

        m_expirationTime = RobotController.getFPGATime() * 1e-6 + m_period;
        updateAlarm();

        while (true) {
            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
            if (curTime == 0) {
                break;
            }

            m_expirationTime += m_period;
            updateAlarm();
            
            loopfunc();
        }
    }

    private void loopfunc() {
        Robot.vl53l1x_jni_i2c.startRanging();
        int[] distances = Robot.vl53l1x_jni_i2c.getDistance();
        Robot.vl53l1x_jni_i2c.stopRanging();

        for (int distance : distances) {
            System.out.println(String.format("INFO: Distance: %03d", distance));
        }
    }

    private void updateAlarm(){
        NotifierJNI.updateNotifierAlarm(m_notifier, (long) (m_expirationTime * 1e6));
    }
}