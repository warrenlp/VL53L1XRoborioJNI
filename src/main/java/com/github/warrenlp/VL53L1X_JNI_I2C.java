package com.github.warrenlp;

import java.util.concurrent.ArrayBlockingQueue;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.RobotController;

public class VL53L1X_JNI_I2C {

    static {
       System.loadLibrary("vl53l1x_i2c");
    }

    private Thread m_acquire_task;
    public static final ArrayBlockingQueue<int[]> distanceBuffer = new ArrayBlockingQueue<>(2);

    private class AcquireTask implements Runnable {
        private VL53L1X_JNI_I2C vl53l1x;
        private double m_expirationTime;
        private final int m_notifier = NotifierJNI.initializeNotifier();
        private final double m_period = .1;

        public AcquireTask(VL53L1X_JNI_I2C vl53l1x_jni) {
            this.vl53l1x = vl53l1x_jni;
        }

        @Override
        public void run() {
            System.out.println("Inside of run:");
            boolean[] sensorInitResults = vl53l1x.sensorInit();
    
            for (boolean sensorInitResult: sensorInitResults) {
                String extra_success = sensorInitResult ? " NOT" : "";
                System.out.println(String.format("INFO: JAVA: Sensor Init was%s successful", extra_success));
            }
    
            vl53l1x.setTimingBudgetInMs(40);
    
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
            vl53l1x.startRanging();
            int[] distances = vl53l1x.getDistance();
            vl53l1x.stopRanging();
    
            byte[] rangeStatuses = vl53l1x.getRangeStatus();
            boolean distanceStatusesOkay[] = {false, false};
            int index = 0;
            for (byte rangeStatus : rangeStatuses) {
                StringBuilder sb = new StringBuilder("INFO: Range Status: ");
                switch (rangeStatus)
                {
                    case 0:
                        sb.append("Good");
                        distanceStatusesOkay[index] = true;
                    break;
                    case 1:
                        sb.append("Signal fail");
                    break;
                    case 2:
                        sb.append("Sigma fail");
                    break;
                    case 7:
                        sb.append("Wrapped target fail");
                    break;
                    default:
                        sb.append("Unknown: ");
                        sb.append(rangeStatus);
                    break;
                }
                if (!distanceStatusesOkay[index]) {
                    System.out.println(sb.toString());
                }
                index++;
            }
            boolean reportDistance = false;
            for (boolean distanceStatusOkay : distanceStatusesOkay) {
                if (distanceStatusOkay) {
                    reportDistance = true;
                } else {
                    reportDistance = false;
                    break;
                }
            }
            if (reportDistance) {
                synchronized(distanceBuffer) {
                    if(distanceBuffer.remainingCapacity()==0) {
                        distanceBuffer.remove();
                    }
                    try {
                        distanceBuffer.put(distances);
                    } catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }
            }
        }
    
        private void updateAlarm(){
            NotifierJNI.updateNotifierAlarm(m_notifier, (long) (m_expirationTime * 1e6));
        }
    }

    public VL53L1X_JNI_I2C() {
        m_acquire_task = new Thread(new AcquireTask(this));
        m_acquire_task.setDaemon(true);
        m_acquire_task.start();
    }

    private native boolean[] sensorInit();

    private native int[] getSensorId();

    private native void startRanging();

    private native int[] getDistance();

    private native void stopRanging();

    private native byte[] getRangeStatus();

    private native void setIntermeasurementPeriod(int period);

    private native void setTimingBudgetInMs(int period);
}