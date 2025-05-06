package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Utility class for reading positional data from an Axon Servo Encoder. Axons are unique in that
 * they have a secondary, 1-pin wire representing the movement from absolute zero rotation.
 * <br><br>
 * Please call {@link #cleanUp()} after the object is no longer needed to avoid floating threads.
 */
public class AxonServoEncoder {
    private Thread thread;
    private final AnalogInput channel;
    private double initDelta; // The difference between starting pos and absolute 0
    private double pos = 0; // The difference between starting pos and now
    private static boolean warningMessageSet = false;
    public AxonServoEncoder(AnalogInput channel){
        this.channel = channel;
        initDelta = channel.getVoltage() / 3.3 * 360;

        thread = new Thread(() -> {
            double normalizedPosLastCycle = getNormalizedPosition();
            ElapsedTime timer = new ElapsedTime();
            while (!thread.isInterrupted()){
                if (timer.milliseconds() > 250 && !warningMessageSet){
                    /*
                    * @ 4.8v it takes 140ms to move 60deg. Because we can effectively track 180deg
                    * of rotation in a single loop, 250ms is a fair warning level.
                    * We want to warn because we could have lost rotation, offsetting automatic controls.
                    */
                    RobotLog.addGlobalWarningMessage("Warning: Servo encoder tracking loop has gone on for more than 250ms. Servo positional data risks being incorrect and affecting automatic controls. Please report this to a programmer.");
                    warningMessageSet = true;
                }
                timer.reset();

                double currentNormalizedPosition = getNormalizedPosition();
                // Determine the delta between the current position and the previous position
                double normalizedDelta = currentNormalizedPosition - normalizedPosLastCycle;

                double trueDelta = normalizedDelta;
                if(normalizedDelta > 180){ // underflow has occurred (i.e. 10 -> 350)
                    // 10 -> 350, 340 delta, -20 true delta
                    trueDelta -= 360;
                } else if (normalizedDelta < -180) { // overflow has occurred (i.e. 350 -> 10)
                    // 350 -> 10, -340 delta, 20 true delta
                    trueDelta += 360;
                }

                // Save the difference to a variable counting our overall rotation
                pos += trueDelta;
                normalizedPosLastCycle = currentNormalizedPosition;

                try {
                    Thread.sleep(50); // wait for a while for new data
                } catch (InterruptedException ignored) {}
            }
        });

        thread.start();
    }

    /**
     * Returns the rotational difference between the starting and current positions, normalized
     * between a variable range of 360 degrees (always limited between [-360, 360]).
     */
    public double getNormalizedPosition(){
        return (channel.getVoltage() / 3.3 * 360) - initDelta;
    }

    /**
     * Returns the rotational difference between the starting and current positions, in degrees.
     */
    public double getPosition(){
        return pos;
    }

    public double getInitDelta() {
        return initDelta;
    }

    public void cleanUp(){ thread.interrupt(); thread = null; }
}
