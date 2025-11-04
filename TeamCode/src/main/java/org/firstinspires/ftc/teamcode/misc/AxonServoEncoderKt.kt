package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog

/**
 * Utility class for reading positional data from an Axon Servo Encoder. Axons are unique in that
 * they have a secondary, 1-pin wire representing the movement from absolute zero rotation.
 *
 * Please call [cleanUp] after the object is no longer needed to avoid floating threads.
 */
class AxonServoEncoderKt(private val channel: AnalogInput) {
    /**
     * The rotational difference between starting position and absolute zero, in degrees.
     */
    var initDelta: Double = channel.voltage / 3.3 * 360
        private set
    /**
     * The rotational difference between the starting and current positions, in degrees.
     */
    var pos: Double = 0.0
        private set

    companion object {
        private var warningMessageSet = false
    }

    private var thread = Thread {
        var normalizedPosLastCycle = getNormalizedPosition()
        val timer = ElapsedTime()
        while (!Thread.currentThread().isInterrupted) {
            if (timer.milliseconds() > 250 && !warningMessageSet){
                /*
                * @ 4.8v it takes 140ms to move 60deg. Because we can effectively track 180deg
                * of rotation in a single loop, 250ms is a fair warning level.
                * We want to warn because we could have lost rotation, offsetting automatic controls.
                */
                RobotLog.addGlobalWarningMessage("Warning: Servo encoder tracking loop has gone on for more than 250ms. Servo positional data risks being incorrect and affecting automatic controls. Please report this to a programmer.")
                warningMessageSet = true
            }
            timer.reset()

            val currentNormalizedPosition = getNormalizedPosition()
            // Determine the delta between the current position and the previous position
            val normalizedDelta = currentNormalizedPosition - normalizedPosLastCycle

            var trueDelta = normalizedDelta
            if (normalizedDelta > 180){ // underflow has occurred (i.e. 10 -> 350)
                // 10 -> 350, 340 delta, -20 true delta
                trueDelta -= 360
            } else if (normalizedDelta < -180) {
                // 350 -> 10, -340 delta, 20 true delta
                trueDelta += 360
            }

            // Save the difference to a variable counting our overall rotation
            pos += trueDelta
            normalizedPosLastCycle = currentNormalizedPosition

            try {
                Thread.sleep(50)
            } catch (ignored: InterruptedException) {}
        }
    }.apply { start() }

    /**
     * Returns the rotational difference between the starting and current positions, normalized
     * between a variable range of 360 degrees (always limited between [-360, 360]).
     */
    fun getNormalizedPosition(): Double { return (channel.voltage / 3.3 * 360) - initDelta }

    fun cleanUp() = thread.interrupt()
}