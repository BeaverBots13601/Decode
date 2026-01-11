package org.firstinspires.ftc.teamcode.misc

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Utility class for reading positional data from an Axon Servo Encoder. Axons are unique in that
 * they have a secondary, 1-pin wire representing the movement from absolute zero rotation.
 *
 * Please call [cleanUp()][cleanUp] after the object is no longer needed to avoid floating threads.
 * @param axonServoName The name of the Axon servo. The servo must be configured in CR mode.
 * @param gearRatio The gear ratio of the gears the axon is connected to. i.e. after rotating the
 * mechanism by 90deg, the detected rotation divided by this number should be 90.
 */
@Config
class AxonDriver(
    hardwareMap: HardwareMap,
    private val axonServoName: String,
    axonEncoderName: String,
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    private val telemetry: Telemetry,
    private val gearRatio: Double = 1.0,
) {
    private val axonServo = hardwareMap.crservo.get(axonServoName).apply { power = 0.0 }
    private val channel = hardwareMap.analogInput.get(axonEncoderName)

    fun start() {
        encoderTimer.reset()
    }

    fun update() {
        updateEncoder()
        updatePID()
    }

    private var thread = Thread {
        while (!Thread.currentThread().isInterrupted) {
            update()
        }
    }//.apply { start() }

    // region Encoder tracking
    var initDelta: Double = run {
        Thread.sleep(1000) // Have to do this to let the axon initialize and send a signal
        channel.voltage / 3.3 * 360
    }
        private set

    /**
     * The rotational difference between the starting and current positions, in degrees.
     *
     * Equivalent to [DcMotor.getCurrentPosition()][com.qualcomm.robotcore.hardware.DcMotor.getCurrentPosition]
     */
    var position: Double = 0.0 // initializer not quite working
        private set

    var normalizedPosLastCycle = internalNormalizedPosition
    val encoderTimer = ElapsedTime()
    private fun updateEncoder() {
        if (encoderTimer.milliseconds() > 250 && !warningMessageSet){
            /*
            * @ 4.8v it takes 140ms to move 60deg. Because we can effectively track 180deg
            * of rotation in a single loop, 250ms is a fair warning level.
            * We want to warn because we could have lost rotation, offsetting automatic controls.
            */
            RobotLog.addGlobalWarningMessage("Warning: Servo encoder tracking loop has gone on for more than 250ms. Servo positional data risks being incorrect and affecting automatic controls. Please report this to a programmer.")
            warningMessageSet = true
        }
        encoderTimer.reset()

        val currentNormalizedPosition = internalNormalizedPosition
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
        position += trueDelta / gearRatio
        normalizedPosLastCycle = currentNormalizedPosition
    }

    /**
     * The rotational difference between the starting and current positions, normalized between [-180, 180)
     */
    private val internalNormalizedPosition: Double
        get() {
            // Get the absolute rotation (pos relative to 0deg as of last reset) (anywhere between [-360, 360])
            val absolutePos = (channel.voltage / 3.3 * 360) - initDelta

            // I don't even know ChatGPT did this
            val a = (((absolutePos % 360) + 360) % 360) // ensure the result is in [-180, 180)
            return PoseKt.normalizeAngleDeg(a)
        }

    val normalizedPosition: Double
        get() {
            return PoseKt.normalizeAngleDeg(position)
        }

    val rawPosition: Double
        get() = channel.voltage / 3.3 * 360

    /**
     * Resets the encoder such that we have moved 0° and are at 0° relative
     */
    fun reset() {
        initDelta = channel.voltage / 3.3 * 360
        position = 0.0
        targetPosition = null
        overridePower = null
        warningMessageSet = false
        RobotLog.clearGlobalWarningMsg()
    }
    // endregion

    // region Control loop
    private var integralSum = 0.0
    private var lastError = 0.0
    private var pastTargetPosition: Double? = null
    private val timer = ElapsedTime()

    private fun updatePID() {
        val overridePower = overridePower // get a local copy to avoid concurrency issues
        if (overridePower != null) {
            axonServo.power = overridePower
            return
        }

        if (targetPosition != pastTargetPosition) {
            timer.reset()
            pastTargetPosition = targetPosition
            lastError = 0.0
            integralSum = 0.0
        }

        // have to get a local copy to avoid concurrency issues
        val targetPosition = targetPosition

        if (targetPosition == null) {
            axonServo.power = 0.0
            return
        }

        val position = this.position
        val error = targetPosition - position

        integralSum += (error * timer.seconds()) // sum of all error over time
        val derivative = (error - lastError) / timer.seconds() // rate of change of error

        val out = if (USE_TUNABLE_VALUES) {
            (Companion.kP * error) + (Companion.kI * integralSum) + (Companion.kD * derivative)
        } else {
            (kP * error) + (kI * integralSum) + (kD * derivative)
        }

        axonServo.power = out

        lastError = error
        this.error = error // error for outward facing

        // Divide by gearRatio so user expected values match up
        telemetry.addData("($axonServoName) PID Position", position / gearRatio)
        telemetry.addData("($axonServoName) PID Normalized Position", normalizedPosition)
        telemetry.addData("($axonServoName) PID Target", targetPosition / gearRatio)
        telemetry.addData("($axonServoName) PID Error", error / gearRatio)
        telemetry.addData("($axonServoName) PID Power", out)

        timer.reset()
    }

    /**
     * The target position of the axon.
     *
     * Set to null to disable control and set the velocity to 0.
     */
    var targetPosition: Double? = null
        set(it) {
            field = it?.times(gearRatio)
        }

    var error: Double = 0.0
        private set

    /**
     * The power the axon will be set to. This will disable the standard control loop.
     *
     * Set to null to disable override control.
     */
    var overridePower: Double? = null
        set(it) {
            field = it
            if (it != null) axonServo.power = it
        }
    // endregion

    fun cleanUp() = thread.interrupt()

    private companion object {
        // Encoder
        private var warningMessageSet = false

        // Control loop
        @JvmField
        var USE_TUNABLE_VALUES = false
        @JvmField
        var kP = 0.0
        @JvmField
        var kI = 0.0
        @JvmField
        var kD = 0.0
    }
}