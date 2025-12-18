package org.firstinspires.ftc.teamcode.misc

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

@Config
class PIDVelocityController(
    private val motor: DcMotorEx,
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    private val telemetry: Telemetry,
) {
    private var integralSum = 0.0
    private var lastError = 0.0
    private var pastTargetVelocity: Double? = -1.0
    private val timer = ElapsedTime()
    private val name = motor.deviceName

    init {
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * Update the motor
     */
    private fun loop(targetVelocity: Double?) {
        if (targetVelocity != pastTargetVelocity) {
            timer.reset()
            pastTargetVelocity = targetVelocity
            lastError = 0.0
            integralSum = 0.0
        }

        if (targetVelocity == null) {
            motor.power = 0.0
            return
        }

        val velocity = motor.velocity
        val error = targetVelocity - velocity

        integralSum += (error * timer.seconds()) // sum of all error over time
        val derivative = (error - lastError) / timer.seconds() // rate of change of error

        @Suppress("RedundantCompanionReference") // Incorrect error
        var out = if (USE_TUNABLE_VALUES) {
            (Companion.kP * error) + (Companion.kI * integralSum) + (Companion.kD * derivative)
        } else {
            (kP * error) + (kI * integralSum) + (kD * derivative)
        }

        out = clamp(out, -1.0, 1.0)

        motor.power = out

        lastError = error

        telemetry.addData("($name) PID Velocity", velocity)
        telemetry.addData("($name) PID Target", targetVelocity)
        telemetry.addData("($name) PID Error", error)
        telemetry.addData("($name) PID Power", out)

        timer.reset()
    }

    /**
     * The current velocity of the motor. Set to null to disable control and set the power to 0.
     * The control loop is ran when you set the velocity, so you should set it every loop.
     */
    val velocity: Double
        get() = motor.velocity

    fun setVelocity(velocity: Double?) {
        loop(velocity)
    }

    companion object {
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