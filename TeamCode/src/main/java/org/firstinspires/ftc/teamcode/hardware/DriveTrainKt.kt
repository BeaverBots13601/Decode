package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.GlobalsKt
import org.firstinspires.ftc.teamcode.misc.PoseKt
import kotlin.math.abs
import kotlin.math.max

/**
 * Responsible for managing our four-wheel Mecanum drive. Includes 3 levels of variable speed.
 */
class DriveTrainKt private constructor(hardwareMap: HardwareMap, data: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    private val driveMotors: Array<DcMotorEx> = createDriveMotors(hardwareMap)
    private val switch: DigitalChannel? = // optional hardware
        runCatching { hardwareMap.digitalChannel.get("switch") }.getOrNull()
    private val referenceAngle = GlobalsKt.robotHeading // saved from auto, or 0 by default.
    private val dashboardEnabled = data.dashboardEnabled
    private var currentSpeedMode = SPEEDS.NORMAL
    private var orientationMode = data.driveMode

    override fun start() {}

    override fun run(data: RunData) {
        orientationMode = if (getSwitchState()) {
            DriveMode.ROBOT // if no switch is attached, fall back to robot mode.
        } else {
            DriveMode.FIELD
        }

        val speedNow = currentSpeedMode.speed

        val tmp_deadzoneadjust = 2

        val stickX = (data.currentGamepadOne.left_stick_x * tmp_deadzoneadjust).toDouble()
        val stickY = (data.currentGamepadOne.left_stick_y * tmp_deadzoneadjust).toDouble() // fixme does this need a negative?
        val stickRotation = (data.currentGamepadOne.right_stick_x * tmp_deadzoneadjust).toDouble()

        telemetry.addData("Current Orientation Mode", orientationMode)
        val directionRotation: Double = if (orientationMode == DriveMode.FIELD) {
            -PoseKt.normalizeAngle(data.imuAngleRad - referenceAngle)
        } else 0.0

        val rotatedPosition = PoseKt.rotatePosition(stickX, stickY, directionRotation)

        val rotatedStickX = rotatedPosition.x
        val rotatedStickY = rotatedPosition.y
        val orientation = data.imuAngleRad
        telemetry.addData("IMU DATA (rads)", orientation)
        telemetry.addData("Reference Angle (rads)", referenceAngle)

        val maxPower = max(abs(stickY) + abs(stickX) + abs(stickRotation), 1.0)

        val leftFrontPower = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speedNow
        val leftBackPower = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speedNow
        val rightFrontPower = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speedNow
        val rightBackPower = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speedNow

        telemetry.addData("Left Front Power", leftFrontPower)
        telemetry.addData("Left Back Power", leftBackPower)
        telemetry.addData("Right Front Power", rightFrontPower)
        telemetry.addData("Right Back Power", rightBackPower)
        telemetry.addData("Current Speed Mode", currentSpeedMode)

        setDriveMotors(arrayOf(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower), RunMode.RUN_WITHOUT_ENCODER)

        // speed controls (gp1)
        if (data.currentGamepadOne.dpadRightWasPressed()){
            currentSpeedMode = SPEEDS.FAST
        }
        if (data.currentGamepadOne.dpadUpWasPressed()){
            currentSpeedMode = SPEEDS.NORMAL
        }
        if (data.currentGamepadOne.dpadLeftWasPressed()){
            currentSpeedMode = SPEEDS.SLOW
        }
        if (data.currentGamepadOne.dpadDownWasPressed() && dashboardEnabled){
            currentSpeedMode = SPEEDS.CUSTOM_FTC_DASHBOARD
        }
    }

    override fun stop() {}

    override val usedButtons = arrayOf(
        GamepadButtons.GP1_LEFT_JOYSTICK,
        GamepadButtons.GP1_RIGHT_JOYSTICK,
        GamepadButtons.GP1_DPAD_RIGHT,
        GamepadButtons.GP1_DPAD_UP,
        GamepadButtons.GP1_DPAD_LEFT,
        GamepadButtons.GP1_DPAD_DOWN,
    )

    private enum class DriveMotorName { // expecting to be same for forseeable future
        leftFront, leftBack, rightFront, rightBack
    }

    private enum class SPEEDS(val speed: Double) {
        NORMAL(0.65),
        FAST(0.80),
        SLOW(0.40),
        CUSTOM_FTC_DASHBOARD(GlobalsKt.CUSTOM_FTC_DASHBOARD_SPEED)
    }

    private fun createDriveMotors(hardwareMap: HardwareMap): Array<DcMotorEx> {
        var out = emptyArray<DcMotorEx>()
        for (driveMotorName in DriveMotorName.entries) {
            val driveMotor = createDefaultMotor(hardwareMap, driveMotorName.name)
            driveMotor.mode = RunMode.RUN_WITHOUT_ENCODER
            out = out.plus(driveMotor)
        }
        return out
    }

    fun setDriveMotors(powers: Array<Double>, mode: RunMode) {
        for (driveMotorName in DriveMotorName.entries){
            driveMotors[driveMotorName.ordinal].mode = mode
            driveMotors[driveMotorName.ordinal].power = powers[driveMotorName.ordinal]
        }
    }

    /**
     * Returns the switch's state. Note that if a switch is not attached (or not configured), this will always return true.
     */
    fun getSwitchState(): Boolean { return switch?.state ?: true }

    companion object : HardwareMechanismSingletonManager<DriveTrainKt>(::DriveTrainKt)
}