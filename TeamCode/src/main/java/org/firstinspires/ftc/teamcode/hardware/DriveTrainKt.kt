package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.misc.PoseKt
import kotlin.math.abs
import kotlin.math.max

/**
 * Responsible for managing our four-wheel Mecanum drive during TeleOp. Includes 3 levels of variable speed, as well as both a field-centric and robot-centric movement mode.
 *
 * Uses an optional [DigitalChannel] named 'switch' for dynamically switching between Robot and Field mode. When not configured, uses the mode of the selected OpMode.
 */
@Config
class DriveTrainKt private constructor(hardwareMap: HardwareMap, data: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    private val driveMotors: Array<DcMotorEx> = createDriveMotors(hardwareMap)
    private val switch: DigitalChannel? = // optional hardware
        runCatching { hardwareMap.digitalChannel.get("switch") }.getOrNull()
    private val referenceAngle = data.referenceAngle // saved from auto, or our current heading by default.
    private val dashboardEnabled = data.dashboardEnabled
    private var currentSpeedMode = SPEEDS.NORMAL
    private var orientationMode = data.driveMode

    override fun start() {}

    override fun run(data: RunData) {
        val switchState = getSwitchState()
        if (switchState != null && switchState) { // if no switch is attached, do nothing
            orientationMode = DriveMode.ROBOT
        } else if (switchState != null) {
            orientationMode = DriveMode.FIELD
        }

        val speedNow = currentSpeedMode.speed

        // Make the joystick more sensitive at powers below what gets capped by maxPower
        // i.e.: stickX =  0.25 -> 0.50 / 1.0 -> forward @ 0.50
        // i.e w/o factor: 0.25 -> 0.25 / 1.0 -> forward @ 0.25
        // i.e.: stickX =  0.75 -> 1.50 / 1.5 -> forward @ 1.00
        // i.e w/o factor: 0.75 -> 0.75 / 1.0 -> forward @ 0.75
        val deadZoneAdjust = 2

        val stickX = (data.currentGamepadOne.left_stick_x * deadZoneAdjust).toDouble()
        val stickY = -(data.currentGamepadOne.left_stick_y * deadZoneAdjust).toDouble()
        val stickRotation = (data.currentGamepadOne.right_stick_x * deadZoneAdjust).toDouble()

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

        setDriveMotors(arrayOf(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower), RunMode.RUN_USING_ENCODER)

        // speed controls (gp1)
        if (data.currentGamepadOne.dpadUpWasPressed()){
            currentSpeedMode = SPEEDS.FAST
        }
        if (data.currentGamepadOne.dpadRightWasPressed()){
            currentSpeedMode = SPEEDS.NORMAL
        }
        if (data.currentGamepadOne.dpadDownWasPressed()){
            currentSpeedMode = SPEEDS.SLOW
        }
        if (data.currentGamepadOne.dpadLeftWasPressed() && dashboardEnabled){
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

    @Suppress("EnumEntryName") // Reason: Enum names map to hardware map config, where
    // convention is camelCase instead of PascalCase
    private enum class DriveMotorName(val direction: DcMotorSimple.Direction) { // Expected to be same for foreseeable future
        leftFront(DcMotorSimple.Direction.REVERSE),
        leftBack(DcMotorSimple.Direction.REVERSE),
        rightFront(DcMotorSimple.Direction.FORWARD),
        rightBack(DcMotorSimple.Direction.FORWARD),
    }

    private enum class SPEEDS(val speed: Double) {
        NORMAL(0.65),
        FAST(0.80),
        SLOW(0.40),
        CUSTOM_FTC_DASHBOARD(CUSTOM_FTC_DASHBOARD_SPEED)
    }

    private fun createDriveMotors(hardwareMap: HardwareMap): Array<DcMotorEx> {
        var out = emptyArray<DcMotorEx>()
        for (driveMotorName in DriveMotorName.entries) {
            val driveMotor = createDefaultMotor(hardwareMap, driveMotorName.name)
            driveMotor.mode = RunMode.RUN_USING_ENCODER
            driveMotor.direction = driveMotorName.direction
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
     * @return The switch's state. Returns null if a switch is not configured
     */
    fun getSwitchState(): Boolean? { return switch?.state }

    companion object : HardwareMechanismSingletonManager<DriveTrainKt>(::DriveTrainKt) {
        /**
         * This speed is designed to be set dynamically within FTC Dashboard.
         */
        @JvmField var CUSTOM_FTC_DASHBOARD_SPEED = 0.65
    }
}