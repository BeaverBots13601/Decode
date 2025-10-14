package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.misc.PoseKt
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.abs
import kotlin.math.max

/**
 * Responsible for managing our four-wheel Mecanum drive. Includes 3 levels of variable speed.
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
    private val teamColor = data.teamColor

    // Roadrunner drive for autonomous movement in teleop. 0,0,0 pose is okay because really we want
    // to be in a specific spot relative to an apriltag, not on the field
    private val roadrunnerDrive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
    private val limelight = LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(
        teamColor = data.teamColor,
        dashboardEnabled = data.dashboardEnabled,
        ), telemetry)

    override fun start() {}

    private var autoTrackEnabled = false
    override fun run(data: RunData) {
        if (data.currentGamepadOne.psWasPressed()) {
            autoTrackEnabled = !autoTrackEnabled
        }

        if (autoTrackEnabled && limelight != null) { // handle moving autonomously to apriltag
            val tags = limelight.poll()

            val relativePose = (tags?.find { // Pose of the tag relative to camera
                if (teamColor == TeamColor.RED) { // select tag on depot
                    it.fiducialId == 24
                } else {
                    it.fiducialId == 20
                }
            }?.targetPoseCameraSpace) ?: return // return if not valid or found
            val currentPose = roadrunnerDrive.localizer.pose

            // 1m = 39.37 inches
            // -z on the ll is +x in roadrunner
            // +x on the ll is +y in roadrunner
            // +deg cw on the ll is -deg in roadrunner
            val calculatedPose = Pose2d((-39.37 * relativePose.position.z) + currentPose.position.x + 52.5,
                (39.37 * relativePose.position.x) + currentPose.position.y,
                -Math.toRadians(relativePose.orientation.yaw) + currentPose.heading.real)

            val finalAction = roadrunnerDrive.actionBuilder(roadrunnerDrive.localizer.pose)
                .strafeToLinearHeading(calculatedPose.position, calculatedPose.heading)
                .build()

            // todo should we reuse these actions for a few cycles to save on cycle times?
            finalAction.run(TelemetryPacket())

            return
        }

        val switchState = getSwitchState()
        if (switchState != null && switchState) { // if no switch is attached, do nothing
            orientationMode = DriveMode.ROBOT
        } else if (switchState != null && !switchState) {
            orientationMode = DriveMode.FIELD
        }

        val speedNow = currentSpeedMode.speed

        val tmp_deadzoneadjust = 2

        val stickX = (data.currentGamepadOne.left_stick_x * tmp_deadzoneadjust).toDouble()
        val stickY = -(data.currentGamepadOne.left_stick_y * tmp_deadzoneadjust).toDouble()
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
        GamepadButtons.GP1_PS,
    )

    private enum class DriveMotorName { // expecting to be same for forseeable future
        leftFront, leftBack, rightFront, rightBack
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
            if (driveMotorName == DriveMotorName.leftBack) {
                driveMotor.direction = DcMotorSimple.Direction.FORWARD
            }
            if (driveMotorName == DriveMotorName.rightBack) {
                driveMotor.direction = DcMotorSimple.Direction.REVERSE
            }
            driveMotor.mode = RunMode.RUN_USING_ENCODER
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
     * @return The switch's state. Returns null if a switch is not attached (or not configured)
     */
    fun getSwitchState(): Boolean? { return switch?.state }

    companion object : HardwareMechanismSingletonManager<DriveTrainKt>(::DriveTrainKt) {
        // This speed is designed to be set dynamically within FTC Dashboard.
        @JvmField var CUSTOM_FTC_DASHBOARD_SPEED = 0.65
    }
}