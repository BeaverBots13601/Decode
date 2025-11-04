package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.misc.PIDVelocityController
import kotlin.math.abs

class OuttakeV2 private constructor(hardwareMap: HardwareMap, initData: InitData, val telemetry: Telemetry) : HardwareMechanismKt() {
    // LEDs (for some reason it makes the most sense to do these as servos)
    private val leftIndicatorLED  = hardwareMap.servo.get("leftIndicatorLED")
    private val rightIndicatorLED = hardwareMap.servo.get("rightIndicatorLED")

    // Color Sensors
    private val leftColorSensor  = hardwareMap.get(RevColorSensorV3::class.java, "leftColorSensor")
    private val rightColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightColorSensor")
    private val lowerColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "lowerColorSensor")

    // Flywheels
    private val leftFlywheel  = PIDVelocityController(
        createDefaultMotor(hardwareMap, "leftFlywheel"),
        0.0175,
        0.000002,
        0.0001,
        telemetry,
    )
    private val rightFlywheel = PIDVelocityController(
        createDefaultMotor(hardwareMap, "rightFlywheel"),
        0.0175,
        0.000002,
        0.0001,
        telemetry,
    )

    // Angling Servos
    private val leftAngleServo  = hardwareMap.crservo.get("leftAngleServo")
    private val rightAngleServo = hardwareMap.crservo.get("rightAngleServo")

    // Kickers
    private val leftKicker  = hardwareMap.servo.get("leftKicker")
    private val rightKicker = hardwareMap.servo.get("rightKicker")

    // Wheel
    private val ferrisWheelMotor = createDefaultMotor(hardwareMap, "ferrisWheelMotor")

    init {
        leftAngleServo.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun start() {}

    private val currentLaunchDistance = LaunchDistance.FAR
    private var awaitingLaunch = ArtifactColors.NONE
    override fun run(data: RunData) {
        val ferrisWheelMoving = abs(ferrisWheelMotor.targetPosition - ferrisWheelMotor.currentPosition) > 50

        val leftSensorValue = leftColorSensor.normalizedColors
        val rightSensorValue = rightColorSensor.normalizedColors
        val lowerSensorValue = lowerColorSensor.normalizedColors

        // todo: works?
        val leftArtifact = if (leftSensorValue.red > 0.5 && leftSensorValue.blue > 0.5) {
            leftIndicatorLED.position = 0.722; ArtifactColors.PURPLE
        } else if (leftSensorValue.green > 0.5) {
            leftIndicatorLED.position = 0.500; ArtifactColors.GREEN
        } else {
            leftIndicatorLED.position = 0.000; ArtifactColors.NONE
        }

        val rightArtifact = if (rightSensorValue.red > 0.5 && rightSensorValue.blue > 0.5) {
            rightIndicatorLED.position = 0.722; ArtifactColors.PURPLE
        } else if (rightSensorValue.green > 0.5) {
            rightIndicatorLED.position = 0.500; ArtifactColors.GREEN
        } else {
            rightIndicatorLED.position = 0.000; ArtifactColors.NONE
        }

        // When this has one, we need it to be rotated up
        val lowerArtifact = if (lowerSensorValue.red > 0.5 && lowerSensorValue.blue > 0.5) {
            ArtifactColors.PURPLE
        } else if (lowerSensorValue.green > 0.5) {
            ArtifactColors.GREEN
        } else {
            ArtifactColors.NONE
        }

        // Bring intake artifacts into launch chamber
        if (lowerArtifact != ArtifactColors.NONE && !ferrisWheelMoving){
            if (leftArtifact == ArtifactColors.NONE) {
                // need to rotate ccw
                ferrisWheelMotor.targetPosition -= FERRIS_WHEEL_ROTATE_TICKS
            } else if (rightArtifact != ArtifactColors.NONE) {
                // need to rotate cw
                ferrisWheelMotor.targetPosition += FERRIS_WHEEL_ROTATE_TICKS
            } // no place to take it
        }

        if (data.currentGamepadTwo.leftBumperWasPressed()) {
            awaitingLaunch = ArtifactColors.GREEN
        }

        if (data.currentGamepadTwo.rightBumperWasPressed()) {
            awaitingLaunch = ArtifactColors.PURPLE
        }

        // launch when a launch is queued and we aren't rotating
        if (!ferrisWheelMoving && awaitingLaunch != ArtifactColors.NONE) {
            // stop motors
            DriveTrainKt.getInstance()?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), DcMotor.RunMode.RUN_USING_ENCODER)
            if (leftArtifact == awaitingLaunch) { // color is same
                runBlocking(launch(Side.LEFT))
            } else if (rightArtifact == awaitingLaunch) { // color is same
                runBlocking(launch(Side.RIGHT))
            }
            awaitingLaunch = ArtifactColors.NONE
        }

        telemetry.addData("Current Launch Mode", currentLaunchDistance)

        // Flywheels (on trigger)
        leftFlywheel.velocity = data.currentGamepadTwo.right_trigger.toDouble() * currentLaunchDistance.velocity
        rightFlywheel.velocity = data.currentGamepadTwo.right_trigger.toDouble() * currentLaunchDistance.velocity
        telemetry.addData("Launch Velocity (tps)", data.currentGamepadTwo.right_trigger.toDouble() * currentLaunchDistance.velocity)
        val velocity = (leftFlywheel.velocity + rightFlywheel.velocity) / 2
        telemetry.addData("Current Velocity (tps)", velocity)
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    fun launch(side: Side): Action {
        return if (side == Side.LEFT) {
            SequentialAction(
                InstantAction { leftKicker.position = LeftKickerPositions.KICK.pos },
                SleepAction(0.5), // frozen thread may be desirable; stop drivers from leaving
                InstantAction { leftKicker.position = LeftKickerPositions.NOT_KICK.pos },
            )
        } else { // right
            SequentialAction(
                InstantAction { rightKicker.position = RightKickerPositions.KICK.pos },
                SleepAction(0.5), // frozen thread may be desirable; stop drivers from leaving
                InstantAction { rightKicker.position = RightKickerPositions.NOT_KICK.pos },
            )
        }
    }

    fun spinUp(side: Side, distance: LaunchDistance): Action {
        return object : Action {
            private var firstRun = true
            private val motor = if (side == Side.LEFT) leftFlywheel else rightFlywheel
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    firstRun = false
                    motor.velocity = distance.velocity
                }

                p.put("Current Speed", motor)

                if (abs(motor.velocity - distance.velocity) < 25) {
                    motor.velocity = 0.0
                    return false
                } else return true
            }
        }
    }

    enum class LaunchDistance(val velocity: Double) {
        FAR(900.0),
        NEAR(0.0),
    }

    private enum class ArtifactColors {
        PURPLE,
        GREEN,
        NONE,
    }

    enum class Side {
        LEFT,
        RIGHT
    }

    private enum class LeftKickerPositions(val pos: Double) {
        KICK(0.0),
        NOT_KICK(0.0),
    }

    private enum class RightKickerPositions(val pos: Double) {
        KICK(0.0),
        NOT_KICK(0.0),
    }

    companion object : HardwareMechanismSingletonManager<OuttakeV2>(::OuttakeV2) {
        @JvmField var FERRIS_WHEEL_ROTATE_TICKS: Int = 0
    }
}