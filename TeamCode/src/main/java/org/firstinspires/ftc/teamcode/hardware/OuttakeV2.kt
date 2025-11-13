package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.misc.PIDVelocityController
import kotlin.math.abs

@Config
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
        leftKicker.direction = Servo.Direction.REVERSE
        leftKicker.position = LeftKickerPositions.NOT_KICK.pos
        rightKicker.position = RightKickerPositions.NOT_KICK.pos
        ferrisWheelMotor.mode = RunMode.RUN_TO_POSITION
        ferrisWheelMotor.targetPosition = 0
    }

    override fun start() {}

    private val currentLaunchDistance = LaunchDistance.FAR_TELEOP
    private var awaitingLaunch = ArtifactColors.NONE
    private var leftRed = true
    private val timer = ElapsedTime()
    private var delta = 0.0
    override fun run(data: RunData) {
        // EMERGENCY !!! REVERSE FLYWHEEL !!!
        if (data.currentGamepadTwo.ps) {
            if (data.currentGamepadTwo.dpad_down) {
                leftFlywheel.velocity = -1000000.0
                rightFlywheel.velocity = -1000000.0
            } else if (data.currentGamepadTwo.dpad_up) {
                leftFlywheel.velocity = 1000000.0
                rightFlywheel.velocity = 1000000.0
            } else if (data.currentGamepadTwo.dpadRightWasPressed()) {
                ferrisWheelMotor.targetPosition -= FERRIS_WHEEL_ROTATE_TICKS
            } else if (data.currentGamepadTwo.dpadLeftWasPressed()) {
                ferrisWheelMotor.targetPosition += FERRIS_WHEEL_ROTATE_TICKS
            }

            ferrisWheelMotor.power = 0.8

            delta += timer.seconds() - delta
            if (delta > 0.75) {
                leftIndicatorLED.position = if (leftRed) 0.28 else 0.611
                rightIndicatorLED.position = if (leftRed) 0.611 else 0.28
                leftRed = !leftRed
                delta = 0.0
                timer.reset()
            }
            return
        }


        val leftSensorValue = leftColorSensor.normalizedColors
        val rightSensorValue = rightColorSensor.normalizedColors
        val lowerSensorValue = lowerColorSensor.normalizedColors

        val leftArtifact = if (leftSensorValue.blue > 0.002
            && leftSensorValue.blue > leftSensorValue.green
        ) {
            leftIndicatorLED.position = 0.722; ArtifactColors.PURPLE
        } else if (leftSensorValue.green > 0.002
            && leftSensorValue.green > leftSensorValue.blue
            && leftSensorValue.green > leftSensorValue.red
        ) {
            leftIndicatorLED.position = 0.500; ArtifactColors.GREEN
        } else {
            leftIndicatorLED.position = 0.000; ArtifactColors.NONE
        }

        val rightArtifact = if (
            rightSensorValue.blue > 0.002
            && rightSensorValue.blue > rightSensorValue.green
        ) {
            rightIndicatorLED.position = 0.722; ArtifactColors.PURPLE
        } else if (
            rightSensorValue.green > 0.002
            && rightSensorValue.green > rightSensorValue.blue
            && rightSensorValue.green > rightSensorValue.red
        ) {
            rightIndicatorLED.position = 0.500; ArtifactColors.GREEN
        } else {
            rightIndicatorLED.position = 0.000; ArtifactColors.NONE
        }

        // When this has one, we need it to be rotated up
        val lowerArtifact = if (lowerSensorValue.blue > 0.002
            && lowerSensorValue.blue > lowerSensorValue.green
        ) {
            ArtifactColors.PURPLE
        } else if (lowerSensorValue.green > 0.002
            && lowerSensorValue.green > lowerSensorValue.blue
            && lowerSensorValue.green > lowerSensorValue.red
        ) {
            ArtifactColors.GREEN
        } else if (data.currentGamepadTwo.crossWasPressed()) {
            ArtifactColors.PURPLE // manual override so it will bring up
        } else {
            ArtifactColors.NONE
        }

        var ferrisWheelMoving =
            abs(ferrisWheelMotor.targetPosition - ferrisWheelMotor.currentPosition) > 5

        // Bring intake artifacts into launch chamber
        if (lowerArtifact != ArtifactColors.NONE && !ferrisWheelMoving) {
            if (leftArtifact == ArtifactColors.NONE) {
                // need to rotate ccw
                ferrisWheelMotor.targetPosition += FERRIS_WHEEL_ROTATE_TICKS
            } else if (rightArtifact == ArtifactColors.NONE) {
                // need to rotate cw
                ferrisWheelMotor.targetPosition -= FERRIS_WHEEL_ROTATE_TICKS
            } // no place to take it
        }

        // Recalculate
        ferrisWheelMoving =
            abs(ferrisWheelMotor.targetPosition - ferrisWheelMotor.currentPosition) > 5

        telemetry.addData("Left Artifact", leftArtifact)
        telemetry.addData("Left Artifact Blue", leftSensorValue.blue)
        telemetry.addData("Left Artifact Red", leftSensorValue.red)
        telemetry.addData("Left Artifact Green", leftSensorValue.green)
        telemetry.addData("Right Artifact", rightArtifact)
        telemetry.addData("Right Artifact Blue", rightSensorValue.blue)
        telemetry.addData("Right Artifact Red", rightSensorValue.red)
        telemetry.addData("Right Artifact Green", rightSensorValue.green)
        telemetry.addData("Lower Artifact", lowerArtifact)
        telemetry.addData("Ferris Wheel Moving", ferrisWheelMoving)
        telemetry.addData("Current Ferris Wheel Position", ferrisWheelMotor.currentPosition)
        telemetry.addData("Current Ferris Wheel Target", ferrisWheelMotor.targetPosition)

        ferrisWheelMotor.power = if (ferrisWheelMoving) 0.4 else 0.0

        if (data.currentGamepadTwo.leftBumperWasPressed()) {
            awaitingLaunch = ArtifactColors.GREEN
        }

        if (data.currentGamepadTwo.rightBumperWasPressed()) {
            awaitingLaunch = ArtifactColors.PURPLE
        }

        // launch when a launch is queued and we aren't rotating
        if (!ferrisWheelMoving && awaitingLaunch != ArtifactColors.NONE) {
            // stop motors
            DriveTrainKt.getInstance()
                ?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), RunMode.RUN_USING_ENCODER)
            if (leftArtifact == awaitingLaunch) { // color is same
                runBlocking(launch(Side.LEFT))
            } else if (rightArtifact == awaitingLaunch) { // color is same
                runBlocking(launch(Side.RIGHT))
            }
            awaitingLaunch = ArtifactColors.NONE
        }

        telemetry.addData("Current Launch Mode", currentLaunchDistance)

        val spinningUpColor = if (data.currentGamepadTwo.right_trigger > 0.5) {
            ArtifactColors.PURPLE
        } else if (data.currentGamepadTwo.left_trigger > 0.5) {
            ArtifactColors.GREEN
        } else {
            ArtifactColors.NONE
        }

        // Flywheels (on trigger)
        val velocity = if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity
        if (spinningUpColor != ArtifactColors.NONE && leftArtifact == spinningUpColor) { // color is same
            leftFlywheel.velocity = velocity
            rightFlywheel.velocity = -1.0
        } else if (spinningUpColor != ArtifactColors.NONE && rightArtifact == spinningUpColor) { // color is same
            leftFlywheel.velocity = -1.0
            rightFlywheel.velocity = velocity
        } else {
            leftFlywheel.velocity = -1.0
            rightFlywheel.velocity = -1.0
        }

        telemetry.addData("Launch Velocity (tps)", velocity)
        // todo fix math
        telemetry.addData("Current Velocity (tps)", leftFlywheel.velocity + rightFlywheel.velocity)
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

    /**
     * Spins up the flywheel until launched, or until 5 seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(side: Side, distance: LaunchDistance): Action {
        return object : Action {
            private var spunUp = false
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            private val motor = if (side == Side.LEFT) leftFlywheel else rightFlywheel
            override fun run(p: TelemetryPacket): Boolean {
                p.put("Spun Up?", spunUp)

                motor.velocity = distance.velocity

                val averageVelocity = motor.velocity
                if (!spunUp && abs(averageVelocity - distance.velocity) < 25) {
                    spunUp = true
                    timer.reset()
                } else if (!spunUp) {
                    p.put("Current Speed", averageVelocity)
                    return true // keep spinning up
                }

                // We are spun up, now monitor for big loss in velocity
                if ((lastVelocity - averageVelocity) > 50.0 || timer.seconds() > 5.0) {
                    p.put("Current Speed", averageVelocity)
                    motor.velocity = -1.0
                    return false // big drop, all done, or timed out
                }

                lastVelocity = averageVelocity
                p.put("Current Speed", averageVelocity)
                return true // keep monitoring
            }
        }
    }

    fun waitForSpunUp(side: Side, distance: LaunchDistance): Action {
        return Action {
            val averageVelocity = if (side == Side.LEFT) leftFlywheel.velocity else rightFlywheel.velocity
            if (abs(averageVelocity - distance.velocity) < 25) {
                false
            } else true
        }
    }

    fun compositionLaunch(side: Side, distance: LaunchDistance): ParallelAction {
        return ParallelAction(
            spinUpUntilLaunched(side, distance),
            SequentialAction(
                waitForSpunUp(side, distance),
                launch(side),
            )
        )
    }

    /**
     * @param side The side to have the artifact be on, or undefined to be the available slot
     */
    fun moveLowerArtifactTo(side: Side?): Action {
        return Action {
            false
        }
    }

    enum class LaunchDistance(val velocity: Double) {
        FAR_TELEOP(100000000000.0),
        FAR_AUTO(900.0),
        CLOSE_TELEOP(0.0),
        CLOSE_AUTO(0.0),

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
        KICK(0.6),
        NOT_KICK(0.2),
    }

    private enum class RightKickerPositions(val pos: Double) {
        KICK(0.6),
        NOT_KICK(0.2),
    }

    companion object : HardwareMechanismSingletonManager<OuttakeV2>(::OuttakeV2) {
        @JvmField var FERRIS_WHEEL_ROTATE_TICKS = 179
        @JvmField var CUSTOM: Double = 0.0
    }
}