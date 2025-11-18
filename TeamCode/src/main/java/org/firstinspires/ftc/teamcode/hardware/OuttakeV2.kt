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
import com.qualcomm.robotcore.util.RobotLog
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

    // Color Sensor
    private val lowerColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "lowerColorSensor")

    // Flywheels
    private val leftFlywheel  = PIDVelocityController(
        createDefaultMotor(hardwareMap, "leftFlywheel"),
        0.005,
        0.0,
        0.0,
        telemetry,
    )
    private val rightFlywheel = PIDVelocityController(
        createDefaultMotor(hardwareMap, "rightFlywheel"),
        0.005,
        0.0,
        0.0,
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

    // Intake
    private val intakeMotor = createDefaultMotor(hardwareMap, "intakeMotor")

    init {
        leftAngleServo.direction = DcMotorSimple.Direction.REVERSE
        leftKicker.position = LeftKickerPositions.NOT_KICK.pos
        rightKicker.position = RightKickerPositions.NOT_KICK.pos
        ferrisWheelMotor.mode = RunMode.RUN_TO_POSITION
        ferrisWheelMotor.targetPosition = 0
        intakeMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun start() {}

    private var currentLaunchDistance = LaunchDistance.FAR
    private var awaitingLaunch = ArtifactColors.NONE

    // Emergency Mode Stuff
    private var leftRed = true
    private val timer = ElapsedTime()
    private var delta = 0.0

    private val artifacts = ArtifactData()
    override fun run(data: RunData) {
        // !!! EMERGENCY !!!
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
            } else if (data.currentGamepadTwo.leftBumperWasPressed()) {
                runBlocking(launch(Position.LEFT))
                artifacts.launched(Position.LEFT)
            } else if (data.currentGamepadTwo.rightBumperWasPressed()) {
                runBlocking(launch(Position.RIGHT))
                artifacts.launched(Position.RIGHT)
            }

            ferrisWheelMotor.power = 0.6

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

        val lowerSensorValue = lowerColorSensor.normalizedColors

        // When this has one, we need it to be rotated up
        val detectedLowerArtifact = if (lowerSensorValue.blue > 0.002
            && lowerSensorValue.blue > lowerSensorValue.green
        ) {
            ArtifactColors.PURPLE
        } else if (lowerSensorValue.green > 0.002
            && lowerSensorValue.green > lowerSensorValue.blue
            && lowerSensorValue.green > lowerSensorValue.red
        ) {
            ArtifactColors.GREEN
        } else {
            ArtifactColors.NONE
        }

        var ferrisWheelMoving =
            abs(ferrisWheelMotor.targetPosition - ferrisWheelMotor.currentPosition) > 5

        if (detectedLowerArtifact != ArtifactColors.NONE && !ferrisWheelMoving) {
            artifacts.intake(detectedLowerArtifact)
            val rotateTo = artifacts.rotate()

            rotateFerrisWheel(rotateTo)

            if (artifacts.lowerArtifact != ArtifactColors.NONE
                && artifacts.leftArtifact != ArtifactColors.NONE
                && artifacts.rightArtifact != ArtifactColors.NONE) {
                //toggleIntake()
            }
        }

        if (data.currentGamepadTwo.squareWasPressed()) {
            toggleIntake()
        }

        if (data.currentGamepadTwo.optionsWasPressed()) {
            currentLaunchDistance = if (currentLaunchDistance == LaunchDistance.FAR) {
                LaunchDistance.CLOSE
            } else {
                LaunchDistance.FAR
            }
        }

        setLEDs()

        // Recalculate
        ferrisWheelMoving =
            abs(ferrisWheelMotor.targetPosition - ferrisWheelMotor.currentPosition) > 5

        telemetry.addData("Lower Artifact", artifacts.lowerArtifact)
        telemetry.addData("Left Artifact", artifacts.leftArtifact)
        telemetry.addData("Right Artifact", artifacts.rightArtifact)
        telemetry.addData("Ferris Wheel Moving", ferrisWheelMoving)
        telemetry.addData("Current Ferris Wheel Position", ferrisWheelMotor.currentPosition)
        telemetry.addData("Current Ferris Wheel Target", ferrisWheelMotor.targetPosition)

        ferrisWheelMotor.power = if (ferrisWheelMoving) 0.6 else 0.0

        if (data.currentGamepadTwo.leftBumperWasPressed()) { awaitingLaunch = ArtifactColors.GREEN }

        if (data.currentGamepadTwo.rightBumperWasPressed()) { awaitingLaunch = ArtifactColors.PURPLE }

        val velocity = if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity

        val spunUpFlywheel = if (velocity > 2000) {
            if (leftFlywheel.velocity > 2000) {
                Position.LEFT
            } else if (rightFlywheel.velocity > 2000) {
                Position.RIGHT
            } else Position.NONE
        } else {
            if (abs(leftFlywheel.velocity - velocity) < 250) {
                Position.LEFT
            } else if (abs(rightFlywheel.velocity - velocity) < 250) {
                Position.RIGHT
            } else Position.NONE
        }

        telemetry.addData("Spun Up Flywheel", spunUpFlywheel)

        // launch when a launch is queued and we aren't rotating
        if (!ferrisWheelMoving && awaitingLaunch != ArtifactColors.NONE) {
            val launchArtifact = artifacts.color(awaitingLaunch, spunUpFlywheel)
            if (launchArtifact == spunUpFlywheel && spunUpFlywheel != Position.NONE) { // launch!
                // stop motors
                DriveTrainKt.getInstance()
                    ?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), RunMode.RUN_USING_ENCODER)
                runBlocking(launch(launchArtifact))
                awaitingLaunch = ArtifactColors.NONE
                artifacts.launched(launchArtifact)
            } else if (launchArtifact != Position.NONE && spunUpFlywheel != Position.NONE) {
                // Queue for rotation and then wait for wheel to move
                // Don't reset awaitingLaunch: we want this to happen next time
                val position = artifacts.rotate(spunUpFlywheel, awaitingLaunch)
                rotateFerrisWheel(position)
            } else {
                // reset; probably done in error
                awaitingLaunch = ArtifactColors.NONE
            }
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
        if (spunUpFlywheel != Position.NONE && spinningUpColor != ArtifactColors.NONE) { // keep current spun up one
            if (spunUpFlywheel == Position.LEFT) {
                leftFlywheel.velocity = velocity
                rightFlywheel.velocity = -1.0
            } else {
                leftFlywheel.velocity = -1.0
                rightFlywheel.velocity = velocity
            }
        } else if (spinningUpColor != ArtifactColors.NONE) { // need to spin one up
            if (artifacts.leftArtifact == spinningUpColor) { // color is same
                leftFlywheel.velocity = velocity
                rightFlywheel.velocity = -1.0
            } else if (artifacts.rightArtifact == spinningUpColor) { // color is same
                leftFlywheel.velocity = -1.0
                rightFlywheel.velocity = velocity
            } else {
                // spin up both because we don't know where it will end up after spinning
                // todo: optimize; we can figure that out somehow
                leftFlywheel.velocity = velocity
                rightFlywheel.velocity = velocity
            }
        } else { // not spun up and/or don't need to
            leftFlywheel.velocity = -1.0
            rightFlywheel.velocity = -1.0
        }

        telemetry.addData("Launch Velocity (tps)", velocity)
        // todo fix math
        telemetry.addData("Current Velocity (tps)", leftFlywheel.velocity + rightFlywheel.velocity)
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        GamepadButtons.GP2_SQUARE,
    )

    /**
     * @param position The position the current lower artifact should end up in
     */
    private fun rotateFerrisWheel(position: Position) {
        if (position == Position.RIGHT) { // move low to left
            // need to rotate ccw
            ferrisWheelMotor.targetPosition += FERRIS_WHEEL_ROTATE_TICKS
        } else if (position == Position.LEFT) { // move low to right
            // need to rotate cw
            ferrisWheelMotor.targetPosition -= FERRIS_WHEEL_ROTATE_TICKS
        }
    }

    private fun setLEDs() {
        when (artifacts.leftArtifact) {
            ArtifactColors.PURPLE -> {
                leftIndicatorLED.position = 0.722
            }
            ArtifactColors.GREEN -> {
                leftIndicatorLED.position = 0.500
            }
            else -> {
                leftIndicatorLED.position = 0.000
            }
        }

        when (artifacts.rightArtifact) {
            ArtifactColors.PURPLE -> {
                rightIndicatorLED.position = 0.722
            }
            ArtifactColors.GREEN -> {
                rightIndicatorLED.position = 0.500
            }
            else -> {
                rightIndicatorLED.position = 0.000
            }
        }
    }

    private var intakeActive = false
    fun toggleIntake() {
        if (intakeActive) {
            intakeMotor.power = 0.0
            intakeActive = false
        } else {
            intakeMotor.power = 0.7
            intakeActive = true
        }
    }

    fun launch(position: Position): Action {
        return if (position == Position.LEFT) {
            SequentialAction(
                InstantAction { leftKicker.position = LeftKickerPositions.KICK.pos },
                SleepAction(0.5), // frozen thread may be desirable; stop drivers from leaving
                InstantAction { leftKicker.position = LeftKickerPositions.NOT_KICK.pos },
                InstantAction { artifacts.launched(Position.LEFT) },
            )
        } else { // right
            SequentialAction(
                InstantAction { rightKicker.position = RightKickerPositions.KICK.pos },
                SleepAction(0.5), // frozen thread may be desirable; stop drivers from leaving
                InstantAction { rightKicker.position = RightKickerPositions.NOT_KICK.pos },
                InstantAction { artifacts.launched(Position.RIGHT) },
            )
        }
    }

    /**
     * Spins up the flywheel until launched, or until 5 seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(position: Position, distance: LaunchDistance): Action {
        return object : Action {
            private var spunUp = false
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            private val motor = if (position == Position.LEFT) leftFlywheel else rightFlywheel
            override fun run(p: TelemetryPacket): Boolean {
                p.put("Spun Up?", spunUp)

                motor.velocity = distance.velocity

                val averageVelocity = motor.velocity
                if (!spunUp && abs(abs(averageVelocity) - distance.velocity) < 50) {
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

    fun waitForSpunUp(position: Position, distance: LaunchDistance): Action {
        return Action {
            val averageVelocity = if (position == Position.LEFT) leftFlywheel.velocity else rightFlywheel.velocity
            if (abs(averageVelocity - distance.velocity) < 25) {
                false
            } else true
        }
    }

    fun compositionLaunch(position: Position, distance: LaunchDistance): ParallelAction {
        return ParallelAction(
            spinUpUntilLaunched(position, distance),
            SequentialAction(
                waitForSpunUp(position, distance),
                launch(position),
                InstantAction { setLEDs() }
            )
        )
    }

    fun intakeUntilIndexed(): Action {
        return object : Action {
            private var firstRun = true
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    toggleIntake()
                    firstRun = false
                }

                val lowerSensorValue = lowerColorSensor.normalizedColors

                // When this has one, we need it to be rotated up
                val detectedLowerArtifact = if (lowerSensorValue.blue > 0.002
                    && lowerSensorValue.blue > lowerSensorValue.green
                ) {
                    ArtifactColors.PURPLE
                } else if (lowerSensorValue.green > 0.002
                    && lowerSensorValue.green > lowerSensorValue.blue
                    && lowerSensorValue.green > lowerSensorValue.red
                ) {
                    ArtifactColors.GREEN
                } else {
                    ArtifactColors.NONE
                }

                if (detectedLowerArtifact != ArtifactColors.NONE) {
                    artifacts.intake(detectedLowerArtifact)
                    val rotateTo = artifacts.rotate()

                    rotateFerrisWheel(rotateTo)

                    toggleIntake()
                    return false
                }
                return true
            }
        }
    }

    fun rotateOptimally(spunUp: Position, nextColor: ArtifactColors) {
        val position = artifacts.rotate(spunUp, nextColor)
        rotateFerrisWheel(position)
    }

    fun tripleLaunch(motif: Motif, distance: LaunchDistance): Action {
        val firstPosition = artifacts.color(motif.first)
        return ParallelAction(
                SequentialAction(compositionLaunch(firstPosition, distance),
                InstantAction { rotateOptimally(firstPosition, motif.second) },
                compositionLaunch(firstPosition, distance),
                InstantAction { rotateOptimally(firstPosition, motif.third) },
                compositionLaunch(firstPosition, distance),
            ),
            SequentialAction(
                spinUpUntilLaunched(firstPosition, distance),
                spinUpUntilLaunched(firstPosition, distance),
                spinUpUntilLaunched(firstPosition, distance),
            ),
        )
    }

    fun loadAutoArtifacts() = artifacts.autoArtifactPreloads()

    enum class LaunchDistance(val velocity: Double) {
        FAR(2800.0),
        CLOSE(0.0),

    }

    enum class ArtifactColors {
        PURPLE,
        GREEN,
        NONE,
    }

    enum class Position {
        LEFT,
        RIGHT,
        LOWER,
        NONE,
    }

    enum class Motif(val first: ArtifactColors, val second: ArtifactColors, val third: ArtifactColors) {
        PURPLE_PURPLE_GREEN(ArtifactColors.PURPLE, ArtifactColors.PURPLE, ArtifactColors.GREEN),
        PURPLE_GREEN_PURPLE(ArtifactColors.PURPLE, ArtifactColors.GREEN, ArtifactColors.PURPLE),
        GREEN_PURPLE_PURPLE(ArtifactColors.GREEN, ArtifactColors.PURPLE, ArtifactColors.PURPLE),
    }

    private enum class LeftKickerPositions(val pos: Double) {
        KICK(0.4),
        NOT_KICK(0.8),
    }

    private enum class RightKickerPositions(val pos: Double) {
        KICK(0.6),
        NOT_KICK(0.2),
    }

    companion object : HardwareMechanismSingletonManager<OuttakeV2>(::OuttakeV2) {
        @JvmField var FERRIS_WHEEL_ROTATE_TICKS = 930
        @JvmField var CUSTOM: Double = 0.0
    }

    private class ArtifactData {
        val artifacts: Array<ArtifactColors> = arrayOf(
            ArtifactColors.NONE, // lower
            ArtifactColors.NONE, // left
            ArtifactColors.NONE, // right
        )

        fun intake(color: ArtifactColors) {
            if (lowerArtifact != ArtifactColors.NONE) {
                RobotLog.ee(
                    "OuttakeV2",
                    "You screwed up: Lower Artifact was not empty when intake was called"
                )
                return
            }
            lowerArtifact = color
        }

        /**
         * Automatically rotate the current artifacts to the most optimal positioning.
         * @param spunUp The flywheel that is currently spun up to shooting speed.
         * @param spunUpColor The color that the flywheel is seeking.
         * @return Position that the current lower artifact should end up in.
         */
        fun rotate(spunUp: Position = Position.NONE, spunUpColor: ArtifactColors = ArtifactColors.NONE): Position {
            val colorPos = color(spunUpColor, spunUp) // where is the color we want?

            // note: this inherently prioritizes firing fast over optimal storage
            // we are looking for an artifact and hold it
            // todo: something not quite right here. what about case where rotate low
            //       so that both of our tops are filled?
            // if we have the ability to decide, we should rotate such that both of our tops are filled
            // and the one we want is in the correct place instead of top/lower
            if (spunUpColor != ArtifactColors.NONE && colorPos != Position.NONE) {
                if (colorPos == spunUp) {
                    return Position.LOWER // don't move: in perfect spot
                }

                if (colorPos == Position.LEFT && spunUp == Position.RIGHT) {
                    // need to rotate lower to left
                    rotate(Position.LEFT)
                    return Position.LEFT
                }

                if (colorPos == Position.RIGHT && spunUp == Position.LEFT) {
                    // need to rotate lower to right
                    rotate(Position.RIGHT)
                    return Position.RIGHT
                }

                // artifact we want is in lower
                if (spunUp == Position.LEFT) {
                    rotate(Position.LEFT)
                    return Position.LEFT
                }

                // must want to go to right
                rotate(Position.RIGHT)
                return Position.RIGHT
            }

            // we aren't looking for an artifact or we don't hold it
            if (leftArtifact == ArtifactColors.NONE || rightArtifact == ArtifactColors.NONE) {
                // never want a 'NONE' in the upper two
                if (leftArtifact == ArtifactColors.NONE) {
                    rotate(Position.RIGHT)
                    return Position.RIGHT
                    // We rotate to RIGHT not LEFT so that if there is an artifact in the RIGHT
                    // it is kept in the high not brought to low
                }

                if (rightArtifact == ArtifactColors.NONE) {
                    rotate(Position.LEFT)
                    return Position.LEFT
                    // above but reversed
                }
            }

            if (leftArtifact == rightArtifact && lowerArtifact != leftArtifact) {
                // if we have the opportunity to, we want the two ready-to-launch artifacts to be different
                rotate(Position.LEFT)
                return Position.LEFT // arbitrary
            }

            return Position.LOWER // don't move: nowhere to move to
        }

        /**
         * Manually rotate the current artifacts.
         * @param position Position that the current lower artifact will end up in.
         */
        fun rotate(position: Position) {
            if (position == Position.LEFT) {
                val temp = leftArtifact
                leftArtifact = lowerArtifact
                lowerArtifact = rightArtifact
                rightArtifact = temp
            } else if (position == Position.RIGHT) {
                val temp = leftArtifact
                leftArtifact = rightArtifact
                rightArtifact = lowerArtifact
                lowerArtifact = temp
            }
        }

        /**
         * @param color The color of the artifact to search for.
         * @param spunUp The flywheel that is currently spun up to shooting speed.
         * @return The position of the artifact that should be shot.
         */
        fun color(color: ArtifactColors, spunUp: Position = Position.NONE): Position {
            // Prefer the spunUp one if possible
            if (spunUp != Position.NONE) {
                if (leftArtifact == color && spunUp == Position.LEFT) {
                    // left is the color we want and is spun up.
                    return Position.LEFT
                } else if (rightArtifact == color && spunUp == Position.RIGHT) {
                    // ditto for right
                    return Position.RIGHT
                }
                // If we are spun up but the wanted is at lower, it's up to the caller to manage that.
            }

            // Nothing is spun up so both are equally desirable.
            // Still don't want to use the bottom if possible, to avoid turning delay
            if (leftArtifact == color) {
                return Position.LEFT
            } else if (rightArtifact == color) {
                return Position.RIGHT
            }

            if (lowerArtifact == color) {
                // Lower is our only option
                return Position.LOWER
            }
            return Position.NONE // don't hold this color
        }

        fun launched(side: Position) {
            if (side == Position.LEFT) {
                leftArtifact = ArtifactColors.NONE
            } else if (side == Position.RIGHT) {
                rightArtifact = ArtifactColors.NONE
            }
        }

        fun autoArtifactPreloads() {
            lowerArtifact = ArtifactColors.PURPLE
            leftArtifact = ArtifactColors.GREEN
            rightArtifact = ArtifactColors.PURPLE
        }

        var leftArtifact
            get() = artifacts[1]
            private set(it) { artifacts[1] = it }

        var rightArtifact
            get() = artifacts[2]
            private set(it) { artifacts[2] = it }


        var lowerArtifact
            get() = artifacts[0]
            private set(it) { artifacts[0] = it }
    }
}