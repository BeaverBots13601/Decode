package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.clamp
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.misc.AxonDriver
import org.firstinspires.ftc.teamcode.misc.GoBildaRGBIndicatorDriver
import org.firstinspires.ftc.teamcode.misc.GoBildaRGBIndicatorDriver.Color
import org.firstinspires.ftc.teamcode.misc.PoseKt
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import java.util.LinkedList
import java.util.Queue
import kotlin.math.abs

class OuttakeV3 private constructor(hardwareMap: HardwareMap, initData: InitData, val telemetry: Telemetry) : HardwareMechanismKt() {
    // LEDs (for some reason it makes the most sense to do these as servos)
    private val leftIndicatorLED  = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("leftIndicatorLED"))
    private val rightIndicatorLED = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("rightIndicatorLED"))

    // Flywheel
    private val flywheel = createDefaultMotor(hardwareMap, "flywheel").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    // Color Sensor
    private val intakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeColorSensor")

    private val hoodAngler = hardwareMap.servo.get("hoodAngler")

    // Intake
    private val intakeMotor = createDefaultMotor(hardwareMap, "intakeMotor").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    // Turntable
    private val turntableAxon = AxonDriver(
        hardwareMap,
        "turntableAxon",
        "turntableEncoder",
        0.0,
        0.0,
        0.0,
        telemetry,
        5.4
    )

    // Limelight
    private val limelight = LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(initData), telemetry)

    // Transfer
    private val leftKicker = hardwareMap.servo.get("leftKickerServo")
    private val rightKicker = hardwareMap.servo.get("rightKickerServo")
    private val transfer = hardwareMap.crservo.get("transferServo")

    init {
        leftKicker.position = LeftKickerPosition.NOT_KICK.pos
        rightKicker.position = RightKickerPosition.NOT_KICK.pos
    }

    override fun start() {}

    // Emergency Mode Stuff
    private var leftRed = true
    private val timer = ElapsedTime()
    private var delta = 0.0

    // Config info
    private val teamColor = initData.teamColor

    // State
    private var currentLaunchDistance = LaunchDistance.CLOSE
    private val artifacts = ArtifactData()
    override fun run(data: RunData) {
        // !!! EMERGENCY !!!
        if (data.currentGamepadTwo.ps) {
            if (data.currentGamepadTwo.dpad_down) {
                flywheel.velocity = -1000000.0
            } else if (data.currentGamepadTwo.dpad_up) {
                flywheel.velocity = 1000000.0
            } else if (data.currentGamepadTwo.rightBumperWasPressed()) {
                runBlocking(launch())
            }

            delta += timer.seconds() - delta
            if (delta > 0.75) {
                leftIndicatorLED.color = if (leftRed) Color.RED else Color.BLUE
                rightIndicatorLED.color = if (leftRed) Color.BLUE else Color.RED
                leftRed = !leftRed
                delta = 0.0
                timer.reset()
            }
            return
        }

        // Artifact detection
        val detectedArtifact = detectArtifact(intakeColorSensor.normalizedColors)
        telemetry.addData("Detected Artifact", detectedArtifact)

        // Intaking
        if (data.currentGamepadTwo.squareWasPressed()) {
            toggleIntake()
        }

        if (data.currentGamepadTwo.triangleWasPressed()) {
            toggleTransfer()
        }

        // Setting launch distance
        if (data.currentGamepadTwo.optionsWasPressed()) {
            currentLaunchDistance = if (currentLaunchDistance == LaunchDistance.FAR) {
                LaunchDistance.CLOSE
            } else {
                LaunchDistance.FAR
            }
        }

        setLEDs()

        // Fire!
        if (data.currentGamepadTwo.crossWasPressed()) {
            DriveTrainKt.getInstance()
                ?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), RunMode.RUN_USING_ENCODER)
            runBlocking(launch())
        }

        // Flywheels (on trigger)
        flywheel.velocity = if (data.currentGamepadTwo.right_trigger < 0.5) {
            telemetry.addData("Target Velocity (tps)", 0.0)
            flywheel.power = 0.0
            0.0
        } else {
            telemetry.addData("Target Velocity (tps)", currentLaunchDistance.velocity)
            flywheel.power = 1.0
            currentLaunchDistance.velocity
        }

        // region Figure out if spun up
        val currentVelocity = flywheel.velocity

        val flywheelSpunUp = if (currentVelocity > 2000) {
            if (flywheel.velocity > 2000) {
                true
            } else false
        } else {
            if (abs(flywheel.velocity - currentVelocity) < 250) {
                true
            } else false
        }
        // endregion

        telemetry.addData("Flywheel Spun Up?", flywheelSpunUp)
        telemetry.addData("Current Launch Mode", currentLaunchDistance)
        telemetry.addData("Current Velocity (tps)", flywheel.velocity)

        // Turret angling
        if (data.currentGamepadTwo.left_bumper) {
            turntableAxon.overridePower = -0.2
        } else if (data.currentGamepadTwo.right_bumper) {
            turntableAxon.overridePower = 0.2
        } else {
            turntableAxon.overridePower = null // let the control loop below handle it

            // get tags
            val depotTag = getDepotTag()

            val delta = if (limelight == null) { // find delta
                null // Not attached; do nothing
            } else if (depotTag != null) {
                -depotTag.orientation.yaw // negative! because positive is left by default
            } else 90.0 // rotate to find the tag

            // rotate
            rotateTurretByDelta(delta)
        }
    }

    override fun stop() {
        turntableAxon.cleanUp()
    }

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        GamepadButtons.GP2_PS,
        GamepadButtons.GP2_SQUARE,
        GamepadButtons.GP2_TRIANGLE,
        GamepadButtons.GP2_OPTIONS,
        GamepadButtons.GP2_RIGHT_BUMPER,
        GamepadButtons.GP2_RIGHT_TRIGGER,
    )

    // region Hardware Functions
    private var intakeActive = false
    fun toggleIntake() {
        if (intakeActive) {
            intakeMotor.power = 0.0
            intakeActive = false
        } else {
            intakeMotor.power = 1.0
            intakeActive = true
        }
    }

    private var transferActive = false
    fun toggleTransfer() {
        if (transferActive) {
            transfer.power = 1.0
            transferActive = false
        } else {
            transfer.power = 0.0
            transferActive = true
        }
    }

    private fun setLEDs() {
        //TODO() temporary noop
    }

    /**
     * Rotates the turret a specific number of degrees from our current location. Please call
     * this function even if the delta is 0.0 as this updates the control loop for our servo.
     *
     * @param delta The delta, in degrees, from current forward on the robot. Negative is left; Positive is right.
     */
    private fun rotateTurretByDelta(delta: Double?) {
        if (delta == null) {
            turntableAxon.targetPosition = null
            return
        }
        // this may underflow or overflow so we're always in [-180, 180]
        var target = PoseKt.normalizeAngleDeg(turntableAxon.position + delta)

        // also clamp to [-130, 130] to meet hardware requirements (allowing 20deg of overshoot - too much?)
        target = clamp(target, -130.0, 130.0)

        turntableAxon.targetPosition = target
    }
    // endregion

    // region Roadrunner Actions
    fun launch(): Action {
        return SequentialAction(
            // bring artifact to top (launches one if in top)
            InstantAction { toggleTransfer() },
            waitUntilLaunched(1.0), // times out when nothing in transfer
            // launch top artifact
            InstantAction { leftKicker.position = LeftKickerPosition.KICK.pos },
            InstantAction { rightKicker.position = RightKickerPosition.KICK.pos },
            SleepAction(0.5),
            InstantAction { leftKicker.position = LeftKickerPosition.NOT_KICK.pos },
            InstantAction { rightKicker.position = RightKickerPosition.NOT_KICK.pos },
            InstantAction { toggleTransfer() },
            // fixme: launches two
        )
    }

    /**
     * Spins up the flywheel until launched, or until *timeout* seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(distance: LaunchDistance, timeout: Double): Action {
        return object : Action {
            private var spunUp = false
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                p.put("Spun Up?", spunUp)

                flywheel.velocity = distance.velocity

                val averageVelocity = flywheel.velocity
                if (!spunUp && abs(abs(averageVelocity) - distance.velocity) < 50) {
                    spunUp = true
                    timer.reset()
                } else if (!spunUp) {
                    p.put("Current Speed", averageVelocity)
                    return true // keep spinning up
                }

                // We are spun up, now monitor for big loss in velocity
                if ((lastVelocity - averageVelocity) > 50.0 || timer.seconds() > timeout) {
                    p.put("Current Speed", averageVelocity)
                    flywheel.velocity = 0.0
                    return false // big drop, all done, or timed out
                }

                lastVelocity = averageVelocity
                p.put("Current Speed", averageVelocity)
                return true // keep monitoring
            }
        }
    }

    /**
     * Spins up the flywheel until launched, or until 2.5 seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(distance: LaunchDistance) = spinUpUntilLaunched(distance, 2.5)

    /**
     * Watches the flywheel until launched, or until 2.5 seconds have passed.
     */
    fun waitUntilLaunched() = waitUntilLaunched(2.5)

    /**
     * Watches the flywheel until launched, or until *timeout* seconds have passed.
     */
    fun waitUntilLaunched(timeout: Double): Action {
        return object : Action {
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                val averageVelocity = flywheel.velocity

                // Monitor for big loss in velocity
                if ((lastVelocity - averageVelocity) > 50.0 || timer.seconds() > timeout) {
                    p.put("Current Speed", averageVelocity)
                    flywheel.velocity = 0.0
                    return false // big drop & all done, or timed out
                }

                lastVelocity = averageVelocity
                p.put("Current Speed", averageVelocity)
                return true // keep monitoring
            }
        }
    }

    fun waitForSpunUp(distance: LaunchDistance): Action {
        return Action {
            val averageVelocity = flywheel.velocity
            if (abs(averageVelocity - distance.velocity) < 25) {
                false
            } else true
        }
    }

    fun compositionLaunch(distance: LaunchDistance): Action {
        return ParallelAction(
            spinUpUntilLaunched(distance),
            SequentialAction(
                waitForSpunUp(distance),
                launch(),
            )
        )
    }

    fun launchAllHeld(distance: LaunchDistance): Action {
        val numArtifacts = artifacts.artifactsHeld

        if (numArtifacts == 0) return InstantAction {}

        if (numArtifacts == 1) return compositionLaunch(distance)

        if (numArtifacts == 2) return SequentialAction(
            compositionLaunch(distance),
            SleepAction(0.5),
            compositionLaunch(distance),
        )

        if (numArtifacts == 3) return SequentialAction(
            compositionLaunch(distance),
            SleepAction(0.5),
            compositionLaunch(distance),
            SleepAction(0.5),
            compositionLaunch(distance),
        )

        RobotLog.ee("OuttakeV2", "Impossible Case! Aaa!")
        return InstantAction {} // impossible case
    }

    fun intakeUntilDetected(): Action {
        TODO()
        return object : Action {
            private var firstRun = true
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    toggleIntake()
                    timer.reset()
                    firstRun = false
                }

                // When this has one, we need it to be rotated up
                val detectedLowerArtifact = detectArtifact(intakeColorSensor.normalizedColors)

                if (detectedLowerArtifact != ArtifactColors.NONE || timer.seconds() > 1.5) {
                    artifacts.intake(detectedLowerArtifact)

                    toggleIntake()
                    return false
                }
                return true
            }
        }
    }
    // endregion

    private class ArtifactData {
        val artifacts: Queue<ArtifactColors> = LinkedList()

        fun intake(color: ArtifactColors) {
            artifacts.add(color)
        }

        fun launched() {
            artifacts.poll()
        }

        fun autoArtifactPreloads() {
            artifacts.addAll(arrayOf(
                ArtifactColors.PURPLE,
                ArtifactColors.GREEN,
                ArtifactColors.PURPLE
            ))
        }

        val artifactsHeld: Int
            get() = artifacts.count()
    }

    // region Utility Functions
    private fun detectArtifact(data: NormalizedRGBA): ArtifactColors {
        return if (data.blue > 0.002 && data.blue > data.green) {
            ArtifactColors.PURPLE
        } else if (data.green > 0.002 && data.green > data.blue && data.green > data.red) {
            ArtifactColors.GREEN
        } else {
            ArtifactColors.NONE
        }
    }

    private fun getDepotTag(): Pose3D? {
        if (limelight == null) return null

        val tags = limelight.poll()

        val tag = (tags?.find { // Pose of the tag relative to camera
            if (teamColor == TeamColor.RED) { // select tag on depot
                it.fiducialId == 24
            } else {
                it.fiducialId == 20
            }
        }?.targetPoseCameraSpace) ?: return null // return if not valid or found

        return tag
    }
    // endregion

    // region Enums
    enum class LaunchDistance(val velocity: Double) {
        FAR(1500.0),
        CLOSE(1500.0),
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

    enum class LeftKickerPosition(val pos: Double) {
        KICK(0.25),
        NOT_KICK(0.8),
    }
    enum class RightKickerPosition(val pos: Double) {
        KICK(0.75),
        NOT_KICK(0.27),
    }
    // endregion

    companion object : HardwareMechanismSingletonManager<OuttakeV3>(::OuttakeV3)
}