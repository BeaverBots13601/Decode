package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
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
import org.firstinspires.ftc.teamcode.misc.PIDVelocityController
import org.firstinspires.ftc.teamcode.misc.PoseKt
import org.firstinspires.ftc.teamcode.misc.ArtifactColors
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.abs

@Config
class OuttakeV3 private constructor(hardwareMap: HardwareMap, initData: InitData, val telemetry: Telemetry) : HardwareMechanismKt() {
    // LEDs
    private val leftIndicatorLED  = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("leftIndicatorLED"))
    private val rightIndicatorLED = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("rightIndicatorLED"))

    // Flywheel
    val flywheel = PIDVelocityController(
        createDefaultMotor(hardwareMap, "flywheel").apply {
            direction = DcMotorSimple.Direction.REVERSE
        },
        0.0175,
        0.000002,
        0.00001,
        telemetry,
    )

    // Color Sensor
    private val intakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeColorSensor")

    private val locker = hardwareMap.servo.get("locker")

    // Intake
    private val intakeMotor = createDefaultMotor(hardwareMap, "intakeMotor").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    // Turntable
    val gearRatio = -11.0 / 3.0
    val turntableAxon = AxonDriver(
        hardwareMap,
        "turntableAxon",
        "turntableEncoder",
        -0.001,
        0.0,
        0.0000005,
        telemetry,
        gearRatio,
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
        locker.position = LockerPosition.LOCK.pos
    }

    override fun start() = turntableAxon.start()


    // Config info
    private val teamColor = initData.teamColor

    // State
    private var currentLaunchDistance = LaunchDistance.CLOSE
    private val artifacts = ArtifactData()
    override fun run(data: RunData) {
        // Artifact detection
        val colors = intakeColorSensor.normalizedColors
        telemetry.addData("Raw Blue", colors.blue)
        telemetry.addData("Raw Green", colors.green)
        telemetry.addData("Raw Red", colors.red)
        val detectedArtifact = detectArtifact(colors)
        telemetry.addData("Detected Artifact", detectedArtifact)

        if (artifacts.midArtifact != ArtifactColors.NONE && detectedArtifact == ArtifactColors.NONE) {
            // Assume a artifact has moved to top
            artifacts.topArtifact = artifacts.midArtifact
        }

        artifacts.midArtifact = detectedArtifact

        telemetry.addData("Top Artifact", artifacts.topArtifact)
        telemetry.addData("Mid Artifact", artifacts.midArtifact)

        // Intaking
        if (data.currentGamepadTwo.squareWasPressed()) {
            toggleIntake()
        } else if (data.currentGamepadOne.squareWasPressed()) {
            toggleIntake()
        } else if (data.currentGamepadTwo.circleWasPressed() || data.currentGamepadOne.circleWasPressed()) {
            intakeReverse()
        }

        // Setting launch distance
        if (data.currentGamepadTwo.dpadUpWasPressed()) {
            currentLaunchDistance = LaunchDistance.FAR
        }

        if (data.currentGamepadTwo.dpadRightWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE_PEAK
        }

        if (data.currentGamepadTwo.dpadDownWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE
        }

        if (data.currentGamepadTwo.dpadLeftWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE_FAR
        }

        setLEDs(detectedArtifact)

        // Fire!
        if (data.currentGamepadTwo.crossWasPressed()) {
            DriveTrainKt.getInstance()
                ?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), RunMode.RUN_USING_ENCODER)
            runBlocking(launch())
        }

        // Flywheels (on trigger)
        flywheel.setVelocity(if (data.currentGamepadTwo.right_trigger < 0.5) {
            telemetry.addData("Target Velocity (tps)", 0.0)
            null // disable control
        } else {
            telemetry.addData("Target Velocity (tps)", currentLaunchDistance.velocity)
            if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity
        })

        // Lock/Unlock
        val flywheelSpunUp = flywheelSpunUp()

        if (data.currentGamepadTwo.share/* || flywheelSpunUp*/) { // open if held or span up
            locker.position = LockerPosition.NOT_LOCK.pos
        } else { // otherwise locked
            locker.position = LockerPosition.LOCK.pos
        }

        telemetry.addData("Flywheel Spun Up?", flywheelSpunUp)
        telemetry.addData("Current Launch Mode", currentLaunchDistance)
        telemetry.addData("Current Velocity (tps)", flywheel.velocity)

        // Turret angling
        var power = 0.0
        if (data.currentGamepadTwo.left_bumper) {
            power = 0.2
        } else if (data.currentGamepadTwo.right_bumper) {
            power = -0.2
        }

        // Turret reset
        if (data.currentGamepadTwo.optionsWasPressed()) {
            turntableAxon.reset()
        }

        if (data.currentGamepadTwo.psWasPressed()) {
            turntableLocked = !turntableLocked
        }

        telemetry.addData("Turntable Auto-gimballing Disabled", turntableLocked)

        if (turntableLocked || power != 0.0) {
            turntableAxon.overridePower = power
        } else {
            turntableAxon.overridePower = null // let the control loop below handle it

            val delta = calculateTagDelta()

            // rotate
            rotateTurretByDelta(delta)
        }

        turntableAxon.update()
    }
    private var turntableLocked = false

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
            transfer.power = 0.0
            intakeActive = false
        } else {
            intakeMotor.power = 0.8
            transfer.power = 1.0
            intakeActive = true
        }
    }

    fun intakeOff() {
        intakeMotor.power = 0.0
        transfer.power = 0.0
        intakeActive = false
    }

    fun intakeOn() {
        intakeMotor.power = 0.8
        transfer.power = 1.0
        intakeActive = true
    }

    fun intakeReverse() {
        intakeMotor.power = -0.8
        transfer.power = -1.0
        intakeActive = true
    }

    fun flywheelSpunUp(): Boolean {
        val velocity = flywheel.velocity
        val target = if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity

        return if (abs(velocity - target) < 100) {
            true
        } else false
    }

    private fun setLEDs(display: ArtifactColors) {
        // Left indicates velocity
        if(flywheelSpunUp()) {
            leftIndicatorLED.color = Color.GREEN
        } else {
            leftIndicatorLED.color = Color.OFF // off
        }

        // Right indicates expected color
        when (display) {
            ArtifactColors.GREEN -> {
                rightIndicatorLED.color = Color.GREEN
            }
            ArtifactColors.PURPLE -> {
                rightIndicatorLED.color = Color.VIOLET
            }
            ArtifactColors.NONE -> {
                rightIndicatorLED.color = Color.OFF
            }
        }
    }

    /**
     * Rotates the turret a specific number of degrees from our current location. Please call
     * this function even if the delta is 0.0 as this updates the control loop for our servo.
     *
     * @param delta The delta, in degrees, from current forward on the robot. Negative is left; Positive is right.
     */
    fun rotateTurretByDelta(delta: Double?) {
        if (delta == null) {
            turntableAxon.targetPosition = null
            return
        }
        // this may underflow or overflow so we're always in [-180, 180]
        var target = PoseKt.normalizeAngleDeg(turntableAxon.position + delta)

        // also clamp to [-110, 130] to meet hardware requirements (too much overshoot allowance?)
        target = clamp(target, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
//        target = clamp(target, -135.0 * -gearRatio, 135.0 * -gearRatio)

        turntableAxon.targetPosition = target
    }
    // endregion

    // region Roadrunner Actions
    /**
     * Launch smartly, deciding whether a kick or a intake launch is required
     */
    fun launch(kick: Boolean? = null): Action {
        return when(kick) {
            true -> {
                SequentialAction(
                    InstantAction { leftKicker.position = LeftKickerPosition.KICK.pos },
                    InstantAction { rightKicker.position = RightKickerPosition.KICK.pos },
                    SleepAction(0.75),
                    InstantAction { leftKicker.position = LeftKickerPosition.NOT_KICK.pos },
                    InstantAction { rightKicker.position = RightKickerPosition.NOT_KICK.pos },
                )
            }
            false -> {
                SequentialAction(
                    // bring artifact to top (launches one if in top)
                    InstantAction { intakeOn() },
                    waitUntilLaunched(1.0),
                    //InstantAction { intakeOff() },
                    // fixme: launches two
                )
            }
            null -> {
                val middle = detectArtifact(intakeColorSensor.normalizedColors)
                if (middle == ArtifactColors.NONE) { // need to kick
                    SequentialAction(
                        InstantAction { leftKicker.position = LeftKickerPosition.KICK.pos },
                        InstantAction { rightKicker.position = RightKickerPosition.KICK.pos },
                        SleepAction(0.75),
                        InstantAction { leftKicker.position = LeftKickerPosition.NOT_KICK.pos },
                        InstantAction { rightKicker.position = RightKickerPosition.NOT_KICK.pos },
                    )
                } else { // artifact will launch when we use the intake
                    SequentialAction(
                        // bring artifact to top (launches one if in top)
                        InstantAction { intakeOn() },
                        waitUntilLaunched(1.0),
                        //InstantAction { intakeOff() },
                        // fixme: launches two
                    )
                }
            }
        }
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

                //flywheel.setVelocity(distance.velocity)

                val averageVelocity = flywheel.velocity
                if (!spunUp && abs(abs(averageVelocity) - distance.velocity) < 100) {
                    spunUp = true
                    timer.reset()
                } else if (!spunUp) {
                    p.put("Current Speed", averageVelocity)
                    return true // keep spinning up
                }

                // We are spun up, now monitor for big loss in velocity
                if ((lastVelocity - averageVelocity) > 50.0 || timer.seconds() > timeout) {
                    p.put("Current Speed", averageVelocity)
                    //flywheel.setVelocity(null)
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
    fun spinUpUntilLaunched(distance: LaunchDistance) = spinUpUntilLaunched(distance, 1.0)

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
                    //flywheel.setVelocity(null)
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
            if (abs(averageVelocity - distance.velocity) < 100) {
                false
            } else true
        }
    }

    fun compositionLaunch(distance: LaunchDistance, kick: Boolean? = null): Action {
        return ParallelAction(
            spinUpUntilLaunched(distance),
            SequentialAction(
                waitForSpunUp(distance),
                launch(kick),
            )
        )
    }

    fun launchAllHeld(distance: LaunchDistance): Action {
        val numArtifacts = 3/*artifacts.artifactsHeld*/

        if (numArtifacts == 0) return InstantAction {}

        if (numArtifacts == 1) return SequentialAction(
            InstantAction { locker.position = LockerPosition.NOT_LOCK.pos },
            SleepAction(0.5),
            compositionLaunch(distance, true),
            InstantAction { locker.position = LockerPosition.LOCK.pos },
        )

        if (numArtifacts == 2) return SequentialAction(
            InstantAction { locker.position = LockerPosition.NOT_LOCK.pos },
            SleepAction(0.5),
            compositionLaunch(distance, false),
            SleepAction(0.5),
            compositionLaunch(distance, true),
            InstantAction { locker.position = LockerPosition.LOCK.pos },
        )

        if (numArtifacts == 3) return SequentialAction(
            InstantAction { locker.position = LockerPosition.NOT_LOCK.pos },
            SleepAction(0.5),
            compositionLaunch(distance, false),
            SleepAction(0.5),
            compositionLaunch(distance, false),
            SleepAction(0.5),
            compositionLaunch(distance, true),
            InstantAction { locker.position = LockerPosition.LOCK.pos },
        )

        RobotLog.ee("OuttakeV2", "Impossible Case! Aaa!")
        return InstantAction {} // impossible case
    }

    fun intakeUntilDetectedOrTimeout(): Action {
        return object : Action {
            private var firstRun = true
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    intakeActive = false
                    //toggleIntake()
                    intakeOn()
                    timer.reset()
                    firstRun = false
                }

                val detectedArtifact = detectArtifact(intakeColorSensor.normalizedColors)

                if ((artifacts.midArtifact == ArtifactColors.NONE && detectedArtifact != ArtifactColors.NONE)
                    || timer.seconds() > 1.5) {
                    artifacts.intake(detectedArtifact)
                    intakeOn()
                    return false
                }
                return true
            }
        }
    }
    // endregion

    private class ArtifactData {
        var topArtifact: ArtifactColors = ArtifactColors.NONE
        var midArtifact: ArtifactColors = ArtifactColors.NONE

        fun intake(color: ArtifactColors) {
            midArtifact = color // can't know anything about lower
        }

        fun launched() {
            topArtifact = ArtifactColors.NONE
        }

        fun autoArtifactPreloads() {
            topArtifact = ArtifactColors.PURPLE
            midArtifact = ArtifactColors.PURPLE
            // lowArtif = ArtifactColors.PURPLE
        }

        val artifactsHeld: Int
            get() {
                var i = 0
                if (topArtifact != ArtifactColors.NONE) i++
                if (midArtifact != ArtifactColors.NONE) i++
                // If we hold both we should assume we hold a third as well since we can't see
                if (i == 2) i++
                return i
            }
    }

    fun loadAutoArtifacts() = artifacts.autoArtifactPreloads()

    // region Utility Functions
    private fun detectArtifact(data: NormalizedRGBA): ArtifactColors {
        return if (data.blue > 0.001 && data.blue > data.green) {
            ArtifactColors.PURPLE
        } else if (data.green > 0.001 && data.green > data.blue && data.green > data.red) {
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

    fun calculateTagDelta(): Double? {
        // get tags
        val depotTag = getDepotTag()

        val delta = if (limelight == null) { // find delta
            null // Not attached; do nothing
        } else if (depotTag != null) {
            -depotTag.position.x * 90 // negative! because positive is left by default
        } else 0.0 // rotate to find the tag

        telemetry.addData("Delta", delta)
        return delta
    }
    // endregion

    // region Enums
    enum class LaunchDistance(val velocity: Double) {
        FAR(1150.0),
        CLOSE_FAR(1000.0),
        CLOSE_PEAK(900.0),
        CLOSE(750.0),
    }

    enum class LeftKickerPosition(val pos: Double) {
        KICK(0.25),
        NOT_KICK(0.8),
    }
    enum class RightKickerPosition(val pos: Double) {
        KICK(0.75),
        NOT_KICK(0.27),
    }

    enum class LockerPosition(val pos: Double) {
        LOCK(0.25),
        NOT_LOCK(0.55),
    }
    // endregion

    companion object : HardwareMechanismSingletonManager<OuttakeV3>(::OuttakeV3) {
        @JvmField var CUSTOM: Double = 0.0
    }
}