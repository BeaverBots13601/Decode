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
import org.firstinspires.ftc.teamcode.misc.DualMotorPIDVelocityController
import org.firstinspires.ftc.teamcode.misc.GoBildaRGBIndicatorDriver
import org.firstinspires.ftc.teamcode.misc.GoBildaRGBIndicatorDriver.Color
import org.firstinspires.ftc.teamcode.misc.PoseKt
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.abs

class OuttakeV4 private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    // region Hardware
    // LEDs
    private val leftIndicatorLED  = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("leftIndicatorLED"))
    private val rightIndicatorLED = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("rightIndicatorLED"))

    // Flywheels
    private val flywheel = DualMotorPIDVelocityController(
        createDefaultMotor(hardwareMap, "flywheel").apply {
            direction = DcMotorSimple.Direction.REVERSE
        },
        createDefaultMotor(hardwareMap, "counterspin"),
        0.0175,
        0.000002,
        0.00001,
        telemetry,
    )
    // flywheel needs to be reversed

    // Kicker
    private val kicker = hardwareMap.servo.get("kicker").apply {
        position = KickerPosition.NOT_KICK.pos
    }

    // Intake
    private val intakeMotor = createDefaultMotor(hardwareMap, "intakeMotor").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val leftIntakePropBar = hardwareMap.servo.get("leftIntakePropBar").apply {
        position = LeftPropBarPosition.DOWN.pos
    }
    private val rightIntakePropBar = hardwareMap.servo.get("rightIntakePropBar").apply {
        position = RightPropBarPosition.DOWN.pos
    }

    // Color Sensors
    private val leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "leftIntakeColorSensor")
    private val rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightIntakeColorSensor")

    // Turntable
    val turntable = AxonDriver(
        hardwareMap,
        "turntableAxon",
        "turntableEncoder",
        0.0,//0.008,
        0.0,//0.096,
        0.0,//0.00044, // todo: tune
        // 0.04 = ku, 1/6 = tu   ??????
        telemetry,
        -2.0,
    )

    // Spindexer
    val spindexer = AxonDriver(
        hardwareMap,
        "spindexerAxon",
        "spindexerEncoder",
        0.002,
        0.012,
        0.00022,
        // 0.01 = ku, 1/3 = tu
        telemetry,
    ).apply {
        targetPosition = 0.0
    }

    // Limelight
    private val limelight = LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(initData), telemetry)

    // endregion

    override fun start() {}

    // Config info
    private val teamColor = initData.teamColor

    // State
    private val artifacts = ArtifactData()
    private var currentLaunchDistance = LaunchDistance.CLOSE
    private var turntableLocked = false
    private var awaitingLaunch = ArtifactColors.NONE

    // Emergency Mode Stuff
    private var leftRed = true
    private val timer = ElapsedTime()
    private var delta = 0.0

    override fun run(data: RunData) {
        // !!! EMERGENCY !!!
        if (data.currentGamepadTwo.ps) {
            if (data.currentGamepadTwo.dpad_down) {
                flywheel.setVelocity(-1000000.0)
            } else if (data.currentGamepadTwo.dpad_up) {
                flywheel.setVelocity(1000000.0)
            } else if (data.currentGamepadTwo.leftBumperWasPressed()) {
                runBlocking(launch(true))
            } else if (data.currentGamepadTwo.rightBumperWasPressed()) {

            }

            spindexer.overridePower = if (data.currentGamepadTwo.dpad_right) {
                0.2
            } else if (data.currentGamepadTwo.dpad_left) {
                -0.2
            } else 0.0

            delta += timer.seconds() - delta
            if (delta > 0.75) {
                leftIndicatorLED.color = if (leftRed) Color.RED else Color.BLUE
                rightIndicatorLED.color = if (leftRed) Color.BLUE else Color.RED
                leftRed = !leftRed
                delta = 0.0
                timer.reset()
            }
            return
        } else {
            spindexer.overridePower = null
        }

        // Detect intake artifact
        val leftColors  = leftIntakeColorSensor.normalizedColors
        val rightColors = rightIntakeColorSensor.normalizedColors
        val detectedArtifact = detectArtifact(leftColors, rightColors)
        telemetry.addData("Detected Artifact", detectedArtifact)

        // region Spindexer Controls
        var spindexerMoving = abs(spindexer.error) > 10

        if (detectedArtifact != ArtifactColors.NONE && !spindexerMoving) {
            artifacts.intake(detectedArtifact)
            val rotateTo = artifacts.rotate()
            rotateSpindexer(rotateTo)

            if (artifacts.intakeArtifact != ArtifactColors.NONE
                && artifacts.outtakeArtifact != ArtifactColors.NONE
                && artifacts.storageArtifact != ArtifactColors.NONE) {
                //intakeOff()
            }
        } else {

        }

        // update pid
        spindexer.targetPosition = spindexer.targetPosition

        artifacts.updateTelemetry(telemetry)

        spindexerMoving = abs(spindexer.error) > 10
        telemetry.addData("Spindexer Moving", spindexerMoving)
        // endregion

        // Intaking controls
        if (data.currentGamepadTwo.squareWasPressed()) {
            toggleIntake()
        } else if (data.currentGamepadOne.squareWasPressed()) {
            toggleIntake()
        } else if (data.currentGamepadTwo.circleWasPressed() || data.currentGamepadOne.circleWasPressed()) {
            intakeReverse()
        }

        // region Setting launch distance
        if (data.currentGamepadTwo.dpadUpWasPressed()) {
            currentLaunchDistance = LaunchDistance.FAR
        }

        if (data.currentGamepadTwo.dpadRightWasPressed()) {
            //currentLaunchDistance = LaunchDistance.CLOSE_PEAK
        }

        if (data.currentGamepadTwo.dpadDownWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE
        }

        if (data.currentGamepadTwo.dpadLeftWasPressed()) {
            //currentLaunchDistance = LaunchDistance.CLOSE_FAR
        }
        // endregion

        setLEDs()

        if (data.currentGamepadTwo.triangleWasPressed()) { awaitingLaunch = ArtifactColors.GREEN }

        if (data.currentGamepadTwo.crossWasPressed()) { awaitingLaunch = ArtifactColors.PURPLE }

        telemetry.addData("Awaiting Launch", awaitingLaunch)

        // Fire! when a launch is queued and we aren't rotating
        if (!spindexerMoving && awaitingLaunch != ArtifactColors.NONE) {
            val launchArtifact = artifacts.color(awaitingLaunch)
            if (launchArtifact == Position.OUTTAKE) { // in right spot, launch!
                // stop motors
                DriveTrainKt.getInstance()
                    ?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), RunMode.RUN_USING_ENCODER)
                runBlocking(launch())
                awaitingLaunch = ArtifactColors.NONE
            } else if (launchArtifact != Position.NONE) { // In wrong spot
                // Queue for rotation and then wait for wheel to move
                // Don't reset awaitingLaunch: we want this to happen next time
                val position = artifacts.rotate(awaitingLaunch)
                rotateSpindexer(position)
            } else {
                // don't hold; reset; probably done in error
                awaitingLaunch = ArtifactColors.NONE
            }
        }

        // region Flywheels (on trigger)
        flywheel.setVelocity(if (data.currentGamepadTwo.right_trigger < 0.5) {
            telemetry.addData("Target Velocity (tps)", 0.0)
            null // disable control
        } else {
            val vel = if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity
            telemetry.addData("Target Velocity (tps)", vel)
            vel
        })

        telemetry.addData("Current Launch Mode", currentLaunchDistance)
        telemetry.addData("Current Velocity (tps)", flywheel.velocity)
        // endregion

        // region Turret management
        var power = 0.0
        if (data.currentGamepadTwo.left_bumper) {
            power = 0.2
        } else if (data.currentGamepadTwo.right_bumper) {
            power = -0.2
        }

        if (data.currentGamepadTwo.optionsWasPressed()) {
            turntable.reset()
        }

        if (data.currentGamepadTwo.psWasPressed()) {
            turntableLocked = !turntableLocked
        }
        telemetry.addData("Turntable Auto-gimballing Disabled", turntableLocked)

        if (turntableLocked || power != 0.0) {
            turntable.overridePower = power
        } else {
            turntable.overridePower = null // let the control loop below handle it

            val delta = calculateTagDelta()

            // rotate
            rotateTurretByDelta(delta)
        }
        // endregion
    }

    override fun stop() {
        turntable.cleanUp()
        spindexer.cleanUp()
    }

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    // region Hardware Functions
    private var intakeActive = false
    fun toggleIntake() {
        if (intakeActive) {
            intakeMotor.power = 0.0
            intakeActive = false
        } else {
            intakeMotor.power = 0.8
            intakeActive = true
        }
    }

    fun intakeOff() {
        intakeMotor.power = 0.0
        intakeActive = false
    }

    fun intakeOn() {
        intakeMotor.power = 0.8
        intakeActive = true
    }

    fun intakeReverse() {
        intakeMotor.power = -0.8
        intakeActive = true
    }

    fun flywheelSpunUp(): Boolean {
        val velocity = flywheel.velocity
        val target = if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity

        return abs(velocity - target) < 100
    }

    fun spindexerAtTarget(): Boolean {
        val target = spindexer.targetPosition ?: return true

        return abs(spindexer.position - target) < 10
    }

    /**
     * Physically rotate the current artifacts.
     * @param position Position that the current intake artifact will end up in.
     */
    fun rotateSpindexer(position: Position) {
        if (position == Position.OUTTAKE) {
            spindexer.targetPosition = spindexer.targetPosition?.minus(SPINDEXER_ROTATE_DEGREES * 2)
        } else if (position == Position.STORAGE) {
            spindexer.targetPosition = spindexer.targetPosition?.minus(SPINDEXER_ROTATE_DEGREES)
        }
    }

    fun rotateSpindexerForLaunch() {
        spindexer.targetPosition = spindexer.targetPosition?.plus(SPINDEXER_ROTATE_DEGREES)
    }

    private fun setLEDs() {
        when (artifacts.outtakeArtifact) {
            ArtifactColors.PURPLE -> {
                leftIndicatorLED.color = Color.VIOLET
            }
            ArtifactColors.GREEN -> {
                leftIndicatorLED.color = Color.GREEN
            }
            else -> {
                leftIndicatorLED.color = Color.OFF
            }
        }

        when (artifacts.storageArtifact) {
            ArtifactColors.PURPLE -> {
                rightIndicatorLED.color = Color.VIOLET
            }
            ArtifactColors.GREEN -> {
                rightIndicatorLED.color = Color.GREEN
            }
            else -> {
                rightIndicatorLED.color = Color.OFF
            }
        }

        return
    }

    /**
     * Rotates the turret a specific number of degrees from our current location. Please call
     * this function even if the delta is 0.0 as this updates the control loop for our servo.
     *
     * @param delta The delta, in degrees, from current forward on the robot. Negative is left; Positive is right.
     */
    fun rotateTurretByDelta(delta: Double?) {
        if (delta == null) {
            turntable.targetPosition = null
            return
        }
        // this may underflow or overflow so we're always in [-180, 180]
        var target = PoseKt.normalizeAngleDeg(turntable.position + delta)

        // also clamp to [-110, 130] to meet hardware requirements (too much overshoot allowance?)
        target = clamp(target, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
//        target = clamp(target, -135.0, 135.0)

        turntable.targetPosition = target
    }
    // endregion

    // region Utility Functions
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

    private fun getMotifTag(): Int? {
        if (limelight == null) return null

        val tags = limelight.poll()

        val tag = (tags?.find { // Pose of the tag relative to camera
            it.fiducialId == 23 || it.fiducialId == 22 || it.fiducialId == 21 // select tag on depot
        }?.fiducialId) ?: return null // return if not valid or found

        return tag
    }

    fun calculateTagDelta(): Double? {
        // get tags
        val depotTag = getDepotTag()
        val motifTag = getMotifTag() // if can see, must rotate

        val delta = if (limelight == null) { // find delta
            null // Not attached; do nothing
        } else if (depotTag != null) {
            -depotTag.position.x * 90 // negative! because positive is left by default
        } else if (motifTag != null) {
            // rotate to find the tag
            if (teamColor == TeamColor.BLUE) 20.0 else -20.0
        } else 0.0

        telemetry.addData("Delta", delta)
        return delta
    }
    // endregion

    fun loadAutoArtifacts() = artifacts.autoArtifactPreloads()

    // region Roadrunner Actions
    /**
     * Launch smartly, deciding whether a kick or a intake launch is required. Assumes the artifact
     * to launch is in the Outtake position.
     */
    fun launch(kick: Boolean? = null): Action {
        return when(kick) {
            true -> {
                SequentialAction(
                    InstantAction { kicker.position = KickerPosition.KICK.pos },
                    SleepAction(0.75),
                    InstantAction { kicker.position = KickerPosition.NOT_KICK.pos },
                    InstantAction { artifacts.launched() },
                )
            }
            false -> {
                SequentialAction(
                    // pushes artifact into launch
                    InstantAction { rotateSpindexerForLaunch() },
                    waitUntilLaunched(1.0),
                    InstantAction { artifacts.launched() },
                )
            }
            null -> {
                // Kick if holding only one
                return launch(artifacts.artifactsHeld == 1)
            }
        }
    }

    /**
     * Spins up the flywheel until launched, or until *timeout* seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(distance: LaunchDistance, timeout: Double): Action { // todo: rework
        return object : Action {
            private var spunUp = false
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                p.put("Spun Up?", spunUp)

                flywheel.setVelocity(distance.velocity)

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
                    flywheel.setVelocity(null)
                    return false // big drop, all done, or timed out
                }

                lastVelocity = averageVelocity
                p.put("Current Speed", averageVelocity)
                return true // keep monitoring
            }
        }
    }

    /**
     * Spins up the flywheel until launched, or until 1.0 seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(distance: LaunchDistance) = spinUpUntilLaunched(distance, 1.0)

    /**
     * Watches the flywheel until launched, or until 2.5 seconds have passed.
     */
    fun waitUntilLaunched() = waitUntilLaunched(2.5)

    /**
     * Watches the flywheel until launched, or until *timeout* seconds have passed.
     */
    fun waitUntilLaunched(timeout: Double): Action { // todo: rework
        return object : Action {
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                val averageVelocity = flywheel.velocity

                // Monitor for big loss in velocity
                if ((lastVelocity - averageVelocity) > 50.0 || timer.seconds() > timeout) {
                    p.put("Current Speed", averageVelocity)
                    flywheel.setVelocity(null)
                    return false // big drop & all done, or timed out
                }

                lastVelocity = averageVelocity
                p.put("Current Speed", averageVelocity)
                return true // keep monitoring
            }
        }
    }

    fun waitForSpunUp(distance: LaunchDistance): Action { // todo: rework
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
        val numArtifacts = artifacts.artifactsHeld

        if (numArtifacts == 0) return InstantAction {}

        if (numArtifacts == 1) return SequentialAction(
            SleepAction(0.5),
            compositionLaunch(distance, true),
        )

        if (numArtifacts == 2) return SequentialAction(
            SleepAction(0.5),
            compositionLaunch(distance, false),
            SleepAction(0.5),
            compositionLaunch(distance, true),
        )

        if (numArtifacts == 3) return SequentialAction(
            SleepAction(0.5),
            compositionLaunch(distance, false),
            SleepAction(0.5),
            compositionLaunch(distance, false),
            SleepAction(0.5),
            compositionLaunch(distance, true),
        )

        RobotLog.ee("OuttakeV4", "Impossible Case! Aaa!")
        return InstantAction {} // impossible case
    }

    fun intakeUntilDetectedOrTimeout(): Action {
        return object : Action {
            private var firstRun = true
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    intakeActive = false
                    intakeOn()
                    timer.reset()
                    firstRun = false
                }

                val detectedArtifact = detectArtifact(leftIntakeColorSensor.normalizedColors, rightIntakeColorSensor.normalizedColors)

                if ((artifacts.intakeArtifact == ArtifactColors.NONE && detectedArtifact != ArtifactColors.NONE)
                    || timer.seconds() > 1.5) {
                    artifacts.intake(detectedArtifact)
                    intakeOff()
                    return false
                }
                return true
            }
        }
    }
    // endregion

    // region Enums
    enum class LaunchDistance(val velocity: Double) {
        FAR(1150.0),
        CLOSE(750.0),
    }

    enum class ArtifactColors {
        PURPLE,
        GREEN,
        NONE,
    }

    enum class Motif(val first: ArtifactColors, val second: ArtifactColors, val third: ArtifactColors) {
        PURPLE_PURPLE_GREEN(ArtifactColors.PURPLE, ArtifactColors.PURPLE, ArtifactColors.GREEN),
        PURPLE_GREEN_PURPLE(ArtifactColors.PURPLE, ArtifactColors.GREEN, ArtifactColors.PURPLE),
        GREEN_PURPLE_PURPLE(ArtifactColors.GREEN, ArtifactColors.PURPLE, ArtifactColors.PURPLE),
    }

    enum class Position {
        OUTTAKE,
        STORAGE,
        INTAKE,
        NONE,
    }

    enum class KickerPosition(val pos: Double) {
        KICK(0.85),
        NOT_KICK(0.08),
    }

    enum class LeftPropBarPosition(val pos: Double) {
        PROP(0.60),
        DOWN(0.82),
    }

    enum class RightPropBarPosition(val pos: Double) {
        PROP(0.65),
        DOWN(0.43),
    }
    // endregion

    class ArtifactData {
        fun intake(color: ArtifactColors) {
            intakeArtifact = color
        }

        /**
         * Automatically rotate the current artifacts to the most optimal positioning.
         * @param wantedColor The color that the flywheel is seeking.
         * @return Position that the current lower artifact should end up in.
         */
        fun rotate(wantedColor: ArtifactColors = ArtifactColors.NONE): Position {
            // fixme: bug situation
            //   one in storage, one in intake
            //   "go outtake" -> pushed into intake and outtake instead of outtake and storage

            val colorPos = color(wantedColor) // where is the color we want?

            // we are looking for an artifact and hold it
            if (wantedColor != ArtifactColors.NONE && colorPos != Position.NONE) {
                if (colorPos == Position.OUTTAKE) {
                    return Position.INTAKE // don't move: in perfect spot
                }

                if (colorPos == Position.STORAGE) {
                    // need to rotate lower to right
                    rotate(Position.STORAGE)
                    return Position.STORAGE
                }

                // artifact we want is in intake
                rotate(Position.OUTTAKE)
                return Position.OUTTAKE
            }

            // we aren't looking for an artifact or we don't hold it
            if (intakeArtifact != ArtifactColors.NONE) {
                // always want a 'NONE' in the intake; we should rotate such that both of our backs are filled
                if (outtakeArtifact == ArtifactColors.NONE) {
                    rotate(Position.STORAGE)
                    return Position.STORAGE
                    // We rotate to RIGHT not LEFT so that if there is an artifact in the RIGHT
                    // it is kept in the rear not brought to intake
                }

                if (storageArtifact == ArtifactColors.NONE) {
                    rotate(Position.OUTTAKE)
                    return Position.OUTTAKE
                    // above but reversed
                }

                // nowhere to put the intakeArtifact
            }

            // todo optimal storage: purple in left (more likely to want)

            return Position.INTAKE // don't move: nowhere to move to
        }

        /**
         * Manually rotate the current artifacts.
         * @param position Position that the current lower artifact will end up in.
         */
        fun rotate(position: Position) {
            if (position == Position.OUTTAKE) {
                val temp = outtakeArtifact
                outtakeArtifact = intakeArtifact
                intakeArtifact = storageArtifact
                storageArtifact = temp
            } else if (position == Position.STORAGE) {
                val temp = outtakeArtifact
                outtakeArtifact = storageArtifact
                storageArtifact = intakeArtifact
                intakeArtifact = temp
            }
        }

        /**
         * @param color The color of the artifact to search for.
         * @return The position of the artifact that should be shot.
         */
        fun color(color: ArtifactColors): Position {
            if (outtakeArtifact == color) {
                return Position.OUTTAKE
            }

            if (storageArtifact == color) {
                return Position.STORAGE
            }

            if (intakeArtifact == color) {
                return Position.INTAKE
            }

            return Position.NONE // don't hold this color
        }

        fun launched() {
            outtakeArtifact = ArtifactColors.NONE
            rotate(Position.OUTTAKE)
        }

        fun autoArtifactPreloads() {
            intakeArtifact = ArtifactColors.PURPLE
            outtakeArtifact = ArtifactColors.GREEN
            storageArtifact = ArtifactColors.PURPLE
        }

        fun updateTelemetry(telemetry: Telemetry) {
            telemetry.addData("Outtake Artifact", outtakeArtifact)
            telemetry.addData("Storage Artifact", storageArtifact)
            telemetry.addData("Intake Artifact", intakeArtifact)
        }

        var outtakeArtifact: ArtifactColors = ArtifactColors.NONE
            private set

        var storageArtifact: ArtifactColors = ArtifactColors.NONE
            private set

        var intakeArtifact: ArtifactColors = ArtifactColors.NONE
            private set

        val artifactsHeld: Int
            get() {
                var num = 0
                if (outtakeArtifact  != ArtifactColors.NONE) { num++ }
                if (storageArtifact != ArtifactColors.NONE) { num++ }
                if (intakeArtifact != ArtifactColors.NONE) { num++ }
                return num
            }
    }

    companion object : HardwareMechanismSingletonManager<OuttakeV4>(::OuttakeV4) {
        // region Static Utility Functions
        fun detectArtifact(data1: NormalizedRGBA, data2: NormalizedRGBA): ArtifactColors {
            val artifact1 = detectArtifact(data1)
            val artifact2 = detectArtifact(data2)

            if (artifact1 == artifact2) {
                return artifact1 // in agreement
            }

            if ((artifact1 == ArtifactColors.NONE && artifact2 != ArtifactColors.NONE)
                || (artifact1 != ArtifactColors.NONE && artifact2 == ArtifactColors.NONE)) {
                // use the one that detects
                return if (artifact1 == ArtifactColors.NONE) artifact2 else artifact1
            }

            // uh oh, there's a conflict... hope for a better reading next time
            return ArtifactColors.NONE
        }

        private fun detectArtifact(data: NormalizedRGBA): ArtifactColors {
            return if (data.blue > 0.003 && data.blue > data.green) {
                ArtifactColors.PURPLE
            } else if (data.green > 0.003 && data.green > data.blue && data.green > data.red) {
                ArtifactColors.GREEN
            } else {
                ArtifactColors.NONE
            }
        }
        // endregion

        @JvmField var CUSTOM: Double = 0.0
        @JvmField var SPINDEXER_ROTATE_DEGREES = 120.0
    }
}