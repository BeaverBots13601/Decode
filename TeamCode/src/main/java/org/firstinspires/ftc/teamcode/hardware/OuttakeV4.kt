package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.clamp
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.acmerobotics.roadrunner.now
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.Servo
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
import org.firstinspires.ftc.teamcode.misc.ArtifactColors
import org.firstinspires.ftc.teamcode.misc.Motif
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.abs

@Config
class OuttakeV4 private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    // region Hardware
    // LEDs
    private val leftIndicatorLED  = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("leftIndicatorLED"))
    private val rightIndicatorLED = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("rightIndicatorLED"))

    // Flywheels
    val flywheel = DualMotorPIDVelocityController(
        createDefaultMotor(hardwareMap, "flywheel").apply {
            direction = DcMotorSimple.Direction.REVERSE
        },
        createDefaultMotor(hardwareMap, "counterspin"),
        0.0175,
        0.000002,
        0.00001,
        telemetry,
    )

    // Booster
    private val booster = createDefaultMotor(hardwareMap, "boosterMotor")

    // Intake
    private val intakeMotor = createDefaultMotor(hardwareMap, "intakeMotor").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val leftIntakePropBar = hardwareMap.servo.get("leftIntakePropBar").apply {
        position = 0.90 // calibrated
    }
    private val rightIntakePropBar = hardwareMap.servo.get("rightIntakePropBar").apply {
        position = 0.45 // calibrated
    }

    // Color Sensors
    private val leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "leftIntakeColorSensor")
    private val rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightIntakeColorSensor")

    // Turntable
    private val turntableAxon = runCatching {
        hardwareMap.servo.get("turntableAxon").apply {
            scaleRange(0.02, 0.93)
            position = 0.5 // Start centered
//            direction = Servo.Direction.REVERSE
        }
    }.getOrNull()

    private val turntable: AxonDriver? = null
//    runCatching {
//        if (turntableAxon != null) throw Error()
//        AxonDriver(
//            hardwareMap,
//            "turntableAxon",
//            "turntableEncoder",
//            0.0,//0.008,
//            0.0,//0.096,
//            0.0,//0.00044, // todo: tune
//            // 0.04 = ku, 1/6 = tu   ??????
//            telemetry,
//            -2.0,
//        )
//    }

    // Spindexer
    private val spindexer = AxonDriver(
        hardwareMap,
        "spindexerAxon",
        "spindexerEncoder",
        0.006, // 0.0066
        0.0005, // 0.0528
        0.00005, // 0.00018
        // ku = 0.011; tu = 1/4
        telemetry,
    ).apply {
        targetPosition = 0.0
    }

    // Limelight
    private val limelight = LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(initData), telemetry)

    // endregion

    override fun start() {
        spindexer.start()
        turntable?.start()
    }

    // Config info
    private val teamColor = initData.teamColor

    // State
    private val artifacts = ArtifactData()
    private var currentLaunchDistance = LaunchDistance.CLOSE_MID
    private var turntableLocked = false
    private var awaitingLaunch = ArtifactColors.NONE
    private var launchAction: Action? = null
    private var numPurple: Int? = null

    // region Emergency Mode Stuff
    private var leftRed = true
    private val timer = ElapsedTime()
    private var timeDelta = 0.0
    private var emergencyModeActive = false
    // endregion

    override fun run(data: RunData) {
        // update pid before the hijack so tracking is maintained
        spindexer.update()
        turntable?.update()

        telemetry.addData("Offset", currentOffset)

        // region !!! EMERGENCY !!!
        if (data.currentGamepadTwo.psWasPressed()) { emergencyModeActive = !emergencyModeActive }

        if (emergencyModeActive) {
            if (data.currentGamepadTwo.dpad_down) {
                flywheel.setVelocity(-1000000.0)
                boosterOn()
            } else if (data.currentGamepadTwo.dpad_up) {
                flywheel.setVelocity(1000000.0)
                boosterOn()
            } else if (data.currentGamepadTwo.dpadLeftWasPressed() || data.currentGamepadTwo.dpadRightWasPressed()) {
                runBlockingAndUpdateTeleOp(launch())
            } else {
                flywheel.setVelocity(null)
                boosterOff()
            }

            spindexer.overridePower = if (data.currentGamepadTwo.right_bumper) {
                0.2
            } else if (data.currentGamepadTwo.left_bumper) {
                -0.2
            } else 0.0

            timeDelta += timer.seconds() - timeDelta
            if (timeDelta > 0.75) {
                leftIndicatorLED.color = if (leftRed) Color.RED else Color.BLUE
                rightIndicatorLED.color = if (leftRed) Color.BLUE else Color.RED
                leftRed = !leftRed
                timeDelta = 0.0
                timer.reset()
            }
            return
        } else {
            spindexer.overridePower = null
        }
        // endregion

        // region Launching multiple artifacts
        if (numPurple != null) { // We're setting the order of artifacts for our launch action

            launchAction = launchAllHeld(currentLaunchDistance)
            numPurple = null
            return

            if (turntable != null) turntable.overridePower = null
            awaitingLaunch = ArtifactColors.NONE
            if (data.currentGamepadTwo.circleWasPressed()) {
                // End evaluation; we can guess the motif from here
                launchAction = when (numPurple) {
                    0 -> {
                        launchAllHeld(currentLaunchDistance, Motif.GREEN_PURPLE_PURPLE)
                    }
                    1 -> {
                        launchAllHeld(currentLaunchDistance, Motif.PURPLE_GREEN_PURPLE)
                    }
                    else -> /* 2 or more */ {
                        launchAllHeld(currentLaunchDistance, Motif.PURPLE_PURPLE_GREEN)
                    }
                }

                numPurple = null
            }

            if (data.currentGamepadTwo.crossWasPressed()) { numPurple = numPurple?.plus(1) }

            if (data.currentGamepadOne.squareWasPressed()) { numPurple = null }

            return
        } else if (launchAction != null) {
            val packet = TelemetryPacket()
            if (launchAction?.run(packet) == false) launchAction = null
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            return
        }
        // endregion

        // Intaking controls
        if (data.currentGamepadTwo.squareWasPressed() || data.currentGamepadOne.squareWasPressed()) {
            toggleIntake()
        } else if (data.currentGamepadTwo.left_trigger > 0.5 || data.currentGamepadOne.left_trigger > 0.5) {
            intakeReverse()
        }

        if (artifacts.artifactsHeld == 3) {
            //intakeOff()
            offsetSpindexer(Offset.STORAGE)
            // lift bars
        }

        // region Spindexer Command
        // Detect intake artifact
        val leftColors  = leftIntakeColorSensor.normalizedColors
        val rightColors = rightIntakeColorSensor.normalizedColors
        val detectedArtifact = detectArtifact(leftColors, rightColors)
        telemetry.addData("Detected Artifact", detectedArtifact)

        var spindexerMoving = abs(spindexer.error) > TOLERANCE_DEGREES

        if (detectedArtifact != ArtifactColors.NONE && !spindexerMoving) {
            if (artifacts.intakeArtifact == ArtifactColors.NONE) { // Just got one
                data.gamepadOneReference.rumble(200)
                data.gamepadTwoReference.rumble(200)
            }

            artifacts.intake(detectedArtifact)
            val rotateTo = artifacts.rotate()
            rotateSpindexer(rotateTo)
        }

        artifacts.updateTelemetry(telemetry)

        spindexerMoving = abs(spindexer.error) > TOLERANCE_DEGREES
        telemetry.addData("Spindexer Moving", spindexerMoving)
        // endregion

        // region Setting launch distance
        if (data.currentGamepadTwo.dpadUpWasPressed()) {
            currentLaunchDistance = LaunchDistance.FAR
        }

        if (data.currentGamepadTwo.dpadRightWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE_PEAK
        }

        if (data.currentGamepadTwo.dpadDownWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE_MID
        }

        if (data.currentGamepadTwo.dpadLeftWasPressed()) {
            currentLaunchDistance = LaunchDistance.CLOSE_FAR
        }
        // endregion

        setLEDs()

        // triangle launch all
        if (data.currentGamepadTwo.triangleWasPressed()) { numPurple = 0; return }

        // single launch
        if (data.currentGamepadTwo.circleWasPressed()) { awaitingLaunch = ArtifactColors.GREEN }

        if (data.currentGamepadTwo.crossWasPressed()) { awaitingLaunch = ArtifactColors.PURPLE }

        telemetry.addData("Awaiting Launch", awaitingLaunch)

        // region Fire!
        // when a single launch is queued and we aren't rotating
        if (!spindexerMoving && awaitingLaunch != ArtifactColors.NONE) {
            val launchArtifact = artifacts.color(awaitingLaunch)
            if (launchArtifact == Position.OUTTAKE) { // in right spot, launch!
                // stop motors
                DriveTrainKt.getInstance()
                    ?.setDriveMotors(arrayOf(0.0, 0.0, 0.0, 0.0), RunMode.RUN_USING_ENCODER)
                runBlockingAndUpdateTeleOp(launch())
                if (artifacts.artifactsHeld != 0) {
                    rotateSpindexer(artifacts.rotate())
                } else {
                    offsetSpindexer(Offset.NONE)
                }
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
        // endregion

        // region Flywheels (on trigger)
        if (data.currentGamepadTwo.right_trigger < 0.5) {
            boosterOff()
            telemetry.addData("Target Velocity (tps)", 0.0)
            flywheel.setVelocity(null) // disable control
        } else {
            boosterOn()
            val vel = if (CUSTOM != 0.0) CUSTOM else currentLaunchDistance.velocity
            telemetry.addData("Target Velocity (tps)", vel)
            flywheel.setVelocity(vel)
        }

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
            turntable?.reset()
        }

        telemetry.addData("Turntable Auto-gimballing Disabled", turntableLocked)

        if (turntableLocked || power != 0.0) {
            if (turntable != null) turntable.overridePower = power
            if (turntableAxon != null) {
                turntableAxon.position += when {
                    power > 0.0 -> {
                        0.01
                    }
                    power < 0.0 -> {
                        -0.01
                    }
                    else -> {
                        // == 0.0
                        0.00
                    }
                }
            }
        } else {
            if (turntable != null) turntable.overridePower = null // let the control loop below handle it

            val delta = calculateTagDelta(true)

            // rotate
            if (turntable != null) rotateTurretByDelta(delta)
            if (turntableAxon != null) rotateTurret(delta)
        }

        telemetry.addData("Turntable Axon Position", turntableAxon?.position)
        // endregion
    }

    override fun stop() {
        turntable?.cleanUp()
        spindexer.cleanUp()
    }

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    // region Hardware Functions
    private var intakeActive = false
    fun toggleIntake() = if (intakeActive) intakeOff() else intakeOn()

    fun intakeOff() {
        intakeMotor.power = 0.0
        intakeActive = false
    }

    fun intakeOn() {
        intakeMotor.power = 0.85
        intakeActive = true
    }

    fun intakeReverse() {
        intakeMotor.power = -0.85
        intakeActive = true
    }

    // Booster
    private var boosterActive = false
    fun toggleBooster() = if (boosterActive) boosterOff() else boosterOn()

    fun boosterOff() {
        booster.power = 0.0
        boosterActive = false
    }

    fun boosterOn() {
        booster.power = 1.0
        boosterActive = true
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

    private var currentOffset = Offset.NONE

    /**
     * Physically rotate the current artifacts. Ends with no offset.
     * @param position Position that the current intake artifact will end up in.
     */
    fun rotateSpindexer(position: Position) {
        val offsetDeg = currentOffset.offsetDeg // how much we are currently offset in negative direction
        currentOffset = Offset.NONE
        if (position == Position.OUTTAKE) {
            spindexer.targetPosition = spindexer.targetPosition?.plus((SPINDEXER_ROTATE_DEGREES * 2) + offsetDeg)
        } else if (position == Position.STORAGE) {
            spindexer.targetPosition = spindexer.targetPosition?.plus(SPINDEXER_ROTATE_DEGREES + offsetDeg)
        }
    }

    /**
     * Ends with [Offset.STORAGE].
     */
    fun rotateSpindexerForLaunch(numToFire: Int) {
        spindexer.targetPosition = when (currentOffset) { // from ??? -> STORAGE + launch
            Offset.NONE -> {
                // Move + add a storage offset because we need to have it
                spindexer.targetPosition?.minus(
                    (SPINDEXER_ROTATE_DEGREES * numToFire) + Offset.STORAGE.offsetDeg
                )
            }
            Offset.STORAGE -> {
                // Move; don't account for difference because we need it
                spindexer.targetPosition?.minus(SPINDEXER_ROTATE_DEGREES * numToFire)
            }
        }
        currentOffset = Offset.STORAGE
    }

    fun offsetSpindexer(newOffset: Offset) {
        if (newOffset == currentOffset) return // Already there
        // Must be going NONE -> STORAGE or vice versa
        if (newOffset == Offset.NONE) {
            // Must be going STORAGE -> NONE; always just undoing our current offset
            spindexer.targetPosition = spindexer.targetPosition?.plus(currentOffset.offsetDeg)
        } else {
            // Must be going NONE -> STORAGE
            spindexer.targetPosition = spindexer.targetPosition?.minus(Offset.STORAGE.offsetDeg)
        }
        currentOffset = newOffset
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
        if (turntable == null) return
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

    fun rotateTurret(delta: Double?) {
        if (turntableAxon == null) return
        telemetry.addData("Turntable Axon Delta", if (delta != null) (delta / 70) else delta)
        val final = turntableAxon.position + (
            // 90 degrees is 0.5 on turret
            if (delta != null) {
                // 0.1 good but too jittery at high range
                // 0.02 good but too low
                ((delta / 70) * 0.067)
            } else {
                calculateUnknownTurretRotation()
            }
        )
        turntableAxon.position = clamp(final, 0.0, 1.0)
    }

    private var goingNegative = true
    fun calculateUnknownTurretRotation(): Double {
        if (turntableAxon == null) return 0.0
        // Don't know where it is, need to find
        if (turntableAxon.position >= 1.0) {
            goingNegative = true
        } else if (turntableAxon.position <= 0.0) {
            goingNegative = false
        }

        // Always move in the correct direction to go find it
        return if (goingNegative) {
            -0.01
        } else {
            0.01
        }
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

    fun calculateTagDelta(teleOp: Boolean = false): Double? {
        // get tags
        val depotTag = getDepotTag()
        //val motifTag = getMotifTag() // if can see, must rotate

        val delta = if (limelight == null) { // find delta
            null // Not attached; do nothing
        } else if (depotTag != null) {
            -depotTag.position.x * 20 // negative! because positive is left by default
//        } else if (motifTag != null) {
            // rotate to find the tag
//            0.0
        } else (if (teleOp) 0.0 else 0.0) // null is go track

        telemetry.addData("Delta", delta)
        telemetry.addData("Depot Tag Data", depotTag?.position)
        return delta
    }
    // endregion

    fun loadAutoArtifacts() {
        artifacts.autoArtifactPreloads()
    }

    // region Roadrunner Actions
    /**
     * Launch an artifact. Assumes the artifact to launch is in the Outtake position.
     */
    fun launch(): Action {
        return SequentialAction(
            // pushes artifact into launch
            InstantAction { intakeOn() },
            InstantAction { rotateSpindexerForLaunch(1) },
            InstantAction { boosterOn() },
            sleepAndUpdate(2.5),
            InstantAction { boosterOff() },
            InstantAction { artifacts.launched() },
            InstantAction { setLEDs() },
            //InstantAction { intakeOff() },
        )
    }

    fun sleepAndUpdate(dt: Double): Action {
        return object : Action {
            private var beginTs = -1.0

            override fun run(p: TelemetryPacket): Boolean {
                flywheel.setVelocity(currentLaunchDistance.velocity)
                turntable?.update()
                spindexer.update()

                val t = if (beginTs < 0) {
                    beginTs = now()
                    0.0
                } else {
                    now() - beginTs
                }

                return t < dt
            }
        }
    }

    fun sleepAndOverrideSpindexer(dt: Double, override: Double?): Action {
        return object : Action {
            private var beginTs = -1.0

            override fun run(p: TelemetryPacket): Boolean {
                spindexer.overridePower = override
                spindexer.update() // for encoder
                turntable?.update()
                flywheel.loop()

                val t = if (beginTs < 0) {
                    beginTs = now()
                    0.0
                } else {
                    now() - beginTs
                }

                return t < dt
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
                boosterOn()

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
                    boosterOff()
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
    fun waitUntilLaunched(timeout: Double): Action {
        return object : Action {
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                val averageVelocity = flywheel.velocity

                flywheel.setVelocity(currentLaunchDistance.velocity)

                // Monitor for big loss in velocity
                if ((lastVelocity - averageVelocity) > 50.0 || timer.seconds() > timeout) {
                    p.put("Current Speed", averageVelocity)
                    flywheel.setVelocity(null)
                    boosterOff()
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

    /**
     * Assumes the artifact you are launching is ready in the OUTTAKE position. Ends with a NONE offset.
     */
    fun compositionLaunch(distance: LaunchDistance): Action {
        return ParallelAction(
            spinUpUntilLaunched(distance),
            SequentialAction(
                waitForSpunUp(distance),
                launch(),
            )
        )
    }

    fun launchAllHeld(distance: LaunchDistance, motif: Motif): Action {
        val numArtifacts = artifacts.artifactsHeld

        if (numArtifacts == 0) return InstantAction {}

        if (numArtifacts == 1) return SequentialAction(
            spin(motif.first),
            sleepAndUpdate(0.5),
            compositionLaunch(distance),
            )

        if (numArtifacts == 2) return SequentialAction(
            spin(motif.first),
            sleepAndUpdate(0.5),
            compositionLaunch(distance),
            spin(motif.second),
            sleepAndUpdate(0.5),
            compositionLaunch(distance),
            )

        if (numArtifacts == 3) return SequentialAction(
            spin(motif.first),
            sleepAndUpdate(0.5),
            compositionLaunch(distance),
            spin(motif.second),
            sleepAndUpdate(0.5),
            compositionLaunch(distance),
            spin(motif.third),
            sleepAndUpdate(0.5),
            compositionLaunch(distance),
        )

        RobotLog.ee("OuttakeV4", "Impossible Case! Aaa!")
        return InstantAction {} // impossible case
    }

    fun launchAllHeld(distance: LaunchDistance): Action {
        val numArtifacts = artifacts.artifactsHeld
        flywheel.setVelocity(distance.velocity)

        if (numArtifacts == 0) return InstantAction {}

        if (numArtifacts == 1) return ParallelAction(
            SequentialAction(
                InstantAction {
                    intakeOn()
                    boosterOn()
                    leftIntakePropBar.position =  .83
                    rightIntakePropBar.position = .52
                },
                sleepAndOverrideSpindexer(1.0, -1.0),
                InstantAction {
                    spindexer.update()
                    spindexer.targetPosition = spindexer.position - spindexer.position.mod(120.0) // cancel rotation
                    spindexer.overridePower = null
                },
                sleepAndUpdate(1.0),
                InstantAction {
                    //intakeOff()
                    boosterOff()
                    leftIntakePropBar.position = .90
                    rightIntakePropBar.position = .45
                    flywheel.setVelocity(null)
                    artifacts.allLaunched()
                },
            )
        )


        if (numArtifacts == 2) return ParallelAction(
            SequentialAction(
                InstantAction {
                    intakeOn()
                    boosterOn()
                    leftIntakePropBar.position =  .83
                    rightIntakePropBar.position = .52
                },
                sleepAndOverrideSpindexer(2.0, -1.0),
                InstantAction {
                    spindexer.update()
                    spindexer.targetPosition = spindexer.position - spindexer.position.mod(120.0) // cancel rotation
                    spindexer.overridePower = null
                },
                sleepAndUpdate(1.0),
                InstantAction {
                    //intakeOff()
                    boosterOff()
                    leftIntakePropBar.position = .90
                    rightIntakePropBar.position = .45
                    flywheel.setVelocity(null)
                    artifacts.allLaunched()
                },
            )
        )

        if (numArtifacts == 3) return ParallelAction(
            SequentialAction(
                InstantAction {
                    intakeOn()
                    boosterOn()
                    leftIntakePropBar.position =  .83
                    rightIntakePropBar.position = .52
                },
//                if (distance == LaunchDistance.FAR) {
                    //SleepAction(1.0)
//                } else InstantAction {},
                sleepAndOverrideSpindexer(3.0, -1.0),
                InstantAction {
                    spindexer.update()
                    spindexer.targetPosition = spindexer.position - spindexer.position.mod(120.0) // cancel rotation
                    spindexer.overridePower = null
                },
                sleepAndUpdate(1.0),
                InstantAction {
                    //intakeOff()
                    boosterOff()
                    leftIntakePropBar.position = .90
                    rightIntakePropBar.position = .45
                    flywheel.setVelocity(null)
                    artifacts.allLaunched()
                },
            )
        )


        RobotLog.ee("OuttakeV4", "Impossible Case all! Aaa!")
        return InstantAction {} // impossible case
    }

    fun intakeUntilIndexedOrTimeout(): Action {
        return object : Action {
            private var firstRun = true
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    intakeOn()
                    timer.reset()
                    firstRun = false
                }

                val detectedArtifact = detectArtifact(leftIntakeColorSensor.normalizedColors, rightIntakeColorSensor.normalizedColors)

                if (artifacts.intakeArtifact == ArtifactColors.NONE && detectedArtifact != ArtifactColors.NONE) {
                    artifacts.intake(detectedArtifact)
                    runBlockingAndUpdate(spin())
                    //intakeOff()
                    return false
                } else if (timer.seconds() > 2.5) {
                    //intakeOff()
                    return false
                }
                return true
            }
        }
    }

    fun spin(color: ArtifactColors = ArtifactColors.NONE): Action {
        return object : Action {
            var firstRun = true
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    rotateSpindexer(artifacts.rotate(color))
                    firstRun = false
                }
                return spindexer.error > TOLERANCE_DEGREES // run until in tolerance range
            }
        }
    }

    // todo: fix the run blockings in this class to use this
    fun runBlockingAndUpdate(a: Action) {
        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        a.preview(c)

        var b = true
        while (b && !Thread.currentThread().isInterrupted) {
            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            spindexer.update()
            flywheel.loop()

            turntable?.update()

            b = a.run(p)

            dash.sendTelemetryPacket(p)
        }
    }

    fun runBlockingAndUpdateTeleOp(a: Action) {
        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        a.preview(c)

        var b = true
        while (b && !Thread.currentThread().isInterrupted) {
            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            spindexer.update()
            flywheel.loop()

            turntable?.update()

            if (turntableAxon != null) {
                val delta = calculateTagDelta(true)
                rotateTurret(delta)
            }

            b = a.run(p)

            dash.sendTelemetryPacket(p)
        }
    }

    fun endAutoAction(): Action {
        return SequentialAction(
            InstantAction { intakeReverse() },
            SleepAction(0.5),
        )
    }
    // endregion

    // region Enums
    enum class LaunchDistance(val velocity: Double) {
        FAR(1065.0),
        CLOSE_PEAK(900.0),
        CLOSE_FAR(950.0),
        CLOSE_MID(850.0),
    }

    enum class Position {
        OUTTAKE,
        STORAGE,
        INTAKE,
        NONE,
    }

    enum class Offset(val offsetDeg: Double) {
        NONE(0.0),
        STORAGE(STORAGE_OFFSET_DEGREES),
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

            // fixme: bug situation
            //   one in storage, one in intake
            //   "go outtake" -> pushed into intake and outtake instead of outtake and storage

            // we aren't looking for an artifact or we don't hold it
            // always want a 'NONE' in the intake, so we should rotate such that both of our backs are filled
            if (outtakeArtifact == ArtifactColors.NONE) {
                rotate(Position.OUTTAKE)
                return Position.OUTTAKE
            }

            if (storageArtifact == ArtifactColors.NONE) {
                // bring this one to outtake because that will also bring the current out to storage
                // and leave intake empty
                rotate(Position.OUTTAKE)
                return Position.OUTTAKE
            }

            // nowhere to put the intakeArtifact

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

        fun allLaunched() {
            outtakeArtifact = ArtifactColors.NONE
            intakeArtifact = ArtifactColors.NONE
            storageArtifact = ArtifactColors.NONE
        }

        fun autoArtifactPreloads() {
            outtakeArtifact = ArtifactColors.GREEN
            intakeArtifact = ArtifactColors.PURPLE
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

            if (artifact1 == ArtifactColors.NONE || artifact2 == ArtifactColors.NONE) {
                // only one has detected; use it
                return if (artifact1 == ArtifactColors.NONE) artifact2 else artifact1
            }

            // uh oh, there's a conflict... hope for a better reading next time
            return ArtifactColors.NONE
        }

        private fun detectArtifact(data: NormalizedRGBA): ArtifactColors {
            return if (data.blue > 0.03 && data.blue > data.green) {
                ArtifactColors.PURPLE
            } else if (data.green > 0.03 && data.green > data.blue && data.green > data.red) {
                ArtifactColors.GREEN
            } else {
                ArtifactColors.NONE
            }
        }
        // endregion

        @JvmField var CUSTOM: Double = 0.0
        @JvmField var SPINDEXER_ROTATE_DEGREES = 120.0
        @JvmField var STORAGE_OFFSET_DEGREES = 10.0
        @JvmField var TOLERANCE_DEGREES = 15.0
    }
}