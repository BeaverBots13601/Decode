package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
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
class InOuttake private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    // Intake
    private val intakeServo = hardwareMap.crservo.get("intakeServo")

    // Outtake
    private val leftFlywheelMotor = PIDVelocityController(
        createDefaultMotor(hardwareMap, "leftFlywheelMotor"),
        0.0175,
        0.000002,
        0.0, // 0.0001?
        telemetry,
    )
    private val rightFlywheelMotor = PIDVelocityController(
        createDefaultMotor(hardwareMap, "rightFlywheelMotor"),
        0.0175,
        0.000002,
        0.0,
        telemetry,
    )
    private val scoopServo = hardwareMap.servo.get("scoopServo")

    // tilt
    private val leftTiltServo = hardwareMap.servo.get("leftTiltServo")
    private val rightTiltServo = hardwareMap.servo.get("rightTiltServo")

    // Indicator LEDs
    private val leftIndicatorLED = hardwareMap.servo.get("leftIndicatorLED")
    private val rightIndicatorLED = hardwareMap.servo.get("rightIndicatorLED")

    init {
        intakeServo.direction = DcMotorSimple.Direction.REVERSE
        intakeServo.power = 0.0 // hitecs suck
        rightTiltServo.direction = Servo.Direction.REVERSE
        leftTiltServo.position = ServoPositions.UP.pos // default up w/ preload
        rightTiltServo.position = ServoPositions.UP.pos
        scoopServo.direction = Servo.Direction.REVERSE
        scoopServo.position = 0.36

        leftIndicatorLED.position = 0.000 // off
        rightIndicatorLED.position = 0.000 // off
    }

    override fun start() {
        scoopServo.position = ScoopPositions.DOWN.pos
    }

    var currentOuttakeSpeed = OuttakeSpeeds.FAR_TELEOP
    private var currentScoopAction: Action? = null // action for running scoop
    private var currentSpace: ServoPositions = ServoPositions.UP
    override fun run(data: RunData) {
        telemetry.addData("Current Launch Mode", currentOuttakeSpeed)

        val triggerSum = data.currentGamepadTwo.right_trigger.toDouble() - data.currentGamepadTwo.left_trigger
        val velocity = if (triggerSum > 0.5) { // fire!!!
            //CUSTOM
            currentOuttakeSpeed.velocity
        } else if (triggerSum < -0.5) { // om nom nom
            -400.0
        } else {
            -1.0 // uncontrolled stop
        }

        //leftFlywheelMotor.velocity = velocity
        //rightFlywheelMotor.velocity = velocity
        telemetry.addData("Launch Velocity (tps)", velocity)

        val avgVelocity = (leftFlywheelMotor.velocity + rightFlywheelMotor.velocity) / 2
        telemetry.addData("Current Velocity (tps)", avgVelocity)

        // Control LEDs
        if(data.currentGamepadTwo.right_trigger > 0.0) {
            val error = abs(avgVelocity - currentOuttakeSpeed.velocity) / currentOuttakeSpeed.velocity
            val color = 0.277 + (0.233 * (1 - error)) // range from red to green
            leftIndicatorLED.position = color
            rightIndicatorLED.position = color
        } else {
            leftIndicatorLED.position = 0.000 // off
            rightIndicatorLED.position = 0.000 // off
        }

        val actionResult = currentScoopAction?.run(TelemetryPacket()) // run after flywheel pid
        // to ensure control continues

        if (actionResult == true) {
            return // hijack loop; keep going.
        } else if (actionResult == false) {
            currentScoopAction = null
        }

        // intake
        if (data.currentGamepadTwo.squareWasPressed()) { // Start
            startIntaking()
            currentSpace = ServoPositions.DOWN
        }

        if (data.currentGamepadTwo.triangleWasPressed()) { // stop intaking / spit out
            if (currentSpace == ServoPositions.DOWN) { // Stop if moving
                stopIntaking()
                currentSpace = ServoPositions.UP
            }
        }

        //  && currentSpace == ServoPositions.UP
        if (data.currentGamepadTwo.share) {
            intakeServo.power = 1.0
        } else if (data.currentGamepadTwo.cross) {
            intakeServo.power = -1.0
        } else {
            intakeServo.power = 0.0
        }

        // outtake
        if (data.currentGamepadTwo.circleWasPressed()) { // launch!!
            currentScoopAction = launch()
            telemetry.addData("Launching?", true)
            return // leave early to hijack loop
        } else {
            telemetry.addData("Launching?", false)
        }
        // near is 52.5in from front of bot to wall

        if (data.currentGamepadTwo.optionsWasPressed()) { // swap speed
            currentOuttakeSpeed = if (currentOuttakeSpeed == OuttakeSpeeds.NEAR_TELEOP) {
                OuttakeSpeeds.FAR_TELEOP
            } else {
                OuttakeSpeeds.NEAR_TELEOP
            }
        }
    }

    override fun stop() {}

    fun startIntaking() {
        //intakeServo.power = 1.0
        // tilt down
        leftTiltServo.position = ServoPositions.DOWN.pos
        rightTiltServo.position = ServoPositions.DOWN.pos
    }

    fun stopIntaking() {
        // tilt up
        leftTiltServo.position = ServoPositions.UP.pos
        rightTiltServo.position = ServoPositions.UP.pos

        //intakeServo.power = 0.0
    }

    fun spinUp(distance: OuttakeSpeeds): Action {
        return object : Action {
            private var firstRun = true
            override fun run(p: TelemetryPacket): Boolean {
                firstRun = false
                //leftFlywheelMotor.velocity = distance.velocity
                //rightFlywheelMotor.velocity = distance.velocity
                p.put("Current Speed", (leftFlywheelMotor.velocity + rightFlywheelMotor.velocity) / 2)
                if (abs(leftFlywheelMotor.velocity - distance.velocity) < 25
                    && abs(rightFlywheelMotor.velocity - distance.velocity) < 25) {
                    runBlocking(SleepAction(1.0))
                    //leftFlywheelMotor.velocity = -1.0
                    //rightFlywheelMotor.velocity = -1.0
                    return false
                } else return true
            }
        }
    }

    /**
     * Spins up the flywheel until launched, or until 5 seconds have passed after being spun up.
     */
    fun spinUpUntilLaunched(distance: OuttakeSpeeds): Action {
        return object : Action {
            private var spunUp = false
            private var lastVelocity = 0.0
            private val timer = ElapsedTime()
            override fun run(p: TelemetryPacket): Boolean {
                p.put("Spun Up?", spunUp)

                //leftFlywheelMotor.velocity = distance.velocity
                //rightFlywheelMotor.velocity = distance.velocity

                val averageVelocity = (leftFlywheelMotor.velocity + rightFlywheelMotor.velocity) / 2
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
                    //leftFlywheelMotor.velocity = -1.0
                    //rightFlywheelMotor.velocity = -1.0
                    return false // big drop, all done, or timed out
                }

                lastVelocity = averageVelocity
                p.put("Current Speed", averageVelocity)
                return true // keep monitoring
            }
        }
    }

    fun compositionLaunchUsingScoop(distance: OuttakeSpeeds): ParallelAction {
        return ParallelAction(
            spinUpUntilLaunched(distance),
            SequentialAction(
                waitForSpunUp(distance),
                launch(),
            )
        )
    }

    fun waitForSpunUp(distance: OuttakeSpeeds): Action {
        return Action {
            val averageVelocity = (leftFlywheelMotor.velocity + rightFlywheelMotor.velocity) / 2
            if (abs(averageVelocity - distance.velocity) < 25) {
                false
            } else true
        }
    }

    fun launch(): Action {
        return SequentialAction(
            InstantAction { scoopServo.position = ScoopPositions.UP.pos },
            SleepAction(1.0), // frozen thread may be desirable; stop drivers from leaving
            InstantAction { scoopServo.position = ScoopPositions.DOWN.pos },
        )
    }

    fun setIntakeServoPower(power: Double) {
        intakeServo.power = power
    }

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        // intake
        GamepadButtons.GP2_TRIANGLE,
        GamepadButtons.GP2_SQUARE,
        // outtake
        GamepadButtons.GP2_RIGHT_TRIGGER,
        GamepadButtons.GP2_OPTIONS,
        GamepadButtons.GP2_CIRCLE,
        GamepadButtons.GP2_LEFT_TRIGGER,
        GamepadButtons.GP2_SHARE,
        GamepadButtons.GP2_CROSS,
    )

    private enum class ServoPositions(val pos: Double) {
        UP(0.44),
        DOWN(0.57),
    }

    private enum class ScoopPositions(val pos: Double) {
        UP(0.78),
        DOWN(0.12),
    }

    enum class OuttakeSpeeds(val velocity: Double) {
        FAR_AUTO(950.0),
        NEAR_AUTO(800.0),
        FAR_TELEOP(1000.0),
        NEAR_TELEOP(875.0),
    }

    companion object : HardwareMechanismSingletonManager<InOuttake>(::InOuttake) {
        @JvmField var CUSTOM = 950.0
    }
}