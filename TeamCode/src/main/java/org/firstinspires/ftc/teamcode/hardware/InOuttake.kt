package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

@Config
class InOuttake(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    // Intake
    private val intakeServo = hardwareMap.crservo.get("intakeServo")
    //private val colorSensor = hardwareMap.get(RevColorSensorV3::class.java,"intakeColorSensor")

    // Outtake
    private val leftFlywheelMotor = createDefaultMotor(hardwareMap, "leftFlywheelMotor")
    private val rightFlywheelMotor = createDefaultMotor(hardwareMap, "rightFlywheelMotor")
    private val scoopServo = hardwareMap.servo.get("scoopServo")

    // tilt
    private val leftTiltServo = hardwareMap.servo.get("leftTiltServo")
    private val rightTiltServo = hardwareMap.servo.get("rightTiltServo")

    init {
        intakeServo.direction = DcMotorSimple.Direction.REVERSE
        intakeServo.power = 0.0 // hitecs suck
        //leftTiltServo.direction = Servo.Direction.REVERSE
        //leftTiltServo.position = ServoPositions.UP.pos // default up w/ preload
        //rightTiltServo.position = ServoPositions.UP.pos
        scoopServo.direction = Servo.Direction.REVERSE
        scoopServo.position = ScoopPositions.DOWN.pos
    }
    override fun start() {}

    private var currentOuttakeSpeed = OuttakeSpeeds.NEAR
    override fun run(data: RunData) {
        // intake
        if (data.currentGamepadTwo.squareWasPressed()) { // Start
            startIntaking()
        }

        if (data.currentGamepadTwo.triangleWasPressed()) { // stop intaking / spit out
            if (intakeServo.power != 0.0) { // Stop if moving
                stopIntaking()
            } else {
                intakeServo.power = -1.0 // Start reversed if not
            }
        } //else if (colorSensor.getDistance(DistanceUnit.CM) < 7.5) {
           // stopIntaking()
        //}

        // outtake
        if (data.currentGamepadTwo.circleWasPressed()) { // launch!!
            runBlocking(launch()) // frozen thread may be desirable; stop drivers from leaving
            telemetry.addData("Launching?", true)
        } else {
            telemetry.addData("Launching?", false)
        }
        // 37.5% limiter works well for far
        // 32.5% limiter works well for near @ 52.5in from front of bot to wall

        if (data.currentGamepadTwo.optionsWasPressed()) { // swap speed
            currentOuttakeSpeed = if (currentOuttakeSpeed == OuttakeSpeeds.NEAR) {
                OuttakeSpeeds.FAR
            } else {
                OuttakeSpeeds.NEAR
            }
        }
        telemetry.addData("Current Launch Mode", currentOuttakeSpeed)

        leftFlywheelMotor.velocity = data.currentGamepadTwo.right_trigger.toDouble() * currentOuttakeSpeed.velocity
        rightFlywheelMotor.velocity = data.currentGamepadTwo.right_trigger.toDouble() * currentOuttakeSpeed.velocity
        telemetry.addData("Launch Velocity (tps)", data.currentGamepadTwo.right_trigger.toDouble() * currentOuttakeSpeed.velocity)
    }

    override fun stop() {}

    fun startIntaking() {
        intakeServo.power = 1.0
        // tilt down
        leftTiltServo.position = ServoPositions.DOWN.pos
        rightTiltServo.position = ServoPositions.DOWN.pos
    }

    fun stopIntaking() {
        intakeServo.power = 0.0
        // tilt up
        leftTiltServo.position = ServoPositions.UP.pos
        rightTiltServo.position = ServoPositions.UP.pos
    }

    fun spinUp(): Action {
        return object : Action {
            private var firstRun = true
            override fun run(p: TelemetryPacket): Boolean {
                if (firstRun) {
                    firstRun = false
                    leftFlywheelMotor.velocity = currentOuttakeSpeed.velocity
                    rightFlywheelMotor.velocity = currentOuttakeSpeed.velocity
                }
                p.put("Current Speed", leftFlywheelMotor.velocity)
                if (abs(leftFlywheelMotor.velocity - currentOuttakeSpeed.velocity) < 100
                    && abs(rightFlywheelMotor.velocity - currentOuttakeSpeed.velocity) < 100) {
                    leftFlywheelMotor.velocity = 0.0
                    rightFlywheelMotor.velocity = 0.0
                    return true
                } else return false
            }
        }
    }

    fun launch(): Action {
        return SequentialAction(
            InstantAction { scoopServo.position = ScoopPositions.UP.pos },
            SleepAction(0.5), // frozen thread may be desirable; stop drivers from leaving
            InstantAction { scoopServo.position = ScoopPositions.DOWN.pos },
        )
    }

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        // intake
        GamepadButtons.GP2_TRIANGLE,
        GamepadButtons.GP2_SQUARE,
        // outtake
        GamepadButtons.GP2_RIGHT_TRIGGER,
        GamepadButtons.GP2_OPTIONS,
        GamepadButtons.GP2_CIRCLE,
    )

    private enum class ServoPositions(val pos: Double) {
        UP(0.60),
        DOWN(0.50),
    }

    private enum class ScoopPositions(val pos: Double) {
        UP(0.90),
        DOWN(0.31),
    }

    private enum class OuttakeSpeeds(val speed: Double, val velocity: Double) {
        FAR(0.375, 1050.0),
        NEAR(0.325, 910.0),
    }

    companion object : HardwareMechanismSingletonManager<InOuttake>(::InOuttake)
}