package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class RiserV2 private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    private val miniLift = hardwareMap.servo.get("miniLift").apply {
        position = LiftPosition.NO_LIFT.pos
    }

    private val leftBrake = hardwareMap.servo.get("leftBrake").apply {
        position = LeftBrakePosition.NO_BRAKE.pos
    }
    private val rightBrake = hardwareMap.servo.get("rightBrake").apply {
        position = RightBrakePosition.NO_BRAKE.pos
    }

    override fun start() {}

    private var lifted = false
    private var brakeRetracted = true
    private val timer = ElapsedTime()
    override fun run(data: RunData) {
        // lifting
        if (data.currentGamepadOne.psWasPressed()) {
            if (lifted) {
                miniLift.position = LiftPosition.NO_LIFT.pos
                rightBrake.position = RightBrakePosition.NO_BRAKE.pos
                brakeRetracted = true
            } else {
                miniLift.position = LiftPosition.LIFT.pos
                rightBrake.position = RightBrakePosition.LIFT.pos
                timer.reset()
                brakeRetracted = false
            }
            lifted = !lifted
        }

        if (!brakeRetracted && timer.seconds() > 1.5) {
            rightBrake.position = RightBrakePosition.NO_BRAKE.pos
            brakeRetracted = true
        }

        // braking
        if (data.currentGamepadOne.triangleWasPressed()) {
            toggleBrakes()
        }
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    var braking = false
    private fun toggleBrakes() {
        if (braking) {
            leftBrake.position = LeftBrakePosition.BRAKE.pos
            rightBrake.position = RightBrakePosition.BRAKE.pos
        } else {
            leftBrake.position = LeftBrakePosition.NO_BRAKE.pos
            rightBrake.position = RightBrakePosition.NO_BRAKE.pos
        }
        braking = !braking
    }


    enum class LiftPosition(val pos: Double) {
        LIFT(1.0),
        NO_LIFT(0.2),
    }

    enum class LeftBrakePosition(val pos: Double) {
        BRAKE(0.55), // todo
        NO_BRAKE(0.8),
    }

    enum class RightBrakePosition(val pos: Double) {
        BRAKE(0.45), // todo
        LIFT(0.8),
        NO_BRAKE(0.2),
    }

    companion object : HardwareMechanismSingletonManager<RiserV2>(::RiserV2)
}