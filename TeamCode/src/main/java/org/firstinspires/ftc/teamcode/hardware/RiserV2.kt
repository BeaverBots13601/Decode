package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class RiserV2 private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    private val miniLift = hardwareMap.servo.get("miniLift").apply {
        position = LiftPosition.NO_LIFT.pos
    }

    override fun start() {}

    private var lifted = false;
    override fun run(data: RunData) {
        if (data.currentGamepadOne.psWasPressed()) {
            if (lifted) {
                miniLift.position = LiftPosition.NO_LIFT.pos
            } else {
                miniLift.position = LiftPosition.LIFT.pos
            }
            lifted = !lifted
        }
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    enum class LiftPosition(val pos: Double) {
        LIFT(0.9),
        NO_LIFT(0.2),
    }

    companion object : HardwareMechanismSingletonManager<RiserV2>(::RiserV2)
}