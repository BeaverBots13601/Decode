package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class RiserV2 private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    // Switching Servos
    private val leftPTOServo  = hardwareMap.servo.get("leftPTOServo")
    private val rightPTOServo = hardwareMap.servo.get("rightPTOServo")

    override fun start() {}

    override fun run(data: RunData) {
        // do not allow lowering
    }

    override fun stop(){}

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    companion object : HardwareMechanismSingletonManager<RiserV2>(::RiserV2)
}