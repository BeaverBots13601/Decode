package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class InOuttake(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    override fun start() {
        TODO("Not yet implemented")
    }

    override fun run(data: RunData) {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    companion object : HardwareMechanismSingletonManager<InOuttake>(::InOuttake)
}