package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class Brakes private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    private val leftBrake = hardwareMap.servo.get("leftBrake")
    private val rightBrake = hardwareMap.servo.get("rightBrake")

    override fun start() {}

    override fun run(data: RunData) {}

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    companion object : HardwareMechanismSingletonManager<Brakes>(::Brakes)
}