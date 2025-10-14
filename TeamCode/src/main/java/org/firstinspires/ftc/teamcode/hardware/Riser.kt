package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class Riser(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    val leftRiserMotor = hardwareMap.dcMotor.get("LeftRiserMotor")
    val rightRiserMotor = hardwareMap.dcMotor.get("RightRiserMotor")

    override fun start() {
        leftRiserMotor.power = 0.0
        rightRiserMotor.power = 0.0

        rightRiserMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun run(data: RunData) {
        leftRiserMotor.power = 1.0
        rightRiserMotor.power = 1.0
    }

    override fun stop() {
        leftRiserMotor.power = 0.0
        rightRiserMotor.power = 0.0
    }

    override val usedButtons: Array<GamepadButtons> = emptyArray()

    companion object : HardwareMechanismSingletonManager<Riser>(::Riser)
}