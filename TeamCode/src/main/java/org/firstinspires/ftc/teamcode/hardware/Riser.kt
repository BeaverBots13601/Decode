package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class Riser(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    val leftRiserMotor = createDefaultMotor(hardwareMap, "leftRiserMotor")
    val rightRiserMotor = createDefaultMotor(hardwareMap, "RightRiserMotor")

    override fun start() {}

    override fun run(data: RunData) {
        if (data.currentGamepadOne.right_bumper) {
            leftRiserMotor.power = -0.5
            rightRiserMotor.power = -0.5
        } else if (data.currentGamepadOne.left_bumper) {
            leftRiserMotor.power = 0.5
            rightRiserMotor.power = 0.5
        } else {
            leftRiserMotor.power = 0.0
            rightRiserMotor.power = 0.0
        }
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        GamepadButtons.GP1_RIGHT_BUMPER,
        GamepadButtons.GP1_LEFT_BUMPER,
    )

    companion object : HardwareMechanismSingletonManager<Riser>(::Riser)
}