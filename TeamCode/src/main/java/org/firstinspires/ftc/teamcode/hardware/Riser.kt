package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class Riser(hardwareMap: HardwareMap, initData: InitData, telemetry: Telemetry) : HardwareMechanismKt() {
    private val leftMotor = hardwareMap.get(DcMotorEx::class.java, "leftRiserMotor")
    private val rightMotor = hardwareMap.get(DcMotorEx::class.java, "rightRiserMotor")

    init {
        rightMotor.direction = Direction.REVERSE
    }

    override fun start() {}

    override fun run(data: RunData) {
        if (data.currentGamepadOne.right_bumper) {
            leftMotor.power = 0.5
            rightMotor.power = 0.5
        } else if (data.currentGamepadOne.left_bumper) {
            leftMotor.power = -0.5
            rightMotor.power = -0.5
        } else {
            leftMotor.power = 0.0
            rightMotor.power = 0.0
        }
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        GamepadButtons.GP1_RIGHT_BUMPER,
    )

    companion object : HardwareMechanismSingletonManager<Riser>(::Riser)
}