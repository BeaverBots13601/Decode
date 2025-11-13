package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcore.external.Telemetry

class IntakeV2 private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    private val intakeMotor = createDefaultMotor(hardwareMap, "intakeMotor")

    init {
        intakeMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun start() {}

    override fun run(data: RunData) {
        if (data.currentGamepadTwo.squareWasPressed()) {
            if (intakeMotor.power == 0.7) {
                intakeMotor.power = 0.0
            } else {
                intakeMotor.power = 0.7
            }
        }
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        GamepadButtons.GP2_SQUARE,
    )

    companion object : HardwareMechanismSingletonManager<IntakeV2>(::IntakeV2)
}