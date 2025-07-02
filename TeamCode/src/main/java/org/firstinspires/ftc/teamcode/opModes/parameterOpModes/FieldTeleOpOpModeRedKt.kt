package org.firstinspires.ftc.teamcode.opModes.parameterOpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOpKt

@TeleOp(name = "Kotlin Red TeleOp Controls (Field)", group = "CompetitionKt")
class FieldTeleOpOpModeRedKt : UnifiedTeleOpKt() {
    override val orientationMode = HardwareMechanismKt.DriveMode.FIELD
    override val teamColor = TeamColor.RED
}