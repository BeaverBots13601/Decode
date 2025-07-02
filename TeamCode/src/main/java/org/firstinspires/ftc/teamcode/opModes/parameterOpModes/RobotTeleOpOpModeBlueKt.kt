package org.firstinspires.ftc.teamcode.opModes.parameterOpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOpKt

@TeleOp(name = "Kotlin Blue TeleOp Controls (Robot)", group = "CompetitionKt")
class RobotTeleOpOpModeBlueKt : UnifiedTeleOpKt() {
    override val orientationMode = HardwareMechanismKt.DriveMode.ROBOT
    override val teamColor = TeamColor.BLUE
}