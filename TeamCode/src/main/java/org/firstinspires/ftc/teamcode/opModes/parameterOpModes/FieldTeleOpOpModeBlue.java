package org.firstinspires.ftc.teamcode.opModes.parameterOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "Blue TeleOp Controls (Field)", group = "Competition")
public class FieldTeleOpOpModeBlue extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = HardwareMechanism.DriveMode.FIELD;
        this.teamColor = TeamColor.BLUE;
        super.runOpMode();
    }
}