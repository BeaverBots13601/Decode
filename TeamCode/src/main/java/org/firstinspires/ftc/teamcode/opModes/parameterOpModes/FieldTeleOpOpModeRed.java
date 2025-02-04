package org.firstinspires.ftc.teamcode.opModes.parameterOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name="Red TeleOp Controls (Field)", group = "Competition")
public class FieldTeleOpOpModeRed extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = HardwareMechanism.DriveMode.FIELD;
        this.teamColor = TeamColor.RED;
        super.runOpMode();
    }
}