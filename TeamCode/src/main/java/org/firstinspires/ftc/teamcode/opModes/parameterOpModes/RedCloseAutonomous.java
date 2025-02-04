package org.firstinspires.ftc.teamcode.opModes.parameterOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Autonomous(name = "Red Close Autonomous", group = "Competition")
public class RedCloseAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.RedClose;
        super.runOpMode();
    }
}