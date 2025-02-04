package org.firstinspires.ftc.teamcode.opModes.parameterOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Autonomous(name = "Red Far Autonomous", group = "Competition")
public class RedFarAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.RedFar;
        super.runOpMode();
    }
}