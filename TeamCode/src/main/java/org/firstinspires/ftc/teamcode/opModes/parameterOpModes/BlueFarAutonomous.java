package org.firstinspires.ftc.teamcode.opModes.parameterOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Autonomous(name = "Blue Far Autonomous", group = "Competition")
public class BlueFarAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.BlueFar;
        super.runOpMode();
    }
}