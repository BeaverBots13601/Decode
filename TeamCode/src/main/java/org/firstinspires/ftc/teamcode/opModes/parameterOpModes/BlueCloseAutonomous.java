package org.firstinspires.ftc.teamcode.opModes.parameterOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Autonomous(name = "Blue Close Autonomous", group = "Competition")
public class BlueCloseAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.BlueClose;
        super.runOpMode();
    }
}
