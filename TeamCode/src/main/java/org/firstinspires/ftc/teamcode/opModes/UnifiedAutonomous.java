package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TelemetryManager;
import org.firstinspires.ftc.teamcode.sensor.SensorDevice;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensor.Limelight;
import org.firstinspires.ftc.teamcode.sensor.WebcamPropIdentification;
import org.firstinspires.ftc.teamcode.sensor.WebcamPropIdentification.PropLocation;

import java.util.List;

@Autonomous(name="Automatic Autonomous")
@Disabled
public class UnifiedAutonomous extends LinearOpMode {
    protected enum Locations {
        BlueClose,
        BlueFar,
        RedClose,
        RedFar,
        Unknown
    }
    protected enum Path {
        STANDARD,
        ALTERNATE
    }
    private PropLocation propLocation;
    protected Locations currentLocation;
    protected Path pathToFollow = Path.STANDARD;
    private WebcamPropIdentification propId;
    private MecanumDrive roadrunnerDrive;
    private Limelight limelight;
    public void runOpMode(){
        Globals.initBulkReads(hardwareMap);
        Globals.robotHeading = 0;
        if(currentLocation == null) currentLocation = Locations.Unknown;
        // Example autonomous code that can be used. Don't be afraid to expand or remodel it as needed
        TelemetryManager robot = new TelemetryManager(telemetry);
        limelight = new Limelight(hardwareMap, new SensorDevice.SensorInitData(), robot::writeToTelemetry);

        if(currentLocation == Locations.Unknown) {
            // limelight apriltag
            List<LLResultTypes.FiducialResult> tags = limelight.poll();
            int iterations2 = 0;
            while (tags.size() == 0 && iterations2 < 500) {
                sleep(10);
                iterations2++;
                tags = limelight.poll();
            }

            @Nullable
            FiducialResult importantTag = null;
            for (LLResultTypes.FiducialResult tag : tags){
                if(tag.getFiducialId() == 12 || tag.getFiducialId() == 15) importantTag = tag; break;
            }

            if (importantTag == null) {
                // todo panic case
            } else if(importantTag.getFiducialId() == 12) {
                //importantTag.
            } else if (importantTag.getFiducialId() == 15){

            }
            /*AprilTagData max = new AprilTagData(); // default
            for (AprilTagData tag : tags) {
                if (tag.getDist() > max.getDist()) max = tag;
            }
            if (max.getId() == 14) { // sees red wall tag
                if (max.getDist() > Globals.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES) {
                    currentLocation = Locations.RedFar; // tag far away
                } else {
                    currentLocation = Locations.RedClose; // tag nearby
                }
            } else if (max.getId() == 11) { // sees blue wall tag
                if (max.getDist() > Globals.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES) {
                    currentLocation = Locations.BlueClose; // tag far away
                } else {
                    currentLocation = Locations.BlueFar; // tag nearby
                }
            } else {
                // uh oh todo make this case
            }*/
        }

        Pose2d startPose = new Pose2d(24, -60, -Math.PI / 2);
        roadrunnerDrive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder toChamberPath = roadrunnerDrive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-6, -30.5));

        robot.writeToTelemetry("INIT STATUS", "READY");
        robot.updateTelemetry();

        waitForStart(); // setup done actually do things

        switch(currentLocation){
            case RedFar:
            case BlueFar: {
                Actions.runBlocking(new SequentialAction());
                break;
            }
            case RedClose:
            case BlueClose: {
                Actions.runBlocking(new SequentialAction());
                break;
            }
        }
    }
}