package org.firstinspires.ftc.teamcode.sensor;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.AprilTagData;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles the scanning and recognition of AprilTags through a dumb webcam.
 * <p>
 * See also Limelight.
 */
public class AprilTagModule extends SensorDevice<List<AprilTagData>> {
    // magic numbers
    private static final String cameraName = "camera";
    private static final int cameraWidth = 1280;
    private static final int cameraHeight = 720;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTagModule(HardwareMap hardwareMap, SensorInitData initData, Telemetry telemetry){
        super(hardwareMap, initData, telemetry);
        WebcamName cameraNameObject;
        try {
            cameraNameObject = hardwareMap.get(WebcamName.class, cameraName);
        } catch (Exception e) {
            available = false;
            return;
        }
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // ang unit doesn't matter
                .setLensIntrinsics(1500.35, 1500.35, 961.278, 563.176) // todo needs recalibration if new camera
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(cameraNameObject);
        builder.setCameraResolution(new Size(cameraWidth, cameraHeight));
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Add our camera to FtcDashboard for viewing
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        available = true;
    }

    public void start() {}

    public void stop() {}

    /**
     * Takes the current camera view and returns information about all visible AprilTags.
     * @return An array of objects, each signifying a detection of an AprilTag and containing data about it.
     */
    public List<AprilTagData> poll(){
        ArrayList<AprilTagData> data = new ArrayList<>();
        for(AprilTagDetection i : aprilTag.getDetections()){
            data.add(new AprilTagData(i.id, i.ftcPose.y, i.hamming));
        }

        return data;
    }
}
