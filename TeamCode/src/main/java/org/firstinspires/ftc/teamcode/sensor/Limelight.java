package org.firstinspires.ftc.teamcode.sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.misc.AprilTagData;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends SensorDevice<List<LLResultTypes.FiducialResult>> {
    private Limelight3A limelight;
    public Limelight(HardwareMap hardwareMap, SensorInitData initData, Telemetry telemetry) {
        super(hardwareMap, initData, telemetry);
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            available = false;
            return;
        }

        limelight.pipelineSwitch(0);

        FtcDashboard.getInstance().startCameraStream(limelight, 0);

        available = true;
    }

    public void start() {
        limelight.start();
    }

    /**
     * @return All AprilTags visible to the Limelight.
     */
    public List<LLResultTypes.FiducialResult> poll() {
        return limelight.getLatestResult().getFiducialResults();
    }

    public void stop() {}

    public ArrayList<AprilTagData> getAprilTags(){
        ArrayList<AprilTagData> out = new ArrayList<>();

        limelight.getLatestResult().getFiducialResults().forEach((LLResultTypes.FiducialResult a) -> out.add(new AprilTagData(a.getFiducialId(), a.getTargetPoseRobotSpace().getPosition().z, 0)));

        return out;
    }

    public void updateIMUData(double angleRad){
        limelight.updateRobotOrientation(angleRad);
    }

    public Pose3D getPositionalData() {
        return limelight.getLatestResult().getBotpose_MT2();
    }
}