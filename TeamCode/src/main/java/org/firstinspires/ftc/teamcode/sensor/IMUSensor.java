package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMUSensor extends SensorDevice<Float> {
    private IMU imu;
    public IMUSensor(HardwareMap hardwareMap, SensorInitData initData, Telemetry telemetry){
        super(hardwareMap, initData, telemetry);
        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception e) {
            available = false;
            return;
        }

        BNO055IMUNew.Parameters imuParameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, // must be adjusted if CH moves
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        boolean worked = imu.initialize(imuParameters);
        imu.resetYaw();
        telemetry.addData("IMU Initialized Goodly?", worked);
    }

    public void start() {}

    /**
     * @return double imu angle around the vertical axis (rotation).
     */
    public Float poll() {
        return this.imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    public void stop() {}
}