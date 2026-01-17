package org.firstinspires.ftc.teamcode.sensor

import com.qualcomm.hardware.bosch.BNO055IMUNew
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

class IMUSensorKt private constructor(hardwareMap: HardwareMap, initData: SensorInitData, private val telemetry: Telemetry) : SensorDeviceKt<Float>() {
    private val imu = hardwareMap.get(IMU::class.java, "imu")

    init {
        val imuParameters = IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ))
        val worked = imu.initialize(imuParameters)
        telemetry.addData("IMU Initialized Goodly?", worked)
    }

    override fun start() {}

    /**
     * @return imu angle around the vertical axis (rotation).
     */
    override fun poll(): Float {
        return imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle
    }

    override fun stop() {}

    companion object : SensorDeviceSingletonManager<IMUSensorKt>(::IMUSensorKt)
}