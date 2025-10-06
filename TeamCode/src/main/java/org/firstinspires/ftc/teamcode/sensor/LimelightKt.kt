package org.firstinspires.ftc.teamcode.sensor

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.misc.AprilTagDataKt

class LimelightKt private constructor(hardwareMap: HardwareMap, initData: SensorInitData, private val telemetry: Telemetry) : SensorDeviceKt<List<LLResultTypes.FiducialResult>?>() {
    private val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

    init {
        limelight.pipelineSwitch(0)
        limelight.start()
        FtcDashboard.getInstance().startCameraStream(limelight, 0.0)
    }

    override fun start() {}

    /**
     * @return All AprilTags visible to the Limelight.
     */
    override fun poll(): List<LLResultTypes.FiducialResult>? {
        val result = limelight.latestResult
        return if (result.isValid) {
            result.fiducialResults
        } else null
    }

    override fun stop() {}

    fun getAprilTags(): List<AprilTagDataKt>? {
        val result = limelight.latestResult
        return if (result.isValid) {
            result.fiducialResults.map { AprilTagDataKt(it.fiducialId, it.targetPoseRobotSpace.position.z, 0) }
        } else null
    }

    fun getPositionalData(): Pose3D? {
        val result = limelight.latestResult
        return if (result.isValid) {
            result.botpose
        } else null
    }

    companion object : SensorDeviceSingletonManager<LimelightKt>(::LimelightKt)
}