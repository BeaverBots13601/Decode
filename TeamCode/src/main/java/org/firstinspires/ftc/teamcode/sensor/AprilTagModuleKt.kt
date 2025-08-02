package org.firstinspires.ftc.teamcode.sensor

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.misc.AprilTagDataKt
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

/**
 * Handles the scanning and recognition of AprilTags through a dumb webcam.
 *
 * See also [Limelight].
 */
class AprilTagModuleKt private constructor(hardwareMap: HardwareMap, initData: SensorInitData, private val telemetry: Telemetry) : SensorDeviceKt<List<AprilTagDataKt>>() {
    // magic numbers
    private val cameraName = "camera"
    private val cameraWidth = 1280
    private val cameraHeight = 720

    private var aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setLensIntrinsics(1500.35, 1500.35, 961.278, 563.176) // todo needs recalibration if new camera
        .build()

    private var visionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, cameraName))
        .setCameraResolution(Size(cameraWidth, cameraHeight))
        .enableLiveView(true)
        .setAutoStopLiveView(false)
        .addProcessor(aprilTag)
        .build()

    init {
        // Add our camera to FtcDashboard for viewing
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0)
    }

    override fun start() {}

    override fun stop() {}

    /**
     * Takes the current camera view and returns information about all visible AprilTags.
     * @return An array of objects, each signifying a detection of an AprilTag and containing data about it.
     */
    override fun poll(): List<AprilTagDataKt> {
        return aprilTag.detections.map { AprilTagDataKt(it.id, it.ftcPose.y, it.hamming) }
    }

    companion object : SensorDeviceSingletonManager<AprilTagModuleKt>(::AprilTagModuleKt)
}