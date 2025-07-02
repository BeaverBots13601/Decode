package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.GlobalsKt
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.PI

@Autonomous(name="Automatic Autonomous")
@Disabled
open class UnifiedAutonomousKt : LinearOpMode() {
    //private val propLocation: WebcamPropIdentificationKt.PropLocation
    //private val propId: WebcamPropIdentificationKt

    // Subclass variables
    protected open val pathToFollow = Path.Standard
    protected open val currentLocation = Locations.Unknown

    private val startPose = Pose2d(24.0, -60.0, -PI / 2)

    private val tel = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    private val roadrunnerDrive by lazy { MecanumDrive(hardwareMap, startPose) }
    private val limelight by lazy { LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(
        currentLocation.teamColor,
        FtcDashboard.getInstance().isEnabled,
    ), tel) ?: throw Exception() }

    override fun runOpMode() {
        GlobalsKt.initBulkReads(hardwareMap)
        GlobalsKt.robotHeading = 0.0

        // Example autonomous code that can be used. Don't be afraid to expand or remodel it as needed

        if(currentLocation == Locations.Unknown) {
            // limelight apriltag
            var tags = limelight.poll()
            var iterations = 0
            while (tags.isEmpty() && iterations < 500) {
                sleep(10)
                iterations++
                tags = limelight.poll()
            }

            var importantTag: FiducialResult? = null
            for (tag in tags) {
                if(tag.fiducialId == 12 || tag.fiducialId == 15) importantTag = tag; break
            }

            if (importantTag == null) {
                // uh oh
            } else if(importantTag.fiducialId == 12) {

            }
        }

        val path = roadrunnerDrive.actionBuilder(startPose)
            .strafeTo(Vector2d(-6.0, -30.5))
            .build()

        tel.addData("INIT STATUS", "READY")
        tel.update()

        waitForStart()

        when(currentLocation) {
            Locations.BlueFar,
            Locations.BlueClose -> {
                runBlocking(SequentialAction(
                    path
                ))
            }
            Locations.RedFar,
            Locations.RedClose -> {
                runBlocking(SequentialAction(
                    path
                ))
            }
            else -> {}
        }

    }

    protected enum class Locations(val teamColor: TeamColor) {
        BlueClose(TeamColor.BLUE),
        BlueFar(TeamColor.BLUE),
        RedClose(TeamColor.RED),
        RedFar(TeamColor.RED),
        Unknown(TeamColor.UNKNOWN)
    }

    protected enum class Path {
        Standard,
        Alternate
    }
}