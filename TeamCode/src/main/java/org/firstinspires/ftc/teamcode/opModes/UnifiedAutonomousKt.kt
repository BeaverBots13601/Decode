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
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.hardware.InOuttake
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.PI

@Autonomous(name="Automatic Autonomous", group="Competition")
@Disabled
open class UnifiedAutonomousKt : LinearOpMode() {
    //private val propLocation: WebcamPropIdentificationKt.PropLocation
    //private val propId: WebcamPropIdentificationKt

    // Subclass variables
    protected open val pathToFollow = Path.Standard
    protected open val currentLocation = Locations.Unknown


    private val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    private val roadrunnerDrive by lazy { MecanumDrive(hardwareMap, currentLocation.startPose) }
    private val limelight by lazy { LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(
        currentLocation.teamColor,
        FtcDashboard.getInstance().isEnabled,
    ), telemetry) ?: throw Exception() }

    override fun runOpMode() {
        val inout = InOuttake.getInstance(hardwareMap, HardwareMechanismKt.InitData(
            currentLocation.teamColor,
            HardwareMechanismKt.DriveMode.ROBOT,
            FtcDashboard.getInstance().isEnabled,
            currentLocation.startPose.heading.real, // todo needs conversion?
        ), telemetry)

        if (inout == null) throw Error("Inout Initialization Failed")
        initBulkReads(hardwareMap)
        blackboard.remove("robotHeading")

        val originTAB = roadrunnerDrive.actionBuilder(currentLocation.startPose)

        // todo smartly rotate poses well instead of duplicating
        val route: SequentialAction = when(currentLocation) {
            Locations.BlueFar -> { // canonical far
                SequentialAction(
                    originTAB.fresh() // back up
                        .strafeToLinearHeading(Vector2d(-63.0, 24.0), PI / 12).build(),
                    inout.spinUp(),
                    inout.launch(),
                    inout.spinUp(),
                    inout.launch(),
                    originTAB.fresh() // park
                        .strafeToConstantHeading(Vector2d(-63.0, 48.0)).build(),
                    )
            }
            Locations.RedFar -> { // blue far but rotated
                SequentialAction(
                    originTAB.fresh() // back up
                        .strafeToConstantHeading(Vector2d(14.0, -14.0)).build(),
                    inout.spinUp(),
                    inout.launch(),
                    inout.spinUp(),
                    inout.launch(),
                    originTAB.fresh() // park
                        .strafeToLinearHeading(Vector2d(66.0, -12.0), -0.0).build(),
                )
            }
            Locations.BlueClose -> { // canonical close
                SequentialAction(
                    originTAB.fresh()
                        .strafeToConstantHeading(Vector2d(14.0, 14.0)).build(),
                    inout.spinUp(),
                    inout.launch(),
                    inout.spinUp(),
                    inout.launch(),
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(66.0, 12.0), 0.0).build(),
                )
            }

            Locations.RedClose -> { // blue close but rotated
                SequentialAction(
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(-63.0, -24.0), -PI / 12).build(),
                    inout.spinUp(),
                    inout.launch(),
                    inout.spinUp(),
                    inout.launch(),
                    originTAB.fresh()
                        .strafeToConstantHeading(Vector2d(-63.0, -48.0)).build(),
                )
            }
            else -> { return }
        }

        telemetry.addData("INIT STATUS", "READY")
        telemetry.update()

        waitForStart() // setup done actually do things

        runBlocking(route)

        blackboard.put("robotHeading", roadrunnerDrive.localizer.pose.heading.toDouble())
    }

    protected enum class Locations(val teamColor: TeamColor, val startPose: Pose2d) {
        BlueClose(TeamColor.BLUE, Pose2d(55.0, 56.0, 3 * -PI / 4)),
        BlueFar(TeamColor.BLUE, Pose2d(-66.0, 24.0, -PI)),
        RedClose(TeamColor.RED, Pose2d(55.0, -56.0, 3 * PI / 4)),
        RedFar(TeamColor.RED, Pose2d(-66.0, -24.0, -PI)),
        Unknown(TeamColor.UNKNOWN, Pose2d(0.0, 0.0, 0.0))
    }

    protected enum class Path {
        Standard,
        Alternate
    }
}

@Autonomous(name = "Blue Close Autonomous", group = "Competition")
class BlueCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueClose
}

@Autonomous(name = "Blue Far Autonomous", group = "Competition")
class BlueFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueFar
}

@Autonomous(name = "Red Close Autonomous", group = "Competition")
class RedCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedFar
}

@Autonomous(name = "Red Far Autonomous", group = "Competition")
class RedFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedClose
}
