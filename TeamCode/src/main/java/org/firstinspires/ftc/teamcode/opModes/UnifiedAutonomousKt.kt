package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.hardware.InOuttake
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.PI

@Autonomous(name="Automatic Autonomous", group="Competition")
@Disabled
open class UnifiedAutonomousKt : LinearOpMode() {
    //private val propLocation: WebcamPropIdentificationKt.PropLocation
    //private val propId: WebcamPropIdentificationKt

    // Subclass variables
    protected open val pathToFollow = Path.Standard
    protected open val currentLocation = Locations.Unknown


    @Suppress("PROPERTY_HIDES_JAVA_FIELD") // intentional
    private val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    private val roadrunnerDrive by lazy { MecanumDrive(hardwareMap, currentLocation.startPose) }

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
                        .strafeToLinearHeading(Vector2d(-63.0, 22.0), (-23 * PI) / 24).build(),
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.FAR_AUTO),
                    InstantAction { inout.setIntakeServoPower(0.25) },
                    SleepAction(3.0),
                    InstantAction { inout.setIntakeServoPower(0.0) },
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.FAR_AUTO),
                    SleepAction(2.5),
                    originTAB.fresh() // park
                        .strafeToLinearHeading(Vector2d(-63.0, 36.0), -PI).build(),
                    )
            }
            Locations.RedFar -> { // blue far but rotated
                SequentialAction(
                    originTAB.fresh() // back up
                        .strafeToLinearHeading(Vector2d(-63.0, -22.0), (22 * PI) / 24).build(),
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.FAR_AUTO),
                    InstantAction { inout.setIntakeServoPower(0.25) },
                    SleepAction(3.0),
                    InstantAction { inout.setIntakeServoPower(0.0) },
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.FAR_AUTO),
                    SleepAction(2.5),
                    originTAB.fresh() // park
                        .strafeToLinearHeading(Vector2d(-63.0, -36.0), -PI).build(),
                )
            }
            Locations.BlueClose -> { // canonical close
                SequentialAction(
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(16.0, 25.0), 13 * -PI / 16).build(),
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.NEAR_AUTO),
                    InstantAction { inout.setIntakeServoPower(0.25) },
                    SleepAction(3.0),
                    InstantAction { inout.setIntakeServoPower(0.0) },
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.NEAR_AUTO),
                    SleepAction(2.5),
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(66.0, 24.0), -PI).build(),
                )
            }

            Locations.RedClose -> { // blue close but rotated
                SequentialAction(
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(16.0, -25.0), 12 * PI / 16).build(),
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.NEAR_AUTO),
                    InstantAction { inout.setIntakeServoPower(0.25) },
                    SleepAction(3.0),
                    InstantAction { inout.setIntakeServoPower(0.0) },
                    inout.compositionLaunchUsingScoop(InOuttake.OuttakeSpeeds.NEAR_AUTO),
                    SleepAction(2.5),
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(66.0, -24.0), -PI).build(),
                )
            }
            else -> { return }
        }

        telemetry.addData("INIT STATUS", "READY")
        telemetry.update()

        waitForStart() // setup done actually do things

        //inout.start() // cursed comment out

        runBlocking(route)

        inout.stop()

        blackboard.put("robotHeading", roadrunnerDrive.localizer.pose.heading.toDouble())
    }

    protected enum class Locations(val teamColor: TeamColor, val startPose: Pose2d) {
        BlueClose(TeamColor.BLUE, Pose2d(55.0, 56.0, 3 * -PI / 4)),
        BlueFar(TeamColor.BLUE, Pose2d(-66.0, 24.0, -PI)),
        RedClose(TeamColor.RED, Pose2d(55.0, -56.0, 3 * PI / 4)),
        RedFar(TeamColor.RED, Pose2d(-66.0, -24.0, -PI)),
        Unknown(TeamColor.UNKNOWN, Pose2d(0.0, 0.0, 0.0)),
    }

    protected enum class Path {
        Standard,
        Alternate,
    }
}

@Autonomous(name = "Blue Close Autonomous", group = "Competition", preselectTeleOp = "Blue TeleOp Controls (Robot)")
class BlueCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueClose
}

@Autonomous(name = "Blue Far Autonomous", group = "Competition", preselectTeleOp = "Blue TeleOp Controls (Robot)")
class BlueFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueFar
}

@Autonomous(name = "Red Close Autonomous", group = "Competition", preselectTeleOp = "Red TeleOp Controls (Robot)")
class RedCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedClose
}

@Autonomous(name = "Red Far Autonomous", group = "Competition", preselectTeleOp = "Red TeleOp Controls (Robot)")
class RedFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedFar
}
