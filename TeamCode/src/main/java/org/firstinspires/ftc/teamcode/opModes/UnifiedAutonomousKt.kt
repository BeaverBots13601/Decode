package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ParallelAction
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
import org.firstinspires.ftc.teamcode.hardware.OuttakeV2
import org.firstinspires.ftc.teamcode.hardware.OuttakeV2.Motif
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.math.PI

@Autonomous(name="Automatic Autonomous", group="Competition")
@Disabled
open class UnifiedAutonomousKt : LinearOpMode() {
    // Subclass variables
    protected open val pathToFollow = Path.Standard
    protected open val currentLocation = Locations.Unknown


    @Suppress("PROPERTY_HIDES_JAVA_FIELD") // intentional
    private val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    private val roadrunnerDrive by lazy { MecanumDrive(hardwareMap, currentLocation.startPose) }
    private val limelight by lazy { LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(
        currentLocation.teamColor,
        FtcDashboard.getInstance().isEnabled
    ), telemetry) }

    override fun runOpMode() {
        val out = OuttakeV2.getInstance(hardwareMap, HardwareMechanismKt.InitData(
            currentLocation.teamColor,
            HardwareMechanismKt.DriveMode.ROBOT,
            FtcDashboard.getInstance().isEnabled,
            currentLocation.startPose.heading.real, // todo needs conversion?
        ), telemetry)
        if (out == null) throw Error("out Initialization Failed")

        out.loadAutoArtifacts()

        initBulkReads(hardwareMap)
        blackboard.remove("robotHeading")

        val motif = when (limelight?.getAprilTags()?.first { it.id == 23 || it.id == 22 || it.id == 21 }?.id) {
            23 -> {
                Motif.PURPLE_PURPLE_GREEN
            }
            22 -> {
                Motif.PURPLE_GREEN_PURPLE
            }
            21 -> {
                Motif.GREEN_PURPLE_PURPLE
            }
            else -> Motif.GREEN_PURPLE_PURPLE // default
        }

        val originTAB = roadrunnerDrive.actionBuilder(currentLocation.startPose)

        // todo smartly rotate poses well instead of duplicating
        val route: SequentialAction = when(currentLocation) {
            Locations.BlueFar -> { // canonical far
                val launchPose = Vector2d(63.0, 22.0)
                val launchHeading = (-23 * PI) / 24
                val launchDistance = OuttakeV2.LaunchDistance.FAR

                val toLaunch = originTAB.fresh() // back up
                    .strafeToLinearHeading(launchPose, launchHeading)

                val offsetPose = Vector2d(0.0, 2.0)
                val firstArtifactPose = Vector2d(36.0, 36.0)
                val fourthArtifactPose = Vector2d(12.0, 36.0)
                val seventhArtifactPose = Vector2d(-12.0, 36.0)

                val firstArtifact = toLaunch.fresh()
                    .strafeToLinearHeading(firstArtifactPose, -PI / 2)
                val secondArtifact = firstArtifact.fresh()
                    .strafeToLinearHeading(firstArtifactPose + offsetPose, -PI / 2)
                val thirdArtifact = secondArtifact.fresh()
                    .strafeToLinearHeading(firstArtifactPose + (offsetPose * 2.0), -PI / 2)

                val toLaunchTwo = thirdArtifact.fresh()
                    .strafeToLinearHeading(launchPose, launchHeading)

                val fourArtifact = toLaunchTwo.fresh()
                    .strafeToLinearHeading(fourthArtifactPose, -PI / 2)
                val fiveArtifact = fourArtifact.fresh()
                    .strafeToLinearHeading(fourthArtifactPose + offsetPose, -PI / 2)
                val sixArtifact = fiveArtifact.fresh()
                    .strafeToLinearHeading(fourthArtifactPose + (offsetPose * 2.0), -PI / 2)

                val toLaunchThree = sixArtifact.fresh()
                    .strafeToLinearHeading(launchPose, launchHeading)

                val sevenArtifact = toLaunchThree.fresh()
                    .strafeToLinearHeading(seventhArtifactPose, -PI / 2)
                val eightArtifact = sevenArtifact.fresh()
                    .strafeToLinearHeading(seventhArtifactPose + offsetPose, -PI / 2)
                val nineArtifact = eightArtifact.fresh()
                    .strafeToLinearHeading(seventhArtifactPose + (offsetPose * 2.0), -PI / 2)

                val toLaunchFour = nineArtifact.fresh()
                    .strafeToLinearHeading(launchPose, launchHeading)
                val toPark = toLaunchFour.fresh() // park
                    .strafeToLinearHeading(Vector2d(63.0, 36.0), -PI).build()
                SequentialAction(
                    toLaunch.build(),
                    out.tripleLaunch(motif, launchDistance),
                    // intake more,
                    ParallelAction(
                        firstArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        secondArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        thirdArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    toLaunchTwo.build(),
                    out.tripleLaunch(motif, launchDistance),
                    // intake more,
                    ParallelAction(
                        fourArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        fiveArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        sixArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    toLaunchThree.build(),
                    out.tripleLaunch(motif, launchDistance),
                    // intake more,
                    ParallelAction(
                        sevenArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        eightArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        nineArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    toLaunchFour.build(),
                    out.tripleLaunch(motif, launchDistance),
                    toPark,
                )
            }
            Locations.RedFar -> { // blue far but rotated
                val launchPose = Vector2d(63.0, -22.0)
                val launchHeading = (22 * PI) / 24
                val launchDistance = OuttakeV2.LaunchDistance.FAR

                val toLaunch = originTAB.fresh() // back up
                    .strafeToLinearHeading(launchPose, launchHeading)

                val offsetPose = Vector2d(0.0, -2.0)
                val firstArtifactPose = Vector2d(36.0, -36.0)
                val fourthArtifactPose = Vector2d(12.0, -36.0)
                val seventhArtifactPose = Vector2d(-12.0, -36.0)

                val firstArtifact = toLaunch.fresh()
                    .strafeToLinearHeading(firstArtifactPose, -PI / 2)
                val secondArtifact = firstArtifact.fresh()
                    .strafeToLinearHeading(firstArtifactPose + offsetPose, -PI / 2)
                val thirdArtifact = secondArtifact.fresh()
                    .strafeToLinearHeading(firstArtifactPose + (offsetPose * 2.0), -PI / 2)

                val toLaunchTwo = thirdArtifact.fresh()
                    .strafeToLinearHeading(launchPose, launchHeading)

                val fourArtifact = toLaunchTwo.fresh()
                    .strafeToLinearHeading(fourthArtifactPose, -PI / 2)
                val fiveArtifact = fourArtifact.fresh()
                    .strafeToLinearHeading(fourthArtifactPose + offsetPose, -PI / 2)
                val sixArtifact = fiveArtifact.fresh()
                    .strafeToLinearHeading(fourthArtifactPose + (offsetPose * 2.0), -PI / 2)

                val toLaunchThree = sixArtifact.fresh()
                    .strafeToLinearHeading(launchPose, launchHeading)

                val sevenArtifact = toLaunchThree.fresh()
                    .strafeToLinearHeading(seventhArtifactPose, -PI / 2)
                val eightArtifact = sevenArtifact.fresh()
                    .strafeToLinearHeading(seventhArtifactPose + offsetPose, -PI / 2)
                val nineArtifact = eightArtifact.fresh()
                    .strafeToLinearHeading(seventhArtifactPose + (offsetPose * 2.0), -PI / 2)

                val toLaunchFour = nineArtifact.fresh()
                    .strafeToLinearHeading(launchPose, launchHeading)
                val toPark = toLaunchFour.fresh() // park
                    .strafeToLinearHeading(Vector2d(63.0, -36.0), -PI).build()
                SequentialAction(
                    toLaunch.build(),
                    out.tripleLaunch(motif, launchDistance),
                    // intake more,
                    ParallelAction(
                        firstArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        secondArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        thirdArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    toLaunchTwo.build(),
                    out.tripleLaunch(motif, launchDistance),
                    // intake more,
                    ParallelAction(
                        fourArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        fiveArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        sixArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    toLaunchThree.build(),
                    out.tripleLaunch(motif, launchDistance),
                    // intake more,
                    ParallelAction(
                        sevenArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        eightArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    ParallelAction(
                        nineArtifact.build(),
                        out.intakeUntilIndexed()
                    ),
                    toLaunchFour.build(),
                    out.tripleLaunch(motif, launchDistance),
                    toPark,
                )
            }
            Locations.BlueClose -> { // canonical close
                SequentialAction(
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(16.0, 25.0), 13 * -PI / 16).build(),
                    out.compositionLaunch(OuttakeV2.Position.LEFT, OuttakeV2.LaunchDistance.CLOSE),
                    out.compositionLaunch(OuttakeV2.Position.RIGHT, OuttakeV2.LaunchDistance.CLOSE),
                    out.tripleLaunch(motif, OuttakeV2.LaunchDistance.CLOSE),
                    out.compositionLaunch(OuttakeV2.Position.LEFT, OuttakeV2.LaunchDistance.CLOSE),
                    SleepAction(2.5),
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(66.0, 24.0), -PI).build(),
                )
            }

            Locations.RedClose -> { // blue close but rotated
                SequentialAction(
                    originTAB.fresh()
                        .strafeToLinearHeading(Vector2d(16.0, -25.0), 12 * PI / 16).build(),
                    out.compositionLaunch(OuttakeV2.Position.LEFT, OuttakeV2.LaunchDistance.CLOSE),
                    out.compositionLaunch(OuttakeV2.Position.RIGHT, OuttakeV2.LaunchDistance.CLOSE),
                    out.tripleLaunch(motif, OuttakeV2.LaunchDistance.CLOSE),
                    out.compositionLaunch(OuttakeV2.Position.LEFT, OuttakeV2.LaunchDistance.CLOSE),
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

        out.start() // cursed comment out
        limelight?.start()

        runBlocking(route)

        out.stop()
        limelight?.stop()

        blackboard["robotHeading"] = roadrunnerDrive.localizer.pose.heading.toDouble()
    }

    protected enum class Locations(val teamColor: TeamColor, val startPose: Pose2d) {
        BlueClose(TeamColor.BLUE, Pose2d(55.0, 56.0, 3 * -PI / 4)),
        BlueFar(TeamColor.BLUE, Pose2d(60.0, 24.0, -PI)),
        RedClose(TeamColor.RED, Pose2d(55.0, -56.0, 3 * PI / 4)),
        RedFar(TeamColor.RED, Pose2d(60.0, -24.0, -PI)),
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
