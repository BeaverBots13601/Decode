package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.hardware.OuttakeV4
import org.firstinspires.ftc.teamcode.misc.Motif
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
        val out = OuttakeV4.getInstance(hardwareMap, HardwareMechanismKt.InitData(
            currentLocation.teamColor,
            HardwareMechanismKt.DriveMode.ROBOT,
            FtcDashboard.getInstance().isEnabled,
            currentLocation.startPose.heading.real, // todo needs conversion?
        ), telemetry)
        if (out == null) throw Error("Out Initialization Failed")

        out.loadAutoArtifacts()

        initBulkReads(hardwareMap)
        runCatching { FtcDashboard.getInstance().startCameraStream(hardwareMap.get(Limelight3A::class.java, "limelight"), 0.0) }
        blackboard.remove("robotHeading")

        val originTAB = roadrunnerDrive.actionBuilder(currentLocation.startPose)

        // todo better pose rotation?
        val routeParameters: RouteParameters = when(currentLocation) {
            Locations.BlueFar -> { // canonical far
                RouteParameters(
                    //Pose2d(-12.0, -12.0, (-25 * PI) / 32),
                    Pose2d(50.0, -12.0, (-28 * PI) / 32),
                    OuttakeV4.LaunchDistance.FAR,
                    ArtifactPositions.BLUE_HUMAN_PLAYER,
                    ArtifactPositions.BLUE_FAR,
                    ArtifactPositions.BLUE_MID,
                    Pose2d(63.0, -36.0, -PI)
                )
            }

            Locations.RedFar -> { // blue far but rotated
                RouteParameters(
                    //Pose2d(-12.0, 12.0, (3 * PI) / 4),
                    Pose2d(50.0, 12.0, (28 * PI) / 32),
                    OuttakeV4.LaunchDistance.FAR,
                    ArtifactPositions.RED_HUMAN_PLAYER,
                    ArtifactPositions.RED_FAR,
                    ArtifactPositions.RED_MID,
                    Pose2d(63.0, 36.0, -PI)
                )
            }

            Locations.BlueClose -> { // canonical close
                RouteParameters(
                    Pose2d(-24.0, -24.0, -3 * PI / 4),
//                    Pose2d(-24.0, -24.0, -21 * PI / 32),
                    OuttakeV4.LaunchDistance.CLOSE_MID,
                    ArtifactPositions.BLUE_CLOSE,
                    ArtifactPositions.BLUE_MID,
                    ArtifactPositions.BLUE_FAR,
                    Pose2d(-62.0, -36.0, -PI)
                )
            }

            Locations.RedClose -> { // blue close but rotated
                RouteParameters(
                    Pose2d(-24.0, 24.0, 3 * PI / 4),
//                    Pose2d(-24.0, 24.0, 21 * PI / 32),
                    OuttakeV4.LaunchDistance.CLOSE_MID,
                    ArtifactPositions.RED_CLOSE,
                    ArtifactPositions.RED_MID,
                    ArtifactPositions.RED_FAR,
                    Pose2d(-62.0, 36.0, -PI)
                )
            }

            else -> {
                throw Error("big uh oh")
            }
        }

        val launchPose = routeParameters.launchPose
        val launchDistance = routeParameters.launchDistance

        val toLaunch = originTAB.fresh() // back up
            .strafeToLinearHeading(launchPose.position, launchPose.heading)
        //.strafeToLinearHeading(launchPose.position, launchPose.heading)

        val firstOffsetPose = Vector2d(0.0, 4.0) * routeParameters.firstArtifactRow.offsetSign
        val firstArtifactPose = routeParameters.firstArtifactRow.pose.position
        val firstArtifactPickupHeading = routeParameters.firstArtifactRow.pose.heading

        val fourthOffsetPose = Vector2d(0.0, 4.0) * routeParameters.secondArtifactRow.offsetSign
        val fourthArtifactPose = routeParameters.secondArtifactRow.pose.position
        val fourthArtifactPickupHeading = routeParameters.secondArtifactRow.pose.heading

        val seventhOffsetPose = Vector2d(0.0, 4.0) * routeParameters.thirdArtifactRow.offsetSign
        val seventhArtifactPose = routeParameters.thirdArtifactRow.pose.position
        val seventhArtifactPickupHeading = routeParameters.thirdArtifactRow.pose.heading

        val firstArtifactBefore = toLaunch.fresh()
            .strafeToLinearHeading(firstArtifactPose - (firstOffsetPose * 2.0), firstArtifactPickupHeading)
//        val firstArtifact = firstArtifactBefore.fresh()
//            .strafeToLinearHeading(firstArtifactPose + (firstOffsetPose * 2.0), firstArtifactPickupHeading)
//        val secondArtifact = firstArtifactBefore.fresh()
//            .strafeToLinearHeading(firstArtifactPose + (firstOffsetPose * 3.0), firstArtifactPickupHeading)
        val thirdArtifact = firstArtifactBefore.fresh()
            .strafeToLinearHeading(firstArtifactPose + (firstOffsetPose * 5.0), firstArtifactPickupHeading)

        val toLaunchTwo = thirdArtifact.fresh()
            .strafeToLinearHeading(launchPose.position, launchPose.heading)

        val fourArtifactBefore = toLaunchTwo.fresh()
            .strafeToLinearHeading(fourthArtifactPose - (fourthOffsetPose * 2.0), fourthArtifactPickupHeading)
//        val fourArtifact = fourArtifactBefore.fresh()
//            .strafeToLinearHeading(fourthArtifactPose + (fourthOffsetPose * 2.0), fourthArtifactPickupHeading)
//        val fiveArtifact = fourArtifactBefore.fresh()
//            .strafeToLinearHeading(fourthArtifactPose + (fourthOffsetPose * 3.0), fourthArtifactPickupHeading)
        val sixArtifact = fourArtifactBefore.fresh()
            .strafeToLinearHeading(fourthArtifactPose + (fourthOffsetPose * 5.0), fourthArtifactPickupHeading)

        val toLaunchThree = sixArtifact.fresh()
            .strafeToLinearHeading(launchPose.position, launchPose.heading)

        val seventhArtifactBefore = toLaunchThree.fresh()
            .strafeToLinearHeading(seventhArtifactPose - (seventhOffsetPose * 2.0), seventhArtifactPickupHeading)
//        val sevenArtifact = seventhArtifactBefore.fresh()
//            .strafeToLinearHeading(seventhArtifactPose + (seventhOffsetPose * 2.0), seventhArtifactPickupHeading)
//        val eightArtifact = seventhArtifactBefore.fresh()
//            .strafeToLinearHeading(seventhArtifactPose + (seventhOffsetPose * 3.0), seventhArtifactPickupHeading)
        val nineArtifact = seventhArtifactBefore.fresh()
            .strafeToLinearHeading(seventhArtifactPose + (seventhOffsetPose * 5.0), seventhArtifactPickupHeading)

        val toLaunchFour = nineArtifact.fresh()
            .strafeToLinearHeading(launchPose.position, launchPose.heading)
        val toPark = toLaunchFour.fresh() // park
            .strafeToLinearHeading(routeParameters.parkPose.position, routeParameters.parkPose.heading).build()

        val toLaunchAction = toLaunch.build()

        val firstGroup = SequentialAction(
            // intake more,
            firstArtifactBefore.build(),
//            ParallelAction(
//                firstArtifact.build(),
//                out.intakeUntilDetectedOrTimeout()
//            ),
//            SleepAction(0.5),
//            ParallelAction(
//                secondArtifact.build(),
//                out.intakeUntilDetectedOrTimeout()
//            ),
//            SleepAction(0.5),
            ParallelAction(
                thirdArtifact.build(),
                out.intakeUntilIndexedOrTimeout(),
                out.intakeUntilIndexedOrTimeout(),
                out.intakeUntilIndexedOrTimeout(),
            ),
            toLaunchTwo.build(),
        )

        val secondGroup = SequentialAction(
            // intake more,
            fourArtifactBefore.build(),
//            ParallelAction(
//                fourArtifact.build(),
//                out.intakeUntilDetectedOrTimeout()
//            ),
//            SleepAction(0.5),
//            ParallelAction(
//                fiveArtifact.build(),
//                out.intakeUntilDetectedOrTimeout()
//            ),
//            SleepAction(0.5),
            ParallelAction(
                sixArtifact.build(),
                out.intakeUntilIndexedOrTimeout(),
                out.intakeUntilIndexedOrTimeout(),
                out.intakeUntilIndexedOrTimeout(),
            ),
            toLaunchThree.build(),
        )

        val thirdGroup = SequentialAction(
            // intake more,
            seventhArtifactBefore.build(),
//            ParallelAction(
//                sevenArtifact.build(),
//                out.intakeUntilDetectedOrTimeout()
//            ),
//            SleepAction(0.5),
//            ParallelAction(
//                eightArtifact.build(),
//                out.intakeUntilDetectedOrTimeout()
//            ),
//            SleepAction(0.50),
            ParallelAction(
                nineArtifact.build(),
                out.intakeUntilIndexedOrTimeout(),
                out.intakeUntilIndexedOrTimeout(),
                out.intakeUntilIndexedOrTimeout(),
            ),
            toLaunchFour.build(),
        )

        telemetry.addData("INIT STATUS", "READY")
        telemetry.update()

        waitForStart() // setup done actually do things

        out.start()
        limelight?.start()

        var motif: Motif? = null
        val timer = ElapsedTime()

        while (motif == null && timer.seconds() < 2.0) { // find tag or abort if too long
            motif = when (limelight?.getAprilTags()?.firstOrNull { it.id == 23 || it.id == 22 || it.id == 21 }?.id) {
                23 -> Motif.PURPLE_PURPLE_GREEN
                22 -> Motif.PURPLE_GREEN_PURPLE
                21 -> Motif.GREEN_PURPLE_PURPLE
                else -> null // default
            }
        }
        telemetry.addData("Detected Motif", motif)
        telemetry.update()
        motif = motif ?: Motif.GREEN_PURPLE_PURPLE // default

        out.runBlockingAndUpdate(toLaunchAction)
        out.intakeOn()
        out.runBlockingAndUpdate(out.launchAllHeld(launchDistance))
        out.runBlockingAndUpdate(firstGroup)
        out.runBlockingAndUpdate(out.launchAllHeld(launchDistance))
        out.runBlockingAndUpdate(secondGroup)
        out.runBlockingAndUpdate(out.launchAllHeld(launchDistance))
        out.runBlockingAndUpdate(thirdGroup)
        out.runBlockingAndUpdate(out.launchAllHeld(launchDistance))
        out.runBlockingAndUpdate(toPark)

        out.stop()
        limelight?.stop()

        blackboard["robotHeading"] = roadrunnerDrive.localizer.pose.heading.toDouble()
    }

    protected enum class Locations(val teamColor: TeamColor, val startPose: Pose2d) {
        //BlueClose(TeamColor.BLUE, Pose2d(-55.0, -56.0, 3 * -PI / 4)), cursed positioning hack below
        BlueClose(TeamColor.BLUE, Pose2d(-55.5, -50.0, PI / 2)),
        BlueFar(TeamColor.BLUE, Pose2d(63.0, -24.0, -PI)),
//        RedClose(TeamColor.RED, Pose2d(-55.0, 50.0, 3 * PI / 4)),
        RedClose(TeamColor.RED, Pose2d(-85.0, 50.0, -PI / 2)),
        RedFar(TeamColor.RED, Pose2d(53.0, 24.0, -PI)),
        Unknown(TeamColor.UNKNOWN, Pose2d(0.0, 0.0, 0.0)),
    }

    private data class RouteParameters(
        val launchPose: Pose2d,
        val launchDistance: OuttakeV4.LaunchDistance,
        val firstArtifactRow: ArtifactPositions,
        val secondArtifactRow: ArtifactPositions,
        val thirdArtifactRow: ArtifactPositions,
        val parkPose: Pose2d,
    )

    private enum class ArtifactPositions(val offsetSign: Double, val pose: Pose2d) { // todo better way to do offset
//        RED_CLOSE(1.0, Pose2d(-12.0, 34.0, PI / 2)),
//        RED_MID(1.0, Pose2d(12.0, 34.0, PI / 2)),
//        RED_FAR(1.0, Pose2d(36.0, 34.0, PI / 2)),
//        BLUE_CLOSE(-1.0, Pose2d(-12.0, -34.0, -PI / 2)),
//        BLUE_MID(-1.0, Pose2d(12.0, -34.0, -PI / 2)),
//        BLUE_FAR(-1.0, Pose2d(36.0, -34.0, -PI / 2)),
        RED_CLOSE(1.0, Pose2d(-18.0, 34.0, PI / 2)),
        RED_MID(1.0, Pose2d(3.0, 34.0, PI / 2)),
        RED_FAR(1.0, Pose2d(27.0, 34.0, PI / 2)),
        RED_HUMAN_PLAYER(1.0, Pose2d(60.0, 64.0, PI / 2)),
        BLUE_CLOSE(-1.0, Pose2d(-18.0, -34.0, -PI / 2)),
        BLUE_MID(-1.0, Pose2d(3.0, -34.0, -PI / 2)),
        BLUE_FAR(-1.0, Pose2d(27.0, -34.0, -PI / 2)),
        BLUE_HUMAN_PLAYER(1.0, Pose2d(60.0, -64.0, -PI / 2)),
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
