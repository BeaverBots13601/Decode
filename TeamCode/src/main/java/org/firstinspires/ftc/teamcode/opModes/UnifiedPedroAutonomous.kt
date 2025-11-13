package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.hardware.InOuttake
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.PI

@Autonomous(name="Automatic Pedro Autonomous", group="Competition")
@Disabled
open class UnifiedPedroAutonomousKt : LinearOpMode() {
    //private val propLocation: WebcamPropIdentificationKt.PropLocation
    //private val propId: WebcamPropIdentificationKt

    // Subclass variables
    protected open val pathToFollow = Path.Standard
    protected open val currentLocation = Locations.Unknown


    @Suppress("PROPERTY_HIDES_JAVA_FIELD") // intentional
    private val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    private val drive by lazy { Constants.createFollower(hardwareMap) }

    override fun runOpMode() {
        val inout = InOuttake.getInstance(hardwareMap, HardwareMechanismKt.InitData(
            currentLocation.teamColor,
            HardwareMechanismKt.DriveMode.ROBOT,
            FtcDashboard.getInstance().isEnabled,
            currentLocation.startPose.heading, // todo needs conversion?
        ), telemetry)

        if (inout == null) throw Error("Inout Initialization Failed")
        initBulkReads(hardwareMap)
        blackboard.remove("robotHeading")

        drive.setStartingPose(currentLocation.startPose)

        // todo smartly rotate poses well instead of duplicating
        when(currentLocation) {
            Locations.BlueFar -> { // canonical far

            }
            Locations.RedFar -> { // blue far but rotated

            }
            Locations.BlueClose -> { // canonical close

            }
            Locations.RedClose -> { // blue close but rotated

            }
            else -> { return }
        }

        telemetry.addData("INIT STATUS", "READY")
        telemetry.update()

        waitForStart() // setup done actually do things

        //inout.start() // cursed comment out

        inout.stop()

        blackboard["robotHeading"] = drive.pose.heading
    }

    protected enum class Locations(val teamColor: TeamColor, val startPose: Pose) {
        BlueClose(TeamColor.BLUE, Pose(55.0, 56.0, 3 * -PI / 4)),
        BlueFar(TeamColor.BLUE, Pose(-66.0, 24.0, -PI)),
        RedClose(TeamColor.RED, Pose(55.0, -56.0, 3 * PI / 4)),
        RedFar(TeamColor.RED, Pose(-66.0, -24.0, -PI)),
        Unknown(TeamColor.UNKNOWN, Pose(0.0, 0.0, 0.0)),
    }

    protected enum class Path {
        Standard,
        Alternate,
    }
}

@Autonomous(name = "Blue Close Pedro Autonomous", group = "Competition", preselectTeleOp = "Blue TeleOp Controls (Robot)")
class BlueClosePedroAutonomousKt : UnifiedPedroAutonomousKt() {
    override val currentLocation = Locations.BlueClose
}

@Autonomous(name = "Blue Far Pedro Autonomous", group = "Competition", preselectTeleOp = "Blue TeleOp Controls (Robot)")
class BlueFarPedroAutonomousKt : UnifiedPedroAutonomousKt() {
    override val currentLocation = Locations.BlueFar
}

@Autonomous(name = "Red Close Pedro Autonomous", group = "Competition", preselectTeleOp = "Red TeleOp Controls (Robot)")
class RedClosePedroAutonomousKt : UnifiedPedroAutonomousKt() {
    override val currentLocation = Locations.RedClose
}

@Autonomous(name = "Red Far Pedro Autonomous", group = "Competition", preselectTeleOp = "Red TeleOp Controls (Robot)")
class RedFarPedroAutonomousKt : UnifiedPedroAutonomousKt() {
    override val currentLocation = Locations.RedFar
}
