package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.sensor.LimelightKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt

@TeleOp
class LimelightDebugger : LinearOpMode() {
    val initData = HardwareMechanismKt.InitData(
        teamColor = TeamColor.RED,
        driveMode = HardwareMechanismKt.DriveMode.ROBOT,
        dashboardEnabled = FtcDashboard.getInstance().isEnabled,
        referenceAngle = 0.0,
    )
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val limelight = LimelightKt.getInstance(hardwareMap, SensorDeviceKt.SensorInitData(initData), telemetry) ?: return
        waitForStart()
        while (!isStopRequested) {
            val tag = limelight.poll()?.firstOrNull() ?: continue
            val data = tag.targetPoseCameraSpace

            telemetry.addData("x pos", data.position.x)
            telemetry.addData("y pos", data.position.y)
            telemetry.addData("z pos", data.position.z)

            telemetry.addData("yaw", data.orientation.yaw)
            telemetry.addData("pitch", data.orientation.pitch)
            telemetry.addData("roll", data.orientation.roll)

            telemetry.addData("ID", tag.fiducialId)

            telemetry.update()
        }
        limelight.stop()
    }
}