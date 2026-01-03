package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.misc.AxonDriver

@TeleOp
class AxonEncoderTester : LinearOpMode() {
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val axon = AxonDriver(
            hardwareMap,
            "spindexerAxon",
            "spindexerEncoder",
            0.0,
            0.0,
            0.0,
            telemetry,
            2.0
        )

        waitForStart()
        axon.targetPosition = 0.0
        while (!isStopRequested) {
            if (gamepad1.leftBumperWasPressed()) {
                //axon.overridePower = 0.2
                axon.targetPosition = axon.targetPosition?.plus(60.0)
            } else if (gamepad1.rightBumperWasPressed()) {
                axon.targetPosition = axon.targetPosition?.minus(60.0)
            } else if (gamepad1.cross) {
                axon.reset()
            } else {
                //axon.overridePower = 0.0
            }

            axon.targetPosition = axon.targetPosition

            telemetry.update()
        }

        axon.cleanUp()
    }
}