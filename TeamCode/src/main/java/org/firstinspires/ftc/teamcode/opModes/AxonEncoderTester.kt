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
            0.002,
            0.012,
            0.00022,
            telemetry,
        )

        waitForStart()
        axon.start()
        axon.targetPosition = 0.0
        while (!isStopRequested) {
            if (gamepad1.leftBumperWasPressed()) {
                //axon.overridePower = 0.2
                axon.targetPosition = axon.targetPosition?.plus(120.0)
            } else if (gamepad1.rightBumperWasPressed()) {
                axon.targetPosition = axon.targetPosition?.minus(120.0)
            } else if (gamepad1.cross) {
                axon.reset()
            } else {
                //axon.overridePower = 0.0
            }

            axon.update()

            telemetry.update()
        }

        axon.cleanUp()
    }
}