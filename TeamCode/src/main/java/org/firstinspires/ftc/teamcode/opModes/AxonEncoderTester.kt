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
            "turntableAxon",
            "turntableEncoder",
            0.0,
            0.0,
            0.0,
            telemetry,
            5.4
        )

        waitForStart()
        while (!isStopRequested) {
            if (gamepad1.left_bumper) {
                axon.overridePower = 0.4
            } else if (gamepad1.right_bumper) {
                axon.overridePower = -0.4
            } else if (gamepad1.x) {
                axon.resetEncoder()
            } else {
                axon.overridePower = 0.0
            }

            telemetry.addData("Normalized Position", axon.normalizedPosition)
            telemetry.addData("Position", axon.position)
            telemetry.update()
        }

        axon.cleanUp()
    }
}