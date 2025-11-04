package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.misc.PIDVelocityController
import kotlin.jvm.java

@TeleOp
class PIDVelocityTuner : LinearOpMode() {
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val a = PIDVelocityController(
            hardwareMap.get(DcMotorEx::class.java, "leftMotor"),
            0.0175,
            0.000002,
            0.00001,
            telemetry,
        )
        val b = hardwareMap.get(DcMotorEx::class.java, "rightMotor")
        b.mode = DcMotor.RunMode.RUN_USING_ENCODER

        waitForStart()

        var c = true
        var d: Double
        while (opModeIsActive()) {
            if (c) {
                a.velocity = 1000.0
                b.velocity = 1000.0
                d = 1000.0

                if (gamepad2.circleWasPressed()) c = false
            } else {
                a.velocity = -1000.0
                b.velocity = -1000.0
                d = -1000.0

                if (gamepad2.circleWasPressed()) c = true
            }

            telemetry.addData("Velocity", ((a.velocity + b.velocity) / 2))
            telemetry.addData("Target", d)
            telemetry.addData("Error", d - ((a.velocity + b.velocity) / 2))
            telemetry.update()
        }
    }
}