package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.misc.PIDVelocityController

@TeleOp
@Config
class FlywheelCalibrationOpmode : LinearOpMode() {
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val fmMotor = hardwareMap.get(DcMotorEx::class.java, "flywheel").apply {
            direction = DcMotorSimple.Direction.REVERSE
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        val flywheel = PIDVelocityController(
            fmMotor,
            0.0175,
            0.000002,
            0.00001,
            telemetry,
        )
        val intakeMotor = hardwareMap.get(DcMotorEx::class.java, "intakeMotor").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        val locker = hardwareMap.servo.get("locker")

        waitForStart()
        intakeMotor.power = 0.0
        locker.position = 0.55
        while (!isStopRequested) {
            if (POWER != 0.0) {
                fmMotor.power = POWER
                telemetry.addData("(Motor) PID Velocity", fmMotor.velocity)
            } else {
                flywheel.setVelocity(SPEED)
            }
            telemetry.update()
        }
    }

    companion object {
        @JvmField var SPEED = 0.0
        @JvmField var POWER = 0.0
    }
}