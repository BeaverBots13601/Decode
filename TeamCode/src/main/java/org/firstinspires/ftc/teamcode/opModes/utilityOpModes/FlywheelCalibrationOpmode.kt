package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.misc.AxonDriver
import org.firstinspires.ftc.teamcode.misc.DualMotorPIDVelocityController

@TeleOp
@Config
class FlywheelCalibrationOpmode : LinearOpMode() {
    override fun runOpMode() {
        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val fMotor = createDefaultMotor(hardwareMap, "flywheel").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        val counterspin = createDefaultMotor(hardwareMap, "counterspin")
        val flywheel = DualMotorPIDVelocityController(
            fMotor,
            counterspin,
            0.0175,
            0.000002,
            0.00001,
            telemetry,
        )
        val intakeMotor = hardwareMap.get(DcMotorEx::class.java, "intakeMotor").apply {
            direction = DcMotorSimple.Direction.REVERSE
        }
        val spindexer = AxonDriver(
            hardwareMap,
            "spindexerAxon",
            "spindexerEncoder",
            0.006,
            0.0005,
            0.00005,
            telemetry,
        ).apply {
            overridePower = -0.2
        }
        val booster = createDefaultMotor(hardwareMap, "boosterMotor")

        waitForStart()
        intakeMotor.power = 0.8
        booster.power = 1.0
        while (!isStopRequested) {
            if (POWER != 0.0) {
                fMotor.power = POWER
                counterspin.power = POWER
                telemetry.addData("(Motor) PID Velocity", (fMotor.velocity + counterspin.velocity) / 2)
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

    private fun createDefaultMotor(hardwareMap: HardwareMap, motorName: String): DcMotorEx {
        val motor = hardwareMap.get(DcMotorEx::class.java, motorName)
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        if (motorName.lowercase().contains("left")){
            motor.direction = DcMotorSimple.Direction.REVERSE
        }
        return motor
    }
}