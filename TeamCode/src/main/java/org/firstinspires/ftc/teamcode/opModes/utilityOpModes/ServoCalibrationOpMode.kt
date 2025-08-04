package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@Config
@TeleOp(name="Servo Calibration Mode")
class ServoCalibrationOpMode : LinearOpMode() {
    override fun runOpMode() {
        val servo = hardwareMap.get(Servo::class.java, SERVO_NAME)
        servo.position = 0.5
        waitForStart()
        while (opModeIsActive()){
            if (gamepad1.left_bumper) servo.position += 0.01
            if (gamepad1.right_bumper) servo.position -= 0.01
            telemetry.addData("Servo Position", servo.position)
            telemetry.update()
            sleep(50)
        }
    }

    companion object {
        @JvmField var SERVO_NAME = "bucketServo"
    }
}