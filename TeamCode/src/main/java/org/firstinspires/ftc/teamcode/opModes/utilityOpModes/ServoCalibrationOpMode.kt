package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@Config
@TeleOp(name="Servo Calibration Mode")
class ServoCalibrationOpMode : LinearOpMode() {
    override fun runOpMode() {
        hardwareMap.crservo.forEach { // stop all servos (hitecs suck)
            it.power = 0.0
        }
        hardwareMap.servo.forEach { // center all servos
            it.position = 0.5
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val servo = hardwareMap.servo.get(SERVO_NAME)
        servo.direction = Servo.Direction.REVERSE
        //val servo2 = hardwareMap.servo.get(SERVO_NAME_2)
        servo.position = 0.5
        //servo2.position = 0.5
        waitForStart()
        while (opModeIsActive()){
            if (gamepad1.left_bumper) {
                servo.position += 0.01
                //servo2.position += 0.01
            }
            if (gamepad1.right_bumper) {
                servo.position -= 0.01
                //servo2.position -= 0.01
            }
            telemetry.addData("Servo Position", servo.position)
            //telemetry.addData("Servo 2 Position", servo2.position)
            telemetry.update()
            sleep(50)
        }
    }

    companion object {
        @JvmField var SERVO_NAME = "firstServo"
        @JvmField var SERVO_NAME_2 = "secondServo"
    }
}