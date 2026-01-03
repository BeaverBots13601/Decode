package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ReadColorSensor : LinearOpMode() {
    override fun runOpMode() {
        val leftColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "leftIntakeColorSensor")
        val rightColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightIntakeColorSensor")
        val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (!isStopRequested) {
            val leftData = leftColorSensor.normalizedColors
            val rightData = rightColorSensor.normalizedColors
            telemetry.addData("Left Red", leftData.red)
            telemetry.addData("Left Blue", leftData.blue)
            telemetry.addData("Left Green", leftData.green)
            telemetry.addData("Right Red", rightData.red)
            telemetry.addData("Right Blue", rightData.blue)
            telemetry.addData("Right Green", rightData.green)
            telemetry.update()
        }
    }
}