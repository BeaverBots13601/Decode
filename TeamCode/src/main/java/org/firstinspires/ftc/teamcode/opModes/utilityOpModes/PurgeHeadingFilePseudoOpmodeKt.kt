package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name="Purge Heading")
class PurgeHeadingFilePseudoOpmodeKt : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("Are you sure you want to purge the heading? (If you're not sure, ask Grayson first.) The current heading is (rads)", blackboard.getOrDefault("robotHeading", "Unset"))
        telemetry.update()
        waitForStart()
        blackboard.remove("robotHeading")
    }
}