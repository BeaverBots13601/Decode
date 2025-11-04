package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.DcMotorSimple

// TODO: Make sure these directions match the current Drivetrain

@Suppress("EnumEntryName") // Reason: Enum names map to hardware map config, where
// convention is camelCase instead of PascalCase
enum class DriveMotors(val direction: DcMotorSimple.Direction) { // Expected to be same for foreseeable future
    leftFront(DcMotorSimple.Direction.REVERSE),
    leftBack(DcMotorSimple.Direction.REVERSE),
    rightFront(DcMotorSimple.Direction.FORWARD),
    rightBack(DcMotorSimple.Direction.FORWARD),
}