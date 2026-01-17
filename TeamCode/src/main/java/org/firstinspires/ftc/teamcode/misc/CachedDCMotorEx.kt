package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.DcMotorEx

class CachedDCMotorEx(val motor: DcMotorEx) : DcMotorEx by motor {
    var cachedPower = motor.power
    override fun setPower(power: Double) {
        if (cachedPower == power) return
        cachedPower = power
        motor.power = power
    }

    var cachedVelocity = motor.velocity
    override fun setVelocity(velocity: Double) {
        if (cachedVelocity == velocity) return
        cachedVelocity = velocity
        motor.velocity = velocity
    }
}