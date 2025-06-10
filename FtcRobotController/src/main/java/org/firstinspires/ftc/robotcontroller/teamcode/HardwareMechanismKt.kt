package org.firstinspires.ftc.robotcontroller.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class HardwareMechanismKt {
    var available: Boolean = false

    abstract fun start()

    abstract fun run(data: RunData)

    abstract fun stop()

    abstract val usedButtons: Array<GamepadButtons>

    open class HardwareMechanismSingletonManager<out T : HardwareMechanismKt>(private val constructor: (HardwareMap, InitData, Telemetry) -> T) {
        @Volatile
        private var instance: T? = null

        @Synchronized
        fun getInstance(hardwareMap: HardwareMap, data: InitData, telemetry: Telemetry): T? {
            if(instance == null) constructor(hardwareMap, data, telemetry).also {
                // .also runs a function with the output as 'it'
                if(it.available) instance = it
            }
            return instance
        }

        fun getInstance(): T? {
            return instance
        }
    }

    protected fun createDefaultMotor(hardwareMap: HardwareMap, motorName: String): DcMotorEx {
        val motor = hardwareMap.get(DcMotorEx::class.java, motorName)
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        if (motorName.lowercase().contains("left")){
            motor.direction = DcMotorSimple.Direction.REVERSE
        }
        return motor
    }

    data class InitData(
        val teamColor: TeamColor,
        val driveMode: DriveMode,
        val dashboardEnabled: Boolean,
    )

    data class RunData(
        val currentGamepadOne: Gamepad,
        val currentGamepadTwo: Gamepad,
        val previousGamepadOne: Gamepad,
        val previousGamepadTwo: Gamepad,
        val imuAngleRad: Double,
    )

    enum class DriveMode {
        FIELD,
        ROBOT,
    }
}