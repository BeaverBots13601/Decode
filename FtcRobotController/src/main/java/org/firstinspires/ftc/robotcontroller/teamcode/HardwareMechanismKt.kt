package org.firstinspires.ftc.robotcontroller.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * A class representing one combined mechanism on the robot, consisting of all related hardware components and functions which are called on start, stop, and every loop. Implementations of this class should expose such functions as are needed for autonomous.
 *
 * All discovered [HardwareMechanismKt]s are loaded automatically within the TeleOp. Within autonomous they must be manually instantiated.
 *
 * Subclasses should include a private constructor(HardwareMap, InitData, Telemetry). Subclasses should also have a companion object extending [HardwareMechanismSingletonManager], supplied with the subclass type and a reference to the constructor.
 */
abstract class HardwareMechanismKt {
    /**
     * Call after doing waitForStart(). Allows the class to do setup that can only legally be done after starting.
     */
    abstract fun start()

    /**
     * The main loop function, meant to be called every TeleOp loop.
     */
    abstract fun run(data: RunData)

    /**
     * Call after ending the main program. Allows the class to do cleanup.
     */
    abstract fun stop()

    /**
     * The list of all [GamepadButtons] this hardware mechanism uses in [run].
     */
    abstract val usedButtons: Array<GamepadButtons>

    open class HardwareMechanismSingletonManager<out T : HardwareMechanismKt>(private val constructor: (HardwareMap, InitData, Telemetry) -> T) {
        @Volatile
        private var instance: T? = null

        @Synchronized
        fun getInstance(hardwareMap: HardwareMap, data: InitData, telemetry: Telemetry): T? {
            instance = null
            runCatching {
                instance = constructor(hardwareMap, data, telemetry)
            }
            return instance
        }

        fun getInstance(): T? {
            return instance
        }
    }

    /**
     * Creates a default motor with the settings [RUN_USING_ENCODER][DcMotor.RunMode.RUN_USING_ENCODER] and ['FLOAT on zero power'][DcMotor.ZeroPowerBehavior.FLOAT].
     * Reverses the direction if the name includes left.
     */
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

    /**
     * Sleeps. Warning: The main loop working well depends on [run] functions taking only a short time.
     * Sleeping usually disrupts this illusion and therefore should only be done in situations
     * where it's okay to completely cease driver control. (Also note there is no abort.)
     */
    protected fun sleep(millis: Long){ kotlin.runCatching { Thread.sleep(millis) } }

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