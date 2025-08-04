package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismClassManagerKt
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt.HardwareMechanismSingletonManager
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt.DriveMode
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.GlobalsKt
import org.firstinspires.ftc.teamcode.sensor.IMUSensorKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.reflect.full.companionObjectInstance

/*
    TODO: Build web-tool that allows robot configuration i.e driver station (ftc-dash)
    TODO: Add a modification to ftc-dash allowing multiple camera sources.
 */

abstract class UnifiedTeleOpKt : LinearOpMode() {
    /** This field may be immediately changed by the switch state update. */
    abstract val orientationMode: DriveMode
    abstract val teamColor: TeamColor

    private val currentGamepadOne = Gamepad()
    private val currentGamepadTwo = Gamepad()

    private val mechanisms: MutableList<HardwareMechanismKt> = mutableListOf()

    // Manually added exceptions to bypass button duplication checks.
    private val buttonDuplicationExceptions: Array<GamepadButtons> = arrayOf()

    override fun runOpMode() {
        GlobalsKt.initBulkReads(hardwareMap)

        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val imu = IMUSensorKt.getInstance(
            hardwareMap,
            SensorDeviceKt.SensorInitData(teamColor, FtcDashboard.getInstance().isEnabled),
            telemetry
        ) ?: throw RuntimeException("Failed to initialize IMU.")

        // InitData
        val initData = HardwareMechanismKt.InitData(
            teamColor = teamColor,
            driveMode = orientationMode,
            dashboardEnabled = FtcDashboard.getInstance().isEnabled
        )

        // Get a list of all HardwareMechanism classes
        val classes = HardwareMechanismClassManagerKt.getMechanisms()
        val buttons: HashSet<GamepadButtons> = hashSetOf()
        for (clazz in classes){
            // Instantiate each
            val mech = (clazz.companionObjectInstance as HardwareMechanismSingletonManager<*>)
                .getInstance(hardwareMap, initData, telemetry) ?: continue

            for (button in mech.usedButtons) {
                if (!buttons.add(button) && !buttonDuplicationExceptions.contains(button)) {
                    // button is already in array & isn't in the exception list
                    throw RuntimeException("WARNING! Duplicate button detected. Button: $button. If this was intentional, you must add the button to the exception list.")
                }
            }
            // If all is well, add the mechanism to our list
            mechanisms.add(mech)
        }

        telemetry.update()

        waitForStart()
        for (mechanism in mechanisms) mechanism.start()

        val timer = ElapsedTime()
        while (opModeIsActive()) {
            timer.reset()

            currentGamepadOne.copy(gamepad1)
            currentGamepadTwo.copy(gamepad2)

            // There are still some situations where code may need to be placed in this loop.
            // For example, purposes which MUST use components from multiple mechanisms or need to
            // temporarily seize control of the entire control loop. That code can go here.

            val runData = HardwareMechanismKt.RunData(
                currentGamepadOne,
                currentGamepadTwo,
                imuAngleRad = imu.poll().toDouble()
            )

            for (mechanism in mechanisms) mechanism.run(runData)

            telemetry.addData("Time This Loop (ms)", timer.milliseconds())
            telemetry.update()
        }

        for (mechanism in mechanisms) mechanism.stop()
    }
}

@TeleOp(name = "Blue TeleOp Controls (Field)", group = "Competition")
class FieldTeleOpOpModeBlueKt : UnifiedTeleOpKt() {
    override val orientationMode = HardwareMechanismKt.DriveMode.FIELD
    override val teamColor = TeamColor.BLUE
}

@TeleOp(name = "Red TeleOp Controls (Field)", group = "Competition")
class FieldTeleOpOpModeRedKt : UnifiedTeleOpKt() {
    override val orientationMode = HardwareMechanismKt.DriveMode.FIELD
    override val teamColor = TeamColor.RED
}

@TeleOp(name = "Blue TeleOp Controls (Robot)", group = "Competition")
class RobotTeleOpOpModeBlueKt : UnifiedTeleOpKt() {
    override val orientationMode = HardwareMechanismKt.DriveMode.ROBOT
    override val teamColor = TeamColor.BLUE
}

@TeleOp(name = "Red TeleOp Controls (Robot)", group = "Competition")
class RobotTeleOpOpModeRedKt : UnifiedTeleOpKt() {
    override val orientationMode = HardwareMechanismKt.DriveMode.ROBOT
    override val teamColor = TeamColor.RED
}
