package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismClassManagerKt
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt.HardwareMechanismSingletonManager
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt.DriveMode
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.sensor.IMUSensorKt
import org.firstinspires.ftc.teamcode.sensor.SensorDeviceKt
import kotlin.reflect.full.companionObjectInstance

/*
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
        initBulkReads(hardwareMap)

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
            dashboardEnabled = FtcDashboard.getInstance().isEnabled,
            referenceAngle = blackboard.getOrDefault("robotHeading", imu.poll().toDouble()) as Double,
        )

        // Get a list of all HardwareMechanism classes
        val classes = HardwareMechanismClassManagerKt.getMechanisms()
        val buttons: HashSet<GamepadButtons> = hashSetOf()
        for (clazz in classes){
            telemetry.addData(clazz.simpleName + " Loaded", false)
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
            telemetry.addData(clazz.simpleName + " Loaded", true)
        }

        telemetry.update()

        waitForStart()
        telemetry.clear()
        for (mechanism in mechanisms) mechanism.start()
        imu.start()

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
                imuAngleRad = 0.0 // moderately safe hack to reduce cycle times
                // how roadrunner stop so fast in manual ff tune??
            )

            for (mechanism in mechanisms) mechanism.run(runData)

            telemetry.addData("Time This Loop (ms)", timer.milliseconds())
            telemetry.addData("Time This Loop (hz)", 1 / timer.seconds())
            telemetry.update()
        }

        for (mechanism in mechanisms) mechanism.stop()
        imu.stop()
    }
}

@TeleOp(name = "Blue TeleOp Controls (Field)", group = "Competition")
class FieldTeleOpOpModeBlueKt : UnifiedTeleOpKt() {
    override val orientationMode = DriveMode.FIELD
    override val teamColor = TeamColor.BLUE
}

@TeleOp(name = "Red TeleOp Controls (Field)", group = "Competition")
class FieldTeleOpOpModeRedKt : UnifiedTeleOpKt() {
    override val orientationMode = DriveMode.FIELD
    override val teamColor = TeamColor.RED
}

@TeleOp(name = "Blue TeleOp Controls (Robot)", group = "Competition")
class RobotTeleOpOpModeBlueKt : UnifiedTeleOpKt() {
    override val orientationMode = DriveMode.ROBOT
    override val teamColor = TeamColor.BLUE
}

@TeleOp(name = "Red TeleOp Controls (Robot)", group = "Competition")
class RobotTeleOpOpModeRedKt : UnifiedTeleOpKt() {
    override val orientationMode = DriveMode.ROBOT
    override val teamColor = TeamColor.RED
}

/**
 * Enables bulk reads which allow faster hardware call times.
 * Set to auto currently but if speed becomes an issue this can be manually configured.
 * TODO: Re-home me.
 */
fun initBulkReads(hardwareMap: HardwareMap){
    val allHubs = hardwareMap.getAll(LynxModule::class.java)
    for (hub in allHubs) {
        hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
    }
}
