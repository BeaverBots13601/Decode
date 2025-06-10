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
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt.DriveMode
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.teamcode.Globals
import org.firstinspires.ftc.teamcode.sensor.IMUSensor
import org.firstinspires.ftc.teamcode.sensor.SensorDevice

@TeleOp
open class UnifiedTeleOpKt : LinearOpMode() {
    protected val orientationMode = DriveMode.ROBOT // override me
    protected val teamColor = TeamColor.BLUE // override me

    private val currentGamepadOne = Gamepad()
    private val currentGamepadTwo = Gamepad()
    private val previousGamepadOne = Gamepad()
    private val previousGamepadTwo = Gamepad()

    private val mechanisms: MutableList<HardwareMechanismKt> = mutableListOf()

    private val buttonDuplicationExceptions: Array<GamepadButtons> = arrayOf()

    override fun runOpMode() {
        Globals.initBulkReads(hardwareMap)

        val telemetry = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        val imu = IMUSensor(hardwareMap, SensorDevice.SensorInitData(), telemetry)

        // InitData
        val initData = HardwareMechanismKt.InitData(
            teamColor = teamColor,
            driveMode = orientationMode,
            dashboardEnabled = FtcDashboard.getInstance().isEnabled
        )

        val classes = HardwareMechanismClassManagerKt.getMechanisms()
        val buttons: HashSet<GamepadButtons> = hashSetOf()
        for (clazz in classes){
            try {
                // Instantiate each
                val mech: HardwareMechanismKt = /*clazz todo*/ null ?: break

                for (button in mech.usedButtons) {
                    if (!buttons.add(button) && !buttonDuplicationExceptions.contains(button)) {
                        throw RuntimeException("WARNING! Duplicate button detected. Button: $button. If this was intentional, you must add the button to the exception list.")
                    }
                }
                // If all is well, add the mechanism to our list
                mechanisms.add(mech)
            } catch (e: Exception) { throw RuntimeException(e) }
        }

        previousGamepadOne.copy(currentGamepadOne)
        previousGamepadTwo.copy(currentGamepadTwo)

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
                previousGamepadOne,
                currentGamepadTwo,
                previousGamepadTwo,
                imuAngleRad = imu.poll().toDouble()
            )

            for (mechanism in mechanisms) mechanism.run(runData)

            previousGamepadOne.copy(currentGamepadOne)
            previousGamepadTwo.copy(currentGamepadTwo)
            telemetry.addData("Time This Loop (ms)", timer.milliseconds())
            telemetry.update()
        }

        for (mechanism in mechanisms) mechanism.stop()
    }
}