package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismClassManager;
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;
import org.firstinspires.ftc.teamcode.sensor.SensorDevice;
import org.firstinspires.ftc.teamcode.sensor.IMUSensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

/*
    TODO: Build web-tool that allows robot configuration i.e driver station (ftc-dash)
    TODO: Add a modification to ftc-dash allowing multiple camera sources.
 */

public abstract class UnifiedTeleOp extends LinearOpMode {
    /** This field may be immediately changed by the switch state update. */
    protected HardwareMechanism.DriveMode orientationMode = HardwareMechanism.DriveMode.ROBOT; // override me
    protected TeamColor teamColor = TeamColor.BLUE; // override me
    private Gamepad currentGamepadOne = new Gamepad();
    private Gamepad previousGamepadOne = new Gamepad();
    private Gamepad currentGamepadTwo = new Gamepad();
    private Gamepad previousGamepadTwo = new Gamepad();
    private ArrayList<HardwareMechanism> mechanisms = new ArrayList<>();

    // Manually added exceptions to bypass button duplication checks.
    private static final List<GamepadButtons> buttonDuplicationExceptions = Arrays.asList();
    public void runOpMode() {
        Globals.initBulkReads(hardwareMap);
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        IMUSensor imu = new IMUSensor(hardwareMap, new SensorDevice.SensorInitData(), telemetry);

        // InitData
        HardwareMechanism.InitData data = new HardwareMechanism.InitData();
        data.teamColor = teamColor;
        data.driveMode = orientationMode;
        data.dashboardEnabled = FtcDashboard.getInstance().isEnabled();

        // Get a list of all HardwareMechanism classes
        List<Class<HardwareMechanism>> classes = HardwareMechanismClassManager.getMechanisms();
        HashSet<GamepadButtons> buttons = new HashSet<>();
        for (Class<HardwareMechanism> clazz : classes){
            try {
                // Instantiate each
                HardwareMechanism mech = clazz.getDeclaredConstructor(HardwareMap.class, HardwareMechanism.InitData.class, Telemetry.class).newInstance(hardwareMap, data, telemetry);
                // we do this sanity checking before determining whether the class is valid to catch issues earlier in dev
                for (GamepadButtons button : mech.getUsedButtons()){
                    if (!buttons.add(button) && !buttonDuplicationExceptions.contains(button)){
                        // button is already in array & isn't in the exception list
                        throw new RuntimeException("WARNING! Duplicate button detected. Button: " + button + ". If this was intentional, you must add the button to the exception list.");
                    }
                }
                // If all is well, add the mechanism to our list
                if (mech.available) mechanisms.add(mech);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        previousGamepadOne.copy(currentGamepadOne);
        previousGamepadTwo.copy(currentGamepadTwo);

        telemetry.update();

        waitForStart();
        for (HardwareMechanism mechanism : mechanisms){
            mechanism.start();
        }

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            timer.reset();

            currentGamepadOne.copy(gamepad1);
            currentGamepadTwo.copy(gamepad2);

            // There are still some situations where code may need to be placed in this loop.
            // For example, purposes which MUST use components from multiple mechanisms or need to
            // temporarily seize control of the entire control loop. That code can go here.

            HardwareMechanism.RunData runData = new HardwareMechanism.RunData();
            runData.currentGamepadOne = currentGamepadOne;
            runData.previousGamepadOne = previousGamepadOne;
            runData.currentGamepadTwo = currentGamepadTwo;
            runData.previousGamepadTwo = previousGamepadTwo;
            runData.imuAngleRad = imu.poll();

            for (HardwareMechanism mechanism : mechanisms){
                mechanism.run(runData);
            }

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);
            telemetry.addData("Time This Loop (ms)", timer.milliseconds());
            telemetry.update();
        }

        for (HardwareMechanism mechanism : mechanisms){
            mechanism.stop();
        }
    }
}