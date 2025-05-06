package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.misc.Pose;

import java.util.Arrays;
import java.util.List;

/**
 * Responsible for managing our four-wheel Mecanum drive. Includes 3 levels of variable speed.
 */
public class DriveTrain extends HardwareMechanism {
    @Nullable
    private static DriveTrain instance = null;

    private DriveMode orientationMode;
    private DcMotorEx[] driveMotors = new DcMotorEx[driveMotorName.values().length];
    @Nullable
    private DigitalChannel switch_ = null;
    private double referenceAngle;
    private boolean dashboardEnabled;
    private SPEEDS currentSpeedMode = SPEEDS.NORMAL;
    private DriveTrain(HardwareMap hardwareMap, InitData data, Telemetry telemetry){
        super(hardwareMap, data, telemetry);
        try {
            createDriveMotors(hardwareMap);
            try {
                // optional hardware
                switch_ = hardwareMap.get(DigitalChannel.class, "switch");
            } catch (Exception ignored){}
        } catch (Exception e) {
            available = false;
            return;
        }

        orientationMode = data.driveMode;
        dashboardEnabled = data.dashboardEnabled;

        referenceAngle = Globals.robotHeading; // saved from auto, or 0 by default.

        available = true;
    }

    public static @Nullable DriveTrain getInstance(){
        return instance;
    }

    public static @Nullable DriveTrain getInstance(HardwareMap hardwareMap, InitData data, Telemetry telemetry){
        instance = new DriveTrain(hardwareMap, data, telemetry);
        if (!instance.available) instance = null;
        return instance;
    }

    public void start() {

    }

    public void run(RunData data) {
        if (getSwitchState()){
            // if no switch is attached, fall back to robot mode.
            orientationMode = HardwareMechanism.DriveMode.ROBOT;
        } else {
            orientationMode = HardwareMechanism.DriveMode.FIELD;
        }

        double speedNow = currentSpeedMode.getNumericalSpeed();

        int tmp_deadzoneadjust = 2;

        float stickX = data.currentGamepadOne.left_stick_x * tmp_deadzoneadjust;
        float stickY = -data.currentGamepadOne.left_stick_y * tmp_deadzoneadjust;
        float stickRotation = data.currentGamepadOne.right_stick_x * tmp_deadzoneadjust;

        telemetry.addData("Current Orientation Mode", orientationMode);
        double directionRotation = 0;
        if (orientationMode == DriveMode.FIELD) {
            directionRotation = -Pose.normalizeAngle(data.imuAngleRad - referenceAngle);
        }

        Pose rotatedPosition = Pose.rotatePosition(stickX, stickY, directionRotation);
        double rotatedStickX = rotatedPosition.getX();
        double rotatedStickY = rotatedPosition.getY();
        double orientation = data.imuAngleRad;
        telemetry.addData("IMU DATA (rads)", orientation);
        telemetry.addData("Reference Angle (rads)", Globals.robotHeading);

        double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

        double leftFrontPower = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speedNow;
        double leftBackPower = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speedNow;
        double rightFrontPower = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speedNow;
        double rightBackPower = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speedNow;

        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Back Power", leftBackPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Back Power", rightBackPower);
        telemetry.addData("Current Speed Mode", currentSpeedMode);

        setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // speed ctrls (gp 1)
        if (data.currentGamepadOne.dpad_right && !data.previousGamepadOne.dpad_right) {
            currentSpeedMode = SPEEDS.FAST;
        }
        if (data.currentGamepadOne.dpad_up && !data.previousGamepadOne.dpad_up) {
            currentSpeedMode = SPEEDS.NORMAL;
        }
        if (data.currentGamepadOne.dpad_left && !data.previousGamepadOne.dpad_left) {
            currentSpeedMode = SPEEDS.SLOW;
        }
        if (data.currentGamepadOne.dpad_down && !data.previousGamepadOne.dpad_down && dashboardEnabled) {
            currentSpeedMode = SPEEDS.CUSTOM_FTC_DASHBOARD;
        }
    }

    public void stop() {}

    public List<GamepadButtons> getUsedButtons() {
        return Arrays.asList(
                GamepadButtons.GP1_LEFT_JOYSTICK,
                GamepadButtons.GP1_RIGHT_JOYSTICK,
                GamepadButtons.GP1_DPAD_RIGHT,
                GamepadButtons.GP1_DPAD_UP,
                GamepadButtons.GP1_DPAD_LEFT,
                GamepadButtons.GP1_DPAD_DOWN
        );
    }

    private enum driveMotorName { // expecting to be same for forseeable future
        leftFront, leftBack, rightFront, rightBack
    }

    private enum SPEEDS {
        NORMAL(0.65),
        FAST(.80),
        SLOW(0.4),
        CUSTOM_FTC_DASHBOARD(Globals.CUSTOM_FTC_DASHBOARD_SPEED);

        private final double speed;
        public double getNumericalSpeed(){
            return speed;
        }
        SPEEDS(double speed){
            this.speed = speed;
        }
    }

    private void createDriveMotors(HardwareMap hardwareMap) {
        for (driveMotorName driveMotorName : driveMotorName.values()) {
            DcMotorEx driveMotor = createDefaultMotor(hardwareMap, driveMotorName.name());
            driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.driveMotors[driveMotorName.ordinal()] = driveMotor;
        }
    }

    public void setDriveMotors(double[] powers, DcMotor.RunMode mode) {
        for (driveMotorName driveMotorName : driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setMode(mode);
            this.driveMotors[driveMotorName.ordinal()].setPower(powers[driveMotorName.ordinal()]);
        }
    }

    /**
     * Returns the switch's state. Note that if a switch is not attached (or not configured), this will always return true.
     */
    public boolean getSwitchState(){
        return switch_ == null ? true : switch_.getState();
    }
}