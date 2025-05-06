package org.firstinspires.ftc.robotcontroller.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * A class representing one combined mechanism on the robot, consisting of all related hardware components and function which is called on initialization, start, and every loop. Implementations of this class should expose such functions as are needed for autonomous.
 * <p>
 * All discovered HardwareMechanisms are loaded automatically within the TeleOp. Within autonomous they must be manually instantiated.
 * <p>
 * Subclasses should implement a singleton design, with a static getInstance() function returning null if unavailable.
 * Subclasses should also include a static getInstance(HardwareMap, InitData, Telemetry) function for initialization.
 */
public abstract class HardwareMechanism {
    public boolean available;
    protected Telemetry telemetry;
    /**
     * WARNING: After using the constructor, you MUST check the field 'available'. Treat a false value as null.
     *
     * @param data          Starting data for the hardware.
     */
    public HardwareMechanism(HardwareMap hardwareMap, InitData data, Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /**
     * Call after doing waitForStart(). Allows the class to do setup that can only legally be done after starting.
     */
    abstract public void start();

    /**
     * The main loop function, meant to be called every TeleOp loop.
     *
     * @param data Relevant data for the class.
     */
    abstract public void run(RunData data);

    /**
     * Call after ending the main program. Allows the class to do cleanup.
     */
    abstract public void stop();

    /**
     * @return The list of all GamepadButtons this hardware mechanism uses in run().
     */
    abstract public List<GamepadButtons> getUsedButtons();

    // static utility members
    protected static Servo setUpServo(HardwareMap hardwareMap, String servoName) {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        return servo;
    }

    /**
     * Creates a default motor with the settings 'RUN_USING_ENCODER' and 'FLOAT on zero power'.
     * Reverses if name includes left.
     */
    protected static DcMotorEx createDefaultMotor(HardwareMap hardwareMap, String motorName) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (motorName.toLowerCase().contains("left")) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return motor;
    }

    /**
     * Sleeps. Warning: The main loop working well depends on run() functions taking only a short time.
     * Sleeping usually disrupts this illusion and therefore should only be done in situations
     * where it's okay to completely cease driver control. (Also note there is no abort.)
     */
    protected static void sleep(long millis){
        try {
            Thread.sleep(millis);
        } catch (Exception ignored){}
    }

    public static class RunData {
        public Gamepad currentGamepadOne;
        public Gamepad currentGamepadTwo;
        public Gamepad previousGamepadOne;
        public Gamepad previousGamepadTwo;
        public double imuAngleRad;
    }

    public static class InitData {
        public TeamColor teamColor;
        public DriveMode driveMode;
        public boolean dashboardEnabled;
    }

    public enum DriveMode {
        FIELD,
        ROBOT
    }
}
