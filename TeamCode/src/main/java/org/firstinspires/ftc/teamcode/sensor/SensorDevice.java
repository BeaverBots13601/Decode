package org.firstinspires.ftc.teamcode.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * A SensorDevice is a general class to handle sensors which are of interest to multiple classes. Unlike HardwareMechanisms, SensorDevices must be manually instantiated.
 * <p>
 * Take note that sensors which are only of interest to a single mechanism should instead be handled within the corresponding HardwareMechanism.
 * @param <T> The principle type which the sensor will return when polled.
 */
public abstract class SensorDevice<T> {
    public boolean available;
    protected Telemetry telemetry;

    public SensorDevice(HardwareMap hardwareMap, SensorInitData initData, Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /**
     * Call after doing waitForStart(). Allows the class to do setup that can only legally be done after starting.
     */
    abstract public void start();

    /**
     * @return The principle output of the sensor. Other outputs may have access provided for through other getters.
     */
    abstract public T poll();

    abstract public void stop();

    public static class SensorInitData {
        public TeamColor teamColor;
        public boolean dashboardEnabled;
    }

    // utility functions
    protected OpenCvCamera setUpCamera(HardwareMap hardwareMap, String cameraName, int cameraWidth, int cameraHeight, OpenCvCameraRotation orientation) {
        WebcamName cameraNameThing = hardwareMap.get(WebcamName.class, cameraName);
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraNameThing);
        // This sets up the camera n stuff. Basically just does settings
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
                webcam.startStreaming(cameraWidth, cameraHeight, orientation);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        return webcam;
    }
}