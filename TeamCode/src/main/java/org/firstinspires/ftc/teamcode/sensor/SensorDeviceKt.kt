package org.firstinspires.ftc.teamcode.sensor

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

/**
 * A SensorDeviceKt is a general class to handle sensors which are of interest to multiple classes. Unlike HardwareMechanisms, SensorDevices must be manually instantiated.
 *
 * Take note that sensors which are only of interest to a single mechanism should instead be handled within the corresponding HardwareMechanism.
 *
 * Subclasses should include a private constructor(HardwareMap, [SensorInitData], Telemetry). Subclasses should also have a companion object extending [SensorDeviceSingletonManager], supplied with the subclass type and a reference to the constructor.
 *
 * @param T The principle type which the sensor will return when polled.
 */
abstract class SensorDeviceKt<T> {
    /**
     * Call after doing waitForStart(). Allows the class to do setup that can only legally be done after starting.
     */
    abstract fun start()

    /**
     * @return The principle output of the sensor. Other outputs may have access provided for through other getters.
     */
    abstract fun poll(): T

    abstract fun stop()

    open class SensorDeviceSingletonManager<T : SensorDeviceKt<*>>(private val constructor: (HardwareMap, SensorInitData, Telemetry) -> T) {
        @Volatile
        private var instance: T? = null

        @Synchronized
        fun getInstance(hardwareMap: HardwareMap, data: SensorInitData, telemetry: Telemetry): T? {
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

    data class SensorInitData(
        val teamColor: TeamColor,
        val dashboardEnabled: Boolean,
    ) {
        constructor(initData: HardwareMechanismKt.InitData) : this(
            initData.teamColor,
            initData.dashboardEnabled,
        )
    }

    protected fun setUpCamera(hardwareMap: HardwareMap, cameraName: String, cameraWidth: Int, cameraHeight: Int, orientation: OpenCvCameraRotation): OpenCvCamera {
        val webcamName = hardwareMap.get(WebcamName::class.java, cameraName)
        val webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName)
        // This sets up the camera n stuff. Basically just does settings
        webcam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW)
                webcam.startStreaming(cameraWidth, cameraHeight, orientation)
            }

            override fun onError(errorCode: Int) {}
        })
        return webcam
    }
}