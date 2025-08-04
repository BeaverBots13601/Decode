package org.firstinspires.ftc.teamcode.sensor

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.max

/**
 * Handles scanning for a colored Team Prop within three regions. The regions it is scanning are
 * visible within FTCDashboard.
 *
 * See also [LimelightKt].
 */
class WebcamPropIdentificationKt private constructor(hardwareMap: HardwareMap, initData: SensorInitData, private val telemetry: Telemetry) : SensorDeviceKt<WebcamPropIdentificationKt.PropLocation>() {
    // magic numbers
    // Changes how "zoomed-in" the camera sees
    // If the camera size is too small, the boxes could end up overlapping and causing problems. For the 22-23 and 23-24 years, the camera was 1280x720.
    private val squareSizePx = 90.0
    private val detectionBoxOffsetSidesPx = 128.0
    /**
     * The percentage, from [0, 1.0], at which values equal to or below will not be used to determine the location of the team prop.
     * Ex. If the highest percentage of color is the center area at 8%, and the value is at 0.1, it will be declared 'unknown' instead of 'center'.
     */
    private val unknownColorThresholdPercent = 0.1

    // camera constants (should these be extracted into an injected class?)
    private val cameraName = "camera"
    private val cameraWidthPx = 1280
    private val cameraHeightPx = 720
    private val orientation = OpenCvCameraRotation.UPRIGHT

    // Color search data
    private val teamColor = initData.teamColor
    private val pipeline = Pipeline()

    init {
        val cam = setUpCamera(
            hardwareMap,
            cameraName,
            cameraWidthPx,
            cameraHeightPx,
            orientation
        )
        cam.setPipeline(pipeline)
        FtcDashboard.getInstance().startCameraStream(cam, 0.0)
    }

    override fun start() {}

    override fun poll(): PropLocation { return pipeline.propLocation }

    override fun stop() {}

    private inner class Pipeline : OpenCvPipeline() {
        // Publicly-accessible output
        var propLocation = PropLocation.UNKNOWN

        // Open-CV variables
        private val HUE_DIFF = 15.0
        private val SAT_DIFF = 205.0
        private val VAL_DIFF = 185.0

        // OpenCV uses the HSV range H(0-180), S(0-255), V(0-255).
        //    The color red is centered around 0, so we need two ranges to measure high-end and low-end values.
        // todo does the implementation of ^ mean that the camera is reading more red val than it should compared to blue?
        // Red color range below 180
        private val redHSVLow1 = Scalar(180 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF)
        private val redHSVHigh1 = Scalar(180.0, 255.0, 255.0)

        // Red color range above 0
        private val redHSVLow2 = Scalar(0.0, 255 - SAT_DIFF, 255 - VAL_DIFF)
        private val redHSVHigh2 = Scalar(0 + HUE_DIFF, 255.0, 255.0)

        // Blue color range
        private val blueHSVLow = Scalar(120 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF)
        private val blueHSVHigh = Scalar(120 + HUE_DIFF, 255.0, 255.0)

        private val hsv = Mat()
        private val grey = Mat()

        private val LeftROI = Rect(
            Point(detectionBoxOffsetSidesPx,
                cameraHeightPx / 2.0 - squareSizePx / 2.0),
            Point(squareSizePx + detectionBoxOffsetSidesPx,
                cameraHeightPx / 2.0 + squareSizePx / 2.0)
        )
        private val CenterROI = Rect(
            Point(cameraWidthPx / 2.0 - squareSizePx / 2.0,
                cameraHeightPx / 2.0 - squareSizePx / 2.0),
            Point(cameraWidthPx / 2.0 + squareSizePx / 2.0,
                cameraHeightPx / 2.0 + squareSizePx / 2.0)
        )
        private val RightROI = Rect(
            Point(cameraWidthPx - squareSizePx - detectionBoxOffsetSidesPx,
                cameraHeightPx / 2.0 - squareSizePx / 2.0),
            Point(cameraWidthPx - detectionBoxOffsetSidesPx,
                cameraHeightPx / 2.0 + squareSizePx / 2.0)
        )

        // color!
        private val purple = Scalar(255.0, 0.0, 255.0)
        private val colorScalar = if (teamColor == TeamColor.BLUE) {
            Scalar(0.0, 0.0, 255.0) // blue
        } else {
            Scalar(255.0, 0.0, 0.0) // red
        }

        override fun processFrame(input: Mat): Mat {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)

            var box: Mat
            var leftPercentage: Double
            var centerPercentage: Double
            var rightPercentage: Double

            if (teamColor == TeamColor.BLUE) {
                Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey)
                box = grey.submat(LeftROI)
                leftPercentage = Core.sumElems(box).`val`[0] / LeftROI.area() / 255

                Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey)
                box = grey.submat(CenterROI)
                centerPercentage = Core.sumElems(box).`val`[0] / CenterROI.area() / 255

                Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey)
                box = grey.submat(RightROI)
                rightPercentage = Core.sumElems(box).`val`[0] / RightROI.area() / 255
            } else {
                // must be red
                // left stuff
                Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey)
                box = grey.submat(LeftROI)
                leftPercentage = Core.sumElems(box).`val`[0] / LeftROI.area() / 255

                Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey)
                box = grey.submat(LeftROI)
                leftPercentage += leftPercentage + Core.sumElems(box).`val`[0] / LeftROI.area() / 255
                leftPercentage /= 2.0

                // center stuff
                Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey)
                box = grey.submat(CenterROI)
                centerPercentage = Core.sumElems(box).`val`[0] / CenterROI.area() / 255

                Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey)
                box = grey.submat(CenterROI)
                centerPercentage += centerPercentage + Core.sumElems(box).`val`[0] / CenterROI.area() / 255
                centerPercentage /= 2.0

                // right stuff
                Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey)
                box = grey.submat(RightROI)
                rightPercentage = Core.sumElems(box).`val`[0] / RightROI.area() / 255

                Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey)
                box = grey.submat(RightROI)
                rightPercentage += rightPercentage + Core.sumElems(box).`val`[0] / RightROI.area() / 255
                rightPercentage /= 2.0
            }

            // Display to the user and save the detection to variable
            val max = max(leftPercentage, max(centerPercentage, rightPercentage))
            if (max <= unknownColorThresholdPercent) {
                propLocation = PropLocation.UNKNOWN
                Imgproc.rectangle(input, LeftROI, colorScalar, 3)
                Imgproc.rectangle(input, CenterROI, colorScalar, 3)
                Imgproc.rectangle(input, RightROI, colorScalar, 3)
            } else if (max == leftPercentage) {
                propLocation = PropLocation.LEFT
                Imgproc.rectangle(input, LeftROI, purple, 3)
                Imgproc.rectangle(input, CenterROI, colorScalar, 3)
                Imgproc.rectangle(input, RightROI, colorScalar, 3)
            } else if (max == centerPercentage) {
                propLocation = PropLocation.CENTER
                Imgproc.rectangle(input, LeftROI, colorScalar, 3)
                Imgproc.rectangle(input, CenterROI, purple, 3)
                Imgproc.rectangle(input, RightROI, colorScalar, 3)
            } else if (max == rightPercentage) {
                propLocation = PropLocation.RIGHT
                Imgproc.rectangle(input, LeftROI, colorScalar, 3)
                Imgproc.rectangle(input, CenterROI, colorScalar, 3)
                Imgproc.rectangle(input, RightROI, purple, 3)
            }

            return input
        }
    }

    enum class PropLocation {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    companion object : SensorDeviceSingletonManager<WebcamPropIdentificationKt>(::WebcamPropIdentificationKt)
}