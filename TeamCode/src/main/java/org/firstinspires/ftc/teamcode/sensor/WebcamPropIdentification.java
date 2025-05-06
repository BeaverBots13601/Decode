package org.firstinspires.ftc.teamcode.sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensor.WebcamPropIdentification.PropLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Handles scanning for a colored Team Prop within three regions. The regions it is scanning are
 * visible within FTCDashboard.
 * <p>
 * See also Limelight.
 */
public class WebcamPropIdentification extends SensorDevice<PropLocation> {
    // magic numbers
    // Changes how "zoomed-in" the camera sees
    // If the camera size is too small, the boxes could end up overlapping and causing problems. For the 22-23 and 23-24 years, the camera was 1280x720.
    private static final double squareSizePx = 90;
    private static final double detectionBoxOffsetSidesPx = 128;
    /**
     * The percentage, from [0, 1.0], at which values equal to or below will not be used to determine the location of the team prop.
     * Ex. If the highest percentage of color is the center area at 8%, and the value is at 0.1, it will be declared 'unknown' instead of 'center'.
     */
    public static final double unknownColorThresholdPercent = 0.1;

    // camera constants (should these be extracted into an injected class?)
    private static final String cameraName = "camera";
    private static final int cameraWidthPx = 1280;
    private static final int cameraHeightPx = 720;
    private static final OpenCvCameraRotation orientation = OpenCvCameraRotation.UPRIGHT;

    // Color search data
    private TeamColor teamColor;
    private Pipeline pipeline;
    private OpenCvCamera camera;
    public WebcamPropIdentification(HardwareMap hardwareMap, SensorInitData initData, Telemetry telemetry){
        super(hardwareMap, initData, telemetry);
        try {
            camera = setUpCamera(hardwareMap, cameraName, cameraWidthPx, cameraHeightPx, orientation);
        } catch (Exception e) {
            available = false;
            return;
        }

        teamColor = initData.teamColor;
        pipeline = new Pipeline(teamColor);
        camera.setPipeline(pipeline);

        // Add to dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        available = true;
    }

    public void start() {}

    public PropLocation poll() {
        return pipeline.propLocation;
    }

    public void stop() {}

    private class Pipeline extends OpenCvPipeline {
        // Publicly-accessible output
        public PropLocation propLocation;

        // Open-CV variables
        private static final double HUE_DIFF = 15;
        private static final double SAT_DIFF = 205;
        private static final double VAL_DIFF = 185;

        // OpenCV uses the HSV range H(0-180), S(0-255), V(0-255).

        //    The color red is centered around 0, so we need two ranges to measure high-end and low-end values.
        // todo does the implementation of ^ mean that the camera is reading more red val than it should compared to blue?
//    Red color range below 180
        private final Scalar redHSVLow1 = new Scalar(180 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
        private final Scalar redHSVHigh1 = new Scalar(180, 255, 255);
        //    Red color range above 0
        private final Scalar redHSVLow2 = new Scalar(0, 255 - SAT_DIFF, 255 - VAL_DIFF);
        private final Scalar redHSVHigh2 = new Scalar(0 + HUE_DIFF, 255, 255);
        //    Blue color range
        private final Scalar blueHSVLow = new Scalar(120 - HUE_DIFF, 255 - SAT_DIFF, 255 - VAL_DIFF);
        private final Scalar blueHSVHigh = new Scalar(120 + HUE_DIFF, 255, 255);

        private final Mat hsv = new Mat();
        private final Mat grey = new Mat();

        private final Rect LeftROI = new Rect(
                new Point(detectionBoxOffsetSidesPx, cameraHeightPx / 2.0 - squareSizePx / 2.0),
                new Point(squareSizePx + detectionBoxOffsetSidesPx, cameraHeightPx / 2.0 + squareSizePx / 2.0)
        );
        private final Rect CenterROI = new Rect(
                new Point(cameraWidthPx / 2.0 - squareSizePx / 2.0,
                        cameraHeightPx / 2.0 - squareSizePx / 2.0),
                new Point(cameraWidthPx / 2.0 + squareSizePx / 2.0,
                        cameraHeightPx / 2.0 + squareSizePx / 2.0)
        );
        private final Rect RightROI = new Rect(
                new Point(cameraWidthPx - squareSizePx - detectionBoxOffsetSidesPx, cameraHeightPx / 2.0 - squareSizePx / 2.0),
                new Point(cameraWidthPx - detectionBoxOffsetSidesPx, cameraHeightPx / 2.0 + squareSizePx / 2.0)
        );

        // color!
        private final Scalar purple = new Scalar(255, 0, 255);
        private final Scalar colorScalar;

        public Pipeline(TeamColor teamColor){
            if(teamColor == TeamColor.BLUE){
                colorScalar = new Scalar(0, 0, 255); // blue
            } else {
                colorScalar = new Scalar(255, 0, 0); // red
            }
        }

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat box;
            double leftPercentage;
            double centerPercentage;
            double rightPercentage;

            if(teamColor == TeamColor.BLUE){
                Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
                box = grey.submat(LeftROI);
                leftPercentage = Core.sumElems(box).val[0] / LeftROI.area() / 255;

                Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
                box = grey.submat(CenterROI);
                centerPercentage = Core.sumElems(box).val[0] / CenterROI.area() / 255;

                Core.inRange(hsv, blueHSVLow, blueHSVHigh, grey);
                box = grey.submat(RightROI);
                rightPercentage = Core.sumElems(box).val[0] / RightROI.area() / 255;
            } else {
                // must be red
                // left stuff
                Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
                box = grey.submat(LeftROI);
                leftPercentage = Core.sumElems(box).val[0] / LeftROI.area() / 255;

                Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
                box = grey.submat(LeftROI);
                leftPercentage += leftPercentage + Core.sumElems(box).val[0] / LeftROI.area() / 255;
                leftPercentage /= 2.0;

                // center stuff
                Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
                box = grey.submat(CenterROI);
                centerPercentage = Core.sumElems(box).val[0] / CenterROI.area() / 255;

                Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
                box = grey.submat(CenterROI);
                centerPercentage += centerPercentage + Core.sumElems(box).val[0] / CenterROI.area() / 255;
                centerPercentage /= 2.0;

                // right stuff
                Core.inRange(hsv, redHSVLow1, redHSVHigh1, grey);
                box = grey.submat(RightROI);
                rightPercentage = Core.sumElems(box).val[0] / RightROI.area() / 255;

                Core.inRange(hsv, redHSVLow2, redHSVHigh2, grey);
                box = grey.submat(RightROI);
                rightPercentage += rightPercentage + Core.sumElems(box).val[0] / RightROI.area() / 255;
                rightPercentage /= 2.0;
            }

            // Display to the user and save the detection to variable
            double max = Math.max(leftPercentage, Math.max(centerPercentage, rightPercentage));
            if(max <= unknownColorThresholdPercent){
                propLocation = PropLocation.UNKNOWN;
                Imgproc.rectangle(input, LeftROI, colorScalar, 3);
                Imgproc.rectangle(input, CenterROI, colorScalar, 3);
                Imgproc.rectangle(input, RightROI, colorScalar, 3);
            } else if (max == leftPercentage) {
                propLocation = PropLocation.LEFT;
                Imgproc.rectangle(input, LeftROI, purple, 3);
                Imgproc.rectangle(input, CenterROI, colorScalar, 3);
                Imgproc.rectangle(input, RightROI, colorScalar, 3);
            } else if (max == centerPercentage) {
                propLocation = PropLocation.CENTER;
                Imgproc.rectangle(input, LeftROI, colorScalar, 3);
                Imgproc.rectangle(input, CenterROI, purple, 3);
                Imgproc.rectangle(input, RightROI, colorScalar, 3);
            } else if (max == rightPercentage) {
                propLocation = PropLocation.RIGHT;
                Imgproc.rectangle(input, LeftROI, colorScalar, 3);
                Imgproc.rectangle(input, CenterROI, colorScalar, 3);
                Imgproc.rectangle(input, RightROI, purple, 3);
            }

            return input;
        }
    }

    public enum PropLocation {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }
}