package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.hardware.OuttakeV4
import org.firstinspires.ftc.teamcode.hardware.OuttakeV4.ArtifactColors
import org.firstinspires.ftc.teamcode.hardware.OuttakeV4.Companion.SPINDEXER_ROTATE_DEGREES
import org.firstinspires.ftc.teamcode.hardware.OuttakeV4.Position
import org.firstinspires.ftc.teamcode.misc.AxonDriver
import kotlin.math.abs

@TeleOp
class SpindexerCalibrationOpmode : LinearOpMode() {
    val telemetry = MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().telemetry)
    val spindexer by lazy { AxonDriver(
        hardwareMap,
        "spindexerAxon",
        "spindexerEncoder",
        0.005, // todo: improve tuning
        0.00001,
        0.0001,
        telemetry,
    ).apply {
        targetPosition = 0.0
    } }

    // Color Sensors

    val artifacts = OuttakeV4.ArtifactData()
    override fun runOpMode() {
        waitForStart()
        val leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "leftIntakeColorSensor")
        val rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightIntakeColorSensor")
        while (!isStopRequested) {
            val leftColors  = leftIntakeColorSensor.normalizedColors
            val rightColors = rightIntakeColorSensor.normalizedColors
            val detectedArtifact = OuttakeV4.detectArtifact(leftColors, rightColors)
            telemetry.addData("Detected Artifact", detectedArtifact)

            // region Spindexer Controls
            var spindexerMoving = abs(spindexer.error) > 5

            if (detectedArtifact != ArtifactColors.NONE && !spindexerMoving) {
                artifacts.intake(detectedArtifact)
                val rotateTo = artifacts.rotate()

                if (rotateTo == Position.OUTTAKE) {
                    // must go right twice to avoid getting launch
                    rotateSpindexer(Position.STORAGE)
                    rotateSpindexer(Position.STORAGE)

                    // fixme: bug situation
                    //   one in storage, one in intake
                    //   "go outtake" -> pushed into intake and outtake instead of outtake and storage
                } else {
                    rotateSpindexer(rotateTo)
                }


                if (artifacts.intakeArtifact != ArtifactColors.NONE
                    && artifacts.outtakeArtifact != ArtifactColors.NONE
                    && artifacts.storageArtifact != ArtifactColors.NONE) {
                    //intakeOff()
                }
            }

            // update pid
            spindexer.targetPosition = spindexer.targetPosition

            spindexerMoving = abs(spindexer.error) > 5
            telemetry.addData("Spindexer Moving", spindexerMoving)
            telemetry.addData("Intake Artifact", artifacts.intakeArtifact)
            telemetry.addData("Outtake Artifact", artifacts.outtakeArtifact)
            telemetry.addData("Storage Artifact", artifacts.storageArtifact)
            // endregion
            telemetry.update()
        }
        spindexer.cleanUp()

    }

    fun rotateSpindexer(position: Position) {
        if (position == Position.OUTTAKE) {
            spindexer.targetPosition = spindexer.targetPosition?.minus(SPINDEXER_ROTATE_DEGREES)
        } else if (position == Position.STORAGE) {
            spindexer.targetPosition = spindexer.targetPosition?.plus(SPINDEXER_ROTATE_DEGREES)
        }
    }
}