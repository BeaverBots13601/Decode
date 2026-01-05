package org.firstinspires.ftc.teamcode.opModes.utilityOpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.OuttakeV4
import org.firstinspires.ftc.teamcode.misc.ArtifactColors
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
        0.006, // todo: improve tuning
        0.0005,
        0.00005,
        telemetry,
    ).apply {
        targetPosition = 0.0
    } }

    // Color Sensors

    val artifacts = OuttakeV4.ArtifactData()
    override fun runOpMode() {
        waitForStart()
        spindexer.start()
        val leftIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "leftIntakeColorSensor")
        val rightIntakeColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightIntakeColorSensor")
        while (!isStopRequested) {
            val leftColors  = leftIntakeColorSensor.normalizedColors
            val rightColors = rightIntakeColorSensor.normalizedColors
            val detectedArtifact = OuttakeV4.detectArtifact(leftColors, rightColors)
            telemetry.addData("Detected Artifact", detectedArtifact)

            // region Spindexer Controls
            var spindexerMoving = abs(spindexer.error) > 15

            if (detectedArtifact != ArtifactColors.NONE && !spindexerMoving) {
                artifacts.intake(detectedArtifact)
                val rotateTo = artifacts.rotate()
                rotateSpindexer(rotateTo)

                if (artifacts.intakeArtifact != ArtifactColors.NONE
                    && artifacts.outtakeArtifact != ArtifactColors.NONE
                    && artifacts.storageArtifact != ArtifactColors.NONE) {
                    //intakeOff()
                }
            } else {

            }

            // update pid
            spindexer.update()

            artifacts.updateTelemetry(telemetry)

            spindexerMoving = abs(spindexer.error) > 15
            telemetry.addData("Spindexer Moving", spindexerMoving)
            // endregion
            telemetry.update()
        }
        spindexer.cleanUp()

    }

    fun rotateSpindexer(position: Position) {
        if (position == Position.OUTTAKE) {
            spindexer.targetPosition = spindexer.targetPosition?.plus(SPINDEXER_ROTATE_DEGREES * 2)
        } else if (position == Position.STORAGE) {
            spindexer.targetPosition = spindexer.targetPosition?.plus(SPINDEXER_ROTATE_DEGREES)
        }
    }
}