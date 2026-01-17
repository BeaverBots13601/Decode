package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismKt
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.misc.GoBildaRGBIndicatorDriver
import org.firstinspires.ftc.teamcode.misc.GoBildaRGBIndicatorDriver.Color

class Riser private constructor(hardwareMap: HardwareMap, initData: InitData, private val telemetry: Telemetry) : HardwareMechanismKt() {
    val leftRiserMotor = createDefaultMotor(hardwareMap, "leftRiser")
    val rightRiserMotor = createDefaultMotor(hardwareMap, "rightRiser")

    // LEDs
    private val leftIndicatorLED  = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("leftIndicatorLED"))
    private val rightIndicatorLED = GoBildaRGBIndicatorDriver(hardwareMap.servo.get("rightIndicatorLED"))

    // Color Sensors
    private val leftColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "leftLiftColorSensor")
    private val rightColorSensor = hardwareMap.get(RevColorSensorV3::class.java, "rightLiftColorSensor")

    private val teamColor = initData.teamColor

    override fun start() {}

    // Emergency Mode Stuff
    private var leftRed = true
    private val timer = ElapsedTime()
    private var delta = 0.0
    override fun run(data: RunData) {
        // !!! EMERGENCY !!!
        if (data.currentGamepadOne.psWasPressed()) {
            // Take over the loop
            while (!Thread.currentThread().isInterrupted) {
                val gamepad1 = Gamepad()
                val gamepad2 = Gamepad()
                gamepad1.copy(data.gamepadOneReference)
                gamepad2.copy(data.gamepadTwoReference)
                DriveTrainKt.getInstance()?.run(RunData(
                    gamepad1,
                    gamepad2,
                    data.gamepadOneReference,
                    data.gamepadTwoReference,
                    0.0
                ))

                // Riser movement
                if (data.gamepadOneReference.right_bumper) {
                    leftRiserMotor.power = -0.5
                    rightRiserMotor.power = -0.5
                } else if (data.gamepadOneReference.left_bumper) {
                    leftRiserMotor.power = 0.5
                    rightRiserMotor.power = 0.5
                } else {
                    leftRiserMotor.power = 0.0
                    rightRiserMotor.power = 0.0
                }


                var leftLEDSet = false
                var rightLEDSet = false

                if (tapeDetected(teamColor, leftColorSensor.normalizedColors)) {
                    leftLEDSet = true
                    leftIndicatorLED.color = Color.GREEN
                }

                if (tapeDetected(teamColor, rightColorSensor.normalizedColors)) {
                    rightLEDSet = true
                    rightIndicatorLED.color = Color.GREEN
                }

                // LEDs
                delta += timer.seconds() - delta
                if (delta > 0.75) {
                    // We can take these over because we've stolen the loop from OuttakeV3
                    if (!leftLEDSet) leftIndicatorLED.color = if (leftRed) Color.RED else Color.BLUE
                    if (!rightLEDSet) rightIndicatorLED.color = if (leftRed) Color.BLUE else Color.RED
                    leftRed = !leftRed
                    delta = 0.0
                    timer.reset()
                }
            }
            Thread.currentThread().interrupt()
            return
        }
    }

    override fun stop() {}

    override val usedButtons: Array<GamepadButtons> = arrayOf(
        GamepadButtons.GP1_RIGHT_BUMPER,
        GamepadButtons.GP1_LEFT_BUMPER,
    )

    fun tapeDetected(teamColor: TeamColor, data: NormalizedRGBA): Boolean {
        return if (teamColor == TeamColor.RED) {
            data.red > 0.01 && data.red > data.green && data.red > data.blue
        } else {
            data.blue > 0.0125 && data.blue > data.green && data.blue > data.red
        }
    }

    companion object : HardwareMechanismSingletonManager<Riser>(::Riser)
}