package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.Servo

/**
 * A simple driver for the GoBilda PWM RGB Indicator (SKU: 3118-0808-0002).
 */
class GoBildaRGBIndicatorDriver(val indicator: Servo) {
    var color: Color = Color.OFF
        set(value) {
            indicator.position = value.servoPos
            field = value
        }

    enum class Color(val servoPos: Double) {
        OFF(0.0),
        RED(0.277),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.5),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0)
    }
}