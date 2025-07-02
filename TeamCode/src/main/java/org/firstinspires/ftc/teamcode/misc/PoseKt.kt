package org.firstinspires.ftc.teamcode.misc

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Class to conveniently manage position data.
 */
data class PoseKt(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val angle: Double = 0.0,
) {
    companion object {
        /**
         * @param x x value of the point (positive = forward?)
         * @param y y value of the point (positive = right?)
         * @param angle angle change of the point (radians)
         * @return rotation of point (x, y) around angle
         */
        fun rotatePosition(x: Double, y: Double, angle: Double): PoseKt {
            return PoseKt(
                x * cos(angle) - y * sin(angle),
                x * sin(angle) + y * cos(angle),
                angle
            )
        }

        /**
         * @return double angle within the range [-π, π]
         */
        fun normalizeAngle(angle: Double): Double {
            var out = angle
            while (out > PI) {
                out -= 2 * PI;
            }
            while (out < -PI) {
                out += 2 * PI;
            }
            return out;
        }
    }
}