package org.firstinspires.ftc.teamcode.misc

class AverageOverTime(val storedValues: Int) {
    private val queue = ArrayDeque<Number>(storedValues)

    fun push(value: Number) {
        if (queue.size == storedValues) {
            queue.removeFirst()
        }
        queue.add(value)
    }

    fun average(): Double {
        var average = 0.0
        queue.forEach { average += it.toDouble() }
        return average / queue.size
    }
}