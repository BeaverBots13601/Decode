package org.firstinspires.ftc.teamcode.opModes.parameterOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomousKt

@Autonomous(name = "Kotlin Red Far Autonomous", group = "CompetitionKt")
class RedFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.RedFar
}