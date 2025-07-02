package org.firstinspires.ftc.teamcode.opModes.parameterOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomousKt

@Autonomous(name = "Kotlin Blue Far Autonomous", group = "CompetitionKt")
class BlueFarAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueFar
}