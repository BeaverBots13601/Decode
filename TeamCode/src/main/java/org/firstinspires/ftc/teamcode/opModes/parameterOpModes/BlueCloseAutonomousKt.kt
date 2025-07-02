package org.firstinspires.ftc.teamcode.opModes.parameterOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomousKt

@Autonomous(name = "Kotlin Blue Close Autonomous", group = "CompetitionKt")
class BlueCloseAutonomousKt : UnifiedAutonomousKt() {
    override val currentLocation = Locations.BlueClose
}