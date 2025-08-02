package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

// class for global variables. todo can this be refactored into a better form?
@Config
object GlobalsKt {
    // Heading @ end of Autonomous, for field mode.
    @JvmField var robotHeading = 0.0

    // Utility global for auto tuning. Read-only except for Dashboard.
    /**
     * The distance, in inches, at which the scanner decides we are at the "Far" position if the AprilTags it sees are beyond.
     * Might have to be fudged a bit if the calibration is wonky
     */
    @JvmField var APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES = 85.0

    // This speed is designed to be set dynamically within FTC Dashboard.
    @JvmField var CUSTOM_FTC_DASHBOARD_SPEED = 0.65

    /**
     * Enables bulk reads which allow faster hardware call times.
     * Set to auto currently but if speed becomes an issue this can be manually configured.
     * TODO: Re-home me.
     */
    fun initBulkReads(hardwareMap: HardwareMap){
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}