package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object GlobalsKt {
    @JvmField var robotHeading = 0.0

    @JvmField var APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES = 85.0

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