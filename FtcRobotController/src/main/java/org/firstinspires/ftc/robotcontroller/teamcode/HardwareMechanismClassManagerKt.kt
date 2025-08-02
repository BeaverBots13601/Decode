package org.firstinspires.ftc.robotcontroller.teamcode

import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.opmode.ClassFilter
import java.lang.reflect.Modifier
import kotlin.reflect.KClass

// TODO: Sinister (https://docs.dairy.foundation/Sinister) is an option. If we lose the ability to do class filtering like this, it may become necessary.
class HardwareMechanismClassManagerKt : ClassFilter {
    companion object {
        private var mechanisms: MutableList<KClass<out HardwareMechanismKt>> = mutableListOf()

        /**
         * @return The list of all Classes that extend HardwareMechanismKt.
         */
        fun getMechanisms(): List<KClass<out HardwareMechanismKt>> {
            if (mechanisms.isEmpty()) {
                // This really should never occur unless somehow we've deleted every single HardwareMechanism, including the drivetrain. (Or a bug.)
                // Still, that could definitely cause some headaches when debugging so let's make the source of the error real obvious.
                RobotLog.addGlobalWarningMessage("Something has gone really terribly wrong in the code and you should tell a programmer this has happened right now. Nothing will work until you do.")
            }
            return mechanisms
        }
    }

    override fun filterAllClassesStart() { mechanisms = mutableListOf() }

    override fun filterClass(clazz: Class<*>) {
        if (!HardwareMechanismKt::class.java.isAssignableFrom(clazz)) return
        if (Modifier.isAbstract(clazz.modifiers)) return

        try {
            // Check the companion object extends the SingletonManager
            if(!HardwareMechanismKt.HardwareMechanismSingletonManager::class.java
                    .isAssignableFrom(clazz.getField("Companion").type)) throw Exception()
        } catch (ignored: Exception) {
            RobotLog.addGlobalWarningMessage("Warning: A class' (" + clazz.simpleName + ") companion object does not extend the required superclass.")
            return
        }

        mechanisms.add(clazz.kotlin as KClass<out HardwareMechanismKt>)
    }



    override fun filterOnBotJavaClassesStart() {}
    override fun filterExternalLibrariesClassesStart() {}
    override fun filterOnBotJavaClass(clazz: Class<*>?) {}
    override fun filterExternalLibrariesClass(clazz: Class<*>?) {}
    override fun filterAllClassesComplete() {}
    override fun filterOnBotJavaClassesComplete() {}
    override fun filterExternalLibrariesClassesComplete() {}
}
