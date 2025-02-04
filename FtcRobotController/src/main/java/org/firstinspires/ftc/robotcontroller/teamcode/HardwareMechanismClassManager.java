package org.firstinspires.ftc.robotcontroller.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.opmode.ClassFilter;
import org.firstinspires.ftc.robotcore.internal.opmode.ClassManager;

import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

// TODO: Sinister (https://docs.dairy.foundation/Sinister) is an option. If we lose the ability to do class filtering like this, it may become necessary.
public class HardwareMechanismClassManager implements ClassFilter {
    private static List<Class<HardwareMechanism>> mechanisms = new ArrayList<>();

    /**
     * @return The list of all Classes that extend HardwareMechanism.
     */
    public static List<Class<HardwareMechanism>> getMechanisms(){
        if (mechanisms.isEmpty()){
            // This really should never occur unless somehow we've deleted every single HardwareMechanism, including the drivetrain. (Or a bug.)
            // Still, that could definitely cause some headaches when debugging so let's make the source of the error real obvious.
            RobotLog.addGlobalWarningMessage("Something has gone really terribly wrong in the code and you should tell a programmer this has happened right now. Nothing will work until you do.");
        }
        return mechanisms;
    }

    //@OnCreate
    public static void registerClassFilter(Context context){
        ClassManager.getInstance().registerFilter(new HardwareMechanismClassManager());
    }

    public void filterAllClassesStart() {
        mechanisms = new ArrayList<>();
    }
    public void filterOnBotJavaClassesStart() {}
    public void filterExternalLibrariesClassesStart() {}

    public void filterClass(Class clazz) {
        if (HardwareMechanism.class.isAssignableFrom(clazz) && !Modifier.isAbstract(clazz.getModifiers())) {
            mechanisms.add((Class<HardwareMechanism>) clazz);
        }
    }
    public void filterOnBotJavaClass(Class clazz) {}
    public void filterExternalLibrariesClass(Class clazz) {}

    public void filterAllClassesComplete() {}
    public void filterOnBotJavaClassesComplete() {}
    public void filterExternalLibrariesClassesComplete() {}
}
