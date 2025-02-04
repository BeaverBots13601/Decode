package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Action;

public interface InterruptableAction extends Action {
    void interrupt();
}