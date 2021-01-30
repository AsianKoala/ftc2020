package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Hardware {
    protected static OpMode parentOpMode;
    public static void loadParentOpMode(OpMode parentOpMod) {
        parentOpMode = parentOpMod;
    }

    abstract void update();
}
