package org.firstinspires.ftc.teamcode.main;

import org.openftc.revextensions2.ExpansionHubMotor;

public class OdometrySet {
    private ExpansionHubMotor parallel, lateral;

    public OdometrySet(ExpansionHubMotor parallel, ExpansionHubMotor lateral) {
        this.parallel = parallel;
        this.lateral = lateral;
    }

    public int getParallelTicks() {
        return parallel.getCurrentPosition();
    }

    public int getLateralTicks() {
        return lateral.getCurrentPosition();
    }
}
