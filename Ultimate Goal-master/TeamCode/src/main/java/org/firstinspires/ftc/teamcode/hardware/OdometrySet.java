package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubMotor;

public class OdometrySet {
    private ExpansionHubMotor parallel, lateral;

    public OdometrySet(ExpansionHubMotor parallel, ExpansionHubMotor lateral) {
        this.parallel = parallel;
        this.lateral = lateral;
    }

    public int getVerticalTicks() {
        return parallel.getCurrentPosition();
    }

    public int getHorizontalTicks() {
        return lateral.getCurrentPosition();
    }
}
