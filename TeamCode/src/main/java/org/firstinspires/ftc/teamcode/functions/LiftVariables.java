package org.firstinspires.ftc.teamcode.functions;

public class LiftVariables {

    //Top to bottom, 0.1, 0.2, 0.25, 3220

    public double lowThreshold;
    public double holdPower;
    public double gravityThreshold;
    public double maxEncoderValue;
    public double heightRestriction;

    public LiftVariables(double maxEncoder, double lowThreshold, double gravityThreshold, double holdPower,
                         double heightRestriction){
        this.lowThreshold = lowThreshold;
        this.maxEncoderValue = maxEncoder;
        this.gravityThreshold = gravityThreshold;
        this.holdPower = holdPower;
        this.heightRestriction = heightRestriction;
    }


}
