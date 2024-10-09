package org.firstinspires.ftc.teamcode.functions;


import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
    private static double lowThreshold = 0.1;
    private static double holdPower = 0.1;
    private static double gravityThreshold = 0.25;
    private static double maxEncoderValue = 3000;
    private static double heightRestriction = 250;

    public static LiftVariables liftVariables= new LiftVariables(maxEncoderValue, lowThreshold, gravityThreshold,
            holdPower, heightRestriction);

    private static double LkP = 0.008;
    private static double LkI = 0.0;
    private static double LkD = 0.002;
    public static PIDCoefficients lift = new PIDCoefficients(LkP, LkI, LkD);


}