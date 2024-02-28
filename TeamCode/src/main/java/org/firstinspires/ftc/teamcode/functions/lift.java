package org.firstinspires.ftc.teamcode.functions;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.functions.config;
import org.firstinspires.ftc.teamcode.functions.LiftVariables;
public class lift{

    //Values that require tuning
    //Limits are the points where power starts to decrease approaching top or bottom, in height percentage
    LiftVariables variables;
    private double rawHeight, liftPower, targetEncoderValue = 0, startOffsetHeight;

    private boolean PIDEnabled = false;

    BasicPID controller = new BasicPID(config.lift);

    private DcMotor lift;

    public lift(HardwareMap hardwareMap) {
        variables = config.liftVariables;
        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){

        rawHeight = lift.getCurrentPosition() + startOffsetHeight;
        double power;
        if (PIDEnabled) {
            power = -(controller.calculate(rawHeight, targetEncoderValue));
        } else power = manualLift(liftPower);
        lift.setPower(power);
    }

    public void enablePID(boolean pid){
        this.PIDEnabled = pid;
    }

    public void setPower(double power){
        liftPower = power;
    }

    public void setTargetHeight(double TargetHeight, double startHeight){
        targetEncoderValue = TargetHeight; //0
        startOffsetHeight = startHeight; //250
    }

    private double manualLift(double power){
        double output;

        if(rawHeight < 30 && power < 0){
            output = 0;
        } else if(rawHeight < 300 && power < 0){
            output = power * 0.3;
        } else if(rawHeight > variables.maxEncoderValue - 50 && power > 0){ //3000
            output = variables.holdPower; //0.1
        } else if(power == 0 && rawHeight > variables.maxEncoderValue*variables.gravityThreshold){ //1000
            output = variables.holdPower; //0.1
        } else {
            output = power;
        }

        return output;


    }
    public double currentPos() {
        double test = lift.getCurrentPosition() + startOffsetHeight;
        return test;
    }
}