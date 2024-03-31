package org.firstinspires.ftc.teamcode.functions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.teleOp;

public class claw {
    private Servo upper_grab, lower_grab, vertical_rotate, hori_rotate;

    final private double clawUpperOpen = 1.0;
    final private double clawUpperClosed = 0.46;
    final private double clawLowerOpen = 0.89;
    final private double clawLowerClosed = 0.29;
    final private double verticalAtIntake = 0.02;
    final private double verticalAtDelivery = 0.87; //0.96
    final private double horiAtIntake = 0.98;//was 0.95
    final private double horiAtDelivery = 0.42;

    private String deliveryArm = ""; //Options intake|delivery
    private String upperClaw = ""; //Options open|close
    private String lowerClaw = ""; //Options open|close

    private Double rotateAngle = horiAtIntake;
    private Double verticalPosition = verticalAtIntake;


    public claw(HardwareMap hardwareMap){
        upper_grab = hardwareMap.servo.get("upper_grab");
        lower_grab = hardwareMap.servo.get("lower_grab");
        vertical_rotate = hardwareMap.servo.get("vertical_rotate");
        hori_rotate = hardwareMap.servo.get("hori_rotate");
    }

    public void update(){
        hori_rotate.setPosition(rotateAngle);
        vertical_rotate.setPosition(verticalPosition);
        if (upperClaw == "open"){
            upper_grab.setPosition(clawUpperOpen);
        } else {
            upper_grab.setPosition(clawUpperClosed);
        }
        if (lowerClaw == "open"){
            lower_grab.setPosition(clawLowerOpen);
        } else {
            lower_grab.setPosition(clawLowerClosed);
        }

    }

    public void upperClaw(Boolean upperOpen){
        if (upperOpen){
            upperClaw = "open";
        } else{
            upperClaw = "close";
        }
    }
    public void lowerClaw(Boolean lowerOpen){
        if (lowerOpen){
            lowerClaw = "open";
        } else{
            lowerClaw = "close";
        }
    }


    public void setDeliverArm(String position){
        if (position == "intake"){
            setRotateAngle("intake",0.0);
            verticalPosition = verticalAtIntake;
        } else {
            verticalPosition = verticalAtDelivery;
        }
        deliveryArm = position;
    }
    public void setRotateAngle(String position, Double manualDirection){
        if (deliveryArm == "intake"){
            rotateAngle = horiAtIntake;
        } else {
            switch (position){
                case "intake":
                    rotateAngle = horiAtIntake;
                    break;
                case "horizontal":
                    rotateAngle = horiAtDelivery;
                    break;
                case "manual":
                    if (manualDirection > 0){
                        rotateAngle = rotateAngle + 0.05;
                    } else {
                        rotateAngle = rotateAngle - 0.05;
                    }
                    break;
            }
        }
    }


}
/*
upper_grab (Expansion hub port 0)
lower_grab (Expansion hub port 0)
vertical_rotate (Expansion hub port 0) (the vertical rotation)
hori_rotate (Expansion hub port 0) (rotates the delivery angle that it drops at)
*/