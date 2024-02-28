package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotor vertical_intake, hori_intake;
    private CRServo intake_roller, intake_belt;
    private double verticalPower, horiPower, beltPower, rollerPower;
    private int horiTargetPosition;
    private int verticalTargetPosition; // Target position for vertical motor
    private boolean isHoriDistanceMode;
    private boolean isVerticalDistanceMode; // Flag for vertical distance mode

    public intake(HardwareMap hardwareMap) {
        vertical_intake = hardwareMap.dcMotor.get("vertical_intake");
        hori_intake = hardwareMap.dcMotor.get("hori_intake");
        intake_roller = hardwareMap.crservo.get("intake_roller");
        intake_belt = hardwareMap.crservo.get("intake_belt");

        vertical_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hori_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hori_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isHoriDistanceMode = false;
        isVerticalDistanceMode = false; // Initially not in vertical distance mode

    }

    public void update() {
        if (isHoriDistanceMode) {
            if (Math.abs(hori_intake.getCurrentPosition() - horiTargetPosition) <= 10) {
                horiPower = 0;
                isHoriDistanceMode = false;
            }
        }

        if (isVerticalDistanceMode) {
            if (Math.abs(vertical_intake.getCurrentPosition() - verticalTargetPosition) <= 10) {
                verticalPower = 0;
                isVerticalDistanceMode = false;
            }
        }

        vertical_intake.setPower(verticalPower);
        hori_intake.setPower(horiPower);
        intake_belt.setPower(beltPower);
        intake_roller.setPower(rollerPower);
    }

    public void horiPower(Double speed) {
        horiPower = -speed;
        isHoriDistanceMode = false;
    }

    public void verticalPower(Double speed) {
        verticalPower = speed;
        isVerticalDistanceMode = false; // Exit distance mode if manually setting power
    }

    public void runHoriDistance(int distance, double speed) {
        int currentPos = hori_intake.getCurrentPosition();
        horiTargetPosition = currentPos + (speed > 0 ? distance : -distance);
        horiPower = speed;
        isHoriDistanceMode = true;
    }

    public void runVerticalDistance(int distance, double speed) {
        int currentPos = vertical_intake.getCurrentPosition();
        verticalTargetPosition = currentPos + (speed > 0 ? distance : -distance);
        verticalPower = speed;
        isVerticalDistanceMode = true; // Enter vertical distance mode
    }

    public void setIntakeRoller(double speed) {
        rollerPower = speed;
    }

    public void setIntakebelt(double speed) {
        beltPower = -speed;
    }
}

