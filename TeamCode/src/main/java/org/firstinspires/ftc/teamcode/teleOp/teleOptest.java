package org.firstinspires.ftc.teamcode.teleOp;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.functions.claw;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.lift;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp

public class teleOptest extends LinearOpMode {
    IMU imu;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor climb;
    private Servo planeGoWee;
    private Servo right_release;
    private Servo left_release;
    private double climbPower = 0.0;

    double delayTimer = 0;
    double delayTimer1 = 0;
    boolean runDelay = false;
    boolean runDelay1 = false;

    public boolean armIn = true;
    public boolean upperClawOpen = true;
    public boolean lowerClawOpen = true;
    public boolean prevupperClawOpen = true;
    public boolean prevlowerClawOpen = true;
    boolean ignore = false;

    private final double HEADING_KP = 0.1; // Proportional control constant for heading alignment
    private final double ROTATIONAL_POWER_SCALE = 20.0; // Scale for increasing rotational power
    private final double STRAFING_SCALE = 1.0; // Scale for strafing power
    private double targetAngleZero = Math.toRadians(0); // Target angle for alignment with right trigger
    private double targetAngleFortyFive = Math.toRadians(-45);
    private Double strafeHeading = null;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    //private DistanceSensor distance;
    //private DistanceSensor distance2;
    private ColorRangeSensor distance;
    private ColorRangeSensor distance2;


    Gamepad.RumbleEffect customRumbleEffect, customRumbleEffect2;

    private double slowDownMultiplier = 1.0;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    double aprilSpeedMultiplier = 1.0;



    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 300)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 300)  //  Rumble right motor 100% for 500 mSec
                .build();
        customRumbleEffect2 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 300)  //  Rumble right motor 100% for 500 mSec
                .build();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        lift lift = new lift(hardwareMap);
        claw claw = new claw(hardwareMap);
        intake intake = new intake(hardwareMap);
        claw.update();
        lift.enablePID(false);
        lift.noLimits = true;
        lift.bringHerHome();

        //distance = hardwareMap.get(DistanceSensor.class, "distance");
        //distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        distance2 = hardwareMap.get(ColorRangeSensor.class, "distance2");

        imu = hardwareMap.get(IMU.class, "imu");
        hardwareMap.dcMotor.get("vertical_intake");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        sleep(2000);

        frontLeft = hardwareMap.dcMotor.get("front_left");
        backLeft = hardwareMap.dcMotor.get("back_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backRight = hardwareMap.dcMotor.get("back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        climb = hardwareMap.dcMotor.get("climb");
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        planeGoWee = hardwareMap.servo.get("drone_launcher");
        planeGoWee.setPosition(0.0);
        right_release = hardwareMap.servo.get("right_release");
        right_release.setPosition(0.92);
        left_release = hardwareMap.servo.get("left_release");
        left_release.setPosition(0.04);

        claw.upperClaw(true);
        upperClawOpen = true;
        claw.lowerClaw(true);
        lowerClawOpen = true;

        initAprilTag();



        telemetry.addData("start", "Ready to start");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();
        lift.noLimits=false;

        lift.resetLift();

        if (isStopRequested()) return;
        resetRuntime();


        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            prevupperClawOpen = upperClawOpen;
            prevlowerClawOpen = lowerClawOpen;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = 0;

            double botHeading;
            if (imu.getRobotYawPitchRollAngles() != null){
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            } else {
                botHeading = 0;
            }

            // Check bumpers and set target angle accordingly
            double targetAngle = (gamepad1.right_bumper) ? targetAngleZero :
                    (gamepad1.left_bumper) ? targetAngleFortyFive : Double.NaN;

            // Align to target angle if either bumper is pressed
            if (!Double.isNaN(targetAngle)) {
                strafeHeading = null; // Reset strafe heading
                double headingError = AngleUnit.normalizeRadians(targetAngle - botHeading);
                double rotationalError = -headingError * HEADING_KP;
                rx = rotationalError * ROTATIONAL_POWER_SCALE;
                rx = Math.signum(rx) * Math.min(Math.abs(rx), 1.0); // Clamp rx within [-1, 1]
            } else {
                // Strafing while maintaining heading
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    if (strafeHeading == null) {
                        strafeHeading = botHeading; // Capture the heading when strafing starts
                    }
                    x = (gamepad1.left_trigger > 0.1) ? -gamepad1.left_trigger * STRAFING_SCALE :
                            (gamepad1.right_trigger > 0.1) ? gamepad1.right_trigger * STRAFING_SCALE : x;
                    rx = maintainHeadingControl(strafeHeading, rx);
                } else {
                    strafeHeading = null; // Reset strafe heading when not strafing
                    rx = gamepad1.right_stick_x; // Normal operation
                }
            }

             // Default speed multiplier
            if (getSmallestYForTagIds1To6(currentDetections) != null && getSmallestYForTagIds1To6(currentDetections) < 20 && y < 0) {
                aprilSpeedMultiplier = 0.6; // Reduced speed multiplier for backward movement
            }

            double rotX = x;
            double rotY = y;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (y < 0) { // Apply the speed multiplier only for backward movement
                frontLeft.setPower(frontLeftPower * aprilSpeedMultiplier * slowDownMultiplier);
                backLeft.setPower(backLeftPower * aprilSpeedMultiplier * slowDownMultiplier);
                frontRight.setPower(frontRightPower * aprilSpeedMultiplier * slowDownMultiplier);
                backRight.setPower(backRightPower * aprilSpeedMultiplier * slowDownMultiplier);
            } else {
                frontLeft.setPower(frontLeftPower * slowDownMultiplier);
                backLeft.setPower(backLeftPower * slowDownMultiplier);
                frontRight.setPower(frontRightPower * slowDownMultiplier);
                backRight.setPower(backRightPower * slowDownMultiplier);
            }
            if (frontLeftPower>0.1){
                aprilSpeedMultiplier = 1;
            }
            telemetry.addData("y",y);

            if (armIn && distance2.getDistance(DistanceUnit.MM) < 15){
                claw.upperClaw(false);
                upperClawOpen = false;
                gamepad2.runRumbleEffect(customRumbleEffect2);
            }

            if (!upperClawOpen && armIn && distance.getDistance(DistanceUnit.MM) < 15){
                claw.lowerClaw(false);
                lowerClawOpen = false;
                //gamepad1.runRumbleEffect(customRumbleEffect);
            }

            if (!lowerClawOpen && !upperClawOpen && armIn){

            }
/*
            if (gamepad2.dpad_up){
                slowDownMultiplier = 0.5;
            } else {
                slowDownMultiplier = 1;
            }
*/





            /////////lift up and down
            lift.setPower(-gamepad2.right_stick_y);



            ////////intake vertical and belt in
            if (gamepad2.right_trigger >0.1 && armIn && lowerClawOpen) {
                intake.verticalPower((double) gamepad2.right_trigger);
                intake.setIntakebelt(gamepad2.right_trigger);
            } else {
                intake.verticalPower(0.0);
                intake.setIntakebelt(0.0);
            }

            ////////intake hori and roller in
            if (gamepad2.left_trigger >0.1 && armIn && lowerClawOpen) {
                intake.horiPower((double) -gamepad2.left_trigger);
                intake.setIntakeRoller(gamepad2.left_trigger);
            } else {
                intake.horiPower(0.0);
                intake.setIntakeRoller(0.0);
            }


            if (gamepad2.a) {
                intake.verticalPower(-0.5);
                intake.setIntakeRoller(-1);
            }
            if (gamepad2.b) {
                intake.horiPower(0.5);
            }

            if (!lowerClawOpen && !upperClawOpen && armIn){
                intake.horiPower(0.4);
                intake.verticalPower(-0.4);
                intake.setIntakeRoller(-1);
                intake.setIntakebelt(1);
            }

            /////claw controls grab
            if (gamepad2.left_bumper && !previousGamepad2.left_bumper && armIn){
                claw.lowerClaw(false);
                lowerClawOpen = false;
                claw.upperClaw(false);
                upperClawOpen = false;
                //gamepad1.runRumbleEffect(customRumbleEffect);
                //gamepad2.runRumbleEffect(customRumbleEffect);
            }
            if (gamepad2.right_bumper && !previousGamepad2.right_bumper && armIn){
                claw.lowerClaw(false);
                lowerClawOpen = false;
                claw.upperClaw(false);
                upperClawOpen = false;
                //gamepad1.runRumbleEffect(customRumbleEffect);
                //gamepad2.runRumbleEffect(customRumbleEffect);
            }

            /////claw controls release
            if (gamepad2.left_bumper && !armIn){
                claw.lowerClaw(true);
                lowerClawOpen = true;
            }
            if (gamepad2.right_bumper && !armIn){
                claw.upperClaw(true);
                upperClawOpen = true;
            }

            if (gamepad2.x){
                claw.upperClaw(true);
                upperClawOpen = true;
                claw.lowerClaw(true);
                lowerClawOpen = true;
            }

            ////out pos
            if (gamepad2.left_stick_y <-0.1 && !lowerClawOpen && !upperClawOpen && !runDelay1 && !ignore && gamepad2.left_trigger < 0.05){
                claw.setDeliverArm("delivery");
                delayTimer = getRuntime();
                runDelay = true;
                ignore = true;
                armIn = false;
            }
            if (getRuntime() > (delayTimer + 0.5) && runDelay){
                claw.setRotateAngle("horizontal", 0.0);
                runDelay = false;
                ignore = false;
            }
            ////in pos
            if ((gamepad2.left_stick_y >0.1 && lowerClawOpen && upperClawOpen && !runDelay && !armIn) || gamepad2.dpad_down){
                claw.setRotateAngle("intake", 0.0);
                delayTimer1 = getRuntime();
                runDelay1 = true;
                //gamepad1.runRumbleEffect(customRumbleEffect);
            }
            if (getRuntime() > (delayTimer1 + 0.3) && runDelay1){
                claw.setDeliverArm("intake");
                runDelay1 = false;
                armIn = true;
            }

            if(gamepad2.start && gamepad2.back){
                right_release.setPosition(0.54);
                left_release.setPosition(0.35);
            }

            if (gamepad1.dpad_up && climb.getCurrentPosition()<5200){
                climbPower = 1;
            } else if (climb.getCurrentPosition()>2000){
                climbPower = 0.3;
            } else{
                climbPower=0;//
            }
            if (gamepad1.dpad_down){
                climbPower = 0;
            }

            if (gamepad1.x && gamepad1.dpad_left){
                planeGoWee.setPosition(0.5);
            }
            if (gamepad1.a){
                planeGoWee.setPosition(0.00);
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.right_bumper) {
                if (slowDownMultiplier==1){
                    slowDownMultiplier =0.5;
                } else {
                    slowDownMultiplier =1;
                }
            }

            if (!lowerClawOpen && !upperClawOpen && (prevupperClawOpen || prevlowerClawOpen)){
                gamepad2.runRumbleEffect(customRumbleEffect);
                gamepad1.runRumbleEffect(customRumbleEffect);
            }

            climb.setPower(climbPower);

            lift.update();
            claw.update();
            intake.update();
            telemetry.addData("distance 1", distance.getDistance(DistanceUnit.MM));
            telemetry.addData("distance 2", distance2.getDistance(DistanceUnit.MM));
            telemetry.addData("April Speed", aprilSpeedMultiplier);
            telemetry.addData("Gamepad1 Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Bot Heading", botHeading);
            telemetry.addData("Strafe Heading", strafeHeading);
            telemetry.addData("Motor Powers", "FL: %.2f, BL: %.2f, FR: %.2f, BR: %.2f", frontLeftPower, backLeftPower, frontRightPower, backRightPower);

// Telemetry for claw and lift states
            telemetry.addData("Claw State", "Upper: %b, Lower: %b", upperClawOpen, lowerClawOpen);
            telemetry.addData("Lift Position", lift.currentPos());

// Update Telemetry
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

        }
        visionPortal.close();

    }

    private double maintainHeadingControl(Double targetHeading, double rx) {
        if (targetHeading == null) {
            return rx; // Return the current rx if no target heading is set
        }
        double heading;
        if (imu.getRobotYawPitchRollAngles() != null){
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } else {
            heading = 0;
        }
        double headingError = AngleUnit.normalizeRadians(targetHeading - heading);
        double rotationalError = -headingError * HEADING_KP;
        rx = rotationalError * ROTATIONAL_POWER_SCALE;
        rx = Math.signum(rx) * Math.min(Math.abs(rx), 1.0); // Clamp rx within [-1, 1]
        return rx;
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 360));

        builder.enableLiveView(true);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public Double getSmallestYForTagIds1To6(List<AprilTagDetection> detections) {
        Double smallestY = null;

        for (AprilTagDetection detection : detections) {
            if (detection.id >= 1 && detection.id <= 6) {
                if (smallestY == null || detection.ftcPose.y < smallestY) {
                    smallestY = detection.ftcPose.y;
                }
            }
        }

        return smallestY; // This will return null if no tags with ID 1-6 are found
    }


}