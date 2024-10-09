package org.firstinspires.ftc.teamcode.autonomous.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public class referenceAuto {
}
/*

                                .setReversed(true)
                                .addTemporalMarker(1, () -> {lift.setTargetHeight(400,0);})
                                .addTemporalMarker(1, () -> {claw.setDeliverArm("delivery");})
                                .addTemporalMarker(1.8, () -> {claw.setRotateAngle("horizontal", 0.0);})
                                .splineToSplineHeading(new Pose2d(14, -30, Math.toRadians(-180)), Math.toRadians(-90))
                                .UNSTABLE_addTemporalMarkerOffset(0,() -> intake.horiPower(-0.8))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> intake.horiPower(0.0))
                                .splineTo(new Vector2d(52, -27.5), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0,() ->  claw.upperClaw(true))
                                .UNSTABLE_addTemporalMarkerOffset(0,() ->  claw.lowerClaw(true))
                                .waitSeconds(0.1)



                                .setReversed(false)
                                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                                .addTemporalMarker(0,() ->  claw.setRotateAngle("intake", 0.0))
                                .addTemporalMarker(0.5,() ->  lift.setTargetHeight(0,0))
                                .addTemporalMarker(0.5,() ->  claw.setDeliverArm("intake"))



package org.firstinspires.ftc.teamcode.Opmodes.auto;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.lynx.LynxModule;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.teamcode.Camera.AprilTagDetectionPipeline;
        import org.firstinspires.ftc.teamcode.Objects.Claw;
        import org.firstinspires.ftc.teamcode.Objects.Lift;
        import org.firstinspires.ftc.teamcode.Objects.Turret;
        import org.firstinspires.ftc.teamcode.Objects.autoToTeleOp;
        import org.firstinspires.ftc.teamcode.Opmodes.LeftTeleOp;
        import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
        import org.firstinspires.ftc.teamcode.RoadRunner.drive.MecanumDrive;
        import org.firstinspires.ftc.teamcode.RoadRunner.drive.trajectorysequence.TrajectorySequence;
        import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

        import java.util.ArrayList;
        import java.util.List;


@Autonomous(name="FourConeAutonomousRightSide", group="FSMAuto", preselectTeleOp="RightTeleOp")
public class FourConeAutonomusRightSide extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private CRServo turret1, turret2;
    private DcMotor turretEncoder;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;



    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int Left = 1;
    int Middle = 2;
    int Right = 3;

    int position = 0;
    BNO055IMU imu;

    AprilTagDetection tagOfInterest = null;

    enum State {
        HighJunctionFromStart,
        ConePickup,
        HighJunction,
        ConePickup2,
        HighJunction2,
        ConePickup3,
        HighJunction3,
        ConePickup4,
        HighJunction4,
        ConePickup5,
        HighJunction5,
        Parking,
        IDLE
    }
    // Default to the idle state and define our start pos
    FourConeAutonomusRightSide.State currentState = FourConeAutonomusRightSide.State.IDLE;
    Pose2d startPose = new Pose2d(0, 0, 0);
    //Pose2d startPose = new Pose2d(-63, -38.7, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {
        //for camera detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            //@Override
            public void onError(int errorCode)
            {

            }
        });
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sleep(1000);
        turret1 = hardwareMap.crservo.get("turret1");
        turret2 = hardwareMap.crservo.get("turret2");
        Lift lift = new Lift(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        LeftTeleOp LeftTeleOp = new LeftTeleOp();

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        double LiftStartHeight = 250;
        double startRotation = -2650;
        double reducedVelocity = 10;
        double pickup = -6700-startRotation;
        double dropoff= 0-startRotation;
        double yDropOff = 22; //need to reduce likely was 25
        double yPickUp =-14.5;
        double xDropOff = 56; //was 56 needs testing.
        double pickupoffset = 1.4;
        double xOffset = 0.1;

        lift.enablePID(true);
        claw.lowerArm(false, false);
        claw.grab(true);
        claw.update();
        Orientation startPos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        autoToTeleOp.imuHeading = startPos.firstAngle;

        currentState = FourConeAutonomusRightSide.State.HighJunctionFromStart;
        boolean set = false;
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        break;
                    }
                }
            }
            if (gamepad2.a){
                set = true;
            }
            if (!set) {
                turretEncoder = hardwareMap.dcMotor.get("backRight");
                turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (tagOfInterest != null){
                telemetry.addData("", tagOfInterest.id);
            }
            telemetry.addData("Position set", set);
            telemetry.update();
            sleep(5);
        }
        if(tagOfInterest == null){
            position = 0;
        } else if(tagOfInterest.id == Left){
            position = 1;
        } else if(tagOfInterest.id == Middle){
            position = 2;
        } else {
            position = 3;
        }

        while (opModeIsActive() && !isStopRequested()) {

            /*for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }*//*

            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.update();
            lift.update();
            turret.update();
            claw.update();


            // Print pose to telemetry
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("current ang", turret.rawTurret);
            //telemetry.addData("current ang", turret.runNumber);
            telemetry.addData("parking", position);

            telemetry.update();
            switch (currentState) {
                case HighJunctionFromStart:
                    TrajectorySequence HighJunction = drive.trajectorySequenceBuilder(startPose)
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2890, LiftStartHeight))
                            .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, true))

                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 1)) //-6500

                            .splineTo(new Vector2d(22, 4), Math.toRadians(0))
                            .splineTo(new Vector2d(40 , 4), Math.toRadians(0))

                            //.UNSTABLE_addTemporalMarkerOffset(0.3, () -> turret.rotateEncoder(000, 1))
                            .splineToSplineHeading(new Pose2d(54.5, 18.5, Math.toRadians(270)), Math.toRadians(90))
                            .splineTo(new Vector2d(54.5, yDropOff), Math.toRadians(90),
                                    MecanumDrive.getVelocityConstraint(reducedVelocity, DriveConstants.MAX_ANG_VEL,
                                            DriveConstants.TRACK_WIDTH),
                                    MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            //.waitSeconds(0.05)
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2600, LiftStartHeight))
                            .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.grab(false))
                            .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, false))
                            //.waitSeconds(0.2)
                            .build();

                    if (!drive.isBusy()) {
                        currentState = State.ConePickup;
                        //Sequence for this case
                        drive.followTrajectorySequenceAsync(HighJunction);
                    }
                    break;
                case ConePickup:
                    if (!drive.isBusy()) {
                        currentState = State.HighJunction;
                        TrajectorySequence ConePickup = drive.trajectorySequenceBuilder(poseEstimate)
                                .UNSTABLE_addTemporalMarkerOffset(pickupoffset+0.3, () -> claw.grab(true))
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 2)) //-4500
                                .UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(430, LiftStartHeight))
                                .splineTo(new Vector2d(52.5, yPickUp), Math.toRadians(270))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> claw.grab(true))
                                //.waitSeconds(0.25)
                                .build();
                        drive.followTrajectorySequenceAsync(ConePickup);
                    }
                    break;
                case HighJunction:
                    if (!drive.isBusy()) {
                        currentState = State.ConePickup2;
                        TrajectorySequence HighJunction1 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, true))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 3)) //-6500
                                .splineTo(new Vector2d(xDropOff, yDropOff), Math.toRadians(90))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2600, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.grab(false))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, false))
                                //.waitSeconds(0.2)
                                .build();
                        drive.followTrajectorySequenceAsync(HighJunction1);
                    }
                    break;
                case ConePickup2:
                    if (!drive.isBusy()) {
                        currentState = State.HighJunction2;
                        TrajectorySequence ConePickup2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .UNSTABLE_addTemporalMarkerOffset(pickupoffset, () -> claw.grab(true))
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 4)) //-4500
                                .UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(333, LiftStartHeight))
                                .splineTo(new Vector2d(53+xOffset, yPickUp), Math.toRadians(270))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> claw.grab(true))
                                //.waitSeconds(0.25)
                                .build();
                        drive.followTrajectorySequenceAsync(ConePickup2);
                    }
                    break;
                case HighJunction2:
                    if (!drive.isBusy()) {
                        currentState = State.ConePickup3;
                        TrajectorySequence HighJunction2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, true))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 5)) //4000
                                .splineTo(new Vector2d(xDropOff, yDropOff), Math.toRadians(90))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2600, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.grab(false))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, false))
                                //.waitSeconds(0.2)
                                .build();
                        drive.followTrajectorySequenceAsync(HighJunction2);
                    }
                    break;
                case ConePickup3:
                    if (!drive.isBusy()) {
                        currentState = State.HighJunction3;
                        TrajectorySequence ConePickup3 = drive.trajectorySequenceBuilder(poseEstimate)
                                .UNSTABLE_addTemporalMarkerOffset(pickupoffset, () -> claw.grab(true))
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 6)) //-4500
                                .UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(210, LiftStartHeight))
                                .splineTo(new Vector2d(53+xOffset, yPickUp), Math.toRadians(270))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> claw.grab(true))
                                //.waitSeconds(0.3)
                                .build();
                        drive.followTrajectorySequenceAsync(ConePickup3);
                    }
                    break;
                case HighJunction3:
                    if (!drive.isBusy()) {
                        currentState = State.ConePickup4;
                        TrajectorySequence HighJunction3 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, true))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 7)) //4000
                                .splineTo(new Vector2d(xDropOff, yDropOff), Math.toRadians(90))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2600, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.grab(false))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, false))
                                //.waitSeconds(0.2)
                                .build();
                        drive.followTrajectorySequenceAsync(HighJunction3);
                    }
                    break;
                case ConePickup4:
                    if (!drive.isBusy()) {
                        currentState = State.HighJunction4;
                        TrajectorySequence ConePickup4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .UNSTABLE_addTemporalMarkerOffset(pickupoffset+0.1, () -> claw.grab(true))
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 8)) //-4500
                                .UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(113, LiftStartHeight))
                                .splineTo(new Vector2d(53+xOffset, yPickUp), Math.toRadians(270))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> claw.grab(true))
                                //.waitSeconds(0.3)
                                .build();
                        drive.followTrajectorySequenceAsync(ConePickup4);
                    }
                    break;
                case HighJunction4:
                    if (!drive.isBusy()) {
                        currentState = State.ConePickup5;
                        TrajectorySequence HighJunction4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, true))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 9)) //4000
                                .splineTo(new Vector2d(xDropOff, yDropOff), Math.toRadians(90))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2600, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.grab(false))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, false))
                                //.waitSeconds(0.2)
                                .build();
                        drive.followTrajectorySequenceAsync(HighJunction4);
                    }
                case ConePickup5:
                    if (!drive.isBusy()) {
                        currentState = State.Parking;
                        TrajectorySequence ConePickup4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .UNSTABLE_addTemporalMarkerOffset(pickupoffset+0.2, () -> claw.grab(true))
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 10)) //-4500
                                .UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(0, LiftStartHeight))
                                .splineTo(new Vector2d(53.2+xOffset, yPickUp), Math.toRadians(270))
                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> claw.grab(true))
                                //.waitSeconds(0.3)
                                .build();
                        drive.followTrajectorySequenceAsync(ConePickup4);
                    }
                    break;
                case HighJunction5:
                    if (!drive.isBusy()) {
                        currentState = State.Parking;
                        TrajectorySequence HighJunction4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, true))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 11)) //4000
                                .splineTo(new Vector2d(xDropOff, yDropOff), Math.toRadians(90))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(2600, LiftStartHeight))
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> claw.grab(false))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.lowerArm(false, false))
                                .waitSeconds(0.2)
                                .build();
                        drive.followTrajectorySequenceAsync(HighJunction4);
                    }


                case Parking:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        if(tagOfInterest.id == Left){
                            TrajectorySequence LeftParking = drive.trajectorySequenceBuilder(poseEstimate)
                                    //.splineTo(new Vector2d(50, yDropOff), Math.toRadians(270))
                                    //.UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                    // .UNSTABLE_addTemporalMarkerOffset(0, () -> turret.rotateEncoder(pickup, 12)) //-4500

                                    .setReversed(true)
                                    .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(500, LiftStartHeight))
                                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 11)) //4000
                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.setTargetHeight(0, LiftStartHeight))

                                    //.UNSTABLE_addTemporalMarkerOffset(0, () -> claw.lowerArm(false, false))
                                    //.UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setTargetHeight(0, LiftStartHeight))
                                    .splineTo(new Vector2d(51, 31), Math.toRadians(90)) ////34
                                    .build(); //-21, -60
                            drive.followTrajectorySequenceAsync(LeftParking);
                        } else if(tagOfInterest.id == Middle){
                            TrajectorySequence MiddleParking = drive.trajectorySequenceBuilder(poseEstimate)
                                    //.UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                    ///////.UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 12)) //-4500
                                    //.UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(0, LiftStartHeight))

                                    .setReversed(true)
                                    .UNSTABLE_addTemporalMarkerOffset(0.01, () -> lift.setTargetHeight(500, LiftStartHeight))
                                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> turret.rotateEncoder(dropoff, 11)) //4000
                                    .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.setTargetHeight(0, LiftStartHeight))


                                    .splineTo(new Vector2d(51, 7), Math.toRadians(90))
                                    .build(); //-21, -60
                            drive.followTrajectorySequenceAsync(MiddleParking);
                        } else {
                            /*TrajectorySequence RightParking = drive.trajectorySequenceBuilder(poseEstimate)
                                    .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setTargetHeight(2890, LiftStartHeight))
                                    //.UNSTABLE_addTemporalMarkerOffset(.5, () -> turret.rotateEncoder(pickup, 12)) //-4500
                                    .UNSTABLE_addTemporalMarkerOffset(.7, () -> lift.setTargetHeight(0, LiftStartHeight))
                                    .splineTo(new Vector2d(52.5, yPickUp), Math.toRadians(270))
                                    .build(); //-21, -60
                            drive.followTrajectorySequenceAsync(RightParking);

                             */


/*
                        }
                    }
                    break;
                case IDLE:
                    //Once Idle is reached autonomous code has ended

                    break;
            }

        }

    }
}
*/