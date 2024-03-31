package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.functions.detection.getPosition;
import static org.firstinspires.ftc.teamcode.functions.detection.setColour;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.functions.claw;
import org.firstinspires.ftc.teamcode.functions.hardwareInit;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.lift;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="RedRightBack2plus4", group="FSMAuto", preselectTeleOp="teleOp")
public class RedRightBack2plus4 extends LinearOpMode {


    private DistanceSensor distance;
    private DistanceSensor distance2;
    boolean lowerClawOpen = true;
    boolean upperClawOpen = false;
    boolean armIn = true;

    private Thread clawIntakeThread;
    private volatile boolean runClawIntakeThread = true;

    private Thread asyncUpdatesThread;
    private volatile boolean asyncThread = true;

    int position = 0;
    IMU imu;


    enum State {
        SpikeDelivery,
        BackboardPixel0,
        BackboardPixel1,
        PixelPickup1,
        BackboardPixel2,
        PixelPickup2,
        BackboardPixel3,
        PixelPickup3,
        BackboardPixel4,
        Parking,
        IDLE
    }
    // Default to the idle state and define our start pos
    RedRightBack2plus4.State currentState = RedRightBack2plus4.State.IDLE;
    Pose2d startPose = new Pose2d(12, -62.75, Math.toRadians(-90));
    //Pose2d startPose = new Pose2d(-63, -38.7, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {
        new hardwareInit(hardwareMap);
        setColour("Red"); //Blue or Red
        lift lift = new lift(hardwareMap);
        intake intake = new intake(hardwareMap);
        claw claw = new claw(hardwareMap);

        distance = hardwareMap.get(DistanceSensor.class, "distance");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        lift.enablePID(true);
        claw.update();

        Runnable clawIntakeControl = new Runnable() {
            @Override
            public void run() {
                try {
                    while (runClawIntakeThread && !Thread.currentThread().isInterrupted()) {
                        if (armIn && distance2.getDistance(DistanceUnit.MM) < 30) {
                            claw.upperClaw(false);
                            upperClawOpen = false;
                        }

                        if (!upperClawOpen && armIn && distance.getDistance(DistanceUnit.MM) < 30) {
                            claw.lowerClaw(false);
                            lowerClawOpen = false;
                        }

                        if (!lowerClawOpen && !upperClawOpen && armIn) {
                            intake.horiPower(0.8);
                            intake.verticalPower(-0.8);
                            intake.setIntakeRoller(-1);
                            intake.setIntakebelt(1);
                        }


                        // Optional: Adjust sleep time as necessary
                        Thread.sleep(10);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        };

        Runnable asyncUpdates = new Runnable() {
            @Override
            public void run() {
                while (asyncThread && !Thread.currentThread().isInterrupted()) {
                    lift.update();
                    claw.update();
                    intake.update();
                }
            }
        };

        clawIntakeThread = new Thread(clawIntakeControl);
        clawIntakeThread.start();
        asyncUpdatesThread = new Thread(asyncUpdates);


        long lastLoopTime = System.nanoTime();
        currentState = RedRightBack2plus4.State.SpikeDelivery;
        boolean set = false;
        while (!isStarted() && !isStopRequested()) {
            position = getPosition();
            telemetry.addData("Position" , position);
            telemetry.update();
        }
        telemetry.addData("parking", position);
        telemetry.update();
        asyncUpdatesThread.start();


        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.update();

            switch (currentState) {
                case SpikeDelivery: //.splineTo(new Vector2d(52, -27.5), Math.toRadians(0))
                    if (position == 1) {
                        TrajectorySequence LeftPos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0.5, () -> {
                                    lift.setTargetHeight(600, 0);
                                })
                                .addTemporalMarker(0.5, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .strafeTo(new Vector2d(16, -50))
                                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(-180)))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(LeftPos);
                        }
                    } else if (position == 2) {
                        TrajectorySequence MiddlePos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0.5, () -> {
                                    lift.setTargetHeight(600, 0);
                                })
                                .addTemporalMarker(0.5, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .strafeTo(new Vector2d(28, -40))
                                .lineToLinearHeading(new Pose2d(26, -26, Math.toRadians(-180)))
                                .build(); //-21, -60

                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(MiddlePos);
                        }
                    } else {
                        TrajectorySequence RightPos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0.5, () -> {
                                    lift.setTargetHeight(600, 0);
                                })
                                .addTemporalMarker(0.5, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .lineToLinearHeading(new Pose2d(34, -36, Math.toRadians(-180)))

                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(RightPos);
                        }
                    }
                    break;
                case BackboardPixel0:
                    if (position == 1) {
                        TrajectorySequence LeftPosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.7))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                .splineTo(new Vector2d(52, -31), Math.toRadians(0))//was28.5
                                .build(); //-21, -60*/
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            drive.followTrajectorySequenceAsync(LeftPosBackboard);
                        }
                    } else if (position == 2) {
                        TrajectorySequence MiddlePosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.5))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                .splineTo(new Vector2d(52, -35.5), Math.toRadians(0))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            drive.followTrajectorySequenceAsync(MiddlePosBackboard);
                        }
                    } else {
                        TrajectorySequence RightPosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.7))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                .splineToConstantHeading(new Vector2d(52, -42.5), Math.toRadians(0)).
                                build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            drive.followTrajectorySequenceAsync(RightPosBackboard);
                        }
                    }
                    break;
                case BackboardPixel1:
                    TrajectorySequence LeftPosBackboarddepart = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .addTemporalMarker(0, () -> {
                                claw.upperClaw(true);
                                upperClawOpen = true;
                            })
                            .addTemporalMarker(0, () -> {
                                claw.lowerClaw(true);
                                lowerClawOpen = true;
                            })
                            .addTemporalMarker(0.8, () -> claw.setRotateAngle("intake", 0.0))
                            .addTemporalMarker(1.2, () -> lift.setTargetHeight(0, 0))
                            .addTemporalMarker(1.2, () -> {
                                claw.setDeliverArm("intake");
                                armIn = true;
                            })
                            .addTemporalMarker(3, () -> intake.horiPower(-1.0))
                            .addTemporalMarker(3, () -> intake.verticalPower(1.0))
                            .addTemporalMarker(3, () -> intake.setIntakeRoller(1.0))
                            .addTemporalMarker(3, () -> intake.setIntakebelt(1.0))
                            .waitSeconds(0.2) //was 0.4
                            .splineTo(new Vector2d(24, -60), Math.toRadians(180))
                            .lineToConstantHeading(new Vector2d(-40, -60)) //.lineToConstantHeading(new Vector2d(-60, -13))
                            .lineToLinearHeading(new Pose2d(-62.5, -40, Math.toRadians(165)))
                            /*.lineToConstantHeading(new Vector2d(-62.5, -41),
                                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .splineTo(new Vector2d(-58, -40), Math.toRadians(180))
                            .lineToConstantHeading(new Vector2d(-62.5, -41),
                                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                            .build(); //-21, -60
                    if (!drive.isBusy()) {
                        currentState = State.PixelPickup1;
                        drive.followTrajectorySequenceAsync(LeftPosBackboarddepart);
                    }
                    break;
                case PixelPickup1: //.splineTo(new Vector2d(24, -13), Math.toRadians(180))
                    if (!drive.isBusy()) {
                        currentState = State.BackboardPixel2;
                    }
                    break;
                case BackboardPixel2:
                    if (!drive.isBusy()) {
                        currentState = State.PixelPickup2;
                        TrajectorySequence BackboardPixel2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(2.5, () -> {
                                    claw.upperClaw(false);
                                    upperClawOpen = false;
                                })
                                .addTemporalMarker(2.5, () -> {
                                    claw.lowerClaw(false);
                                    lowerClawOpen = false;
                                })
                                //.waitSeconds(0.2) //was0.4
                                .splineTo(new Vector2d(-40, -60), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(12, -60))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    lift.setTargetHeight(800, 0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .splineTo(new Vector2d(52, -44), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.horiPower(0.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.verticalPower(0.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.setIntakeRoller(0.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.setIntakebelt(0.0);
                                })
                                .build();
                        drive.followTrajectorySequenceAsync(BackboardPixel2);
                    }
                    break;
                case PixelPickup2:
                    if (!drive.isBusy()) {
                        currentState = State.BackboardPixel3;
                        TrajectorySequence PixelPickup2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
                                .addTemporalMarker(0, () -> {
                                    claw.upperClaw(true);
                                    upperClawOpen = true;
                                })
                                .addTemporalMarker(0, () -> {
                                    claw.lowerClaw(true);
                                    lowerClawOpen = true;
                                })
                                .addTemporalMarker(0.8, () -> claw.setRotateAngle("intake", 0.0))
                                .addTemporalMarker(1.2, () -> lift.setTargetHeight(0, 0))
                                .addTemporalMarker(1.2, () -> {
                                    claw.setDeliverArm("intake");
                                    armIn = true;
                                })
                                .addTemporalMarker(3, () -> intake.horiPower(-1.0))
                                .addTemporalMarker(3, () -> intake.verticalPower(1.0))
                                .addTemporalMarker(3, () -> intake.setIntakeRoller(1.0))
                                .addTemporalMarker(3, () -> intake.setIntakebelt(1.0))
                                .waitSeconds(0.4) //was 0.6
                                .splineTo(new Vector2d(24, -60), Math.toRadians(180))
                                .lineToConstantHeading(new Vector2d(-40, -60)) //.lineToConstantHeading(new Vector2d(-60, -13))
                                .lineToLinearHeading(new Pose2d(-62.5, -40, Math.toRadians(165)))
                                //.lineToConstantHeading(new Vector2d(-62.5, -41),
                                  //      SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    //    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
                        drive.followTrajectorySequenceAsync(PixelPickup2);
                    }
                    break;
                case BackboardPixel3:
                    if (!drive.isBusy()) {
                        currentState = State.PixelPickup3;
                    }
                    break;
                case PixelPickup3:
                    if (!drive.isBusy()) {
                        currentState = State.BackboardPixel4;
                        TrajectorySequence PixelPickup3 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(2.5, () -> {
                                    claw.upperClaw(false);
                                    upperClawOpen = false;
                                })
                                .addTemporalMarker(2.5, () -> {
                                    claw.lowerClaw(false);
                                    lowerClawOpen = false;
                                })
                                //.waitSeconds(0.2) //was0.4
                                .splineTo(new Vector2d(-40, -60), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(12, -60))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    lift.setTargetHeight(1200, 0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .splineTo(new Vector2d(52, -46), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.horiPower(0.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.verticalPower(0.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.setIntakeRoller(0.0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intake.setIntakebelt(0.0);
                                })
                                .build();
                        drive.followTrajectorySequenceAsync(PixelPickup3);
                    }
                    break;
                case BackboardPixel4:
                    if (!drive.isBusy()) {
                        currentState = State.Parking;
                        TrajectorySequence BackboardPixel4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
                                .addTemporalMarker(0, () -> {
                                    claw.upperClaw(true);
                                    upperClawOpen = true;
                                })
                                .addTemporalMarker(0, () -> {
                                    claw.lowerClaw(true);
                                    lowerClawOpen = true;
                                })
                                .addTemporalMarker(0.4, () -> claw.setRotateAngle("intake", 0.0))
                                .addTemporalMarker(0.7, () -> lift.setTargetHeight(0, 0))
                                .addTemporalMarker(0.7, () -> {
                                    claw.setDeliverArm("intake");
                                    armIn = true;
                                })
                                .waitSeconds(0.2) //was 0.4
                                .lineToLinearHeading(new Pose2d(44, -50, Math.toRadians(180)))
                                .build();
                        drive.followTrajectorySequenceAsync(BackboardPixel4);
                    }
                    break;
                case Parking:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;

                    }
                    break;
                case IDLE:

                    break;

            }



        }

        runClawIntakeThread = false; // Signal the thread to stop
        clawIntakeThread.interrupt();
        try {
            clawIntakeThread.join(); // Wait for the thread to finish
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        asyncThread = false;
        asyncUpdatesThread.interrupt();
        try {
            asyncUpdatesThread.join(); // Wait for the thread to finish
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }



    }


}
