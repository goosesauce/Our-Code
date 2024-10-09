package org.firstinspires.ftc.teamcode.autonomous.old;

import static org.firstinspires.ftc.teamcode.functions.detection.getPosition;
import static org.firstinspires.ftc.teamcode.functions.detection.setColour;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.functions.claw;
import org.firstinspires.ftc.teamcode.functions.hardwareInit;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.lift;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="SafeBlueAutoLeftSide", group="FSMAuto", preselectTeleOp="RightTeleOp")
@Disabled
public class SafeBlueAutoLeftSide extends LinearOpMode {
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
    SafeBlueAutoLeftSide.State currentState = SafeBlueAutoLeftSide.State.IDLE;
    Pose2d startPose = new Pose2d(12, 63, Math.toRadians(90));



    @Override
    public void runOpMode() throws InterruptedException {
        new hardwareInit(hardwareMap);
        setColour("Blue"); //Blue or Red
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
                            intake.horiPower(-0.4);
                            intake.verticalPower(-0.4);
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

        asyncUpdatesThread = new Thread(asyncUpdates);

        currentState = SafeBlueAutoLeftSide.State.SpikeDelivery;
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
                case SpikeDelivery:
                    if (position == 1) {
                        TrajectorySequence LeftPos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.5))
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.))
                                .addTemporalMarker(0.5, () -> {
                                    lift.setTargetHeight(400, 0);
                                })
                                .addTemporalMarker(0.5, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .lineToLinearHeading(new Pose2d(34, 30, Math.toRadians(-180)))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(LeftPos);
                        }
                    } else if (position == 2) {
                        TrajectorySequence MiddlePos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.5))
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.))
                                .addTemporalMarker(0.5, () -> {
                                    lift.setTargetHeight(400, 0);
                                })
                                .addTemporalMarker(0.5, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .lineToLinearHeading(new Pose2d(24, 25, Math.toRadians(-180)))
                                .build(); //-21, -60

                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(MiddlePos);
                        }
                    } else {
                        TrajectorySequence RightPos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.5))
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.))
                                .addTemporalMarker(0.5, () -> {
                                    lift.setTargetHeight(400, 0);
                                })
                                .addTemporalMarker(0.5, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .strafeTo(new Vector2d(18, 50))
                                .lineToLinearHeading(new Pose2d(12, 37, Math.toRadians(-180)))
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
                                .addTemporalMarker(0, () -> intake.horiPower(-0.7))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                .splineTo(new Vector2d(51, 44), Math.toRadians(0))//was28.5
                                .build(); //-21, -60*/
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            drive.followTrajectorySequenceAsync(LeftPosBackboard);
                        }
                    } else if (position == 2) {
                        TrajectorySequence MiddlePosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(-0.5))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                .splineTo(new Vector2d(51, 39), Math.toRadians(0))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            drive.followTrajectorySequenceAsync(MiddlePosBackboard);
                        }
                    } else {
                        TrajectorySequence RightPosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(-0.8))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))

                                .splineToConstantHeading(new Vector2d(51, 33), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).
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
                                claw.update();
                                upperClawOpen = true;
                            })
                            .addTemporalMarker(0, () -> {
                                claw.lowerClaw(true);
                                lowerClawOpen = true;
                            })
                            .addTemporalMarker(0.1, () -> {claw.update();})
                            .waitSeconds(1)
                            .addTemporalMarker(1.4, () -> claw.setRotateAngle("intake", 0.0))
                            .addTemporalMarker(1.7, () -> lift.setTargetHeight(0, 0))
                            .addTemporalMarker(1.7, () -> {
                                claw.setDeliverArm("intake");
                                armIn = true;
                            })
                            .addTemporalMarker(4, () -> intake.horiPower(1.0))
                            .addTemporalMarker(4, () -> intake.verticalPower(1.0))
                            .addTemporalMarker(4, () -> intake.setIntakeRoller(1.0))
                            .addTemporalMarker(4, () -> intake.setIntakebelt(1.0))
                            .addTemporalMarker(4,() -> clawIntakeThread.start())
                            .splineTo(new Vector2d(24, 11), Math.toRadians(180),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineToConstantHeading(new Vector2d(-56, 12),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) //.lineToConstantHeading(new Vector2d(-60, -13))
                            .lineToConstantHeading(new Vector2d(-61, 12),//-2
                                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build(); //-21, -60
                    if (!drive.isBusy()) {
                        currentState = State.PixelPickup1;
                        drive.followTrajectorySequenceAsync(LeftPosBackboarddepart);
                    }
                    break;
                case PixelPickup1:
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
                                .lineToConstantHeading(new Vector2d(12, 12), //-2
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    lift.setTargetHeight(750, 0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .waitSeconds(5)
                                .splineTo(new Vector2d(52, 35.5), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                    }
                    break;
                case BackboardPixel4:
                    if (!drive.isBusy()) {
                        currentState = State.Parking;
                        TrajectorySequence BackboardPixel4 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
                                .addTemporalMarker(0, () -> {
                                    claw.upperClaw(true);
                                    claw.update();
                                    upperClawOpen = true;
                                })
                                .addTemporalMarker(0, () -> {
                                    claw.lowerClaw(true);
                                    lowerClawOpen = true;
                                })
                                .addTemporalMarker(1.4, () -> claw.setRotateAngle("intake", 0.0))
                                .addTemporalMarker(1.7, () -> lift.setTargetHeight(0, 0))
                                .addTemporalMarker(1.7, () -> {
                                    claw.setDeliverArm("intake");
                                    armIn = true;
                                })
                                .waitSeconds(1) //was 0.4
                                .lineToLinearHeading(new Pose2d(44, 10, Math.toRadians(180))) //-2
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