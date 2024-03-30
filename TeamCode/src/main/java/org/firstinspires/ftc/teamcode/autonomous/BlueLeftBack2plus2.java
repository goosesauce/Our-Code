package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.functions.detection.getPosition;
import static org.firstinspires.ftc.teamcode.functions.detection.setColour;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name="BlueLeftBack2plus2", group="FSMAuto", preselectTeleOp="RightTeleOp")
public class BlueLeftBack2plus2 extends LinearOpMode {
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
    BlueLeftBack2plus2.State currentState = BlueLeftBack2plus2.State.IDLE;
    Pose2d startPose = new Pose2d(11.5, 62.75, Math.toRadians(90));

    double distance2var = 0.0;
    double distance1var = 0.0;



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
                        distance2var = distance2.getDistance(DistanceUnit.MM);
                        distance1var = distance.getDistance(DistanceUnit.MM);

                        if (armIn && distance2var < 30) {
                            claw.upperClaw(false);
                            upperClawOpen = false;
                        }

                        if (!upperClawOpen && armIn && distance1var < 30) {
                            claw.lowerClaw(false);
                            lowerClawOpen = false;
                        }

                        if (!lowerClawOpen && !upperClawOpen && armIn) {
                            intake.horiPower(1.0);
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



        clawIntakeThread = new Thread(clawIntakeControl);


        currentState = BlueLeftBack2plus2.State.SpikeDelivery;
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        while (!isStarted() && !isStopRequested()) {
            position = getPosition();
            telemetry.addData("Position" , position);
            telemetry.update();
        }
        telemetry.addData("parking", position);
        telemetry.update();


        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.update();
            lift.update();
            claw.update();
            intake.update();

            switch (currentState) {
                case SpikeDelivery:
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
                                .lineToLinearHeading(new Pose2d(34, 32, Math.toRadians(-180)))
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
                                .lineToLinearHeading(new Pose2d(24, 25, Math.toRadians(-180)))
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
                                .strafeTo(new Vector2d(16, 50))
                                .lineToLinearHeading(new Pose2d(12, 32, Math.toRadians(-180)))
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
                                .lineToLinearHeading(new Pose2d(52.5, 44, Math.toRadians(180)),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(20))
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
                                .splineTo(new Vector2d(52.5, 39.5), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(20))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            drive.followTrajectorySequenceAsync(MiddlePosBackboard);
                        }
                    } else {
                        TrajectorySequence RightPosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(0.6))
                                //.waitSeconds(0.3)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))

                                .splineTo(new Vector2d(52.5, 33), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(20)).
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

                            .splineTo(new Vector2d(24, 60), Math.toRadians(180),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineToConstantHeading(new Vector2d(-40, 60),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)) //.lineToConstantHeading(new Vector2d(-60, -13))
                            .lineToLinearHeading(new Pose2d(-55, 45, Math.toRadians(195)),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                intake.horiPower(-1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                intake.verticalPower(1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                intake.setIntakeRoller(1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                intake.setIntakebelt(1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                clawIntakeThread.start();
                            })
                            .lineToConstantHeading(new Vector2d(-61, 37),//-2
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
                                .lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)),//-2
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            runClawIntakeThread = false;
                        })
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
                                .lineToConstantHeading(new Vector2d(12, 60), //-2
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    lift.setTargetHeight(1000, 0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .waitSeconds(1)
                                .splineTo(new Vector2d(52.5, 35.5), Math.toRadians(0),
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
                                .lineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180))) //-2
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