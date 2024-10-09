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


@Autonomous(name="BlueRightBack2plus3BrokenMaybe", group="FSMAuto", preselectTeleOp="RightTeleOp")
@Disabled
public class BlueRightBack2plus3BrokenMaybe extends LinearOpMode {
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
    BlueRightBack2plus3BrokenMaybe.State currentState = BlueRightBack2plus3BrokenMaybe.State.IDLE;
    Pose2d startPose = new Pose2d(-35.75, 62.75, Math.toRadians(90));
    //Pose2d startPose = new Pose2d(11.5, 62.75, Math.toRadians(90));

    //47.25 difference



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

        currentState = BlueRightBack2plus3BrokenMaybe.State.SpikeDelivery;
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
        asyncUpdatesThread.start();


        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.update();


            switch (currentState) {
////////////////////////////////////////////   Move 1    //////////////////////////////////////////
                case SpikeDelivery:
                    if (position == 1) {
                        TrajectorySequence LeftPos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-42, 44.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-32, 41, Math.toRadians(315)))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(LeftPos);
                        }
                    } else if (position == 2) {
                        TrajectorySequence MiddlePos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-40, 44.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40, 36.75, Math.toRadians(270)))
                                .build(); //-21, -60

                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(MiddlePos);
                        }
                    } else {
                        TrajectorySequence RightPos = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-44.75, 44.75, Math.toRadians(270)))
                                //.splineTo(new Vector2d(-44.75, 44.75), Math.toRadians(270))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel0;
                            drive.followTrajectorySequenceAsync(RightPos);
                        }
                    }
                    break;
////////////////////////////////////////////   Move 2    //////////////////////////////////////////
                case BackboardPixel0:
                        TrajectorySequence LeftPosBackboard = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> intake.horiPower(1.0))
                                .addTemporalMarker(0, () -> claw.lowerClaw(true))
                                .addTemporalMarker(0, () -> lowerClawOpen = true)
                                .waitSeconds(0.5)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                //.lineToLinearHeading(new Pose2d(-50, 50, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-50, 50, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-50, 47, Math.toRadians(225)))
                                .build(); //-21, -60*/
                        if (!drive.isBusy()) {
                            currentState = State.BackboardPixel1;
                            //currentState = State.IDLE;
                            drive.followTrajectorySequenceAsync(LeftPosBackboard);
                        }


                    break;
////////////////////////////////////////////   Move 3    //////////////////////////////////////////
                case BackboardPixel1:
                    TrajectorySequence LeftPosBackboarddepart = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                intake.horiPower(-1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                intake.verticalPower(1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                intake.setIntakeRoller(1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                intake.setIntakebelt(1.0);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                clawIntakeThread.start();
                            })
                            .lineToLinearHeading(new Pose2d(-61, 37, Math.toRadians(195)),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))

                            .build(); //-21, -60
                    if (!drive.isBusy()) {
                        currentState = State.PixelPickup1;
                        drive.followTrajectorySequenceAsync(LeftPosBackboarddepart);
                    }
                    break;

                    //move 4/////////////////////
                case PixelPickup1:
                    TrajectorySequence BackboardPixel22229 = drive.trajectorySequenceBuilder(poseEstimate)
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

                            //.lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)),//-2
                            .splineTo(new Vector2d(-35, 59), Math.toRadians(0),
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .addTemporalMarker(4, () -> {
                                runClawIntakeThread = false;
                                intake.horiPower(0.0);
                                intake.verticalPower(0.0);
                                intake.setIntakeRoller(0.0);
                                intake.setIntakebelt(0.0);
                            })

                            .lineToConstantHeading(new Vector2d(12, 59), //-2
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))

                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.BackboardPixel2;
                        drive.followTrajectorySequenceAsync(BackboardPixel22229);
                    }
                    break;
////////////////////////////////////////////   Move 4    //////////////////////////////////////////
                case BackboardPixel2:
                    if (position == 1) {

                    } else if (position == 2) {

                    } else {
                        TrajectorySequence RightPosBackboard12211 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .addTemporalMarker(0, () -> {
                                    lift.setTargetHeight(1000, 0);
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                    runClawIntakeThread = false;
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                })
                                .addTemporalMarker(0.8, () -> {claw.setRotateAngle("horizontal", 0.0);})
                                .splineTo(new Vector2d(52.5, 33), Math.toRadians(0),//-2
                                        SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(40))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.PixelPickup2;
                            drive.followTrajectorySequenceAsync(RightPosBackboard12211);
                        }
                    }
                    break;
////////////////////////////////////////////   Move 5    //////////////////////////////////////////
                case PixelPickup2:
                    TrajectorySequence LeftPosBackboarddepart69123 = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .addTemporalMarker(0, () -> {
                                claw.upperClaw(true);
                                upperClawOpen = true;

                            })
                            .addTemporalMarker(0, () -> {
                                claw.lowerClaw(true);
                                lowerClawOpen = true;
                            })
                            .addTemporalMarker(0.1, () -> {claw.update();})
                            .waitSeconds(0.5)
                            .addTemporalMarker(1.4, () -> claw.setRotateAngle("intake", 0.0))
                            .addTemporalMarker(1.7, () -> lift.setTargetHeight(0, 0))
                            .addTemporalMarker(1.7, () -> {
                                claw.setDeliverArm("intake");
                                armIn = true;
                            })
                            .splineTo(new Vector2d(12, 59), Math.toRadians(180),
                                    //.lineToLinearHeading(new Pose2d(24, 60, Math.toRadians(180)),//-2
                                    SampleMecanumDrive.getVelocityConstraint(20, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))


                            .build();

                    if (!drive.isBusy()) {
                        currentState = State.BackboardPixel3;
                        drive.followTrajectorySequenceAsync(LeftPosBackboarddepart69123);
                    }

                    break;
////////////////////////////////////////////   Move 6    //////////////////////////////////////////
                case BackboardPixel3:
                    TrajectorySequence LeftPosdepart6912333 = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .lineToConstantHeading(new Vector2d(-35, 59),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40)) //.lineToConstantHeading(new Vector2d(-60, -13))
                            .addTemporalMarker(1, () ->{
                                intake.horiPower(-1.0);
                                intake.verticalPower(1.0);
                                intake.setIntakeRoller(1.0);
                                intake.setIntakebelt(1.0);
                                clawIntakeThread.start();
                            })

                            .lineToLinearHeading(new Pose2d(-61, 37, Math.toRadians(195)),//-2
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))


                            .build();

                    if (!drive.isBusy()) {
                        currentState = State.PixelPickup3;

                        drive.followTrajectorySequenceAsync(LeftPosdepart6912333);
                    }
                    break;



////////////////////////////////////////////   Move 7    //////////////////////////////////////////
                case PixelPickup3:
                    TrajectorySequence BackboardPixel5126 = drive.trajectorySequenceBuilder(poseEstimate)



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
                            //.lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)),//-2
                            .splineTo(new Vector2d(-35, 59), Math.toRadians(0),
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))

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
                            .lineToConstantHeading(new Vector2d(12, 59), //-2
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
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
                            .waitSeconds(0.5)
                            .splineTo(new Vector2d(52.5, 35.5), Math.toRadians(0),
                                    SampleMecanumDrive.getVelocityConstraint(30, 40, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
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
                    if (!drive.isBusy()) {
                        currentState = State.BackboardPixel4;

                        drive.followTrajectorySequenceAsync(BackboardPixel5126);
                    }
                    break;
////////////////////////////////////////////   Move 8    //////////////////////////////////////////
                case BackboardPixel4:
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
                            .addTemporalMarker(0.4, () -> claw.setRotateAngle("intake", 0.0))
                            .addTemporalMarker(0.9, () -> lift.setTargetHeight(0, 0))
                            .addTemporalMarker(0.7, () -> {
                                claw.setDeliverArm("intake");
                                armIn = true;
                            })
                            .waitSeconds(0.3) //was 0.4
                            .lineToLinearHeading(new Pose2d(50, 50, Math.toRadians(180))) //-2
                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.Parking;

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
        ////////////////////////////////////////////   END    //////////////////////////////////////////

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