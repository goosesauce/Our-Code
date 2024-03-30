package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.functions.detection.getPosition;
import static org.firstinspires.ftc.teamcode.functions.detection.setColour;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.functions.claw;
import org.firstinspires.ftc.teamcode.functions.hardwareInit;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.lift;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="testingtest", group="FSMAuto", preselectTeleOp="RightTeleOp")
public class testingtest extends LinearOpMode {
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
    boolean AutoReject = false;



    enum State {
        State1,
        State2,
        State3,
        State4,
        State5,
        State6,
        State7,
        State8,
        State9,
        State10,
        State11,
        State12,
        State13,
        State14,
        IDLE
    }
    // Default to the idle state and define our start pos
    testingtest.State currentState = testingtest.State.IDLE;
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


        currentState = testingtest.State.State1;
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


            if (AutoReject) {
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
                    AutoReject = false;
                }
            }


            switch (currentState) {
////////////////////////////////////////////   Move 1    //////////////////////////////////////////
                case State1:
                    if (position==1){
                        TrajectorySequence State1SeqPos1 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-42, 44.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-32, 41, Math.toRadians(315)))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.State2;
                            drive.followTrajectorySequenceAsync(State1SeqPos1);
                        }
                        break;

                    } else if (position ==2){
                        TrajectorySequence State1SeqPos2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                //.lineToLinearHeading(new Pose2d(-40, 44.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40, 36.75, Math.toRadians(270)))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.State2;
                            drive.followTrajectorySequenceAsync(State1SeqPos2);
                        }
                        break;

                    } else {
                        TrajectorySequence State1SeqPos3 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-44.75, 44.75, Math.toRadians(270)))
                                //.splineTo(new Vector2d(-44.75, 44.75), Math.toRadians(270))
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.State2;
                            drive.followTrajectorySequenceAsync(State1SeqPos3);
                        }
                        break;
                    }
////////////////////////////////////////////   Move 2    //////////////////////////////////////////
                case State2:
                    if (position==3) {
                        TrajectorySequence State2Seq = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .waitSeconds(0.5)
                                .addTemporalMarker(0, () -> intake.horiPower(1.0))
                                .addTemporalMarker(0, () -> claw.lowerClaw(true))
                                .addTemporalMarker(0, () -> lowerClawOpen = true)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                .lineToConstantHeading(new Vector2d(-44.75, 47))
                                .build(); //-21, -60*/
                        if (!drive.isBusy()) {
                            currentState = State.State14;
                            drive.followTrajectorySequenceAsync(State2Seq);
                        }


                        break;
                    } else {
                        currentState = State.State14;
                        break;
                    }

                case State14:
                    if (position==3) {
                        TrajectorySequence State14Seq3 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-50, 47, Math.toRadians(225)))//y47
                                .build(); //-21, -60*/
                        if (!drive.isBusy()) {
                            currentState = State.State3;
                            drive.followTrajectorySequenceAsync(State14Seq3);
                        }


                        break;
                    } else{
                        TrajectorySequence State14Seq = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .waitSeconds(0.5)
                                .addTemporalMarker(0, () -> intake.horiPower(1.0))
                                .addTemporalMarker(0, () -> claw.lowerClaw(true))
                                .addTemporalMarker(0, () -> lowerClawOpen = true)
                                .addTemporalMarker(0.5, () -> intake.horiPower(0.0))
                                //.lineToConstantHeading(new Vector2d(-44.75, 47))
                                .lineToLinearHeading(new Pose2d(-50, 47, Math.toRadians(225)))//y47
                                .build(); //-21, -60*/
                        if (!drive.isBusy()) {
                            currentState = State.State3;
                            drive.followTrajectorySequenceAsync(State14Seq);
                        }


                        break;
                    }
////////////////////////////////////////////   Move 3    //////////////////////////////////////////
                case State3:
                    TrajectorySequence State3Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .addTemporalMarker(0, () -> {
                                intake.horiPower(-0.50);
                                intake.verticalPower(1.0);
                                intake.setIntakeRoller(1.0);
                                intake.setIntakebelt(1.0);
                                AutoReject = true;
                            })
                            .lineToLinearHeading(new Pose2d(-60, 40, Math.toRadians(195)))//was y 39

                            .build(); //-21, -60
                    if (!drive.isBusy()) {
                        currentState = State.State4;
                        drive.followTrajectorySequenceAsync(State3Seq);
                    }
                    break;

                    //move 4/////////////////////
                case State4:
                    TrajectorySequence State4Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .waitSeconds(0.5)
                            .setReversed(true)
                            .addTemporalMarker(0, () -> {
                                intake.horiPower(1.0);
                                intake.setIntakeRoller(-1);
                            })
                            .addTemporalMarker(0, () -> {
                                intake.verticalPower(-0.4);
                            })


                            .lineToLinearHeading(new Pose2d(-35, 59, Math.toRadians(180)))
                            //.splineTo(new Vector2d(-35, 59), Math.toRadians(0))

                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State5;
                        drive.followTrajectorySequenceAsync(State4Seq);
                    }
                    break;
////////////////////////////////////////////   Move 4    //////////////////////////////////////////
                case State5:
                    ////////here's where an if statement would be
                        TrajectorySequence State5Seq = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(12, 59))
                                .addTemporalMarker(0, () -> {
                                    claw.upperClaw(false);
                                    upperClawOpen = false;
                                    claw.lowerClaw(false);
                                    lowerClawOpen = false;
                                })
                                .addTemporalMarker(0, () -> {
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                    AutoReject=false;

                                })
                                .build(); //-21, -60
                        if (!drive.isBusy()) {
                            currentState = State.State6;
                            drive.followTrajectorySequenceAsync(State5Seq);
                        }
                        break;
////////////////////////////////////////////   Move 5    //////////////////////////////////////////
                case State6:
                    if (position==1){
                        TrajectorySequence State6SeqPos1 = drive.trajectorySequenceBuilder(poseEstimate)

                                .addTemporalMarker(0, () -> {
                                    lift.setTargetHeight(1000, 0);
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(52.5, 42, Math.toRadians(180)))
                                //.splineTo(new Vector2d(52.5, 33), Math.toRadians(0))


                                .build();

                        if (!drive.isBusy()) {
                            currentState = State.State7;
                            drive.followTrajectorySequenceAsync(State6SeqPos1);
                        }
                        break;

                    } else if (position ==2){
                        TrajectorySequence State6SeqPos2 = drive.trajectorySequenceBuilder(poseEstimate)

                                .addTemporalMarker(0, () -> {
                                    lift.setTargetHeight(1000, 0);
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(52.5, 38, Math.toRadians(180)))
                                //.splineTo(new Vector2d(52.5, 33), Math.toRadians(0))


                                .build();

                        if (!drive.isBusy()) {
                            currentState = State.State7;
                            drive.followTrajectorySequenceAsync(State6SeqPos2);
                        }
                        break;

                    } else {
                        TrajectorySequence State6SeqPos3 = drive.trajectorySequenceBuilder(poseEstimate)

                                .addTemporalMarker(0, () -> {
                                    lift.setTargetHeight(1000, 0);
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                })
                                .addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(52.5, 33, Math.toRadians(180)))
                                //.splineTo(new Vector2d(52.5, 33), Math.toRadians(0))


                                .build();

                        if (!drive.isBusy()) {
                            currentState = State.State7;
                            drive.followTrajectorySequenceAsync(State6SeqPos3);
                        }
                        break;
                    }
////////////////////////////////////////////   Move 6    //////////////////////////////////////////
                case State7:
                    TrajectorySequence State7Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(180)))
                            .waitSeconds(0.4)
                            .addTemporalMarker(0, () -> {
                                claw.upperClaw(true);
                                upperClawOpen = true;
                                claw.lowerClaw(true);
                                lowerClawOpen = true;
                            })
                            .addTemporalMarker(0.2, () -> claw.setRotateAngle("intake", 0.0))
                            .addTemporalMarker(0.2, () -> lift.setTargetHeight(0, 0))
                            .addTemporalMarker(0.4, () -> {
                                claw.setDeliverArm("intake");
                                armIn = true;
                            })



                            .build();

                    if (!drive.isBusy()) {
                        currentState = State.State8;
                        drive.followTrajectorySequenceAsync(State7Seq);
                    }
                    break;
////////////////////////////////////////////   Move 7    //////////////////////////////////////////
                case State8:
                    TrajectorySequence State8Seq = drive.trajectorySequenceBuilder(poseEstimate)

                            .setReversed(false)
                            .lineToConstantHeading(new Vector2d(-35, 60))



                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State9;
                        drive.followTrajectorySequenceAsync(State8Seq);
                    }
                    break;
////////////////////////////////////////////   Move 8    //////////////////////////////////////////
                case State9:
                    TrajectorySequence State9Seq = drive.trajectorySequenceBuilder(poseEstimate)

                            .setReversed(false)
                            .lineToLinearHeading(new Pose2d(-60, 40, Math.toRadians(195)))
                            .addTemporalMarker(0, () -> {
                                intake.horiPower(-1.0);
                                intake.verticalPower(1.0);
                                intake.setIntakeRoller(1.0);
                                intake.setIntakebelt(1.0);
                                AutoReject = true;
                            })


                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State10;

                        drive.followTrajectorySequenceAsync(State9Seq);
                    }
                    break;
                case State10:
                    TrajectorySequence State10Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(-35, 59, Math.toRadians(180)))
                            //.splineTo(new Vector2d(-35, 59), Math.toRadians(0))
                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State11;
                        drive.followTrajectorySequenceAsync(State10Seq);
                    }
                    break;

                case State11:
                    TrajectorySequence State11Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .addTemporalMarker(0, () -> {
                                claw.upperClaw(false);
                                upperClawOpen = false;
                                claw.lowerClaw(false);
                                lowerClawOpen = false;
                            })
                            .addTemporalMarker(0, () -> {
                                intake.horiPower(0.0);
                                intake.verticalPower(0.0);
                                intake.setIntakeRoller(0.0);
                                intake.setIntakebelt(0.0);
                                AutoReject=false;

                            })
                            .setReversed(true)
                            .lineToConstantHeading(new Vector2d(12, 59))

                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State12;
                        drive.followTrajectorySequenceAsync(State11Seq);
                    }
                    break;
                case State12:
                    TrajectorySequence State12Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(52.5, 35.5, Math.toRadians(180)))
                            .addTemporalMarker(0, () -> {
                                lift.setTargetHeight(1000, 0);
                                claw.setDeliverArm("delivery");
                                armIn = false;
                                intake.horiPower(0.0);
                                intake.verticalPower(0.0);
                                intake.setIntakeRoller(0.0);
                                intake.setIntakebelt(0.0);
                            })
                            .addTemporalMarker(0.8, () -> {claw.setRotateAngle("horizontal", 0.0);})

                            //.splineTo(new Vector2d(52.5, 35.5), Math.toRadians(180))

                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State13;
                        drive.followTrajectorySequenceAsync(State12Seq);
                    }
                    break;
                case State13:
                    TrajectorySequence State13Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .addTemporalMarker(0, () -> {
                                claw.upperClaw(true);
                                upperClawOpen = true;
                                claw.lowerClaw(true);
                                lowerClawOpen = true;
                            })
                            .addTemporalMarker(0.2, () -> claw.setRotateAngle("intake", 0.0))
                            .addTemporalMarker(0.2, () -> lift.setTargetHeight(0, 0))
                            .addTemporalMarker(0.4, () -> {
                                claw.setDeliverArm("intake");
                                armIn = true;
                            })
                            .waitSeconds(0.2)
                            .setReversed(false)
                            .lineToLinearHeading(new Pose2d(46, 40, Math.toRadians(180))) //-2
                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        drive.followTrajectorySequenceAsync(State13Seq);
                    }
                    break;
                case IDLE:

                    break;

            }
        }
        ////////////////////////////////////////////   END    //////////////////////////////////////////




    }

}