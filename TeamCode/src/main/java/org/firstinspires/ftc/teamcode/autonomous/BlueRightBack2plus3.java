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
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="BlueRightBack2plus3", group="FSMAuto", preselectTeleOp="teleOp")
public class
BlueRightBack2plus3 extends LinearOpMode {
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
        State15,
        IDLE
    }
    // Default to the idle state and define our start pos
    BlueRightBack2plus3.State currentState = BlueRightBack2plus3.State.IDLE;
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


        currentState = BlueRightBack2plus3.State.State1;
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
                if (distance.getDistance(DistanceUnit.MM) < 30) {
                    intake.verticalPower(0.6);
                }


                if (!lowerClawOpen && !upperClawOpen && armIn) {
                    intake.horiPower(1.0);
                    intake.verticalPower(-0.7);
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
                                .lineToLinearHeading(new Pose2d(-40, 46, Math.toRadians(0)))//y44.75
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
                                .lineToLinearHeading(new Pose2d(-43, 38, Math.toRadians(290))) //was36.75 and 270
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
                                .addTemporalMarker(0, () -> intake.horiPower(0.6))
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
                                intake.horiPower(-0.60);
                                intake.verticalPower(1.0);
                                intake.setIntakeRoller(1.0);
                                intake.setIntakebelt(1.0);
                                AutoReject = true;
                            })
                            .lineToLinearHeading(new Pose2d(-60, 40, Math.toRadians(195)))//was y 39

                            .build(); //-21, -60
                    if (!drive.isBusy()) {
                        currentState = State.State15;
                        drive.followTrajectorySequenceAsync(State3Seq);
                    }
                    break;

                case State15:
                    TrajectorySequence State15Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(false)
                            .addTemporalMarker(0, () -> {
                                intake.horiPower(0.90);
                                intake.setIntakeRoller(-1);
                            })
                            .addTemporalMarker(0, () -> {
                                intake.verticalPower(-0.8);
                            })
                            .lineToLinearHeading(new Pose2d(-58, 40, Math.toRadians(195)))

                            //was y 39

                            .build(); //-21, -60
                    if (!drive.isBusy()) {
                        currentState = State.State4;
                        drive.followTrajectorySequenceAsync(State15Seq);
                    }
                    break;

                    //move 4/////////////////////
                case State4:
                    TrajectorySequence State4Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            //.waitSeconds(0.5)
                            .setReversed(true)
                            /*.addTemporalMarker(0, () -> {
                                intake.horiPower(1.0);
                                intake.setIntakeRoller(-1);
                            })
                            .addTemporalMarker(0, () -> {
                                intake.verticalPower(-0.4);
                            })*/


                            .lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)))
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
                                .lineToConstantHeading(new Vector2d(12, 60))
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
                                    lift.setTargetHeight(800, 0);
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                })
                                /*.addTemporalMarker(0.8, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })*/

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(53, 44.3, Math.toRadians(180)))//52.5
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
                                    lift.setTargetHeight(700, 0);
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
                                .lineToLinearHeading(new Pose2d(53.5, 40, Math.toRadians(180)))//52.5
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
                                    lift.setTargetHeight(725, 0);
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
                                .lineToLinearHeading(new Pose2d(53, 33, Math.toRadians(180)))//52.5
                                //.splineTo(new Vector2d(52.5, 33), Math.toRadians(0))


                                .build();

                        if (!drive.isBusy()) {
                            currentState = State.State7;
                            drive.followTrajectorySequenceAsync(State6SeqPos3);
                        }
                        break;
                    }
/////////////////////////////////////////////////////////////////////////// ////////////////changing from here
                case State7:
                    if (!drive.isBusy()) {
                        currentState = State.State8;
                        TrajectorySequence PixelPickup2 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(false)
                                .addTemporalMarker(0, () -> {
                                    claw.upperClaw(true);
                                    upperClawOpen = true;
                                })
                                .addTemporalMarker(0.2, () -> {
                                    claw.lowerClaw(true);
                                    lowerClawOpen = true;
                                })
                                .addTemporalMarker(0.8, () -> claw.setRotateAngle("intake", 0.0))
                                .addTemporalMarker(1.2, () -> lift.setTargetHeight(0, 0))
                                .addTemporalMarker(1.2, () -> {
                                    claw.setDeliverArm("intake");
                                    armIn = true;
                                })
                                .addTemporalMarker(2, () -> intake.horiPower(-1.0))
                                .addTemporalMarker(2, () -> intake.verticalPower(1.0))
                                .addTemporalMarker(2, () -> intake.setIntakeRoller(1.0))
                                .addTemporalMarker(2, () -> intake.setIntakebelt(1.0))
                                .addTemporalMarker(2, () -> AutoReject=true)
                                .waitSeconds(0.4) //was 0.6
                                .splineTo(new Vector2d(24, 61), Math.toRadians(-180))
                                .lineToLinearHeading(new Pose2d(-40, 62, Math.toRadians(-180)))
                                //.lineToConstantHeading(new Vector2d(-40, 60)) //.lineToConstantHeading(new Vector2d(-60, -13))
                                .lineToLinearHeading(new Pose2d(-59.5, 41, Math.toRadians(-165))) //-62.5, 40
                                //.lineToConstantHeading(new Vector2d(-62.5, -41),
                                //      SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                //    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
                        drive.followTrajectorySequenceAsync(PixelPickup2);
                    }
                    break;
////////////////////////////////////////////   Move 7    //////////////////////////////////////////
                case State8:
                    if (!drive.isBusy()) {
                        currentState = State.State9;
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
                                .splineTo(new Vector2d(-40, 60), Math.toRadians(-0))
                                .lineToConstantHeading(new Vector2d(12, 60))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    lift.setTargetHeight(1200, 0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    claw.setDeliverArm("delivery");
                                    armIn = false;
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                                    claw.setRotateAngle("horizontal", 0.0);
                                })
                                .splineTo(new Vector2d(53, 44), Math.toRadians(-0))
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
////////////////////////////////////////////   Move 8    //////////////////////////////////////////
                case State9:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
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
                                .lineToLinearHeading(new Pose2d(44, 50, Math.toRadians(-180)))
                                .build();
                        drive.followTrajectorySequenceAsync(BackboardPixel4);
                    }
                    break;


                    /////////////////////////////////////////////////////////////////////////////////endddddddd
                case State10:
                   /* TrajectorySequence State10Seq = drive.trajectorySequenceBuilder(poseEstimate)
                            .setReversed(true)
                            .waitSeconds(0.1)
                            .addTemporalMarker(0.1, () -> {
                                intake.horiPower(0.60);
                                intake.setIntakeRoller(-1);
                            })
                            .lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)))
                            //.splineTo(new Vector2d(-35, 59), Math.toRadians(0))
                            .build();
                    if (!drive.isBusy()) {
                        if (position==1){
                            currentState = State.State12;
                            drive.followTrajectorySequenceAsync(State10Seq);
                        } else {
                            currentState = State.State11;
                            drive.followTrajectorySequenceAsync(State10Seq);
                        }
                    }*/
                    break;

                case State11:
                   /* TrajectorySequence State11Seq = drive.trajectorySequenceBuilder(poseEstimate)

                            .addTemporalMarker(0, () -> {
                                intake.horiPower(0.0);
                                intake.verticalPower(0.0);
                                intake.setIntakeRoller(0.0);
                                intake.setIntakebelt(0.0);


                            })
                            .setReversed(true)
                            .lineToConstantHeading(new Vector2d(12, 60))

                            .build();
                    if (!drive.isBusy()) {
                        currentState = State.State12;
                        drive.followTrajectorySequenceAsync(State11Seq);
                    }*/
                    break;
                case State12:
                    /*if (position == 1){
                        TrajectorySequence State12Seq1 = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(180)))//was 35.5
                                .addTemporalMarker(0, () -> {
                                    AutoReject=false;
                                    intake.horiPower(0.0);
                                    intake.verticalPower(0.0);
                                    intake.setIntakeRoller(0.0);
                                    intake.setIntakebelt(0.0);
                                })

                                //.splineTo(new Vector2d(52.5, 35.5), Math.toRadians(180))

                                .build();
                        if (!drive.isBusy()) {
                            currentState = State.State13;
                            drive.followTrajectorySequenceAsync(State12Seq1);
                        }
                        break;
                    } else {
                        TrajectorySequence State12Seq = drive.trajectorySequenceBuilder(poseEstimate)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(52.5, 42, Math.toRadians(180)))//was 35.5
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

                                //.splineTo(new Vector2d(52.5, 35.5), Math.toRadians(180))

                                .build();
                        if (!drive.isBusy()) {
                            currentState = State.State13;
                            drive.followTrajectorySequenceAsync(State12Seq);
                        }
                        break;
                    }*/
                    break;
                case State13:
                    /*if (position == 1){
                        currentState = State.IDLE;
                        break;
                    } else {
                        TrajectorySequence State13Seq = drive.trajectorySequenceBuilder(poseEstimate)
                                .addTemporalMarker(0, () -> {
                                    claw.upperClaw(true);
                                    upperClawOpen = true;
                                    claw.lowerClaw(true);
                                    lowerClawOpen = true;
                                })
                                .addTemporalMarker(0.2, () -> claw.setRotateAngle("intake", 0.0))
                                .addTemporalMarker(0.3, () -> lift.setTargetHeight(0, 0))
                                .addTemporalMarker(0.3, () -> {
                                    claw.setDeliverArm("intake");
                                    armIn = true;
                                })
                                .waitSeconds(0.2)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(46, 46, Math.toRadians(180))) //was y-40
                                .build();
                        if (!drive.isBusy()) {
                            currentState = State.IDLE;
                            drive.followTrajectorySequenceAsync(State13Seq);
                        }
                        break;
                    }*/
                    break;
                case IDLE:

                    break;

            }
        }
        ////////////////////////////////////////////   END    //////////////////////////////////////////




    }

}