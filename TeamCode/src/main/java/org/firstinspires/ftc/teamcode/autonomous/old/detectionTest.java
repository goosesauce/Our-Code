package org.firstinspires.ftc.teamcode.autonomous.old;

import static org.firstinspires.ftc.teamcode.functions.detection.getPosition;
import static org.firstinspires.ftc.teamcode.functions.detection.setColour;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.functions.hardwareInit;

@Autonomous(name = "DetectionTest", group = "Robot")
@Disabled
public class detectionTest extends LinearOpMode {
    int position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        new hardwareInit(hardwareMap);
        setColour("Red"); //Blue or Red
        while (!isStarted() && !isStopRequested()) {
            position = getPosition();
            telemetry.addData("Position" , position);
            telemetry.update();
        }
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Final Position" , position);
            telemetry.update();
            sleep(5000);
        }
    }
}

