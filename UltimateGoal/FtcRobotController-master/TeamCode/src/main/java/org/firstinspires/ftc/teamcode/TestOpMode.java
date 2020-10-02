package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "FirstTest",group = "Linear OpMode")
public class TestOpMode extends LinearOpMode {
    public DcMotor leftFrontDrive = null;
    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode()
    {
        leftFrontDrive = hardwareMap.get(DcMotor.class,"testmotor");
        while(runtime.milliseconds()<10000)
        {
            leftFrontDrive.setPower(.2);
        }
        leftFrontDrive.setPower(0);

    }
}
