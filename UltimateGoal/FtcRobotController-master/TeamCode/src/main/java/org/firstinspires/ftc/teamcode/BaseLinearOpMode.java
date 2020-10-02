/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public abstract class BaseLinearOpMode extends LinearOpMode
{

    public static double SKYRATIO=1.15;
    public static double NORMRATIO=1.0;
    public static double GAIN=-.05;

    public static int MAX_DISPLACEMENT=1000000000;
    public String vuforiaKey="ARfiIjT/////AAABmVENNuLySUY6o8kMgGpUW4M+vdodovxEqYYyxuXyX0XfusMtaXFekAop3fh0TjC6eZSpCBLPhLwu1P988oIoJjPIawXeIDYF0ZvLW0C6zrMtUsEmiU7Od+jEtTT3kBFmFDGdPF6V5A1uXnGxFLjVULo/Ra2S6ZK8+VBbp5ax0gmnoCOQi4d0ofe7kwwoiC6C4lhkMKMIVrNqeXsA8eXU1qLzXuvSLJhdpWc/k2POTOBszl5tF39ovq0P1oA7vABtj4J+QmSjXGcuXyyn2BIwy0HnoM10M8xAJMMTOJBQFS45wdWERH4cOhZdhy9GYly0S4HeB0Cf8bgSSnrw1uRSFmLHdReMo4TP/sjb5vRj9rAh";
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;


    public BNO055IMU imu;


    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    public int INCH = 38;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void initVariables() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_Rear_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_Front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_Rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_Front_drive");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        initImu();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void test(){
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        double MAX_SPEED = 1.0;
        holonomic(Speed, Turn, Strafe, MAX_SPEED );
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED){

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = Math.abs(Speed) + Math.abs(Turn) + Math.abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        leftFrontDrive.setPower(Math.scalb((Speed + Turn - Strafe),(int)Math.round(Magnitude)));




        if (leftRearDrive != null) {
            leftRearDrive.setPower(Math.scalb((Speed + Turn + Strafe),(int)Math.round(Magnitude)));
        }
        rightFrontDrive.setPower(Math.scalb((Speed - Turn + Strafe),(int)Math.round(Magnitude)));
        if (rightRearDrive != null) {
            rightRearDrive.setPower(Math.scalb((Speed - Turn - Strafe),(int)Math.round(Magnitude)));
        }
    }



    // Left and Right Strafes are reversed

    public void strafeMode()
    {
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftRearDrive.setPower(v3);
        rightRearDrive.setPower(v4);
    }



    public void move(double speed, double direction, double time, boolean useGyro)
    {
        ElapsedTime runtime = new ElapsedTime();
        double r = speed;
        double robotAngle = direction + Math.PI/4;
        double rightX = 0;
        final double v1 = r * Math.cos(robotAngle) * -1;
        final double v2 = r * Math.sin(robotAngle)* -1;
        final double v3 = r * Math.sin(robotAngle)* -1;
        final double v4 = r * Math.cos(robotAngle)* -1;

        runtime.reset();

        leftFrontDrive.setPower(v1);

        rightFrontDrive.setPower(v2);
        leftRearDrive.setPower(v3);
        rightRearDrive.setPower(v4);
        while(runtime.milliseconds()<time && opModeIsActive())
        {

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);


    }

    public void move(double speed, double direction,  boolean useGyro)
    {
        ElapsedTime runtime = new ElapsedTime();
        double r = speed;
        double robotAngle = direction + Math.PI/4;
        double rightX = 0;
        final double v1 = r * Math.cos(robotAngle) * -1;
        final double v2 = r * Math.sin(robotAngle)* -1;
        final double v3 = r * Math.sin(robotAngle)* -1;
        final double v4 = r * Math.cos(robotAngle)* -1;

        runtime.reset();

        leftFrontDrive.setPower(v1);

        rightFrontDrive.setPower(v2);
        leftRearDrive.setPower(v3);
        rightRearDrive.setPower(v4);



    }

    public void stopAll()
    {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public void rotate(double speed, double time)
    {
        ElapsedTime runtime = new ElapsedTime();
        double r = speed;

        double rightX = 0;
        final double v1 = r;
        final double v2 = r * -1;
        final double v3 = r ;
        final double v4 = r * -1;

        runtime.reset();

        while(runtime.milliseconds()<time)
        {
            leftFrontDrive.setPower(v1);
            rightFrontDrive.setPower(v2);
            leftRearDrive.setPower(v3);
            rightRearDrive.setPower(v4);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }












    public void moveGyro(double speed, double direction, double time, boolean useGyro)
    {
       /* telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();*/

        ElapsedTime runtime = new ElapsedTime();
        double r = speed;
        double robotAngle = direction + Math.PI/4;
        double rightX = 0;
        final double v1 = r * Math.cos(robotAngle) * -1;
        final double v2 = r * Math.sin(robotAngle)* -1;
        final double v3 = r * Math.sin(robotAngle)* -1;
        final double v4 = r * Math.cos(robotAngle)* -1;

        runtime.reset();


        while(runtime.milliseconds()<time && opModeIsActive())
        {

            correction = checkDirection();
            leftFrontDrive.setPower(v1-correction);

            rightFrontDrive.setPower(v2+correction);
            leftRearDrive.setPower(v3-correction);
            rightRearDrive.setPower(v4+correction);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);


    }

    public void initImu()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

       /* telemetry.addData("Z angle:",angles.firstAngle);
        telemetry.addData("Y angle:",angles.secondAngle);
        telemetry.addData("X angle:",angles.thirdAngle);
        telemetry.update();*/
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = GAIN;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftFrontDrive.setPower(leftPower);
        leftRearDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightRearDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void moveDistance(double power,double angle, int distance)
    {

        //The motors move in reverse, so you have to flip the distance

        int totalDist =distance;

        double r = power;
        double robotAngle = angle + Math.PI/4;
        double rightX = 0;
        final double v1 = r * Math.cos(robotAngle) * -1;
        final double v2 = r * Math.sin(robotAngle)* -1;
        final double v3 = r * Math.sin(robotAngle)* -1;
        final double v4 = r * Math.cos(robotAngle)* -1;


        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setTargetPosition(totalDist);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRearDrive.setTargetPosition(totalDist);


        while(Math.abs(rightRearDrive.getCurrentPosition())<totalDist && !this.isStopRequested()&& opModeIsActive())
        {
            correction = checkDirection();
            leftFrontDrive.setPower(v1-correction);

            rightFrontDrive.setPower(v2+correction);
            leftRearDrive.setPower(v3-correction);
            rightRearDrive.setPower(v4+correction);
            telemetry.addData("Encoder",rightRearDrive.getCurrentPosition());
            telemetry.update();
        }

        leftRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        telemetry.addData("Encoder",rightRearDrive.getCurrentPosition());
        telemetry.update();
    }


   /* public void moveGyro(double speed, double direction, CheckCondition condition)
    {
       /* telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        double r = speed;
        double robotAngle = direction + Math.PI/4;
        double rightX = 0;
        final double v1 = r * Math.cos(robotAngle) * -1;
        final double v2 = r * Math.sin(robotAngle)* -1;
        final double v3 = r * Math.sin(robotAngle)* -1;
        final double v4 = r * Math.cos(robotAngle)* -1;

        runtime.reset();


        while(opModeIsActive() && condition.eval())
        {


            telemetry.update();
            correction = checkDirection();
            leftFrontDrive.setPower(v1-correction);

            rightFrontDrive.setPower(v2+correction);
            leftRearDrive.setPower(v3-correction);
            rightRearDrive.setPower(v4+correction);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);


    }*/


}
