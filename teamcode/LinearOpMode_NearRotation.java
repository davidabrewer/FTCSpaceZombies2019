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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="NearSide Autonomous Program using Rotation", group="Linear Opmode")
@Disabled
public class LinearOpMode_NearRotation extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private DcMotor spoolMotor =null;
    private Servo clawServo = null;
    static final double     FORWARD_SPEED = 1.0;  // Full power = 17 inches/sec
    static final double     TURN_SPEED    = 0.5;
    static final double     ARM_SPEED = 0.4;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //TODO: Use Vuphoria to find where you are

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  //This motor is installed opposite what you expect
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);  //This rotates the arm forward
        clawServo.setDirection(Servo.Direction.REVERSE);  //Positive will close the claw. Negative will open
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        runtime.reset();

        int rotCount =0;
        // run until the end of the match (driver presses STOP)
        //Start copy
        while (opModeIsActive() && runtime.milliseconds() < 1500) {


            // Send calculated power to wheels
            leftDrive.setPower(FORWARD_SPEED);
            rightDrive.setPower(FORWARD_SPEED);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        runtime.reset();
        //End copy
                while (opModeIsActive() && runtime.milliseconds() < 850) {
            leftDrive.setPower(FORWARD_SPEED * -1);
            rightDrive.setPower(FORWARD_SPEED);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Start Copy 2
        while (opModeIsActive() && runtime.milliseconds() < 4300) {


            // Send calculated power to wheels
            leftDrive.setPower(FORWARD_SPEED);
            rightDrive.setPower(FORWARD_SPEED);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        runtime.reset();
        //End copy
        while (opModeIsActive() && runtime.milliseconds() < 1375) {
            leftDrive.setPower(FORWARD_SPEED );
            rightDrive.setPower(FORWARD_SPEED* -1)
;        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        runtime.reset();
        // End Copy 2
        // Start Copy 3
        while (opModeIsActive() && runtime.milliseconds() < 2852) {


            // Send calculated power to wheels
            leftDrive.setPower(FORWARD_SPEED);
            rightDrive.setPower(FORWARD_SPEED);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        runtime.reset();
        // End Copy 3


        while(opModeIsActive() && runtime.milliseconds() < 1000)
        {
            armMotor.setPower(ARM_SPEED);
            telemetry.addData("Status", "Arm Move");
        }
        armMotor.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 100)
        {
            clawServo.setPosition(.5);
            sleep(50);
            clawServo.setPosition(0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 100)
        {
            clawServo.setPosition(1);

        }
        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds() < 800)
        {
            armMotor.setPower(ARM_SPEED *-1);
            telemetry.addData("Status", "Arm Move");
        }
        armMotor.setPower(0);
        // /start copy
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 150) {
            leftDrive.setPower(FORWARD_SPEED);
            rightDrive.setPower(FORWARD_SPEED * -1);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 4529) {


        // Send calculated power to wheels
        leftDrive.setPower(-1*FORWARD_SPEED);
        rightDrive.setPower(-1*FORWARD_SPEED);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    }   //end copy
}