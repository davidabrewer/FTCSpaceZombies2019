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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


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


@Autonomous(name="Grab2SkyStoneBlue", group="RedAutonomous")
public class SpaceEngineersOpModeLinearBlue6 extends BaseLinearOpMode {


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        initVariables();
        waitForStart();


/*
        //move forward to bass

        moveGyro(1,0,1100,true);

        //grab base

        moveVerticalArm(0.5,2000,true);
        moveHorizontalArm(0.5,750,true);
        moveVerticalArm(0.5,1500,false);
        moveHorizontalArm(0.5,400,false);

        //drive back

        moveGyro(.6,Math.PI,1500,true);

        //raise arm then strafe left

        moveVerticalArm(0.5,1300,true);
        moveGyro(1,-Math.PI/2,1000,false);
        moveVerticalArm(0.5,900,false);
        moveGyro(1,-Math.PI/2,600,false);
*/
        //move forward to edge of STONES
        openClaw();

        moveGyro(1,0,875,true);
        moveGyro(.5, -Math.PI / 2, new CheckCondition() {
            @Override
            public boolean eval() {
                return checkAlpha(rightSensor,60);
            }
        });
        stopAll();

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        //strafe right to find sky stone
        moveGyro(.5, -Math.PI / 2, new CheckCondition() {
            @Override
            public boolean eval() {
                return checkRedGreater(1.25);
            }
        });

        telemetry.addData("Found skystone",rightSensor.getNormalizedColors().toColor());
        telemetry.update();
      //  sleep(2000);


        moveGyro(.5, -Math.PI / 2, new CheckCondition() {
            @Override
            public boolean eval() {
                return checkBlueGreater(1.25);
            }
        });
        telemetry.addData("Found skystone far edges",rightSensor.getNormalizedColors().toColor());
        telemetry.update();

        stopAll();

        //open claw

        //extend arm
        moveHorizontalArm(0.5,1700,true);
        //close claw
        closeClaw();
        sleep(500);
        //retract arm
        moveHorizontalArm(0.5,1750,false);
        moveGyro( 1,Math.PI, 250, true);
        final int displacement = Math.abs(rightFrontDrive.getCurrentPosition());
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

/*
        while(Math.abs(rightFrontDrive.getCurrentPosition())<displacement)
        {moveGyro( 1,Math.PI/2,200,true  );
        }
*/
        moveGyro(.5, Math.PI / 2, new CheckCondition() {
            @Override
            public boolean eval() {
                return Math.abs(rightFrontDrive.getCurrentPosition())<Math.abs(displacement);
            }
        });
        //strafe left
        moveGyro( .75,Math.PI/2,1250,true  );

        //release block
        openClaw();
        moveGyro(1,0,600,true);
        moveGyro(1,Math.PI,300, true);

if(displacement < MAX_DISPLACEMENT) {
    moveGyro(0.75, -Math.PI / 2, 1300, true);
    moveGyro(.5, -Math.PI / 2, new CheckCondition() {
        @Override
        public boolean eval() {
            return Math.abs(rightFrontDrive.getCurrentPosition()) < Math.abs(displacement);
        }
    });
//should be in front of next stone
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    moveGyro(.5, -Math.PI / 2, new CheckCondition() {
        @Override
        public boolean eval() {
            return checkRedGreater(1.25);
        }
    });

    telemetry.addData("Found skystone",rightSensor.getNormalizedColors().toColor());
    telemetry.update();
    moveGyro(.5, -Math.PI / 2, new CheckCondition() {
        @Override
        public boolean eval() {
            return checkBlueGreater(1.25);
        }
    });
    telemetry.addData("Found skystone far edges",rightSensor.getNormalizedColors().toColor());
    telemetry.update();
    moveHorizontalArm(0.5,1700,true);
    //close claw
    closeClaw();
    sleep(500);
    //retract arm
    moveHorizontalArm(0.5,1750,false);
    moveGyro( 1,Math.PI, 250, true);
    final int displacement2 = Math.abs(rightFrontDrive.getCurrentPosition());
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    moveGyro(.5, Math.PI / 2, new CheckCondition() {
        @Override
        public boolean eval() {
            return Math.abs(rightFrontDrive.getCurrentPosition())<(Math.abs(displacement)+Math.abs(displacement2));
        }
    });

    //strafe left
    moveGyro( .75,Math.PI/2,1250,true  );

    //release block
    openClaw();
    moveGyro(1,0,600,true);
    moveGyro(1,Math.PI,300, true);
    moveGyro(0.75,-Math.PI/2,750,true);
}
else
{
    moveGyro(.75,-Math.PI/2,750,true);
}
    }
}
