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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="WithMechanoWheels", group="Linear Opmode")
//@Disabled
public class MeccanoWheels extends LinearOpMode {

    // Declare OpMode variables.
    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotor LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor;

    //Power assigned to the motor
    private double LeftFrontMotorPower, LeftBackMotorPower, RightFrontMotorPower, RightBackMotorPower;

    //Joystick inputs
    private double leftjoystickXvalue, leftjoystickYvalue;
    private boolean leftBumper = Boolean.FALSE;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Starting the program");
        telemetry.update();

        /*
         * Initialize the hardware variables. Note that the strings used in deviceName for the
         * hardwareMap.get() calls must correspond to the names assigned during the
         * robot configuration step using the FTC Robot Controller app on the phone.
         */

        LeftFrontMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LeftBackMotor   = hardwareMap.get(DcMotor.class, "LB Motor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        RightBackMotor  = hardwareMap.get(DcMotor.class, "RB Motor");

        /*
         * Initialize the left motors to move front
         */
        LeftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftBackMotor.setDirection(DcMotor.Direction.FORWARD);

        /*
         * As the right motors rotate in reverse direction, set the directions
         * as reverse so that all motors rotate in the same directions, and we
         * can control the motors with the joystick.
         */

        RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        RightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            driveRobot();
            grabObjects();
            depositObjects();
        }
    }

    private void driveRobot() {

        /*
         * Get the Joystick Left control value for X axis and Y axis;
         */
        leftjoystickXvalue = gamepad1.left_stick_x;
        leftjoystickYvalue = gamepad1.left_stick_y;
        leftBumper = gamepad1.left_bumper;

        /*
         * Calculate the Forward and Backward movements.
         * For straight, X = 0,
         *  all wheels should have +Y for forward
         *  all wheels should have -Y for backward.
         * For Clockwise Arc Both Y and X are postive:
         *  LeftFront and LeftBack are positive, and the RightFront and RightBack are negative
         * For Counterclockwise Arc, Y is postive but X is negative:
         *  RightFront and RightBack are positive, and the LeftFront and LeftBack are negative
         * For Right shift, x is positive:
         *  LeftFront and RightBack should have a positive value, and the RightFront and
         *  LeftBack are negative
         * For Left Shift, x is positive:
         *  RightFront and LeftBack should have positive value and the LeftFront and RightBack
         *  negative
         */

        if (leftBumper == Boolean.FALSE) {
            /*
             * Left bumper is not used. Move the robot as usual.
             */
            LeftFrontMotorPower = leftjoystickYvalue + leftjoystickXvalue;
            RightFrontMotorPower = leftjoystickYvalue - leftjoystickXvalue;
            LeftBackMotorPower = leftjoystickYvalue + leftjoystickXvalue;
            RightBackMotorPower = leftjoystickYvalue - leftjoystickXvalue;
        } else {
            //Left Bumper is used. Only X axis is used. Drift the robot.
            LeftFrontMotorPower = leftjoystickXvalue;
            RightFrontMotorPower = -leftjoystickXvalue;
            LeftBackMotorPower = -leftjoystickXvalue;
            RightBackMotorPower = leftjoystickXvalue;
        }

        //Limit the Motor Power Range between -1.0 and 1.0. Use RangClip.
        LeftFrontMotorPower  = Range.clip(LeftFrontMotorPower, -1.0, 1.0) ;
        LeftBackMotorPower  = Range.clip(LeftBackMotorPower, -1.0, 1.0) ;
        RightFrontMotorPower = Range.clip(RightFrontMotorPower, -1.0, 1.0) ;
        RightBackMotorPower = Range.clip(RightBackMotorPower, -1.0, 1.0) ;

        // Send calculated power to wheels
        LeftFrontMotor.setPower(LeftFrontMotorPower);
        LeftBackMotor.setPower(LeftBackMotorPower);
        RightFrontMotor.setPower(RightFrontMotorPower);
        RightBackMotor.setPower(RightBackMotorPower);

        // Show the elapsed game time and wheel powers.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors Power", "Left Front(%.4f)," +
                        " Left Back(%.4f), Right Front(%.4f), Right Back (%.4f)",
                LeftFrontMotorPower, LeftBackMotorPower, RightFrontMotorPower,
                RightBackMotorPower);
        telemetry.update();
    }

    private void grabObjects() {
        return;
    }

    private void depositObjects() {
        return;
    }
}
