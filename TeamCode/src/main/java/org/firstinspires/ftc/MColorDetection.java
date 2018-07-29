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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "MColorDetection", group = "Concept")
//@Disabled
public class MColorDetection extends LinearOpMode {

    static final double INCREMENT = 0.01;
    static final int CYCLE_MS = 50;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;


    Servo servo;
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    double position = (MAX_POS - MIN_POS) / 2;
    boolean rampUp = true;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        leftMotor = hardwareMap.dcMotor.get("Left Motor");
        rightMotor = hardwareMap.dcMotor.get("Right Motor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            while(sensorColor.red() != 22) {
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    position += INCREMENT;
                    if (position >= MAX_POS) {
                        position = MAX_POS;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                } else {
                    // Keep stepping down until we hit the min value.
                    position -= INCREMENT;
                    if (position <= MIN_POS) {
                        position = MIN_POS;
                        rampUp = !rampUp;  // Switch ramp direction
                    }
                }

                // Display the current value
                telemetry.addData("Servo Position", "%5.2f", position);

                // Set the servo to the new position and pause;
                servo.setPosition(position);
                idle();}
            if (servo.getPosition() < 0.5) {
                rightMotor.setPower(-.5);
                leftMotor.setPower(.5);
                sleep(500);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                System.exit(0);
            }


            if (servo.getPosition() > .5) {
                rightMotor.setPower(.5);
                leftMotor.setPower(-.5);
                sleep(500);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                System.exit(0);

            }
            if (servo.getPosition() == .5) {
                rightMotor.setPower(-.5);
                rightMotor.setPower(-.5);
                sleep(500);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                System.exit(0);
            }
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }


            });
        }

    }
}