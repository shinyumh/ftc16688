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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="AutoMoveAway", group="Pushbot")
public class AutoStrafe3TilesRed extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMap21         robot   = new HardwareMap21();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.FleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // send telemetry message to show current wheels position
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.FrightDrive.getCurrentPosition(),
                robot.FleftDrive.getCurrentPosition(),
                robot.BleftDrive.getCurrentPosition(),
                robot.BrightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // strafe right three tiles
        encoderDrive(DRIVE_SPEED,  5,  -5, -5,5,9);
        telemetry.addData("Path", "Strafe");
        telemetry.update();


        //stop
        robot.FrightDrive.setPower(0);
        robot.FleftDrive.setPower(0);
        robot.BrightDrive.setPower(0);
        robot.BleftDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    // define function encoderDrive - umm might need to change this lol
    public void encoderDrive(double speed, double FleftInches, double FrightInches, double BleftInches,double BrightInches, double timeoutS) {

        // define variables
        int newFleftTarget;
        int newBleftTarget;
        int newFrightTarget;
        int newBrightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFleftTarget = robot.FleftDrive.getCurrentPosition() + (int) (FleftInches * COUNTS_PER_INCH);
            newBleftTarget = robot.BleftDrive.getCurrentPosition() + (int) (BleftInches * COUNTS_PER_INCH);
            newFrightTarget = robot.FrightDrive.getCurrentPosition() + (int) (FrightInches * COUNTS_PER_INCH);
            newBrightTarget = robot.BrightDrive.getCurrentPosition() + (int) (BrightInches * COUNTS_PER_INCH);

            robot.FleftDrive.setTargetPosition(newFleftTarget);
            robot.BleftDrive.setTargetPosition(newBleftTarget);
            robot.FrightDrive.setTargetPosition(newFrightTarget);
            robot.BrightDrive.setTargetPosition(newBrightTarget);

            // Turn On RUN_TO_POSITION
            robot.FleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FleftDrive.setPower(Math.abs(speed));
            robot.BrightDrive.setPower(Math.abs(speed));
            robot.FrightDrive.setPower(Math.abs(speed));
            robot.BleftDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FleftDrive.isBusy() && robot.FrightDrive.isBusy() && robot.BrightDrive.isBusy() && robot.BleftDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d",
                        newFleftTarget,
                        newBleftTarget,
                        newFrightTarget,
                        newBrightTarget
                        );
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.FleftDrive.getCurrentPosition(),
                        robot.FrightDrive.getCurrentPosition(),
                        robot.BleftDrive.getCurrentPosition(),
                        robot.BrightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FleftDrive.setPower(0);
            robot.BleftDrive.setPower(0);
            robot.FrightDrive.setPower(0);
            robot.BrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
