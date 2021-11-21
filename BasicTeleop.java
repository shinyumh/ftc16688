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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="BasicTeleop", group="TeleOp")

public class BasicTeleop extends LinearOpMode {

    // declare opmode members
    HardwareMap21 robot = new HardwareMap21();
    private ElapsedTime runtime = new ElapsedTime();
    double speedAdjust = 5;
    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    @Override
    public void runOpMode() {

        // update telemetry
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.update();

        // set run mode
        robot.BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set direction of wheels
        robot.BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.FleftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.FrightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.leftIntake.setDirection(DcMotor.Direction.REVERSE);
        robot.rightIntake.setDirection(DcMotor.Direction.FORWARD);

        // set servo positions
        robot.pulley.setPosition(0);
        robot.pushey.setPosition(0);
        robot.closey.setPosition(0);

        // wait for driver to press start
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* POV Mode uses left stick to go forward, and right stick to turn.
            // uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            robot.BleftDrive.setPower(leftPower);
            robot.BrightDrive.setPower(rightPower);
            robot.FleftDrive.setPower(leftPower);
            robot.FrightDrive.setPower(rightPower);
            */

            /* working strafing, wanna check without right_stick_x
            robot.BleftDrive.setPower((gamepad1.left_stick_y +  gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.BrightDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FleftDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FrightDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
            */

            robot.BleftDrive.setPower((gamepad1.left_stick_y +  gamepad1.left_stick_x) * (-speedAdjust / 10));
            robot.BrightDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * (-speedAdjust / 10));
            robot.FleftDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * (-speedAdjust / 10));
            robot.FrightDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * (-speedAdjust / 10));


            // A button - activate the intakes
            if (gamepad1.a) {
                robot.leftIntake.setPower(2);
                robot.rightIntake.setPower(-2);
            } else {
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            }

            // B button - pseudo outtake
            if (gamepad1.b) {
                robot.leftIntake.setPower(-2);
                robot.rightIntake.setPower(2);
            } else {
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            }

            // X button - move servo for clasp
            if (gamepad1.x) {
                robot.pushey.setPosition(-1);
            } else {
                robot.pushey.setPosition(0);
            }

            if (gamepad1.right_bumper) {
                robot.closey.setPosition(-1);
            } else {
                robot.pushey.setPosition(0);
            }

            // Y button - move pulley
            if (gamepad1.y) {
                robot.pulley.setPosition(-1);
            } else {
                robot.pulley.setPosition(0);
            }

            //(optional) carousel - B button

            // show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
