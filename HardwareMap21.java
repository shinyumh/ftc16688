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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMap21
{

    // motors
    public DcMotor FleftDrive  = null;
    public DcMotor BleftDrive  = null;
    public DcMotor FrightDrive = null;
    public DcMotor BrightDrive = null;
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;
    public DcMotor carousel = null;

    // servos
    public CRServo pushey = null;
    public CRServo pulley = null;
    public CRServo liftey = null;
    public CRServo outtake1 = null;
    public CRServo outtake2 = null;
    public CRServo outtake3 = null;
    public CRServo outtake4 = null;
    public CRServo outtake5 = null;
    public CRServo outtake6 = null;
    public Servo closey = null;

    // define numerical variables
    public static final double CLOSED_SERVO   =  0 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap21(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // reference to hardware map
        hwMap = ahwMap;

        // define and initialize motors & servos
        FleftDrive  = hwMap.get(DcMotor.class, "FleftDrive");
        BleftDrive  = hwMap.get(DcMotor.class, "BleftDrive");
        FrightDrive = hwMap.get(DcMotor.class, "FrightDrive");
        BrightDrive = hwMap.get(DcMotor.class, "BrightDrive");
        leftIntake = hwMap.get(DcMotor.class, "leftIntake");
        rightIntake = hwMap.get(DcMotor.class, "rightIntake");
        carousel = hwMap.get(DcMotor.class, "carousel");
        pushey = hwMap.get(CRServo.class, "pushey");
        pulley = hwMap.get(CRServo.class, "pulley");
        liftey = hwMap.get(CRServo.class, "liftey");
        outtake1 = hwMap.get(CRServo.class, "outtake1");
        outtake2 = hwMap.get(CRServo.class, "outtake2");
        outtake3 = hwMap.get(CRServo.class, "outtake3");
        outtake4 = hwMap.get(CRServo.class, "outtake4");
        outtake5 = hwMap.get(CRServo.class, "outtake5");
        outtake6 = hwMap.get(CRServo.class, "outtake6");
        closey = hwMap.get(Servo.class, "closey");

        // set power for motors
        FleftDrive.setPower(0);
        BleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BrightDrive.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        carousel.setPower(0);

        //set power for cr servos
        pulley.setPower(0);
        pushey.setPower(0);
        liftey.setPower(0);
        outtake1.setPower(0);
        outtake2.setPower(0);
        outtake3.setPower(0);
        outtake4.setPower(0);
        outtake5.setPower(0);
        outtake6.setPower(0);

        // set mode (use encoder!)
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // set direction
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);
        // unsure about carousel direction ^^ test

        pulley.setDirection(CRServo.Direction.FORWARD);
        pushey.setDirection(CRServo.Direction.FORWARD);
        liftey.setDirection(CRServo.Direction.FORWARD);

        outtake1.setDirection(CRServo.Direction.REVERSE);
        outtake2.setDirection(CRServo.Direction.REVERSE);
        outtake3.setDirection(CRServo.Direction.REVERSE);
        outtake4.setDirection(CRServo.Direction.FORWARD);
        outtake5.setDirection(CRServo.Direction.FORWARD);
        outtake6.setDirection(CRServo.Direction.FORWARD);

        // initialize all servos
        closey.setPosition(CLOSED_SERVO);
    }
}