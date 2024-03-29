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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    /* Public OpMode members. */
    //Revolutionary Lift Mechanism
    public DcMotor WM40;
    public DcMotor RLM;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public Servo claw;
    public DigitalChannel leftTouch;
    public DigitalChannel rightTouch;
    public ColorSensor clawColor;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Devices
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        RLM = hwMap.get(DcMotor.class, "RLM");      //left side (looking behind the robot)
        WM40 = hwMap.get(DcMotor.class, "WM40");    //right side
        claw = hwMap.get(Servo.class, "claw");
        leftTouch = hwMap.digitalChannel.get("leftTouch");
        rightTouch = hwMap.digitalChannel.get( "rightTouch");
        clawColor = hwMap.colorSensor.get("clawColor");


        // Setting motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        RLM.setDirection(DcMotor.Direction.REVERSE);
        WM40.setDirection(DcMotor.Direction.REVERSE);
        leftTouch.setMode(DigitalChannel.Mode.INPUT);
        rightTouch.setMode(DigitalChannel.Mode.INPUT);
    }
}

