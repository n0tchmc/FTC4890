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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class RobotAuto {

    /* Public OpMode members. */
    public DcMotor arm;
    public DcMotor pivot;
    public DcMotor carousel;
    public CRServo claw;
    public CRServo clawGrab;
    public DigitalChannel armSensor;
    public DigitalChannel clawSensor;
    public ColorSensor clawColor;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public RobotAuto() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Devices
        claw = hwMap.crservo.get("claw");
        clawGrab = hwMap.crservo.get("clawGrab");
        arm = hwMap.dcMotor.get("arm");
        pivot = hwMap.dcMotor.get("pivot");
        carousel = hwMap.dcMotor.get("carousel");
        armSensor = hwMap.digitalChannel.get("ArmSensor");
        clawSensor = hwMap.digitalChannel.get("ClawSensor");
        clawColor = hwMap.colorSensor.get("clawColor");


        armSensor.setMode(DigitalChannel.Mode.INPUT);
        clawSensor.setMode(DigitalChannel.Mode.INPUT);

        // Setting motor directions
        carousel.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        clawGrab.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double powerFunction(double x) {
        if (x < 0) {
            return -(x*x);
        }

        return x*x;
//        if (x >= 0) {
//            return (((Math.pow(Math.E, Math.abs(x)))/ Math.E) - (1/Math.E)) * (Math.E/(Math.E-1));
//        } else if (x < 0) {
//            return -((((Math.pow(Math.E, Math.abs(x)))/ Math.E) - (1/Math.E)) * (Math.E/(Math.E-1)));
//        }
//        return (((Math.pow(Math.E, Math.abs(x)))/ Math.E) - (1/Math.E)) * (Math.E/(Math.E-1));
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

