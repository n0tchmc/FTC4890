package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {
    Robot robot = new Robot();

    Gamepad.RumbleEffect customRumbleEffect;
    Gamepad.RumbleEffect endGameWarn;
    Gamepad.RumbleEffect sixSecRemain;

    boolean rumble = true;
    int robotCycle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.8, 0.8, 500)  //  Rumble both motors 80% for 500 mSec
                .build();
        endGameWarn = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Rumble both motor 0% for 300 mSec
                .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec
                .build();
        sixSecRemain = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble both motor 0% for 200 mSec
                .addStep(1.0, 1.0, 1000)  //  Rumble left motor 100% for 1000 mSec
                .build();

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + robot.runtime.toString());
        telemetry.update();

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red", robot.clawColor.red());
            telemetry.addData("Green", robot.clawColor.green());
            telemetry.addData("Blue", robot.clawColor.blue());
            telemetry.addData("Robot Cycle", robotCycle);
            telemetry.addData("Match Time (s)", getRuntime());
            telemetry.update();

            //controller 1 functions
            double FrontLeftVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
            double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
            double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }
            robot.frontLeft.setPower(FrontLeftVal * 0.63);
            robot.frontRight.setPower(FrontRightVal * 0.63);
            robot.backLeft.setPower(BackLeftVal * 0.63);
            robot.backRight.setPower(BackRightVal * 0.63);


            //clockwise carousel
            if (gamepad1.x) {
                robot.carousel.setPower(-0.75);
            }
            //counter-clockwise carousel
            else if (gamepad1.b) {
                robot.carousel.setPower(0.75);
            } else {
                robot.carousel.setPower(0);
            }

            //Pivot up forward
            if (gamepad2.right_trigger != 0) {
                robot.pivot.setPower(gamepad2.right_trigger * 0.60);
            }
            //Pivot up backward
            else if (gamepad2.left_trigger != 0) {
                robot.pivot.setPower(-gamepad2.left_trigger * 0.60);
            } else {
                robot.pivot.setPower(0);
            }

            //Arm mover
            if (gamepad2.left_stick_y != 0) {
                robot.arm.setPower(-gamepad2.left_stick_y);
            } else {
                robot.arm.setPower(0);
            }

            //Claw mover
            if (gamepad2.right_stick_y != 0) {
                robot.claw.setPower(-gamepad2.right_stick_y);
            } else {
                robot.claw.setPower(0);
            }

            //claw intake
            if (gamepad2.a && (robot.clawColor.green() < 1500) && (robot.clawColor.red() < 1500)) {
                robot.clawGrab.setPower(1);
            }

            //claw outtake
            else if (gamepad2.y) {
                robot.clawGrab.setPower(-1);
                rumble = true;
                robotCycle = 0;
            }

            //claw off (first is manual, second is auto)
            else if (gamepad2.b) {
                robot.clawGrab.setPower(0);
            } else if ((robot.clawColor.green() >= 1500 && robot.clawColor.red() >= 1500 && robot.clawGrab.getPower() != -1)) {
                if (robotCycle > 20) {
                    if (rumble == true) {
                        gamepad2.runRumbleEffect(customRumbleEffect);
                        rumble = false;
                    }
                    robot.clawGrab.setPower(-0.1);
                    robot.clawGrab.setPower(0);
                    robotCycle = 0;
                } else {
                    robotCycle++;
                }
            }

            // arm stopper
            if (!robot.armSensor.getState()) {
                robot.arm.setPower(0);
            }

            if (!robot.clawSensor.getState()) {
                robot.claw.setPower(0);
            }

            //endGame 5 second warning
            if (getRuntime() >= 84 && getRuntime() <= 86) {
                gamepad1.runRumbleEffect(endGameWarn);
                gamepad2.runRumbleEffect(endGameWarn);
            } else if (getRuntime() >= 113 && getRuntime() <= 115) {
                gamepad1.runRumbleEffect(sixSecRemain);
                gamepad2.runRumbleEffect(sixSecRemain);
            }

            idle();
        }
    }
}
