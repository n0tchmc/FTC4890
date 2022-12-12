package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

@TeleOp(name = "EncoderTeleop4890")
public class EncoderTeleop4890 extends LinearOpMode {
    Robot robot = new Robot();

    Gamepad.RumbleEffect customRumbleEffect;
    Gamepad.RumbleEffect endGameWarn;
    Gamepad.RumbleEffect roundEndWarn;

    int robotCycle = 0;
    int encoderValueLeft = 0;
    int encoderValueRight = 0;
    double slowToggle;

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
        roundEndWarn = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 200)  //  Rumble both motor 0% for 200 mSec
                .addStep(1.0, 1.0, 1000)  //  Rumble left motor 100% for 1000 mSec
                .build();

        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.WM40.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RLM.setTargetPosition(0);
        robot.WM40.setTargetPosition(0);
        robot.RLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WM40.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Robot Cycle", robotCycle);
            telemetry.addData("Match Time (s)", getRuntime());
            telemetry.addData("Arm Motor Power", robot.RLM.getPower());
            telemetry.addData("Left Stick Y value", gamepad2.left_stick_y);
            telemetry.addData("Right Stick Y value", gamepad2.right_stick_y);
            telemetry.addData("Left Encoder Count", robot.RLM.getCurrentPosition());
            telemetry.addData("Right Encoder Count", robot.WM40.getCurrentPosition());
            telemetry.update();

            //controller 1 functions
            double FrontLeftVal = gamepad1.left_stick_y - gamepad1.left_stick_x + (-gamepad1.right_stick_x * 0.7);
            double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - (-gamepad1.right_stick_x * 0.7);
            double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + (-gamepad1.right_stick_x * 0.7);
            double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - (-gamepad1.right_stick_x * 0.7);


            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
            }
            robot.frontLeft.setPower(-FrontLeftVal * 1 * slowToggle);
            robot.frontRight.setPower(-FrontRightVal * 1 * slowToggle);
            robot.backLeft.setPower(-BackLeftVal * 1 * slowToggle);
            robot.backRight.setPower(-BackRightVal * 1 * slowToggle);

            //player 2 functions

            /*
            if ((gamepad2.left_stick_y < 0) && gamepad2.right_stick_y < 0) {
                robot.RLM.setPower(Math.abs(gamepad2.left_stick_y));
                robot.WM40.setPower(Math.abs(gamepad2.left_stick_y));
            }
            else if ((gamepad2.left_stick_y > 0) && (gamepad2.right_stick_y > 0)) { //make sure this condition is first
                robot.RLM.setPower(-Math.abs(gamepad2.left_stick_y));
                robot.WM40.setPower(-Math.abs(gamepad2.right_stick_y));
            }
            else if (gamepad2.right_stick_y > 0) {
                robot.WM40.setPower(-Math.abs(gamepad2.right_stick_y));
            }
            else if (gamepad2.left_stick_y > 0) {
                robot.RLM.setPower(-Math.abs(gamepad2.left_stick_y));
            }
            else {
                robot.RLM.setPower(0);
                robot.WM40.setPower(0);
            }*/

            if(gamepad2.dpad_left) { //low
                encoderValueLeft = 2240;
                encoderValueRight = 3975;
            }
            else if (gamepad2.dpad_up) { //med
                encoderValueLeft = 3205;
                encoderValueRight = 5995;
            }
            else if (gamepad2.dpad_right) { // high
                encoderValueLeft = 4600;
                encoderValueRight = 8050;
            }
            else if (gamepad2.dpad_down) { // ground
                encoderValueLeft = 0;
                encoderValueRight = 0;
            }
            else if (gamepad2.left_bumper) { //1st in stack
                encoderValueLeft = 1285;
                encoderValueRight = 2230;
            }
            else if (gamepad2.right_bumper) { //200ish ticks down per each cone
                encoderValueLeft -= 200;
                encoderValueRight -= 200;
            }

            //left side encoders
            if (robot.RLM.getCurrentPosition() < encoderValueLeft) {
                robot.RLM.setTargetPosition(encoderValueLeft);
                robot.RLM.setPower(1);
            }
            else if (robot.RLM.getCurrentPosition() > encoderValueLeft) {
                robot.RLM.setTargetPosition(encoderValueLeft);
                robot.RLM.setPower(-1);
            }
            else {
                robot.RLM.setPower(0);
            }
            //right side encoders
            if (robot.WM40.getCurrentPosition() < encoderValueRight) {
                robot.WM40.setTargetPosition(encoderValueRight);
                robot.WM40.setPower(1);
            }
            else if (robot.WM40.getCurrentPosition() > encoderValueRight) {
                robot.WM40.setTargetPosition(encoderValueRight);
                robot.WM40.setPower(-1);
            }
            else {
                robot.WM40.setPower(0);
            }

            if (gamepad2.b) {
                robot.claw.setPosition(0.30);
            }
            else if (gamepad2.a) {
                robot.claw.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                slowToggle = 0.25;
            }
            else {
                slowToggle = 1;
            }


            //endGame 5 second warning
            if (getRuntime() >= 87 && getRuntime() <= 89) {
                gamepad1.runRumbleEffect(endGameWarn);
                gamepad2.runRumbleEffect(endGameWarn);
            } else if (getRuntime() >= 117 && getRuntime() <= 119) {
                gamepad1.runRumbleEffect(roundEndWarn);
                gamepad2.runRumbleEffect(roundEndWarn);
            }

            idle();
        }
    }
}
