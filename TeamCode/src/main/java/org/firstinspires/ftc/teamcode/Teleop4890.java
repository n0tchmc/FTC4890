package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;

@TeleOp(name = "Teleop4890")
public class Teleop4890 extends LinearOpMode {
    Robot robot = new Robot();

    Gamepad.RumbleEffect armDownAlert;
    Gamepad.RumbleEffect endGameWarn;
    Gamepad.RumbleEffect roundEndWarn;
    Gamepad.RumbleEffect coneInAlert;


    double slowToggle;
    boolean armUp;
    boolean coneIn;

    @Override
    public void runOpMode() throws InterruptedException {

        armDownAlert = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.8, 700)  //  Rumble both motors 80% for 700 mSec
                .build();
        coneInAlert = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.3, 700)  //  Rumble both motors 80% for 700 mSec
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

        robot.RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.WM40.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            telemetry.addData("Match Time (s)", getRuntime());
            telemetry.addData("FL Count", robot.frontLeft.getCurrentPosition());
            telemetry.addData("FR Count", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL Count", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR Count", robot.backRight.getCurrentPosition());
            telemetry.addData("Left Arm Count", robot.RLM.getCurrentPosition());
            telemetry.addData("Right Arm Count", robot.WM40.getCurrentPosition());
            //telemetry.addData("Left Touch", robot.leftTouch.getState());
            //telemetry.addData("Right Touch", robot.rightTouch.getState());
            //telemetry.addData("Arm Up", armUp);
            telemetry.addData("Red", robot.clawColor.red());
            telemetry.addData("Green", robot.clawColor.green());
            telemetry.addData("Blue", robot.clawColor.blue());
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

            if ((gamepad2.left_stick_y < 0) && gamepad2.right_stick_y < 0) {
                robot.RLM.setPower(Math.abs(gamepad2.left_stick_y));
                robot.WM40.setPower(Math.abs(gamepad2.left_stick_y));
            }
            else if ((gamepad2.left_stick_y > 0) && (gamepad2.right_stick_y > 0) && (robot.leftTouch.getState()) && (robot.rightTouch.getState())) { //make sure this condition is first
                robot.RLM.setPower(-Math.abs(gamepad2.left_stick_y));
                robot.WM40.setPower(-Math.abs(gamepad2.right_stick_y));
            }
            else if (gamepad2.right_stick_y > 0 && (robot.rightTouch.getState())) {
                robot.WM40.setPower(-Math.abs(gamepad2.right_stick_y));
            }
            else if (gamepad2.left_stick_y > 0 && (robot.leftTouch.getState())) {
                robot.RLM.setPower(-Math.abs(gamepad2.left_stick_y));
            }
            else {
                robot.RLM.setPower(0);
                robot.WM40.setPower(0);
            }

            if (gamepad2.b) {
                robot.claw.setPosition(0.4);
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


            //endGame 5 second warning, round end 3 second warning
            if (getRuntime() >= 87 && getRuntime() <= 89) {
                gamepad1.runRumbleEffect(endGameWarn);
                gamepad2.runRumbleEffect(endGameWarn);
            } else if (getRuntime() >= 117 && getRuntime() <= 119) {
                gamepad1.runRumbleEffect(roundEndWarn);
                gamepad2.runRumbleEffect(roundEndWarn);
            }

            //alerts if arm is down completely
            if ((robot.leftTouch.getState() == false || robot.rightTouch.getState() == false) && armUp == true) {
                armUp = false;
                gamepad2.runRumbleEffect(armDownAlert);
            }
            else if (robot.leftTouch.getState() == true && robot.rightTouch.getState() == true) {
                armUp = true;
            }

            //alerts if a cone is in the claw
            if ((robot.clawColor.red() >= 350 || robot.clawColor.blue() >= 350) && coneIn == false) {
                coneIn = true;
                gamepad1.runRumbleEffect(coneInAlert);
                gamepad2.runRumbleEffect(coneInAlert);
            }
            else if ((robot.clawColor.red() < 350 && robot.clawColor.blue() < 350)) {
                coneIn = false;
            }

            idle();
        }
    }
}
