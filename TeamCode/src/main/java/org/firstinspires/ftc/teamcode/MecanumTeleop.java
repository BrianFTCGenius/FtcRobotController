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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumTeleop", group="Pushbot")
@Disabled
public class MecanumTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();   // Use a Pushbot's hardware
        public void brake() {
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.tractor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.rearLeft.setPower(0);
            robot.rearRight.setPower(0);
        }

    @Override
    public void runOpMode() {
        double yaw;
        double surge;
        double sway;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tractor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.tractor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double slowMode = (gamepad1.right_bumper ? .4 : 1);
            yaw = gamepad1.right_stick_x * slowMode * .75
            ;
            surge = -gamepad1.left_stick_y * slowMode;
            sway = -gamepad1.left_stick_x * slowMode;

            if (gamepad1.b) {

                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.rearLeft.setPower(0);
                robot.rearRight.setPower(0);

                robot.frontLeft.setTargetPosition(-2669);
                robot.frontRight.setTargetPosition(2598);
                robot.rearLeft.setTargetPosition(-2598);
                robot.rearRight.setTargetPosition(2669);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.8); //change to 0.8
                robot.frontRight.setPower(0.8);
                robot.rearLeft.setPower(0.8);
                robot.rearRight.setPower(0.8);
                while ((robot.frontRight.getCurrentPosition() != 2598) && (robot.frontRight.getCurrentPosition() != -2598) && (robot.frontRight.getCurrentPosition() != 2699)) {
                    sleep(10);
                }

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }


            ///////////////
            // GAMEPAD 1 //
            ///////////////

            // hard reset of encoder ticks; EMERGENCY USE ONLY
            if (gamepad1.x) {
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // feeder
            if (gamepad2.a) {
                robot.feeder.setPower(1);
            } else if (gamepad2.b) {
                robot.feeder.setPower(-1);
            } else {
                robot.feeder.setPower(0);
            }

            // driving
            double[] driveTrainPower = new double[]{
                    (-yaw + sway - surge),
                    (-yaw + sway + surge),
                    (-yaw - sway - surge),
                    (-yaw - sway + surge),
            };

            if (gamepad1.left_bumper) {
                brake();
            } else {
                robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontLeft.setPower(driveTrainPower[0]);
                robot.frontRight.setPower(driveTrainPower[1]);
                robot.rearLeft.setPower(driveTrainPower[2]);
                robot.rearRight.setPower(driveTrainPower[3]);
                telemetry.addData("Brake", false);
            }

            ///////////////
            // GAMEPAD 2 //
            ///////////////

            // kicker

            robot.kicker.setPower(gamepad1.left_trigger * 0.7);
//            robot.trigger.setPower(-gamepad1.right_trigger * 0.24);

            // shooter
            if (gamepad2.y) {
                robot.flywheel.setPower(.9);
            } else if (gamepad2.x) {
                robot.flywheel.setPower(0.8);
            }
            else if (gamepad2.dpad_left) {
                robot.flywheel.setPower(0.7);
            }
            else if (gamepad2.dpad_right) {
                robot.flywheel.setPower(0.6);
                }
            else {
                robot.flywheel.setPower(gamepad2.right_trigger);
            }

            telemetry.addLine("left")
                    .addData("forward", robot.frontLeft.getCurrentPosition())
                    .addData("rear", robot.rearLeft.getCurrentPosition());
            telemetry.addLine("right")
                    .addData("forward", robot.frontRight.getCurrentPosition())
                    .addData("rear", robot.rearRight.getCurrentPosition());
            telemetry.addLine("tractor")
                    .addData("tractor position", robot.tractor.getCurrentPosition());
            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(25);

            //wobble goal
            if (gamepad2.dpad_up) {
                robot.tractor.setPower(0.4);
            } else if (gamepad2.dpad_down) {
                robot.tractor.setPower(-0.4);

            }
            else {
                robot.tractor.setPower(0);
            }

            //claw
            if(gamepad2.right_bumper) {
                robot.claw.setPower(1);
            }
            else if (gamepad2.left_bumper) {
                robot.claw.setPower(-1);
            }
            else{
                robot.claw.setPower(0);
            }

        }
    }
}
