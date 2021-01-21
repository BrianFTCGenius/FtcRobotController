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

import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;


@TeleOp(name="MecanumTeleopv2", group="Mecanum")
@Disabled

public class MecanumTeleopV2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum robot = new HardwareMecanum();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    //public static PIDFCoefficients Kcoefficients = new PIDFCoefficients(5.2, 5.4 , 0, 60);

    @Override
    public void runOpMode() {
        boolean flyIdle = false;
        int turnTicks = 100;
        boolean latch = false;
        boolean readyGrip = true;
        double flySet = 0;
        boolean triggered = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.tractor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.kicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.tractor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.kicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.tractor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            ///////////////
            // GAMEPAD 1 //
            ///////////////

            // DRIVING
            drive();

            //AUTOADJUST SHOOTER POSITION
            // Forwards  =   + - + -
            // Backwards =   - + - +
            // Pivot right = + + + +
            // Pivot left = - - - -

            // KICKER
            // around 825 ticks = 1 turn
            if (gamepad1.left_bumper) {
                runtime.reset();
                robot.kicker.setPower(1);
                robot.kicker.setTargetPosition(820);
                while (robot.kicker.getCurrentPosition() < 818 && runtime.milliseconds() < 3000) {
                    robot.kicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad1.back || gamepad1.right_bumper) {
                runtime.reset();
                robot.kicker.setPower(1);
                robot.kicker.setTargetPosition(2480);
                while (robot.kicker.getCurrentPosition() < 2478 && runtime.milliseconds() < 3000) {
                    robot.kicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad2.left_trigger > .25) {
                robot.kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.kicker.setPower(-gamepad2.left_trigger * .4);
            } else {
                robot.kicker.setTargetPosition(0);
                robot.kicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.kicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.kicker.setPower(gamepad2.right_trigger);
            }


            int frontLeftCurPos = robot.frontLeft.getCurrentPosition();
            int frontRightCurPos = robot.frontRight.getCurrentPosition();
            int rearLeftCurPos = robot.rearLeft.getCurrentPosition();
            int rearRightCurPos = robot.rearRight.getCurrentPosition();


            if (gamepad1.x) {
                robot.frontLeft.setPower(.7);
                robot.frontRight.setPower(.7);
                robot.rearRight.setPower(.7);
                robot.rearLeft.setPower(.7);

                robot.frontLeft.setTargetPosition(frontLeftCurPos + turnTicks);
                robot.frontRight.setTargetPosition(frontRightCurPos + turnTicks);
                robot.rearLeft.setTargetPosition(rearLeftCurPos + turnTicks);
                robot.rearRight.setTargetPosition(rearRightCurPos + turnTicks);
                while (robot.frontLeft.getCurrentPosition() < turnTicks - 50) {
                    robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else {
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1.b) {

                robot.frontLeft.setPower(-.7);
                robot.frontRight.setPower(-.7);
                robot.rearRight.setPower(-.7);
                robot.rearLeft.setPower(-.7);

                robot.frontLeft.setTargetPosition(frontLeftCurPos - turnTicks);
                robot.frontRight.setTargetPosition(frontRightCurPos - turnTicks);
                robot.rearLeft.setTargetPosition(rearLeftCurPos - turnTicks);
                robot.rearRight.setTargetPosition(rearRightCurPos - turnTicks);

                while (robot.frontLeft.getCurrentPosition() > -turnTicks + 50) {
                    robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else {
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            ///////////////
            // GAMEPAD 2 //
            ///////////////

            // INTAKE
            if (gamepad2.right_bumper) {
                robot.feeder.setPower(1);
            } else if (gamepad2.left_bumper) {
                robot.feeder.setPower(-1);
            } else {
                robot.feeder.setPower(0);
            }

            // WOBBLE GOAL // Tractor limits are 0 (all the way up) and -2300 (all the way down)
            if (gamepad2.dpad_up) {
                if (robot.tractor.getCurrentPosition() > -5) {
                    robot.tractor.setPower(-.1);
                } else {
                    robot.tractor.setPower(0.55);
                }
            } else if (gamepad2.dpad_down) {
                readyGrip = true;
                robot.tractor.setTargetPosition(-2200);
                robot.tractor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tractor.setPower(-0.5);
            } else {
                robot.tractor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.tractor.setPower(0);
            }

            // CLAW
            if (gamepad2.dpad_left) {
                robot.claw.setPower(.25);
                latch = false;
            } else if (gamepad2.dpad_right || latch) {
                runtime.reset();
                int tractorPos = robot.tractor.getTargetPosition() - 300;
                robot.tractor.setTargetPosition(tractorPos);
                while (runtime.milliseconds() < 500 && readyGrip) {
                    if (runtime.milliseconds() < 400) {
                        robot.tractor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.tractor.setPower(-.25);
                    } else {
                        robot.tractor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.tractor.setPower(0);
                        readyGrip = false;
                    }
                }
                robot.claw.setPower(-.25);
                latch = true;
            } else {
                robot.claw.setPower(0);
            }

            // SHOOTER
            if (gamepad2.x) {
                flySet = .73;
                //flySet = 2280;
                flyIdle = false;
            } else if (gamepad2.y) {
                flySet = 0.7;
                // flySet = 2050;
                flyIdle = false;
            } else if (gamepad2.b) {
                flySet = .60;
                //flySet = 1330;
                flyIdle = false;
            } else if (gamepad2.a) {
                flySet = 0;
                flyIdle = true;
            }
            robot.flywheel.setPower(flySet);

            // TELEMETRY
            telemetry.addLine("left")
                    .addData("forward", robot.frontLeft.getCurrentPosition())
                    .addData("rear", robot.rearLeft.getCurrentPosition());
            telemetry.addLine("right")
                    .addData("forward", robot.frontRight.getCurrentPosition())
                    .addData("rear", robot.rearRight.getCurrentPosition());
            telemetry.addData("tractor position", robot.tractor.getCurrentPosition());
            telemetry.addLine("Flywheel")
                    .addData("Flywheel Speed", flySet)
                    .addData("FlywheelIdle", flyIdle);
            telemetry.addLine("Velocity)")
                    .addData("shooter", robot.flywheel.getVelocity());
            telemetry.addLine("Trigger")
                    .addData(" Pos ", robot.kicker.getCurrentPosition())
                    .addData(" Tar ", robot.kicker.getTargetPosition());
            telemetry.addLine("Tractor")
                    .addData(" Pos ", robot.tractor.getCurrentPosition())
                    .addData(" Power ", robot.tractor.getPower())
                    .addData(" Tar ", robot.tractor.getTargetPosition());
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }

    public void drive() {
        double[] direction = new double[3];
        double slowMode = 1;//(gamepad1.right_bumper ? .7 : 1);
        direction[0] = -gamepad1.right_stick_x * slowMode * .9;
        direction[1] = -gamepad1.left_stick_x * slowMode;
        direction[2] = -gamepad1.left_stick_y * slowMode;

        double[] driveTrainPower = new double[]{
                (direction[0] + direction[1] - direction[2]),
                (direction[0] + direction[1] + direction[2]),
                (direction[0] - direction[1] - direction[2]),
                (direction[0] - direction[1] + direction[2]),
        };

        robot.frontLeft.setPower(driveTrainPower[0]);
        robot.frontRight.setPower(driveTrainPower[1]);
        robot.rearLeft.setPower(driveTrainPower[2]);
        robot.rearRight.setPower(driveTrainPower[3]);
    }
}