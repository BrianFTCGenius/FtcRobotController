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
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="MecanumTeleopv2Odo", group="Mecanum")
@Disabled

public class MecanumTeleopODOMETRY extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanumV3 robot = new HardwareMecanumV3();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime motorTime = new ElapsedTime();
    private float absX = 0;
    private float absY = 0;
    private int[] lastTick = {0, 0, 0, 0};
    private Orientation lastAngles = new Orientation();
    double globalAngle = 0;
    private static double ang = 45;
    private static double tolerence = 0;
    private static double speed = 0.5;


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
        robot.trigger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.trigger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.tractor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.tractor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.trigger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.tractor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // PID HISTORY
        //robot.flywheel.setVelocityPIDFCoefficients(1.343, .0743, 0.1, 13.43);
        //mathematically computed original values: 1.707, 0.1707, 0, 17.07
        //Mr. Allsworth's baller status idea #1: 2, 0.1707, 0, 17.07
        //Mr. Allsworth's LeBron James idea #2: 1.707, 0.1800, 0, 17.07
        //John Wang's poopoo idea #1: 1.727, 0.1800, 0, 17.07
        //John Wang's "idea" #2: 1.707, 0.1790, 0, 17.07
        //Massimo's dank idea #1: 1.475, 0.182, 0, 17.07
        // John's CRAZY idea 5, .45, 0, 50.07
        //1.07, .1825, 0, 17.07

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            ///////////////
            // GAMEPAD 1 //
            ///////////////

            // a =
            // b = slight turn right
            // x = slight turn left
            // y =
            // rbumper = slow mode
            // lbumper =
            // rtrigger =
            // ltrigger =
            // dpad_up = barrier up
            // dpad_down =
            // dpad_left = barrier down
            // dpad_right =
            // start = fire 3 disks
            // back = fire 1 disk

            // SINGLE TURN ON trigger
//            if (gamepad1.left_bumper){
//                if (runtime.seconds() > 1) {
//                    triggered = true;
//                    runtime.reset();
//                    }
//            } else if (triggered) {
//                robot.trigger.setPower(1);
//                telemetry.addData("Single Shot", true);
//                telemetry.update();
//                if (runtime.milliseconds() > 400) {
//                    triggered = false;
//                    robot.trigger.setPower(0);
//                    runtime.reset();
//                }
//            } else {
//                robot.pretrigger.setPower(gamepad2.left_trigger * 1);
//                //robot.preTrigger.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * 1);
//            }

            // DRIVING
            drive();

            // trigger
            // around 825 ticks = 1 turn
            if (gamepad1.left_bumper){
                runtime.reset();
                robot.trigger.setPower(1);
                robot.trigger.setTargetPosition(820);
                while (robot.trigger.getCurrentPosition() < 818 && runtime.milliseconds() < 3000) {
                    robot.trigger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            else if (gamepad1.right_bumper){
                runtime.reset();
                robot.trigger.setPower(1);
                robot.trigger.setTargetPosition(2480);
                while (robot.trigger.getCurrentPosition() < 2478 && runtime.milliseconds() < 3000) {
                    robot.trigger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            if (gamepad1.b) {
                shootPos();
            }

            else if (gamepad2.left_trigger > .25){
                robot.trigger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.trigger.setPower(-gamepad2.left_trigger*.4);
            }
            else{
                robot.trigger.setTargetPosition(0);
                robot.trigger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.trigger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.trigger.setPower(gamepad2.right_trigger);
            }

            // BARRIER (OBSOLETE)
//            if (gamepad1.dpad_left) {
//                robot.barrier.setPower(0.7);
//            }
//            else if (gamepad1.dpad_right) {
//                robot.barrier.setPower(-0.7);
//            }
//            else if(gamepad2.left_stick_y > 10) {
//                robot.barrier.setPower(gamepad2.left_stick_y);
//            }
//            else if (gamepad2.left_stick_y < 10) {
//                robot.barrier.setPower(gamepad2.left_stick_y);
//            }else {
//                robot.barrier.setPower(0);
//            }

            // TURN

            int frontLeftCurPos = robot.frontLeft.getCurrentPosition();
            int frontRightCurPos = robot.frontRight.getCurrentPosition();
            int rearLeftCurPos = robot.rearLeft.getCurrentPosition();
            int rearRightCurPos = robot.rearRight.getCurrentPosition();



            ///////////////
            // GAMEPAD 2 //
            ///////////////
            // a = shooter off
            // b = shooter slow
            // x = shooter fast
            // y = shooter mid
            // rbumper = feed_in
            // lbumper = feed_out
            // rtrigger = trigger in
            // ltrigger = trigger out
            // dpad_up = wobbleGoal_up
            // dpad_down = wobbleGoal_down
            // dpad_left = claw open
            // dpad_right = claw close
            // start =
            // back =

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
                if (robot.tractor.getCurrentPosition() > -5){
                    robot.tractor.setPower(-.1);
                }
                else {
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
                }
            else if (gamepad2.dpad_right || latch){
                    runtime.reset();
                    int tractorPos = robot.tractor.getTargetPosition() - 300;
                    robot.tractor.setTargetPosition(tractorPos);
                    while (runtime.milliseconds() < 500 && readyGrip) {
                        if (runtime.milliseconds() < 400) {
                            robot.tractor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.tractor.setPower(-.25);
                        }

                        else{
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
            telemetry.addLine("left | ")
                    .addData("forward", robot.frontLeft.getCurrentPosition())
                    .addData("rear", robot.rearLeft.getCurrentPosition());

            telemetry.addLine("right | ")
                    .addData("forward", robot.frontRight.getCurrentPosition())
                    .addData("rear", robot.rearRight.getCurrentPosition());

            telemetry.addLine("Flywheel | ")
                    .addData("Speed: ", flySet)
                    .addData("Idle: ", flyIdle)
                    .addData("Velocity: ", robot.flywheel.getVelocity());

            telemetry.addLine("Trigger | ")
                    .addData(" Pos: ", robot.trigger.getCurrentPosition())
                    .addData(" Tar: ", robot.trigger.getTargetPosition());

            telemetry.addLine("Tractor | ")
                    .addData(" Pos ", robot.tractor.getCurrentPosition())
                    .addData(" Power ", robot.tractor.getPower())
                    .addData(" Tar ", robot.tractor.getTargetPosition());

            telemetry.addLine("Imu | ")
                    .addData("heading :", lastAngles.firstAngle)
                    .addData("global heading", getAngle());

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }
    }

    public void drive(){
        double[] direction = new double[3];
        direction[0] = -gamepad1.right_stick_x * .9 + gamepad1.right_trigger / 4 - gamepad1.left_trigger / 4; //Turn
        direction[1] = -gamepad1.left_stick_y;   // Forward
//        double[] speed = {(robot.frontLeft.getCurrentPosition() - lastTick[0]) / motorTime.seconds(),
//                (robot.frontRight.getCurrentPosition() - lastTick[1]) / motorTime.seconds(),
//                (robot.rearLeft.getCurrentPosition() - lastTick[2]) / motorTime.seconds(),
//                (robot.rearRight.getCurrentPosition() - lastTick[3]) / motorTime.seconds()};
//
//
//        motorTime.reset();
        int changeL = (robot.frontLeft.getCurrentPosition()  - lastTick[0]);
        int changeR = (robot.frontRight.getCurrentPosition()  - lastTick[1]);
        float forward = (changeL + changeR) / 2;
        float turn = changeR - changeL;
        telemetry.addLine("Velocity")
                .addData("Forward", forward)
                .addData("Turn", turn);
        absX += forward * Math.cos(turn);
        absY += forward * Math.sin(turn);
        lastTick = new int[] {robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.rearLeft.getCurrentPosition(), robot.rearRight.getCurrentPosition()};


        double[] driveTrainPower = new double[]{
                (-direction[0] + direction[1]),
                (direction[0] + direction[1]),
                (-direction[0] + direction[1]),
                (direction[0] + direction[1]),
        };

        robot.frontLeft.setPower(driveTrainPower[0]);
        robot.frontRight.setPower(driveTrainPower[1]);
        robot.rearLeft.setPower(driveTrainPower[2]);
        robot.rearRight.setPower(driveTrainPower[3]);
    }

    public void shootPos() {
        double rawDif = ang - lastAngles.firstAngle;
        double dif = Math.abs(rawDif) - tolerence;
        double turn = lastAngles.firstAngle;
        if (turn < 0 ) {
            turn += 360;
        }
        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(-speed);
        robot.rearLeft.setPower(speed);
        robot.rearRight.setPower(-speed);
        runtime.reset();
        while ((turn < ang + tolerence || turn > ang - tolerence) && runtime.milliseconds() < 5000 && opModeIsActive()) {
            turn = lastAngles.firstAngle;
            telemetry.addLine("Imu | ")
                    .addData("heading :", lastAngles.firstAngle)
                    .addData("global heading", getAngle());
            telemetry.update();
        }
    }

    private double getAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void driveNew() {
        final double x = Math.pow(-gamepad1.left_stick_x, 3);
        final double y = Math.pow(-gamepad1.left_stick_y, 3);

        final double rotation = Math.pow(-gamepad1.right_stick_x, 3);
        final double direction = Math.atan2(x,y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double fr = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fl = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double rl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        robot.frontLeft.setPower(fl);
        robot.frontRight.setPower(fr);
        robot.rearLeft.setPower(rl);
        robot.rearRight.setPower(rr);
    }

    public void driveV3(){
        double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI/4;
        double rightX = -gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        robot.frontLeft.setPower(v1);
        robot.frontRight.setPower(v2);
        robot.rearLeft.setPower(v3);
        robot.rearRight.setPower(v4);
    }
}
