/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@TeleOp(name="Teleop Vuforia Odo", group ="Concept")
//@Disabled
public class MecanumTeleopODOVu extends LinearOpMode {

    HardwareMecanumV3 robot = new HardwareMecanumV3();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime motorTime = new ElapsedTime();
    private float absX = 0;
    private float absY = 0;
    private int[] lastTick = {0, 0, 0, 0};
    private Orientation lastAngles = new Orientation();
    double globalAngle = 0;
    private static int shootingAngle = 0;
    double offset = 0;

    List<VuforiaTrackable> allTrackables;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWdbNWH/////AAABmU59dFu3g0lCnaPyHiCQUKQqxHMDG1C7Op/URaB7VkGLS95G7" +
                    "OUQMIu8MNQMjMjMjY67Re/OUDUNWzPAG4zfQe/MV5lG8osn4z8TDcndGx0jWfGRxr5iivD+" +
                    "/RmdCg/857Cq8fZ63uP4ZCqre+Gigvg0jDM3161z7sYniu5oI8CoPajV5s1bYpmFWjXjatunS/3" +
                    "3XwDC5R4OiEFu/8S9vM3DLUsZsCUGthojS2KZxNDNNHnOkLWNI3oeZfdGpqBWBSQtMb4hnHZk3eiTH9" +
                    "fKMfps2BJCwpozyL8Ls50zDLs0TPFEvG4fkQiFwPZLFO2sZwUZxN8htRTPL2WdxGbS3kI7msBCIxzl4ozfuVZUykz3 ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;


    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override public void runOpMode() {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

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


            if (gamepad1.a) {
                //shootingAdjust();
                turnToAngle(0);
            }

            if (gamepad1.b) {
                align();
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


            // TURN
            int frontLeftCurPos = robot.frontLeft.getCurrentPosition();
            int frontRightCurPos = robot.frontRight.getCurrentPosition();
            int rearLeftCurPos = robot.rearLeft.getCurrentPosition();
            int rearRightCurPos = robot.rearRight.getCurrentPosition();



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


            vuforiaUpdate();
            // TELEMETRY
//            telemetry.addLine("left | ")
//                    .addData("forward", robot.frontLeft.getCurrentPosition())
//                    .addData("rear", robot.rearLeft.getCurrentPosition());
//
//            telemetry.addLine("right | ")
//                    .addData("forward", robot.frontRight.getCurrentPosition())
//                    .addData("rear", robot.rearRight.getCurrentPosition());

            telemetry.addLine("Flywheel | ")
                    .addData("Speed: ", flySet)
                    .addData("Idle: ", flyIdle)
                    .addData("Velocity: ", robot.flywheel.getVelocity());

            telemetry.addLine("Trigger | ")
                    .addData(" Pos: ", robot.trigger.getCurrentPosition())
                    .addData(" Tar: ", robot.trigger.getTargetPosition());

            telemetry.addLine("Imu | ")
                    .addData("heading :", lastAngles.firstAngle)
                    .addData("global heading", getAngle());

            targetsUltimateGoal.activate();
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;


            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(25);
        }

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    public void drive(){
        double[] direction = new double[3];
        direction[0] = -gamepad1.right_stick_x - gamepad1.right_trigger / 4 + gamepad1.left_trigger / 4; //Turn
        direction[1] = -gamepad1.left_stick_y;   // Forward
        direction[2] = -gamepad1.left_stick_x;  //Strafe

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
        motorMove(direction);
    }

    private void motorMove(double[] direction) {
        double[] driveTrainPower = new double[]{
                (-direction[0] + direction[1] - direction[2]),
                (direction[0] + direction[1] + direction[2]),
                (-direction[0] + direction[1] + direction[2]),
                (direction[0] + direction[1] - direction[2]),
        };
        robot.frontLeft.setPower(driveTrainPower[0]);
        robot.frontRight.setPower(driveTrainPower[1]);
        robot.rearLeft.setPower(driveTrainPower[2]);
        robot.rearRight.setPower(driveTrainPower[3]);
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

    public void turnToAngle(int angle){
        float heading = lastAngles.firstAngle;
        double speed = Math.abs(((heading)/40));
        double tolerance = 2;
        //double speed = .3;
        if (heading < angle - tolerance){
            robot.frontLeft.setPower(-speed);
            robot.frontRight.setPower(speed);
            robot.rearLeft.setPower(-speed);
            robot.rearRight.setPower(speed);
        }
        else if (heading > angle + tolerance){
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(-speed);
            robot.rearLeft.setPower(speed);
            robot.rearRight.setPower(-speed);
        }
        else{
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.rearLeft.setPower(0);
            robot.rearRight.setPower(0);
        }
    }

    public void vuforiaUpdate() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            offset = translation.get(1)/mmPerInch;
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.addData("offset",  offset);
        } else {
            telemetry.addData("Visible Target", "none");
        }
    }


    public void align() {
        double speed = 0.25;
        double target = -17.5;
        double tolereance = 3;
        if (offset > target - tolereance) {
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(-speed);
            robot.rearLeft.setPower(-speed);
            robot.rearRight.setPower(speed);
        } else if (offset < target + tolereance) {
            robot.frontLeft.setPower(-speed);
            robot.frontRight.setPower(speed);
            robot.rearLeft.setPower(speed);
            robot.rearRight.setPower(-speed);
        } else {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.rearLeft.setPower(0);
            robot.rearRight.setPower(0);
        }


    }

}
