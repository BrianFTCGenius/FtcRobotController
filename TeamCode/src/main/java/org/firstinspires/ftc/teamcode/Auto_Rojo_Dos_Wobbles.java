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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.w3c.dom.Element;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_Rojo_Dos_Wobbles", group="Pushbot")
//@Disabled
public class Auto_Rojo_Dos_Wobbles extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    String Element;

    /* Declare OpMode members. */
    HardwareMecanum robot   = new HardwareMecanum();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    int[] currentPos = {0, 0, 0, 0};

    private static final String VUFORIA_KEY =
            "AWdbNWH/////AAABmU59dFu3g0lCnaPyHiCQUKQqxHMDG1C7Op/URaB7VkGLS95G7" +
                    "OUQMIu8MNQMjMjMjY67Re/OUDUNWzPAG4zfQe/MV5lG8osn4z8TDcndGx0jWfGRxr5iivD+" +
                    "/RmdCg/857Cq8fZ63uP4ZCqre+Gigvg0jDM3161z7sYniu5oI8CoPajV5s1bYpmFWjXjatunS/3" +
                    "3XwDC5R4OiEFu/8S9vM3DLUsZsCUGthojS2KZxNDNNHnOkLWNI3oeZfdGpqBWBSQtMb4hnHZk3eiTH9" +
                    "fKMfps2BJCwpozyL8Ls50zDLs0TPFEvG4fkQiFwPZLFO2sZwUZxN8htRTPL2WdxGbS3kI7msBCIxzl4ozfuVZUykz3 ";

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
        robot.init(hardwareMap);
        //robot.claw.setPower(-.45);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tractor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.kicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.kicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.kicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.tractor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Robot motion
        // FL, FR, RL, RR
        // Forwards  =   + - + -
        // Backwards =   - + - +
        // Pivot right = + + + +
        // Pivot left = - - - -
        // Claw - = close; + = open

        // BEGINNING OF AUTO
        // Start reving flywheel
        robot.flywheel.setPower(0.69);
        // Drive to launch line
        encoderDrive(new int[]{2560, -2600, 2560, -2600}, 0.55, 5000);
        // Wait for flywheel to reach target speed
        sleep(2000);
        // Shoot disks
        shootTrip();
        sleep(1200);
        robot.kicker.setPower(0);
        robot.flywheel.setPower(0);

        if (opModeIsActive()) {
            runtime.reset();
            while(opModeIsActive() && Element == null && (runtime.milliseconds() < 1000)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            int i = 0;
                            Element = recognition.getLabel();
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (Element == "Quad" ) {
            // Drive towards far square
            encoderDrive(new int[]{295, 295, 295, 295}, 0.6, 5000); //was 2000
            encoderDrive(new int[]{3000, -3000, 3000, -3000}, 0.7, 5000);

            // Open claw and drop arm
            clawFix(.25, 800);
            //robot.tractor.setPower(-.2);
            encoderDrive(new int[]{-230, -230, -230, -230}, 0.5, 5000);
            //tractorDown(true,3000);
            encoderDrive(new int[]{-1700, 1700, -1700, 1700}, 0.6, 5000);
            clawFix(-.25, 800);


            //encoderDrive(new int[]{-200, 200, -200, 200}, 0.6, 5000);

        } else if (Element == "Single") {
            // Turn towards second square
            encoderDrive(new int[]{260, 260, 260, 260}, 0.40, 5000);
            // Drive towards second square
            encoderDrive(new int[]{1400, -1400, 1400, -1400}, 0.55, 5000);

            // Open claw to drop wobble goal #1
            clawFix(.25, 1000);
            tractorDown(true,3000);
            //robot.claw.setPower(0);
            // Lower arm into position
//            robot.tractor.setPower(-.4);
//            sleep (1550);
//            robot.tractor.setPower(0);

            // Back up slightly
            encoderDrive(new int[]{-200, 200, -200, 200}, 0.40, 5000);
            // Turn towards wobble goal #2
            encoderDrive(new int[]{-410, -410, -410, -410}, 0.40, 5000);

            encoderDrive(new int[]{-1350, 1350, -1350, 1350}, 0.60, 5000);
            // Curve towards wobble goal #2
            curveDrive(new int[]{-800, 900, -800, 900}, 0.40, .65, 5000);
            encoderDrive(new int[]{-170, 170, -170, 170}, 0.20, 5000);


            // Lower arm
//            robot.tractor.setPower(-.7); //down
//            sleep (310);
//            robot.tractor.setPower(0);
            // Grab wobble goal #2
            clawFix(-.25, 1000);
            // Raise up arm
//            robot.tractor.setPower(.4);
//            sleep (900);
//            robot.tractor.setPower(0);
            tractorDown(false, 3000);

            encoderDrive(new int[]{1500, -1500, 1500, -1500}, 0.40, 5000);
            encoderDrive(new int[]{1000, 1000, 1000, 1000}, 0.50, 5000);
            //encoderDrive(new int[]{-700, 700, -700, 700}, 0.40, 5000);

//            robot.tractor.setPower(-.4);
//            sleep (200);
//            robot.tractor.setPower(0);
            tractorDown(true, 3000);
            clawFix(.25, 2000);
//            robot.tractor.setPower(.5);
//            sleep(500);
//            robot.tractor.setPower(0);
            tractorDown(false, 3000);

            encoderDrive(new int[]{100, -100, 100, -100}, 0.50, 5000);



        } else {
            // Drive towards 1st square
            encoderDrive(new int[]{ 500, 500, 500, 500}, 0.6, 5000); //was 2000
            encoderDrive(new int[]{1900, -1900, 1900, -1900}, 0.6, 5000);

            // Open claw
            clawFix(.25, 800);
            // Lower arm
            tractorDown(true, 3000);
            /*robot.tractor.setPower(-.55);
            sleep (1200);
            robot.tractor.setPower(0);*/
            sleep (500);

            // Turn and drive towards 2nd wobble goal
            encoderDrive(new int[]{-200, 200, -200, 200}, 0.55, 5000);
            encoderDrive(new int[]{ -520, -520, -520, -520}, 0.4, 5000);
            encoderDrive(new int[]{-1225, 1225, -1225, 1225}, 0.5, 5000);
            encoderDrive(new int[]{-100, 100, -100, 100}, 0.2, 5000);

            // Lower arm and grip wobble goal

            //tractorDown();
//            robot.tractor.setPower(-.6); //down
//            sleep (220);
//            robot.tractor.setPower(0);
            tractorDown(false, 3000);
            clawFix(-.25, 1000);
//            robot.tractor.setPower(.4);
//            sleep (900);
//            robot.tractor.setPower(0);

            // Turn and back up into 1st square
            encoderDrive(new int[]{ 1060, 1060, 1060, 1060}, 0.3, 5000);
            encoderDrive(new int[]{-845, 845, -845, 845}, 0.6, 5000);

            // Lower arm and drop wobble goal
//            robot.tractor.setPower(-.4);
//            sleep (200);
//            robot.tractor.setPower(0);
            tractorDown(true, 3000);
            clawFix(.25, 800);
//            robot.tractor.setPower(.5);
//            sleep(500);
//            robot.tractor.setPower(0);
            tractorDown(false, 3000);
            encoderDrive(new int[]{100, -100, 100, -100}, 0.2, 5000);



//            encoderDrive(new int[]{-400, 400, -400, 400}, 0.5, 5000);
//            encoderDrive(new int[]{-300, -300, -300, 300}, 0.6, 5000);
//            encoderDrive(new int[]{600, -600, 600, -600}, 0.5, 5000);
        }

        //encoderDrive(new int[]{700, -700, 700, -700}, 0.75, 2);


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(int[] direction, double speed, int time){
        int[] calcPos = currentPos;
        for (int i = 0; i < 4; i ++) {
            calcPos[i] = calcPos[i] - direction[i];
        }

        robot.frontLeft.setTargetPosition(calcPos[0]);
        robot.frontRight.setTargetPosition(calcPos[1]);
        robot.rearLeft.setTargetPosition(calcPos[2]);
        robot.rearRight.setTargetPosition(calcPos[3]);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(speed);
        robot.rearLeft.setPower(speed);
        robot.rearRight.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < time)
                && (robot.frontRight.isBusy() && robot.frontLeft.isBusy())
                && robot.rearLeft.isBusy() && robot.rearLeft.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Claw")
                    .addData("Power", robot.claw.getPower());
            telemetry.addLine("left")
                    .addData("forward", robot.frontLeft.getCurrentPosition())
                    .addData("rear", robot.rearLeft.getCurrentPosition());
            telemetry.addLine("right")
                    .addData("forward", robot.frontRight.getCurrentPosition())
                    .addData("rear", robot.rearRight.getCurrentPosition());
            telemetry.addData("Time Elapsed", runtime.milliseconds());
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void curveDrive(int[] direction, double speedLeft, double speedRight, int time){
        int[] calcPos = currentPos;
        for (int i = 0; i < 4; i ++) {
            calcPos[i] = calcPos[i] - direction[i];
        }

        robot.frontLeft.setTargetPosition(calcPos[0]);
        robot.frontRight.setTargetPosition(calcPos[1]);
        robot.rearLeft.setTargetPosition(calcPos[2]);
        robot.rearRight.setTargetPosition(calcPos[3]);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(speedLeft);
        robot.frontRight.setPower(speedRight);
        robot.rearLeft.setPower(speedLeft);
        robot.rearRight.setPower(speedRight);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < time)
                && (robot.frontRight.isBusy() && robot.frontLeft.isBusy())
                && robot.rearLeft.isBusy() && robot.rearLeft.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Claw")
                    .addData("Power", robot.claw.getPower());
            telemetry.addLine("left")
                    .addData("forward", robot.frontLeft.getCurrentPosition())
                    .addData("rear", robot.rearLeft.getCurrentPosition());
            telemetry.addLine("right")
                    .addData("forward", robot.frontRight.getCurrentPosition())
                    .addData("rear", robot.rearRight.getCurrentPosition());
            telemetry.addData("Time Elapsed", runtime.milliseconds());
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void shoot(double power, double revTime) {
        robot.flywheel.setPower(power);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < revTime) {
            telemetry.addData("Flywheel", power);
            telemetry.update();
        }
        robot.kicker.setPower(0.7);
        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds() < 400) {
            telemetry.addData("Flywheel", power);
            telemetry.addData("Time Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.kicker.setPower(0);
        //robot.flywheel.setPower(0.5);
    }

    public void shootTrip(){
        runtime.reset();
        robot.kicker.setTargetPosition(2480);
        robot.kicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.kicker.setPower(.95);
        while (robot.kicker.getCurrentPosition() < 2478 && runtime.milliseconds() < 3500 && robot.kicker.isBusy()) {
            telemetry.addData("Shooting",true);
        }
        robot.kicker.setPower(0);
    }

    public void tractorDown(boolean up, int itime){
        robot.tractor.setTargetPosition(-2150);
        robot.tractor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.tractor.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < itime)
                && (robot.tractor.isBusy())) {
            telemetry.addLine("Tractor")
                    .addData("Position", robot.tractor.getCurrentPosition());
            telemetry.update();
        }
        robot.tractor.setPower(0);
        robot.tractor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void clawFix(double power, int time) {
        runtime.reset();
        while (runtime.milliseconds() < time){
            robot.claw.setPower(power);
        }
    }
}
