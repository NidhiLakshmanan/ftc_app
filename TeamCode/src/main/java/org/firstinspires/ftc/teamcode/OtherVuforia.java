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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Concept: VuMark Id", group ="Concept")
//@Disabled
public class OtherVuforia extends LinearOpMode {

    Caydens_New_Hardware_Configuration robot = new Caydens_New_Hardware_Configuration();   // Use a Pushbot's hardware

    public static boolean SeeVumark = false;
    public static final String TAG = "Vuforia VuMark Sample";
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor left_drive = null;
    public DcMotor right_drive = null;
    public DcMotor arm = null;
    public Servo left_claw = null;
    public Servo right_claw = null;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    OpenGLMatrix lastLocation = null;


    // {@link #vuforia} is the variable we will use to store our instance of the Vuforia
    //localization engine.

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Say", "SALUTATIONS HOMO SAPIENS, GRLORY GREATEST ARSTOTZKA");    //
            telemetry.update();

            robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

robot.right_claw.setPosition(-90);
            encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, 3, -3, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(TURN_SPEED, 9, -9, 1);
            encoderDrive(DRIVE_SPEED, 18, 18, 1);
            encoderDrive(TURN_SPEED, -12, 12, 1);
            encoderDrive(DRIVE_SPEED, 3, 3, 1);
            /*if (SeeVumark = true) {
                robot.left_claw.setPosition(180);
                robot.right_claw.setPosition(-180);
                encoderDrive(DRIVE_SPEED, 2, 2, 1);
            }*/
        }
    }

        //* To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        //* If no camera monitor is desired, use the parameterless constructor instead (commented out below).

      //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
         * and paste it in to your code onthe next line, between the double quotes.

        parameters.vuforiaLicenseKey = "AQ0MtNn/////AAAAGQ/3nNdxSEJSkTfrnWlhW5EDBvUPqk4UmpmDicTe3TkHbsAhMKgOxevE4k5Fy36B0vePHLTV/C3gcjvp3yYKyQWFIvsj+ucjWLzjMtz2CVAtnl3zMl4SSBxae2ta8FwcryVFAmJbUiEIosmd/IVfRy1k7snTyugqHABTb94/ofb8qFdkBWgHPig10rKNFqMbk4QZiuBWlESwM02jQ2k78HWCC9WaFJ/bVfscytFlJOqE4LF9KzIqrAI5A2lIixxUxA/57" + "zLu/zVtummHKUMwj84/85N98agKUMeHsn5xE6LC2VTIWUA7VwFWatD9SOfwggXc0RjnUIBlGbK1Uku5NIBjfAZ/AC/0Ud0qlbg4uqiW";
       parameters.vuforiaLicenseKey = "AQ0MtNn/////AAAAGQ/3nNdxSEJSkTfrnWlhW5EDBvUPqk4UmpmDicTe3TkHbsAhMKgOxevE4k5Fy36B0vePHLTV/C3gcjvp3yYKyQWFIvsj+ucjWLzjMtz2CVAtnl3zMl4SSBxae2ta8FwcryVFAmJbUiEIosmd/IVfRy1k7snTyugqHABTb94/ofb8qFdkBWgHPig10rKNFqMbk4QZiuBWlESwM02jQ2k78HWCC9WaFJ/bVfscytFlJOqE4LF9KzIqrAI5A2lIixxUxA/57zLu/zVtummHKUMwj84/85N98agKUMeHsn5xE6LC2VTIWUA7VwFWatD9SOfwggXc0RjnUIBlGbK1Uku5NIBjfAZ/AC/0Ud0qlbg4uqiW";

         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

    while (opModeIsActive()) {
            //program robot to push opposing team's jewel off panel.

        }


          See if any of the instances of {@link relicTemplate} are currently visible.

          {@link RelicRecoveryVuMark} is an enum which can have the following values:
          UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
           UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 // on which VuMark was visible.
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                  // we illustrate it nevertheless, for completeness.
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            final Telemetry.Item pose1 = telemetry.addData("Pose", format(pose));

                 We further illustrate how to decompose the pose into useful rotational and
                  //translational components
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;


            }
            // String format; (OpenGLMatrix, transformationMatrix); {
            //   return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";}
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            runtime.reset();
            //while (runtime.seconds() < 9) {
              //  encoderDrive(TURN_SPEED, 9, -9, 1);
                //encoderDrive(DRIVE_SPEED, 15, 15, 1);
                //encoderDrive(TURN_SPEED, -12, 12, 1);
                //encoderDrive(DRIVE_SPEED, 3, 3, 1);
                //SeeVumark = true;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                runtime.reset();
                //while (runtime.seconds() < 9) {
                //  encoderDrive(TURN_SPEED, 9, -9, 1);
                //encoderDrive(DRIVE_SPEED, 18, 18, 1);
                // encoderDrive(TURN_SPEED, -12, 12, 1);
                //encoderDrive(DRIVE_SPEED, 3, 3, 1);
                //SeeVumark = true;
            }
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    runtime.reset();
                    //while (runtime.seconds() < 9) {
                      //  encoderDrive(TURN_SPEED, 9, -9, 1);
                        //encoderDrive(DRIVE_SPEED, 21, 21, 1);
                        //encoderDrive(TURN_SPEED, -12, 12, 1);
                        //encoderDrive(DRIVE_SPEED, 3, 3, 1);
                        //SeeVumark = true;
                    }
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            left_drive.setPower(0.25);
            right_drive.setPower(0.25);
        }
                 else{
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
    }

}
*/





    public void encoderDrive(double speed,
        double leftInches,double rightInches,
        double timeoutS) {

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left_drive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right_drive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.left_drive.setTargetPosition(newLeftTarget);
            robot.right_drive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_drive.setPower(Math.abs(speed));
            robot.right_drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_drive.isBusy() && robot.right_drive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_drive.getCurrentPosition(),
                        robot.right_drive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              sleep(250);   // optional pause after each move

        }


        }
    }


