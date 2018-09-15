
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The VuForia function
 * is  packaged as utility class within the main opMmode class (inner class). The findVuMark class
 * is generic usable for any single VuMark. It could be moved out of this example to a separate
 * class or a library class.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 *
 *  VuMark is like a bar code. It is an image that contains encoded variable information. For the
 *  Relic Recovery game, the VuMark is the image of a temple. Encoded on that image in hexagonal
 *  dots is a code indicating left, center and right. Vuforia is used to locate the image in the
 *  camera field of view and extract the code returning that to your program. FIRST included a
 *  custom enum class to display the code (also called an instance id) as text.
 */

@Autonomous(name="Red_Platform_Away_From_Audience", group ="Exercises")
//@Disabled
public class Red_Platform_Away_From_Audience_Regional extends LinearOpMode
{
    Caydens_New_Hardware_Configuration robot = new Caydens_New_Hardware_Configuration();   // Use Cayden's Hardware Configuration
    private ElapsedTime runtime = new ElapsedTime();

    Boolean SawRed = false;
    Boolean SawBlue = false;
    VuMarkFinder        vmf;
    RelicRecoveryVuMark vuMark;
public Boolean rightglyph = false;
    public Boolean leftglyph = false;
    public Boolean centerglyph = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Create an instance of VuMarkFinder. This can take some time to complete.
        robot.init(hardwareMap);

        vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", true, VuforiaLocalizer.CameraDirection.BACK);

        telemetry.addData("Mode", "Press Play to start");
        telemetry.update();
// hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "DATLOF TENEBO");    //
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start

            telemetry.addData("Red Level", robot.color_sensor.red());
            telemetry.addData("Blue Level", robot.color_sensor.blue());
            telemetry.update();
            idle();
        }

        waitForStart();
        robot.left_claw.setPosition(.5);
        robot.right_claw.setPosition(.5);
        // Start VuForia background process looking for vumarks in camera field of view.
        vmf.activate();

        for (int i = 0; i<40; i++)
        {
            // See if a vumark is visible.
            if (vmf.findVuMark())
            {
                // Convert vumark instance  id to game specific id.
                vuMark = RelicRecoveryVuMark.from(vmf.instanceId);

                telemetry.addData("VuMark", "%s visible", vuMark);

                //telemetry.addData("Pose", vmf.formatPose(vmf.pose));

                telemetry.addData("X Y Z", "X=%f  Y=%f  Z=%f", vmf.tX, vmf.tY, vmf.tZ);
                if (vuMark == RelicRecoveryVuMark.RIGHT){
                    rightglyph = true;

                }
                if (vuMark == RelicRecoveryVuMark.LEFT){
                    leftglyph = true;

                }
                if (vuMark == RelicRecoveryVuMark.CENTER){
                    centerglyph = true;

                }
            }
            else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();

            idle();
            sleep(100);
        }
robot.color_arm.setPosition(.25);
        sleep(700);
        if (robot.color_sensor.red() > robot.color_sensor.blue()) {
            SawRed = true;
        } else if (robot.color_sensor.red() < robot.color_sensor.blue()) {
            SawBlue = true;
        } else {
            telemetry.addData(" I can't see the color", robot.color_sensor.red());
            telemetry.update();
        }
        if (SawRed) {
            telemetry.addData("Saw Red, Datlof continet, semper Datlof!", robot.color_sensor.red());
            telemetry.update();
            sleep(1000);
            robot.left_drive.setPower(-.2);
            robot.right_drive.setPower(-.2);
            robot.Wheel_3.setPower(-.2);
            sleep(150);
            robot.left_drive.setPower(.0);
            robot.right_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            robot.color_arm.setPosition(1);
            sleep(350);
            robot.left_drive.setPower(.2);
            robot.right_drive.setPower(.2);
            robot.Wheel_3.setPower(.2);
            sleep(150);
            robot.right_drive.setPower(0);
            robot.left_drive.setPower(0);
            robot.Wheel_3.setPower(0);
            sleep(300);
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            sleep(300);
            } else if (SawBlue) {
            telemetry.addData("Saw Blue, " +
                    "Datlof continet, semper Datlof " +
                    "(Laqueus mustaches, & thea!)!" +
                    "Salvete Datlof", robot.color_sensor.red());
            telemetry.update();
            sleep(1000);
            robot.left_drive.setPower(.2);
            robot.right_drive.setPower(.2);
            robot.Wheel_3.setPower(.2);
            sleep(150);
            robot.left_drive.setPower(.0);
            robot.right_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            robot.color_arm.setPosition(1);
            sleep(350);
            robot.left_drive.setPower(-.2);
            robot.right_drive.setPower(-.2);
            robot.Wheel_3.setPower(-.2);
            sleep(150);
            robot.right_drive.setPower(0);
            robot.left_drive.setPower(0);
            robot.Wheel_3.setPower(0);
            sleep(300);
        }
        if (rightglyph == true){
            //close claw
            robot.right_claw.setPosition(.5);
            robot.left_claw.setPosition(.5);
            sleep(1600);
            //move forward
            robot.right_drive.setPower(-0.4);
            robot.left_drive.setPower(0.4);
            sleep(1600);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(600);
            //move forward
            robot.right_drive.setPower(-.4);
            robot.left_drive.setPower(.4);
            robot.Wheel_3.setPower(.0);
            sleep(900);
            //do nothing and open claw
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            robot.right_claw.setPosition(1);
            robot.left_claw.setPosition(0);
            sleep(200);
            //move backwards
            robot.right_drive.setPower(.4);
            robot.left_drive.setPower(-.4);
            robot.Wheel_3.setPower(.0);
            sleep(300);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(700);
        }
        if (centerglyph == true){
            //close claw
            robot.right_claw.setPosition(.5);
            robot.left_claw.setPosition(.5);
            sleep(1400);
            //move forward
            robot.right_drive.setPower(-0.4);
            robot.left_drive.setPower(0.4);
            sleep(1700);
            //turns
            robot.right_drive.setPower(.3);
            robot.left_drive.setPower(.3);
            robot.Wheel_3.setPower(.3);
            sleep(180);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(600);
            //move forward
            robot.right_drive.setPower(-.4);
            robot.left_drive.setPower(.4);
            robot.Wheel_3.setPower(.0);
            sleep(1450);
            //do nothing and open claw
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            robot.right_claw.setPosition(1);
            robot.left_claw.setPosition(0);
            sleep(200);
            //move backwards
            robot.right_drive.setPower(.4);
            robot.left_drive.setPower(-.4);
            robot.Wheel_3.setPower(.0);
            sleep(500);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(700);
        }
        if (leftglyph == true){
            //close claw
            robot.right_claw.setPosition(.5);
            robot.left_claw.setPosition(.5);
            sleep(1400);
            //move forward
            robot.right_drive.setPower(-0.4);
            robot.left_drive.setPower(0.4);
            sleep(1700);
            //turns
            robot.right_drive.setPower(.3);
            robot.left_drive.setPower(.3);
            robot.Wheel_3.setPower(.3);
            sleep(500);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(600);
            //move forward
            robot.right_drive.setPower(-.4);
            robot.left_drive.setPower(.4);
            robot.Wheel_3.setPower(.0);
            sleep(1700);
            robot.right_drive.setPower(-.3);
            robot.left_drive.setPower(-.3);
            robot.Wheel_3.setPower(-.3);
            sleep(500);
            robot.right_drive.setPower(-.4);
            robot.left_drive.setPower(.4);
            robot.Wheel_3.setPower(.0);
            sleep(100);
            //do nothing and open claw
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            robot.right_claw.setPosition(1);
            robot.left_claw.setPosition(0);
            sleep(200);
            //move backwards
            robot.right_drive.setPower(.4);
            robot.left_drive.setPower(-.4);
            robot.Wheel_3.setPower(.0);
            sleep(500);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(700);
        }
        else if(rightglyph == false && centerglyph == false && leftglyph == false ) {
            //close claw
            robot.right_claw.setPosition(.5);
            robot.left_claw.setPosition(.5);
            sleep(1750);
            //move forward
            robot.right_drive.setPower(-0.4);
            robot.left_drive.setPower(0.4);
            sleep(1800);
            //turn 90 degrees
            robot.right_drive.setPower(-.3);
            robot.left_drive.setPower(-.3);
            robot.Wheel_3.setPower(-.3);
            sleep(700);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(600);
            //move forward
            robot.right_drive.setPower(-.4);
            robot.left_drive.setPower(.4);
            sleep(700);
            //do nothing and open claw
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            robot.right_claw.setPosition(1);
            robot.left_claw.setPosition(0);
            sleep(200);
            //open claw
            robot.right_drive.setPower(.4);
            robot.left_drive.setPower(-.4);
            robot.Wheel_3.setPower(.0);
            sleep(300);
            //do nothing
            robot.right_drive.setPower(.0);
            robot.left_drive.setPower(.0);
            robot.Wheel_3.setPower(.0);
            sleep(700);
        }

    }

    /**
     * VuForia VuMark finder class.
     */
    public static class VuMarkFinder
    {
        private VuforiaLocalizer vuforia;
        private VuforiaTrackables trackables;
        private VuforiaTrackable template;

        public VuMarkInstanceId instanceId;
        public OpenGLMatrix pose;
        public double               tX, tY, tZ, rX, rY, rZ;

        /** Constructor.
         * Create an instance of the class.
         * @param hMap HardwareMap object.
         * @param assetName Name of the asset file containing the VuMark definition.
         * @param includeViewer True to display camera viewer on RC phone.
         * @param camera Front or Back camera choice.
         */
        public VuMarkFinder(HardwareMap hMap,
                            String assetName,
                            boolean includeViewer,
                            VuforiaLocalizer.CameraDirection camera)
        {
            VuforiaLocalizer.Parameters parameters;

            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
             * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
             */

            if (includeViewer)
            {
                int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
                parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            }
            else
                // OR...  Do Not Activate the Camera Monitor View, to save power
                parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = "AQdDh+P/////AAAAGYG4khX9T0Mai5pYz9oTllp2KuZI24ZwM9ostcBXs2A90ddi/sJDOAabZEVM/5jhWNRN40BJ32nrSkbKTnqMnZ10v1A/PjDvnKwLG7zpA/wATnngFrhODfBwaHvP1WouKc+9f8QPOfLJnoGAlohWpfNWmdSe0UiyAeVoNCRW6TlLHECp85fs/acyk0eOy3qvUmJSFOTIsa5sJHVHscqpofheFgzhfmC7c+VUHGB8fIDiFBLdJBK9My1B2BBsJhblTZWgeVjOFI28qEHiEm7ADigF4zkH890YMfBRDr70ajPRJfOuzPAQA2QmOatQyL3tO/s9VmiIkcPDirMkTdwPbfBxUYkkCBGUDQMtYstBS58G";

            /*
             * We also indicate which camera on the RC that we wish to use.
             * Here we chose the back (HiRes) camera (for greater range), but
             * for a competition robot, the front camera might be more convenient.
             */
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.useExtendedTracking = false;
            vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /*
             * Load the data set containing the VuMark. This code supports 1 VuMark.
             */
            trackables = vuforia.loadTrackablesFromAsset(assetName);
            template = trackables.get(0);
            template.setName(assetName); // can help in debugging; otherwise not necessary
        }

        /**
         * Activate VuForia image processing. Call after waitForStart().
         */
        public void activate()
        {
            trackables.activate();
        }

        /**
         * Call to find out if VuMark is visible to the phone camera.
         * @return True if VuMark found, false if not.
         */
        public boolean findVuMark()
        {
            // See if any of the instances of the template are currently visible.
            instanceId = ((VuforiaTrackableDefaultListener) template.getListener()).getVuMarkInstanceId();

            if (instanceId != null)
            {
                pose = ((VuforiaTrackableDefaultListener) template.getListener()).getPose();

                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }

                return true;
            }
            else
            {
                pose = null;
                return false;
            }
        }

        /**
         * Format pose object for human viewing.
         * @param pose Pose object returned when VuMark is found.
         * @return Pose description.
         */
        String formatPose(OpenGLMatrix pose)
        {
            return (pose != null) ? pose.formatAsTransform() : "null";
        }
    }

}