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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Caydens_New_Hardware_Configuration {
    /* Public OpMode members. */
    public CRServo linear_slide = null;
    public DcMotor left_drive = null;
    public DcMotor right_drive = null;
    public DcMotor Wheel_3 =null;
    public DcMotor arm = null;
    public DcMotor launcher = null;
    public DcMotor spool = null;
    public Servo clawrotater = null;
    public Servo left_claw = null;
    public Servo right_claw = null;
    public Servo toprightclaw = null;
    public Servo topleftclaw = null;
 //   public Servo color_arm = null;
    public ColorSensor color_sensor = null;
    public Servo color_arm = null;
   // public Servo wrist = null;
    //public NormalizedColorSensor sensor_color = null;
    public static final double MID_SERVO = 1;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    public static final double SERVO_HOME =0.0;
    public static final double SERVO_MIN =0.0;
    public static final double SERVO_MAX =0.65;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Caydens_New_Hardware_Configuration() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        left_drive = hwMap.get(DcMotor.class, "left_drive");
        right_drive = hwMap.get(DcMotor.class, "right_drive");
      //  Wheel_3 = hwMap.get(DcMotor.class, "Wheel_3");
        arm = hwMap.get(DcMotor.class, "arm");
        launcher = hwMap.get(DcMotor.class, "launcher");
        left_drive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right_drive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
      //  Wheel_3.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        left_drive.setPower(0);
        right_drive.setPower(0);
       // Wheel_3.setPower(0);
        arm.setPower(0);
        launcher.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // Wheel_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos
        /*right_claw = hwMap.get(Servo.class, "right_claw");
        left_claw = hwMap.get(Servo.class, "left_claw");
        topleftclaw = hwMap.get(Servo.class, "topleftclaw");
        toprightclaw = hwMap.get(Servo.class, "toprightclaw");
        color_arm = hwMap.get(Servo.class, "color_arm");
        color_arm = hwMap.get(Servo.class, "color_arm");
        clawrotater = hwMap.get(Servo.class,"linear_slide");*/
     //   wrist = hwMap.get(Servo.class,"wrist");

       // wrist.setPosition(MID_SERVO);
       // color_arm.setPosition(SERVO_HOME);
        // left_claw.setPosition(MID_SERVO);
       // right_claw.setPosition(SERVO_HOME);

        // Define and initialize Rev Color sensor
        color_sensor = hwMap.get(ColorSensor.class, "color_sensor");




    }
}

