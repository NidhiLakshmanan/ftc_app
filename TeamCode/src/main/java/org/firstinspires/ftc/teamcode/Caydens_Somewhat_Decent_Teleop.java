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

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="Caydens_Somewhat_Decent_Teleop", group="Hermit")
//@Disabled
public class Caydens_Somewhat_Decent_Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    Caydens_New_Hardware_Configuration robot           = new Caydens_New_Hardware_Configuration();   // Use a Pushbot's hardware
    double  color_arm_offset = 0;
    double linearslideoffset = 0;
    double wristoffset = 0;
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.01 ;                   // sets rate to move servo
    final double    Wrist_Speed     =0.01;
    public SoundPool mySound;
    public int beepID;
    @Override
    public void runOpMode() {
        double left ;
        double right ;
        double drive ;
        double turn ;
        double max;
        double ArmPower;
        boolean ArmLocked = false;

        boolean ElbowLocked = false;
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = mySound.load(hardwareMap.appContext, R.raw.chimes, 1); // PSM
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "SALUTATIONS HOMO SAPIENS, DATLOF CONTINET SEMPER DATLOF, BOOGIEWOOGIEWOOGIE");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //double CurrentElbowPower = (double)robot.spool.getPower();
      //  double CurrentArmPower =(double) robot.arm.getPower();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //This makes the robot move forward and backwards when the right stick is moved forwards and backwards, with the
            //and turn when the right stick is moved to the left or right, with the two combined on one stick for
            //for easy, fluid movement.  The input is placed into a function for smoother movement.
            drive = gamepad1.right_stick_y;
            turn = gamepad1.right_stick_x;
            // Combine drive and turn for blended motion.
            left = drive/1.75  + (turn/1.75 );
            right = drive/1.75  - (turn/1.75 );

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.left_drive.setPower(-left);
            robot.right_drive.setPower(right);
            mySound.play(beepID,gamepad1.right_stick_x + gamepad1.right_stick_y,gamepad1.right_stick_y+gamepad1.right_stick_x,1,3,1);
            // Use gamepad left & right Bumpers to open and close the claw
          /*  if (gamepad2.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.left_bumper)
                clawOffset -= CLAW_SPEED;
            clawOffset = Range.clip(clawOffset, -1, 1);
            robot.right_claw.setPosition( 1 - clawOffset);
            robot.left_claw.setPosition(0 + clawOffset);*/
            /*if (gamepad2.right_trigger != 0)
            wristoffset += Wrist_Speed ;
            if(gamepad2.left_trigger!=0)
                wristoffset -= Wrist_Speed;
            wristoffset= Range.clip(wristoffset,-.5,.5);
            robot.wrist.setPosition(0.0 + wristoffset);*/
            // Move both servos to new position.  Assume servos are mirror image of each other.

          /*  if (gamepad2.left_stick_y != 0 && ElbowLocked == false)
                robot.spool.setPower(gamepad2.left_stick_y/3);
            else if (ElbowLocked == false)robot.spool.setPower(0.0); */
            // Use gamepad buttons to move arm up and down on the y axis
          //  if (gamepad2.right_stick_y != 0 /*&& ArmLocked == false  && gamepad2.right_stick_y/time > 0*/ )
            {
                ArmPower = (gamepad2.right_stick_y);
            robot.arm.setPower(-ArmPower);
            robot.launcher.setPower(ArmPower);
            }
        //else
            //robot.arm.setPower(0.0);
           /* else if (gamepad2.right_stick_y/time ==0) {
                robot.arm.setPower(CurrentArmPower*.5);
            }*/
            /*else
                robot.arm.setPower(0.0);
            /*if (gamepad2.a) {
                ArmLocked = !ArmLocked;
                robot.arm.setPower(CurrentArmPower);
            }*/
           /* if (gamepad2.x){
                ElbowLocked = !ElbowLocked;
                robot.arm.setPower(CurrentElbowPower);
            }*/
           //Sets the power for the horizontal linear slide that moves the arm horizontally
/*if (gamepad2.left_stick_x != 0){
robot.linear_slide.setPower(-gamepad2.left_stick_x/1.5);
            }
            else robot.linear_slide.setPower(0.0);*/
//linearslideoffset = Range.clip(linearslideoffset,-1,1);
//robot.linear_slide.setPosition(0.5 + linearslideoffset);
            // Use gamepad left & right triggers for raising and lowering the color sensor stick
           /* if (gamepad1.right_trigger != 0)
                color_arm_offset += CLAW_SPEED;
            else if (gamepad1.left_trigger != 0)
                color_arm_offset -= CLAW_SPEED;
            color_arm_offset = Range.clip(color_arm_offset,-1,1);
            robot.color_arm.setPosition(0 + color_arm_offset);*/
        /*    if (gamepad2.dpad_up == true){
                robot.color_arm.setPosition(.5);
            }
            else if (gamepad2.dpad_up == true)
                robot.color_arm.setPosition(1);*/
            // Move both servos to new
            // position.  Assume servos are mirror image of each other.
           // clawOffset = Range.clip(color_arm_offset, -0.5, 0.5);
           // robot.left_claw.setPosition(robot.MID_SERVO + color_arm_offset);
            //robot.right_claw.setPosition(robot.MID_SERVO - color_arm_offset);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(5);
    }
}}
