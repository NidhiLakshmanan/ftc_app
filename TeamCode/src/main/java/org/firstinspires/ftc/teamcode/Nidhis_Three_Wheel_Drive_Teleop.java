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
import android.media.MediaPlayer;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Nidhi's Three Wheel Drive Teleop", group="Hermit")
//@Disabled
public class Nidhis_Three_Wheel_Drive_Teleop extends LinearOpMode {

    /* Declare OpMode members. */

    Caydens_New_Hardware_Configuration robot           = new Caydens_New_Hardware_Configuration();   // Use a Pushbot's hardware
    double  color_arm_offset = 0;
    double linearslideoffset = 0;
    double wristoffset = 0;
    double FinalArmPower = 0;
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    final double    Wrist_Speed     =0.01;
    double top_claw_offset;
    int armoffset = 0;
    MediaPlayer mp;
    @Override
    public void runOpMode() {
        double left ;
        double right ;
        double drive ;
        double turn ;
        double max;
        double ArmPower;
        boolean ArmLocked = false;
        double GamepadTheta;
        double GamepadRadial;
        double speed;
        boolean ElbowLocked = false;

        //sound declarations
          SoundPool mySound;
          int chimesID;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "SALUTATIONS HOMO SAPIENS, DATLOF CONTINET SEMPER DATLOF, BOOGIEWOOGIEWOOGIE");    //
        telemetry.update();
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
       chimesID = mySound.load(hardwareMap.appContext, R.raw.chimes, 1); // PSM


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //double CurrentElbowPower = (double)robot.spool.getPower();
       // double CurrentArmPower =(double) robot.arm.getPower();
        // run until the end of the match (driver presses STOP)


        while (opModeIsActive()) {

            if (gamepad1.a){
                mySound.play(chimesID,1,1,1,0,1);}




            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.\
            //
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ) {
                double Drivestick_y = gamepad1.left_stick_y;
                double Drivestick_x = -gamepad1.left_stick_x;
                GamepadTheta = Math.atan2(Drivestick_y, Drivestick_x);
                GamepadRadial = Math.sqrt(Math.pow(Drivestick_y, 2) + Math.pow(Drivestick_x, 2));
                speed = GamepadRadial;
                double Wheel1Thing = 150 - GamepadTheta;
                double Wheel2Thing = 30 - GamepadTheta;
                double Wheel3Thing = 270 - GamepadTheta;
                double Wheel_1_Speed = speed * Math.cos(Wheel1Thing);
                double Wheel_2_Speed = speed * Math.cos(Wheel2Thing);
                 double Wheel_3_Speed = speed * Math.cos(Wheel3Thing);
                // Makes it so the values do not exceed +/- 1
            if (Wheel_1_Speed > 1)
                Wheel_1_Speed /= Wheel_1_Speed;
            if (Wheel_2_Speed > 1)
                Wheel_2_Speed /= Wheel_2_Speed;
            if (Wheel_3_Speed < 1)
                    Wheel_3_Speed /= 1;
            if (Wheel_1_Speed < -1)
                    Wheel_1_Speed /= Wheel_1_Speed;
            if (Wheel_2_Speed < -1)
                    Wheel_2_Speed /= Wheel_2_Speed;
            if (Wheel_3_Speed < -1)
                    Wheel_3_Speed /= -1;
                // Output the safe vales to the motor drives.
                if((Drivestick_x >= .1 || Drivestick_x <= -.1) && (Drivestick_y <= .2 && Drivestick_y >= -.2)|| ((Drivestick_x >=.3 || Drivestick_x <=-.3) &&(Drivestick_y >= .3 || Drivestick_y<=.3))){
                  robot.left_drive.setPower(Wheel_1_Speed/1.25);
                robot.right_drive.setPower(-Wheel_2_Speed/1.25);
                robot.Wheel_3.setPower(-Wheel_3_Speed/1.208);}
                if(Drivestick_y >= .1 || Drivestick_y <= -.1 && (Drivestick_x <= .2 && Drivestick_x >= -.2)){
                    robot.left_drive.setPower(Wheel_1_Speed/1.25);
                    robot.right_drive.setPower(-Wheel_2_Speed/1.25);
                    robot.Wheel_3.setPower(0);}
            }
            else if(gamepad1.right_stick_x != 0) {
                  robot.left_drive.setPower(-gamepad1.right_stick_x/2.08);
                robot.right_drive.setPower(-gamepad1.right_stick_x/2.08);
                robot.Wheel_3.setPower(-gamepad1.right_stick_x/3.125);}
            else  if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0){
                robot.left_drive.setPower(0);
                robot.right_drive.setPower(0);
                robot.Wheel_3.setPower(0);}
            telemetry.addData("Arm Power",FinalArmPower);
            telemetry.addData("left drive power",robot.left_drive.getPower());
            telemetry.addData("right drive power",robot.right_drive.getPower());
            telemetry.addData("Back Wheel power",robot.Wheel_3.getPower());
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.update();


            //sleep(100);
            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_trigger != 0){

                clawOffset += CLAW_SPEED;
                }
            else if (gamepad2.left_trigger != 0){
                clawOffset -= CLAW_SPEED;}
            clawOffset = Range.clip(clawOffset, -1, 1);
            robot.right_claw.setPosition(.5 - clawOffset);
            robot.left_claw.setPosition(0.5 + clawOffset);
              //  clawOffset -= CLAW_SPEED;
            if (gamepad1.right_trigger != 0){

                color_arm_offset += CLAW_SPEED;
            }else if (gamepad1.left_trigger != 0){
                color_arm_offset -= CLAW_SPEED;}
                robot.color_arm.setPosition(1+ color_arm_offset);
           // robot.right_claw.setPosition( 1 - clawOffset);
            //robot.left_claw.setPosition(0 + clawOffset);
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
            if (gamepad2.left_stick_y != 0 /*&& ArmLocked == false  && gamepad2.right_stick_y/time > 0*/ )
            /*{
               ArmPower = (0.01   * gamepad2.left_stick_y);
                FinalArmPower += ArmPower;
               // ArmPower = Math.pow(gamepad2.left_stick_y,7)/10;
                robot.arm.setPower(FinalArmPower);
                //armoffset += ArmPower;
            //robot.arm.setPower(ArmPower);
           // robot.arm.setTargetPosition(armoffset);
        }*/
            if (gamepad2.left_stick_y != 0 /*&& ArmLocked == false  && gamepad2.right_stick_y/time > 0*/ )
            {
                ArmPower = (gamepad2.left_stick_y/1.5);
                FinalArmPower += ArmPower;
                // ArmPower = Math.pow(gamepad2.left_stick_y,7)/10;
                robot.arm.setPower(ArmPower);
                //armoffset += ArmPower;
                //robot.arm.setPower(ArmPower);
                // robot.arm.setTargetPosition(armoffset);
            }


           //else if (gamepad2.left_stick_y/time ==0) {
             //   robot.arm.setPower(0.5);
            //}
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
/*if (gamepad2.a){
robot.clawrotater.setPosition(1);
            }
            else if (gamepad2.x)
                robot.clawrotater.setPosition(0.0);*/
//linearslideoffset = Range.clip(linearslideoffset,-1,1);
//robot.linear_slide.setPosition(0.5 + linearslideoffset);
            // Use gamepad left & right bumpers to open and close the claw
            if (gamepad2.right_bumper)
                top_claw_offset -= CLAW_SPEED;
            else if (gamepad2.left_bumper)
                top_claw_offset += CLAW_SPEED;
            else
                top_claw_offset -= 0;
            top_claw_offset = Range.clip(top_claw_offset,-1,1);
           robot.toprightclaw.setPosition(0 - top_claw_offset);
            robot.topleftclaw.setPosition(1 + top_claw_offset);
            if (gamepad2.a){
                clawOffset = .5;
            }
            if (gamepad2.x){
                top_claw_offset = -.5;}

         // use gamepad2 ???? button to play ????

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

          //  telemetry.addData("Wheel 1 Speed",  "%.2f", Wheel_1_Speed);
           // telemetry.addData("Wheel 2 Speed", "%.2f", Wheel_2_Speed);
           // telemetry.addData("Wheel 3 Speed", "%.2f", Wheel_3_Speed);
           // telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(5);
    }
}}
