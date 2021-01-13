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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_BrainBotz;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
// List where other files are located that are used in this OpMode

/**
 * This OpMode uses the HardwareMap_Example class to define the devices on the robot.
 */
// CHAWKS: Name it something useful!
@TeleOp(name="TeleOp POV BrainBotz", group="TeleOp")
// CHAWKS: What does @Disabled mean? what happens if we remove it?
//@Disabled
public class TeleOp_BrainBotzDual extends LinearOpMode {

    /* CHAWKS: Call and declare the robot here */
    HardwareMap_BrainBotz robot   = new HardwareMap_BrainBotz();    // Use the Example hardware map

    // MUST HAVE
    @Override
    public void runOpMode() {

        // Below are VARIABLES that must be outside the "while (opModeIsActive())" loop
        double leftF;
        double rightF;
        double leftB;
        double rightB;
        double drive;
        double turn;
        double strafe;
        double max,max1,max2;
        double maxSpeed=1;
        double turnReducer = 1.0;

        boolean triggerOn=false;
        double lineAngle=0.2;
        double wallAngle=0.5;


        /*
            CHAWKS: On Driver Station, telemetry will be display!
                    How is this useful for debugging?
        */
        // Send telemetry message to Driver Station
        telemetry.addData("Status: ", "Hit [Init] to Initialize robot");    //
        telemetry.update();

        /*
            CHAWKS: Step 0. Initialize OUR ROBOT
        */
        // MUST HAVE THIS LINE BELOW
        robot.init(hardwareMap);
        robot.target.setPosition(wallAngle);
        robot.wobble.setPosition(1);
        robot.grabber.setPosition(0);


        // Send telemetry message to "Driver Station" signify robot waiting;
        telemetry.addData("Status: ", "Hit [PLAY] to start!");    //


        /*
            CHAWKS: Step 1. Hit Play to run through the code!
        */
        // MUST HAVE THIS LINE BELOW
        waitForStart();


        /*
            CHAWKS: Remember opModeIsActive?! It's a loop!
        */
        // run until the end of the match (driver presses [STOP])
        // MUST HAVE!
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            /*
                CHAWKS: gamepad1 vs. gamepad2
                        what are our limits?
           */
            drive = -gamepad1.left_stick_y;
            strafe  = gamepad1.left_stick_x;
            turn  = turnReducer*(gamepad1.right_stick_x);

            if (gamepad1.right_bumper || gamepad1.left_bumper) { // Low gear
                maxSpeed = 0.5;
                turnReducer = 0.75;
            } else if (gamepad1.right_bumper && gamepad1.left_bumper) { // Super slow
                maxSpeed = 0.33;
                turnReducer = 1;
            } else if ((!gamepad1.right_bumper) && (!gamepad1.left_bumper)) { // Full speed
                maxSpeed = 1.0;
                turnReducer = 1.0;
            }



            // Combine drive and turn for blended motion.
            leftB  = (-strafe+drive + turn)*maxSpeed;
            rightB = (strafe+drive - turn)*maxSpeed;
            leftF  = (strafe+drive + turn)*maxSpeed;
            rightF = (-strafe+drive - turn)*maxSpeed;

            // Normalize the values so neither exceed +/- 1.0
            max1 = Math.max(Math.abs(leftF), Math.abs(rightF));
            max2 = Math.max(Math.abs(leftB), Math.abs(rightB));
            max=Math.max(max1,max2);
            if (max > 1.0)
            {
                leftF /= max;
                rightF /= max;
                leftB /= max;
                rightB /= max;
            }

            //intake
            if(gamepad2.left_trigger>0.1) {
               robot.intake.setPower(1.0);
            }
            else if (gamepad2.right_trigger>0.1) {
                robot.intake.setPower(-1.0);
            }
            else if((gamepad2.left_trigger<=0.1)&&(gamepad2.right_trigger<=0.1)) {
                robot.intake.setPower(0);
            }
            //shooter
            if (-gamepad2.left_stick_y>.1) {
                robot.shooter.setPower(0.75);
            }
                else if (-gamepad2.left_stick_y<=.1){
                    robot.shooter.setPower(0);
            }
                // turn on press button on and off
            if (gamepad2.a){
                robot.trigger.setPosition(1.0);
                sleep(50);
            }
              else if(gamepad2.x) {
                robot.trigger.setPosition(0.5);
                sleep(50);
            }
            if  (gamepad2.left_bumper){
                robot.target.setPosition(wallAngle);
                sleep(50);
        }
            if (gamepad2.right_bumper) {
                robot.target.setPosition(lineAngle);
                sleep(50);
            }
            if (gamepad2.dpad_down){
                robot.wobble.setPosition(1);
                sleep(1000);
                robot.grabber.setPosition(1);
                sleep(50);
            }
            if (gamepad2.dpad_up){
                robot.grabber.setPosition(0);
                sleep(1000);
                robot.wobble.setPosition(0.5);
            }
            // Output the safe vales to the motor drives.
            /*
                CHAWKS: Put in the power number!
                        Which motors are we missing?
                        What would happen if you apply power to half the wheels?
             */

            robot.leftFront.setPower(leftF);
            robot.rightFront.setPower(rightF);
            robot.leftBack.setPower(leftB);
            robot.rightBack.setPower(rightB);


            // Send telemetry message to signify robot running;
            telemetry.addData("leftFront",  "%.2f", leftF);
            telemetry.addData("rightFront", "%.2f", rightF);
            telemetry.addData("leftBack",  "%.2f", leftB);
            telemetry.addData("rightBack", "%.2f", rightB);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);

            /*
                CHAWKS: We are the end! What happens now?
            */
        }
    }
}
