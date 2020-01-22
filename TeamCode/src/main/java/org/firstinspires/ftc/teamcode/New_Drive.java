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


@TeleOp(name="TeleopDrive - New")
public class New_Drive extends LinearOpMode {

    //Declare OpMode members
    RR_Hardware robot  = new RR_Hardware();

    @Override
    public void runOpMode() {


        // Initialize the hardware variables.
        // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);
        robot.arm_gripper.setPosition(0.5);
        double grip_start_pos = robot.arm_gripper.getPosition();
        double swap = 1.0;
        double speed_reduction = .75;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.addData("Gripper Start ", robot.arm_gripper.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //PROGRAM STARTS HERE -------------------------------------------------------------------------------------------------------------------

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* Gamepad 1*/


            if (gamepad1.start)     swap = -1.0;
            else if (gamepad1.back) swap = 1.0;

            telemetry.addData("Servo Position", "%5.2f", robot.servo_left.getPosition());
            telemetry.addData("Servo Position", "%5.2f", robot.servo_right.getPosition());
            telemetry.addData("Servo Position", "%5.2f", robot.arm_gripper.getPosition());
            telemetry.addData("Speed Multiplier", speed_reduction);
            telemetry.update();

            // Move both servos to new position.  Assume servos are mirror image of each other.
            if (gamepad1.left_bumper){
                robot.servo_right.setPosition(0.2);
                robot.servo_left.setPosition(0.2);
            }
            if (gamepad1.right_bumper){
                robot.servo_right.setPosition(0.5);
                robot.servo_left.setPosition(0.5);
            }

            //Apply gripper transformation
            robot.arm_gripper.setPosition(grip_start_pos + gamepad2.left_stick_y);

            //Fancy math to calculate mecanum wheels direction
            if      (gamepad1.a) speed_reduction = .35;
            else if (gamepad1.y) speed_reduction = .75;
            double r          = Math.hypot((gamepad1.left_trigger-gamepad1.right_trigger), gamepad1.left_stick_y);
            double robotangle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_trigger-gamepad1.right_trigger)) - Math.PI / 4;
            double rightX     = gamepad1.right_stick_x * swap;
            double V1 = r * Math.cos(robotangle) + rightX;
            double V2 = r * Math.sin(robotangle) - rightX;
            double V3 = r * Math.sin(robotangle) + rightX;
            double V4 = r * Math.cos(robotangle) - rightX;
            /*
            if (r <= .75) V1 *= speed_reduction;
            if (r <= .75) V2 *= speed_reduction;
            if (r <= .75) V3 *= speed_reduction;
            if (r <= .75) V4 *= speed_reduction;
            */

            //Apply wheel drive
            robot.left_front    .setPower(V1 * swap * speed_reduction);
            robot.right_front   .setPower(V2 * swap * speed_reduction);
            robot.left_rear     .setPower(V3 * swap * speed_reduction);
            robot.right_rear    .setPower(V4 * swap * speed_reduction);


            /* Gamepad 2*/


            robot.arm_extender.setPower(gamepad2.right_stick_y);

            if (gamepad2.a) robot.capstone.setPosition(.6);
            else robot.capstone.setPosition(0.0);


            // PROGRAM ENDS HERE ----------------------------------------------------------------------------------------------------------------
        }
    }
}
