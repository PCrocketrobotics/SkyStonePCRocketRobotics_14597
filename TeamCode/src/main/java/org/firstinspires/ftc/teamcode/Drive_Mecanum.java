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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

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

@TeleOp(name="TeleOp: TeleopDrive")
public class Drive_Mecanum extends LinearOpMode {

    /* Declare OpMode members. */
    RR_Hardware   robot  = new RR_Hardware();

    @Override
    public void runOpMode() {
        double drive;
        double rotation;
        double Mag;
        double angle;
        double v1;
        double v2;
        double v3;
        double v4;
        double servo;
        double clawOffset = 0;
        final double CLAW_SPEED  = .5;
        double  servoleftpos;
        double servorightpos;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //PROGRAM STARTS HERE -----------------------------------------------------------------------------------------------

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            rotation = gamepad1.left_stick_x;
servoleftpos = robot.servo_left.getPosition();
servorightpos = robot.servo_right.getPosition();
            telemetry.addData("Servo Position", "%5.2f", servoleftpos);
            telemetry.addData("Servo Position", "%5.2f", servorightpos);

            telemetry.update();



            if (gamepad1.dpad_up){
                robot.servo_right.setPosition(0.1);
                robot.servo_left.setPosition(0.2);
            }
            if (gamepad1.dpad_down){
                robot.servo_right.setPosition(0);
                robot.servo_left.setPosition(0);
            }
            // Move both servos to new position.  Assume servos are mirror image of each other.


            // Move both servos to new position.  Assume servos are mirror image of each other.
            //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            //robot.servo_left.setPosition(robot.MID_SERVO + clawOffset);
            //robot.servo_right.setPosition(robot.MID_SERVO - clawOffset);

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotangle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            final double V1 = r * Math.cos(robotangle) + rightX;
            final double V2 = r * Math.sin(robotangle) - rightX;
            final double V3 = r * Math.sin(robotangle) + rightX;
            final double V4 = r * Math.cos(robotangle) - rightX;

            robot.left_front.setPower(V1);
            robot.right_front.setPower(V2);
            robot.left_rear.setPower((V3));
            robot.right_rear.setPower(V4);


            //Gamepad2

            //set the shoulder to move when left stick is moved up or down


            //set the arm extender to extend or retract when right stick is moved up or down

            telemetry.addData("Servo Position", "%5.2f", robot.servo_left.getPosition());
            telemetry.addData("Servo Position", "%5.2f", robot.servo_right.getPosition());








            // Send telemetry message to signify robot running;
            //telemetry.addData("left",  "%.2f", leftPower);
            //telemetry.addData("right", "%.2f", rightPower);

            telemetry.update();
            // PROGRAM ENDS HERE -------------------------------------------------------------------------------------
        }
    }
}
