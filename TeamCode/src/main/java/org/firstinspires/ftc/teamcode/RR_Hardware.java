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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class RR_Hardware
{
    /* Public OpMode members. */
    public DcMotor  left_rear   = null;
    public DcMotor  right_rear  = null;
    public DcMotor  left_front   = null;
    public DcMotor  right_front    = null;
    public Orientation lastAngles = new Orientation();
    public DistanceSensor distanceSensor;
    public ColorSensor colorSensor;
    public BNO055IMU imu_hub1;
    public BNO055IMU imu_hub10;
    public Servo servo_left = null;
    public Servo servo_right = null;
    public static final double MID_SERVO = 0;
    public static final double max_pos = 0.25;
    public static final double start_pos = 0;
    public DcMotor  arm_extender = null;
    public Servo arm_gripper = null;



    // public DcMotor  leftArm     = null;
    //public Servo    leftClaw    = null;
    //public Servo    rightClaw   = null;

    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RR_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu_hub1 = hwMap.get(BNO055IMU.class, "imu_hub1");
        imu_hub1.initialize(parameters);


        // Define and Initialize Motors
        left_front        = hwMap.get(DcMotor.class, "leftFront");
        left_rear         = hwMap.get(DcMotor.class, "leftRear");
        right_front       = hwMap.get(DcMotor.class, "rightFront");
        right_rear        = hwMap.get(DcMotor.class, "rightRear");
        arm_extender      = hwMap.get(DcMotor.class,  "armExtender");
        arm_gripper       = hwMap.get(Servo.class,  "armGripper");
        servo_left        = hwMap.get(Servo.class, "servoLeft");
        servo_right       = hwMap.get(Servo.class,"servoRight");

        colorSensor       = hwMap.get(ColorSensor.class, "sensor_color_distance");



        left_rear.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        left_front.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        right_rear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right_front.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        left_front.setPower(0);
        right_front.setPower(0);
        left_rear.setPower(0);
        right_rear.setPower(0);
        //leftArm.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo_right.scaleRange(-1, 1);
        servo_left.scaleRange(-1, 1);

        servo_left.setDirection(Servo.Direction.REVERSE);
        servo_right.setDirection(Servo.Direction.FORWARD);

        arm_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_extender.setPower(0);

        arm_gripper.scaleRange(0.3, 0.9);

        //TO DO: SET MOTORS TO COAST AND BREAK
    }
 }

