package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Park_Drive_Skystone_Sensor - Final", group="Production")
@Disabled
public class Park_Drive_Skystone_Sensor extends LinearOpMode {

    /* Declare OpMode members. */
    RR_Hardware robot = new RR_Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final int COUNTS_PER_MOTOR_REV = 28;    // Motor with 1:1 gear ratio
    static final double DRIVE_GEAR_REDUCTION = 10.5;     // Rev Ultraplanetary Motor 12:1 but actual is 10.5:1
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH_DOUBLE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Init Hardware");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            robot.left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left_rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.left_front.setPower(-0.18);
            robot.right_front.setPower(-0.18);
            robot.right_rear.setPower(-0.18);
            robot.left_rear.setPower(-0.18);


            float hsvValues[] = {0F, 0F, 0F};

            final float values[] = hsvValues;

            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;

            Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                    (int) (robot.colorSensor.green() * SCALE_FACTOR),
                    (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            // Loop to stop robot when its over the blue or red tape under bridge.
            // Blue value from color sensor is ~4500
            while ((robot.colorSensor.blue() < 2500) || (robot.colorSensor.red() > 2500)) {
                telemetry.addData("Blue", robot.colorSensor.blue());
                telemetry.update();
            }
            robot.left_front.setPower(0);
            robot.right_front.setPower(0);
            robot.right_rear.setPower(0);
            robot.left_rear.setPower(0);
        }
    }
}
