package org.firstinspires.ftc.teamcode.tele_code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utility_code.Robot;

@TeleOp(name = "Calibrate TeleOp", group = "tele-op")
public class CalibrationTeleOp extends OpMode {
    Robot robot;
    double x1, y1, x2;
    double liftSpeed;
    public void initRobot() {
        robot = new Robot();
    }

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        initRobot();
        robot.init(hardwareMap, true);
        robot.angleTracker.setOrientation(180);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > .2) {
            y1 = -gamepad1.left_stick_y;
        } else {
            y1 = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) > .2) {
            x1 = gamepad1.left_stick_x;
        } else {
            x1 = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) > .2) {
            x2 = gamepad1.right_stick_x;
        } else {
            x2 = 0;
        }

        //with reverse button, only need to reverse for forward and back (not turning)
        x1 *= .5;
        y1 *= .5;
        x2 *= .5;

        robot.drive(y1, x1, x2, telemetry);

        //was just robot.drive(y1, x1, x2, telemetry); before

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}