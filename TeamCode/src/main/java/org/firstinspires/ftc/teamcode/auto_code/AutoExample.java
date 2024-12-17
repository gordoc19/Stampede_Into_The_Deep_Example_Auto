package org.firstinspires.ftc.teamcode.auto_code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utility_code.Robot;
import org.firstinspires.ftc.teamcode.utility_code.DriveTo;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;

@Autonomous(name = "AUTO", group = "Autonomous")
public class AutoExample extends OpMode {
    boolean isRed = false;
    boolean recentRed = false;
    boolean isNet = false;
    boolean recentNet = false;
    DriveTo driveTo;
    Robot robot;
    public float coords[] = new float[5];
    // in the class variables
    String nextState = "actionStart";
    // we'll set this when we need to wait for an action to complete rather than
    // check if the lift or drive is busy.
    double wait = 0;

    //for where coords are on the field for our different auto modes (diff start positions, ect.)
    HashMap<String, double[]> drivePositionsNetRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsNetBlue = new HashMap<>();
    HashMap<String, double[]> drivePositionsObsRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsObsBlue = new HashMap<>();
    HashMap<String, double[]> drivePositions;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, true);

        driveTo = new DriveTo(robot, telemetry);

        //x, y, heading for start positions
        drivePositionsNetRed.put("start", new double[]{-36, -63, 90});
        drivePositionsNetBlue.put("start", new double[]{36, 63, -90});
        drivePositionsObsRed.put("start", new double[]{24, -63, 90});
        drivePositionsObsBlue.put("start", new double[]{-24, 63, -90});

        final double ROBOT_TO_BUCKET = 12;

        drivePositionsNetRed.put("Sample Score Pose", new double[]{-68 + ROBOT_TO_BUCKET / Math.sqrt(2), -68 + ROBOT_TO_BUCKET / Math.sqrt(2), 45});
        drivePositionsNetBlue.put("Sample Score Pose", new double[]{68 - ROBOT_TO_BUCKET / Math.sqrt(2), 68 - ROBOT_TO_BUCKET / Math.sqrt(2), -135});

        drivePositionsNetRed.put("Ascent Park Waypoint", new double[]{-39, -30, 90});
        drivePositionsNetBlue.put("Ascent Park Waypoint", new double[]{39, 30, -90});

        drivePositionsNetRed.put("Ascent Park", new double[]{-24, -12, 90});
        drivePositionsNetBlue.put("Ascent Park", new double[]{24, 12, -90});

        drivePositionsObsRed.put("Obs Park", new double[]{48, -63, 90});
        drivePositionsObsBlue.put("Obs Park", new double[]{-48, 63, -90});

        /*double ROBOT_TO_PIXEL_CENTER = 8.5;
        drivePositionsNetRed.put("propLEFT", new double[]{-47.5 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 135});
        drivePositionsNetRed.put("propMIDDLE", new double[]{-36, -24.5 - ROBOT_TO_PIXEL_CENTER, 90});
        drivePositionsNetRed.put("propRIGHT", new double[]{-24.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 45});

        drivePositionsObsRed.put("propLEFT", new double[]{0.5 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 135});
        drivePositionsObsRed.put("propMIDDLE", new double[]{12, -24.5 - ROBOT_TO_PIXEL_CENTER, 90});
        drivePositionsObsRed.put("propRIGHT", new double[]{23.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 45});

        drivePositionsNetBlue.put("propLEFT", new double[]{-24.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -45});
        drivePositionsNetBlue.put("propMIDDLE", new double[]{-36, 24.5 + ROBOT_TO_PIXEL_CENTER, -90});
        drivePositionsNetBlue.put("propRIGHT", new double[]{-49 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -135});

        drivePositionsObsBlue.put("propLEFT", new double[]{23.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -45});
        drivePositionsObsBlue.put("propMIDDLE", new double[]{12, 24.5 + ROBOT_TO_PIXEL_CENTER, -90});
        drivePositionsObsBlue.put("propRIGHT", new double[]{-1 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -135});

        drivePositionsNetRed.put("propApproachLEFT", new double[]{-38, -40, 135});
        drivePositionsNetRed.put("propApproachMIDDLE", new double[]{-36, -40, 90});
        drivePositionsNetRed.put("propApproachRIGHT", new double[]{-36, -40, 45});
        drivePositionsObsRed.put("propApproachLEFT", new double[]{12, -40, 135});
        drivePositionsObsRed.put("propApproachMIDDLE", new double[]{12, -40, 90});
        drivePositionsObsRed.put("propApproachRIGHT", new double[]{14, -40, 45});
        drivePositionsNetBlue.put("propApproachLEFT", new double[]{-36, 40, -45});
        drivePositionsNetBlue.put("propApproachMIDDLE", new double[]{-36, 40, -90});
        drivePositionsNetBlue.put("propApproachRIGHT", new double[]{-38, 40, -135});
        drivePositionsObsBlue.put("propApproachLEFT", new double[]{14, 40, -45});
        drivePositionsObsBlue.put("propApproachMIDDLE", new double[]{12, 40, -90});
        drivePositionsObsBlue.put("propApproachRIGHT", new double[]{12, 40, -135});*/
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {
            if (!recentNet) {
                isNet = !isNet;
                recentNet = true;
            }
        } else {
            recentNet = false;
        }
        if (gamepad1.dpad_left) {
            if (!recentRed) {
                isRed = !isRed;
                recentRed = true;
            }
        } else {
            recentRed = false;
        }
        telemetry.addData("Team : ", "%s", isRed ? "RED" : "BLUE");
        telemetry.addData("Position : ", "%s", isNet ? "NET" : "OBSERVATION");
        telemetry.update();
    }

    @Override
    public void start() {
        //Pick our hashmap for color and side
        if (isNet && isRed) {
            drivePositions = drivePositionsNetRed;
        } else if (isNet && !isRed) {
            drivePositions = drivePositionsNetBlue;
        } else if (!isNet && isRed) {
            drivePositions = drivePositionsObsRed;
        } else {
            drivePositions = drivePositionsObsBlue;
        }

        //setting start position, 0 is x, 1 is y, 2 is heading
        robot.xFieldPos = drivePositions.get("start")[0];
        robot.yFieldPos = drivePositions.get("start")[1];
        robot.headingField = drivePositions.get("start")[2];
        robot.angleTracker.setOrientation(robot.headingField);
    }

    @Override
    public void loop() {

        double[] positionChange = robot.positionChange();
        robot.updateFieldPosition(positionChange[0], positionChange[1], positionChange[2]);
        telemetry.addData("Field Position (Coordinates)", "%.2f, %.2f, %.2f", robot.xFieldPos, robot.yFieldPos, robot.headingField);
        telemetry.addData("IMU Orientation", "IMU %.2f", robot.angleTracker.getOrientation());
        telemetry.addData("Next action", nextState);

        driveTo.sendTelemetry(telemetry);
        driveTo.updateDrive();

        if (!isBusy()) {
            //variable used in getDeclaredMethod cannot be changed within the loop because it is being used, so we create another variable so that we may change nextState!
            String currentState = nextState;
            try {
                // inspect our own class to see if we have an action method with the name we specified

                Method stateMethod = this.getClass().getMethod(currentState, new Class[]{});
                // invoke it with our class instance
                stateMethod.invoke(this);
                // catch exceptions to keep the compiler happy
            } catch (NoSuchMethodException exc) {
                // this will be caught when we haven't defined the method (e.g., for "done")
            } catch (IllegalAccessException exc) {
                // we don't expect this one
                telemetry.addData("exception", "IllegalAccessException when calling " + currentState + " " + exc);
            } catch (InvocationTargetException exc) {
                // we don't expect this one
                telemetry.addData("exception", "InvocationTargetException when calling " + currentState + " " +
                        exc.getTargetException() + " " + exc.getTargetException().getStackTrace()[0]);
            }
        }
    }

    @Override
    public void stop() {
        robot.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
    }

    //conditions that the robot is busy in
    Boolean isBusy() {
        //if robot isn't there yet its busy
        if (!driveTo.areWeThereYet) {
            return true;
        }
        /*
        You can check for other busy conditions like this

        if (robot.isLiftBusy()) {
            return true;
        }
        */
        if (getRuntime() < wait) {
            return true;
        }
        return false;
    }

    public void actionStart() {
        driveTo.setTargetPosition(drivePositions.get(isNet ? "Sample Score Pose" : "Obs Park"), 1, true);
        nextState = isNet ? "actionSampleScore" : "actionStop";
    }

    public void actionSampleScore() {
        wait = getRuntime() + 1;
        nextState = "actionAscentParkWay";
    }

    public void actionAscentParkWay() {
        driveTo.setTargetPosition(drivePositions.get("Ascent Park Waypoint"), 1, false);
        nextState = "actionAscentPark";
    }

    public void actionAscentPark() {
        driveTo.setTargetPosition(drivePositions.get("Ascent Park"), 1, false);
        nextState = "actionStop";
    }

    public void actionStop() {
        robot.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
        nextState = "actionDone";
    }
}