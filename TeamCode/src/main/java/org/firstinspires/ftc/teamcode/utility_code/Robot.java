package org.firstinspires.ftc.teamcode.utility_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.LUDecomposition;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Robot {
    /* Public OpMode members. */
    public DcMotorEx DriveFrontLeft = null;
    public DcMotorEx DriveFrontRight = null;
    public DcMotorEx DriveRearLeft = null;
    public DcMotorEx DriveRearRight = null;
    public DcMotorEx lift = null;
    public Servo basket = null;

    /**
     * FORWARD_ENCODER_COUNTS_PER_INCH, RIGHT_ENCODER_COUNTS_PER_INCH, CW_ENCODER_COUNTS_PER_DEGREE are used when using wheel encoders (not odometry pods)
     * <p>
     * We drove 120 inches or 3600 degrees 3 times. These were the wheel encoder counts for each of the wheels (keep them all positive).
     * We divide by the distance traveled to get the average encoder count per unit per wheel.
     * You might get negative encoder values but use the absolute value in the equation.
     */
    /**
     * Total left odometry pod encoder count when traveling a decided forward distance (IF its negative, keep the negative sign)
     */
    static double LEFT_ENCODER_FORWARD_VALUE = -47837 + -47978 + -47925 + -47989;
    /**
     * Total right odometry pod encoder count when traveling a decided forward distance (IF its negative, keep the negative sign)
     */
    static double RIGHT_ENCODER_FORWARD_VALUE = 48145 + 48163 + 48117 + 48070;
    /**
     * Total middle odometry pod encoder count when traveling a decided forward distance (IF its negative, keep the negative sign)
     */
    static double MIDDLE_ENCODER_FORWARD_VALUE = -602 + -304 + -275 + -282;
    /**
     * Decided distance from encoder forward value tests (we drove forward 96in) times number of tests (in inches)
     */
    static double FORWARD_TRAVEL = 96 * 4;

    //vals are whatever, left, right, middle
    /**
     * Total left odometry pod encoder count when traveling a decided strafe distance (IF its negative, keep the negative sign)
     */
    static double LEFT_ENCODER_STRAFE_VALUE = 326 + 148 + 193 + 438;
    /**
     * Total right odometry pod encoder count when traveling a decided strafe distance (IF its negative, keep the negative sign)
     */
    static double RIGHT_ENCODER_STRAFE_VALUE = 102 + -173 + -67 + 62;
    /**
     * Total middle odometry pod encoder count when traveling a decided strafe distance (IF its negative, keep the negative sign)
     */
    static double MIDDLE_ENCODER_STRAFE_VALUE = -48175 + -48011 + -48200 + -48155;
    /**
     * Decided distance from encoder strafe value tests (we strafed right 96in) times number of tests (in inches)
     */
    static double STRAFE_TRAVEL = 96 * 4;
    /**
     * Total left odometry pod encoder count when spinning a decided amount (IF its negative, keep the negative sign)
     */
    static double LEFT_ENCODER_CW_TURN = -69604 + -67945 + -67228 + -66051;
    /**
     * Total right odometry pod encoder count when spinning a decided amount (IF its negative, keep the negative sign)
     */
    static double RIGHT_ENCODER_CW_TURN = -130500 + -132140 + -132853 + -133990;
    /**
     * Total middle odometry pod encoder count when spinning a decided amount (IF its negative, keep the negative sign)
     */
    static double MIDDLE_ENCODER_CW_TURN = 194724 + 194350 + 195156 + 195039;
    /**
     * Decided amount from encoder spinning value tests [we spun clockwise (CW) ten times (3600 degrees)] times number of tests (in inches)
     */
    static double CW_TURN_DEGREES = 3600 + 3600 + 3600 + 3600;
    static Array2DRowRealMatrix odomat = new Array2DRowRealMatrix(new double[][]{
            {LEFT_ENCODER_FORWARD_VALUE, MIDDLE_ENCODER_FORWARD_VALUE, RIGHT_ENCODER_FORWARD_VALUE},
            {LEFT_ENCODER_STRAFE_VALUE, MIDDLE_ENCODER_STRAFE_VALUE, RIGHT_ENCODER_STRAFE_VALUE},
            {LEFT_ENCODER_CW_TURN, MIDDLE_ENCODER_CW_TURN, RIGHT_ENCODER_CW_TURN}
    });
    public static double[] FORWARD_PARAMS = new LUDecomposition(odomat).getSolver().solve(new ArrayRealVector(new double[]{FORWARD_TRAVEL, 0, 0})).toArray();
    public static double[] STRAFE_PARAMS = new LUDecomposition(odomat).getSolver().solve(new ArrayRealVector(new double[]{0, STRAFE_TRAVEL, 0})).toArray();
    public static double[] CW_TURN_PARAMS = new LUDecomposition(odomat).getSolver().solve(new ArrayRealVector(new double[]{0, 0, CW_TURN_DEGREES})).toArray();
    public AngleTrackerIMU angleTracker;

    int flLastPosition = 0;
    int frLastPosition = 0;
    int rlLastPosition = 0;
    int rrLastPosition = 0;
    public double xFieldPos = 0, yFieldPos = 0, headingField = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public Robot() {

    }

    public DcMotorEx setUpEncoderMotor(
            String identifier, DcMotor.Direction direction,
            double pidf_p, double pidf_i, double pidf_d, double pidf_f) {
        return setUpEncoderMotor(identifier, direction, pidf_p, pidf_i, pidf_d, pidf_f, false);
    }

    @SuppressWarnings("SameParameterValue")
    public DcMotorEx setUpEncoderMotor(
            String identifier, DcMotor.Direction direction,
            double pidf_p, double pidf_i, double pidf_d, double pidf_f, boolean withEncoder) {
        DcMotorEx drive = hwMap.get(DcMotorEx.class, identifier);
        drive.setDirection(direction);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setZeroPowerBehavior(BRAKE);
        drive.setPower(0);
        // we might want to make the default RUN_TO_POSITION.  RUN_TO_POSITION
        // targets a specific final position of the motor based on its internal
        // encoders.  RUN_USING_ENCODER targets a specific velocity using the
        // encoders.
        if (!withEncoder) {
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients drivePidf = new PIDFCoefficients(pidf_p, pidf_i, pidf_d, pidf_f);
            drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drivePidf);
        }

        return drive;
    }

    public void initWheelHardware() {
        //was 12,10,0,0
        DriveFrontLeft = setUpEncoderMotor("fl", DcMotor.Direction.FORWARD, 12, 10, 0.0, 5.0, false);
        DriveFrontRight = setUpEncoderMotor("fr", DcMotor.Direction.REVERSE, 12, 10, 0.0, 5.0, false);
        DriveRearLeft = setUpEncoderMotor("rl", DcMotor.Direction.FORWARD, 12, 10, 0.0, 5.0, false);
        DriveRearRight = setUpEncoderMotor("rr", DcMotor.Direction.REVERSE, 12, 10, 0.0, 5.0, false);
    }

    /**
     * Initializes the angle tracker (IMU)
     */
    public void initTracker() {
        // angleTracker = new AngleTrackerIMU(this);
        angleTracker = new AngleTrackerIMU(hwMap.get(IMU.class, "imu"));
    }

    public void initOdometry() {

    }

    public void initOtherHardware() {
        lift = setUpEncoderMotor("lift", DcMotor.Direction.REVERSE, 12, 10, 0.0, 5.0, true);
        basket = hwMap.get(Servo.class, "basket");
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param ahwMap     Hardware map from op mode
     * @param initWheels Boolean that is true as long as you want the wheels to move
     */
    public void init(HardwareMap ahwMap, boolean initWheels) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        if (initWheels) {
            initWheelHardware();
            initOdometry();
            initTracker();
        }
        initOtherHardware();
    }

    public void init(HardwareMap ahwMap) {
        init(ahwMap, true);
    }

    /**
     * Calculate speed for the wheels, sets the power for the wheels, and sending telemetry
     * This is for mechanum robots
     *
     * @param forward     speed forward [-1, 1]
     * @param strafeRight speed right [-1, 1]
     * @param turnCW      speed turn [-1, 1]
     * @param telemetry
     */
    public void drive(double forward, double strafeRight, double turnCW, Telemetry telemetry) {

        double speedfr = forward + strafeRight - turnCW;
        double speedfl = forward + strafeRight + turnCW;
        double speedrl = forward - strafeRight + turnCW;
        double speedrr = forward - strafeRight - turnCW;

        double max = Math.max(Math.max(Math.abs(speedfl), Math.abs(speedfr)), Math.max(Math.abs(speedrl), Math.abs(speedrr)));

        if (max > 1) { //Use abs value so we don't divide a neg by a neg and have it go forward unexpectedly
            speedfl /= max;
            speedfr /= max;
            speedrl /= max;
            speedrr /= max;
        }

        DriveFrontLeft.setPower(speedfl);
        DriveRearLeft.setPower(speedrl);
        DriveFrontRight.setPower(speedfr);
        DriveRearRight.setPower(speedrr);

        telemetry.addData("position", "%9d:%9d:%9d:%9d",
                DriveFrontLeft.getCurrentPosition(),
                DriveFrontRight.getCurrentPosition(),
                DriveRearLeft.getCurrentPosition(),
                DriveRearRight.getCurrentPosition()
        );
        telemetry.addData("speed fl, fr, rl, rr", "%5.2f:%5.2f:%5.2f:%5.2f",
                speedfl, speedfr, speedrl, speedrr);
        telemetry.addData("v1", DriveFrontLeft.getVelocity());
        telemetry.addData("v2", DriveFrontRight.getVelocity());
        telemetry.addData("v3", DriveRearLeft.getVelocity());
        telemetry.addData("v4", DriveRearRight.getVelocity());

    }


    /**
     * How far the robot has moved since it last checked
     *
     * @return array of forward (in inches), strafeRight (in inches), and turnCW (in degrees)
     */
    public double[] positionChange() {
        int flPosition = DriveFrontLeft.getCurrentPosition();
        int frPosition = DriveFrontRight.getCurrentPosition();
        int rlPosition = DriveRearLeft.getCurrentPosition();
        int rrPosition = DriveRearRight.getCurrentPosition();

        //how much did we move since we last asked
        int flPositionChange = flPosition - flLastPosition;
        int frPositionChange = frPosition - frLastPosition;
        int rlPositionChange = rlPosition - rlLastPosition;
        int rrPositionChange = rrPosition - rrLastPosition;

        int rightPod = rlPositionChange;
        int leftPod = rrPositionChange;
        int midPod = frPositionChange;

        //how far has robot moved, how many inches and degrees changed (forward, strafe, turn)
        double forwardDistance = FORWARD_PARAMS[0] * leftPod + FORWARD_PARAMS[1] * midPod + FORWARD_PARAMS[2] * rightPod;
        double rightDistance = STRAFE_PARAMS[0] * leftPod + STRAFE_PARAMS[1] * midPod + STRAFE_PARAMS[2] * rightPod;
        double cwTurnAngle = CW_TURN_PARAMS[0] * leftPod + CW_TURN_PARAMS[1] * midPod + CW_TURN_PARAMS[2] * rightPod;

        //updating new last position
        flLastPosition = flPosition;
        frLastPosition = frPosition;
        rlLastPosition = rlPosition;
        rrLastPosition = rrPosition;

        //this is where the IMU thinks heading is
        headingField = angleTracker.getOrientation();

        return new double[]{forwardDistance, rightDistance, cwTurnAngle};
    }

    /**
     * @param forward forward travel distance (in inches)
     * @param right   right travel distance (in inches)
     * @param cwTurn  turn travel angle (in degrees)
     */
    public void updateFieldPosition(double forward, double right, double cwTurn) {
        //same as double averageHeading = (headingField + (headingField - cwTurn)) / 2;
        double avgHeading = headingField - cwTurn / 2;
        headingField -= cwTurn;
        //effect of forward movement
        xFieldPos += forward * Math.cos(Math.toRadians(avgHeading));
        yFieldPos += forward * Math.sin(Math.toRadians(avgHeading));
        //effect of right movement
        xFieldPos += right * Math.sin(Math.toRadians(avgHeading));
        yFieldPos -= right * Math.cos(Math.toRadians(avgHeading));

    }

    /**
     * Compute the distance the robot needs to travel from the current field position to a target
     * position.
     *
     * @param xTargetPos    Target position in field coordinates in inches.
     * @param yTargetPos    Target position in field coordinates in inches.
     * @param headingTarget Target heading in field coordinates in degrees.
     * @return Distances to travel forward in inches, right in inches, and turn clockwise in degrees.
     */
    public double[] getTravelValues(double xTargetPos, double yTargetPos, double headingTarget) {
        double deltaX = xTargetPos - xFieldPos;
        double deltaY = yTargetPos - yFieldPos;
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double fieldBearing = Math.atan2(deltaY, deltaX);
        double robotBearing = fieldBearing - Math.toRadians(headingField);
        double forwardTravel = Math.cos(robotBearing) * distance;
        double leftTravel = Math.sin(robotBearing) * distance;
        //if rightTravel is positive go right and if leftTravel is positive go left
        double rightTravel = -leftTravel;
        double cwTurnAngle = headingField - headingTarget;
        //to get angle between 0-360
        cwTurnAngle = ((cwTurnAngle % 360) + 360) % 360;
        //to turn the shortest possible distance
        if (cwTurnAngle > 180) {
            cwTurnAngle -= 360;
        }
        return new double[]{forwardTravel, rightTravel, cwTurnAngle};

    }

    /**
     * @param angle1 in degrees
     * @param angle2 in degrees
     * @return angle 1 - angle 2 out of 360 (in degrees)
     */
    public double angleDifference(double angle1, double angle2) {
        double angleDiff = angle1 - angle2;
        //to get angle between 0-360
        angleDiff = ((angleDiff % 360) + 360) % 360;
        //to turn the shortest possible distance
        if (angleDiff > 180) {
            angleDiff -= 360;
        }
        return angleDiff;
    }

    /**
     * @return maximum drive speed out of all the motors
     */
    public double currentAbsDriveSpeed() {
        return Math.max(
                Math.max(Math.abs(DriveFrontLeft.getPower()), Math.abs(DriveFrontRight.getPower())),
                Math.max(Math.abs(DriveRearLeft.getPower()), Math.abs(DriveRearRight.getPower()))
        );
    }
}
