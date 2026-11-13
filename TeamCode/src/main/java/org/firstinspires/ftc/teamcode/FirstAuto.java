package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Multi-Waypoint Autonomous", group = "StarterBot")
public class FirstAuto extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private SparkFunOTOS otos = null;

    static final int LAUNCHER_TARGET_VELOCITY = 1250;
    static final int LAUNCHER_MIN_VELOCITY = 1200;

    Waypoint[] path = {
            new Waypoint(0.0,  5.0,   0.0, 0.35, false, 0.0),
            new Waypoint(0.0,  0.0,   0.0, 0.35, true,  3.0),
            new Waypoint(0.0, -5.0,   0.0, 0.35, false, 0.0)
    };

    int currentWaypointIndex = 0;
    int lastProcessedWaypoint = -1;
    boolean autonomousFinished = false;

    double absoluteTargetX = 0.0;
    double absoluteTargetY = 0.0;
    double absoluteTargetHeading = 0.0;

    double waitStartTime = 0.0;
    boolean isWaiting = false;

    static final double FORWARD_SIGN = -1.0;
    static final double STRAFE_SIGN = -1.0;

    static final double MIN_AUTO_POWER = 0.20;
    static final double DRIVE_GAIN = 0.04;
    static final double TURN_GAIN = 0.015;
    static final double POSITION_TOLERANCE_IN = 0.75;
    static final double HEADING_TOLERANCE_DEG = 2.0;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public static class Waypoint {
        public double deltaX, deltaY, heading, maxPower, waitTimeSec;
        public boolean shoot;

        public Waypoint(double deltaX, double deltaY, double heading, double maxPower, boolean shoot, double waitTimeSec) {
            this.deltaX = deltaX;
            this.deltaY = deltaY;
            this.heading = heading;
            this.maxPower = maxPower;
            this.shoot = shoot;
            this.waitTimeSec = waitTimeSec;
        }
    }

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        configureOtos();

        telemetry.addData("Status", "Initialized - Ready for Waypoints");
        telemetry.update();
    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();

        if (!autonomousFinished) {
            Waypoint wp = path[currentWaypointIndex];

            if (currentWaypointIndex != lastProcessedWaypoint) {
                absoluteTargetX += wp.deltaX;
                absoluteTargetY += wp.deltaY;
                absoluteTargetHeading = wp.heading;
                lastProcessedWaypoint = currentWaypointIndex;
                isWaiting = false;
            }

            if (!isWaiting) {
                boolean reachedWaypoint = driveToTarget(pos, absoluteTargetX, absoluteTargetY, absoluteTargetHeading, wp.maxPower);

                if (reachedWaypoint) {
                    if (wp.waitTimeSec > 0 || wp.shoot) {
                        isWaiting = true;
                        waitStartTime = getRuntime();
                    } else {
                        currentWaypointIndex++;
                        if (currentWaypointIndex >= path.length) {
                            autonomousFinished = true;
                        }
                    }
                }
            } else {
                mecanumDrive(0, 0, 0);

                if (wp.shoot) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                        windmillServo.setPower(1.0);
                        intake.setPower(0.5);
                        leftIntakeServo.setPower(0.5);
                        rightIntakeServo.setPower(0.5);
                    } else {
                        windmillServo.setPower(0);
                        intake.setPower(0);
                        leftIntakeServo.setPower(0);
                        rightIntakeServo.setPower(0);
                    }
                } else {
                    launcher.setVelocity(0);
                    windmillServo.setPower(0);
                    intake.setPower(0);
                    leftIntakeServo.setPower(0);
                    rightIntakeServo.setPower(0);
                }

                if ((getRuntime() - waitStartTime) >= wp.waitTimeSec) {
                    launcher.setVelocity(0);
                    windmillServo.setPower(0);
                    intake.setPower(0);
                    leftIntakeServo.setPower(0);
                    rightIntakeServo.setPower(0);

                    isWaiting = false;
                    currentWaypointIndex++;
                    if (currentWaypointIndex >= path.length) {
                        autonomousFinished = true;
                    }
                }
            }
        } else {
            mecanumDrive(0, 0, 0);
            launcher.setVelocity(0);
            windmillServo.setPower(0);
            intake.setPower(0);
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
        }

        telemetry.addData("Status", autonomousFinished ? "Autonomous Complete!" : "Running Waypoint " + (currentWaypointIndex + 1) + " of " + path.length);
        if (!autonomousFinished) {
            telemetry.addData("Target X", absoluteTargetX);
            telemetry.addData("Target Y", absoluteTargetY);
            telemetry.addData("Shooting?", path[currentWaypointIndex].shoot);
            telemetry.addData("Waiting?", isWaiting);
        }
        telemetry.addData("Current X (in)", "%.2f", pos.x);
        telemetry.addData("Current Y (in)", "%.2f", pos.y);
        telemetry.addData("Heading (deg)", "%.1f", pos.h);
        telemetry.update();
    }

    boolean driveToTarget(SparkFunOTOS.Pose2D pos, double targetX, double targetY, double targetHeading, double maxPower) {
        double errX = targetX - pos.x;
        double errY = targetY - pos.y;
        double errH = wrapDegrees(targetHeading - pos.h);

        double distance = Math.hypot(errX, errY);

        if (distance < POSITION_TOLERANCE_IN && Math.abs(errH) < HEADING_TOLERANCE_DEG) {
            mecanumDrive(0, 0, 0);
            return true;
        }

        double h = Math.toRadians(pos.h);
        double strafeErr = errX * Math.cos(h) + errY * Math.sin(h);
        double forwardErr = -errX * Math.sin(h) + errY * Math.cos(h);

        double deadbandLimit = POSITION_TOLERANCE_IN * DRIVE_GAIN;
        double forward = FORWARD_SIGN * clampWithMin(forwardErr * DRIVE_GAIN, deadbandLimit, maxPower);
        double strafe = STRAFE_SIGN * clampWithMin(strafeErr * DRIVE_GAIN, deadbandLimit, maxPower);

        double rotate = -errH * TURN_GAIN;
        if (Math.abs(errH) < HEADING_TOLERANCE_DEG) rotate = 0;
        rotate = Math.max(-maxPower, Math.min(maxPower, rotate));

        mecanumDrive(forward, strafe, rotate);
        return false;
    }

    double clampWithMin(double power, double deadband, double maxPower) {
        if (Math.abs(power) < deadband) return 0;
        double sign = Math.signum(power);
        double mag = Math.max(MIN_AUTO_POWER, Math.min(maxPower, Math.abs(power)));
        return sign * mag;
    }

    double wrapDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }
}