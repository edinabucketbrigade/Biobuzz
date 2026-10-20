package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Your teleop + a "go to (0, 0)" auto mode.
 *   Press A  -> robot slowly drives itself to X = 0 in, Y = 0 in (and heading 0).
 *   Press A again -> back to normal driver control.
 *
 * The robot's position comes from the SparkFun OTOS sensor, which must be
 * named "sensor_otos" in your robot configuration.
 * (0, 0) is wherever the robot was sitting when you pressed INIT.
 * Press Y at any time to re-zero the position to where the robot is now.
 */
@TeleOp(name = "Mec BioBuzz Teleop + Auto Toggle", group = "StarterBot")
public class FirstAuto extends OpMode {

    // ---------- Hardware ----------
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

    // ---------- Launcher ----------
    public final int LAUNCHER_TARGET_VELOCITY = 1400;
    public final int LAUNCHER_MIN_VELOCITY = 1350;

    // ---------- Auto "go to" tuning (change these!) ----------
    static final double TARGET_X_IN = 0;       // inches
    static final double TARGET_Y_IN = 0;       // inches
    static final double TARGET_HEADING_DEG = 0;

    // Flip any of these to -1 if the robot drives the wrong way on that axis
    static final double FORWARD_SIGN = -1;      // Y was flipped on this robot
    static final double STRAFE_SIGN = -1;

    static final double MAX_AUTO_POWER = 0.30;  // "slowly" -> raise to go faster
    static final double MIN_AUTO_POWER = 0.20;  // enough power to actually move
    static final double DRIVE_GAIN = 0.04;      // power per inch of error
    static final double TURN_GAIN = 0.015;      // power per degree of error
    static final double POSITION_TOLERANCE_IN = 0.75;
    static final double HEADING_TOLERANCE_DEG = 2.0;

    // ---------- State ----------
    boolean autoMode = false;
    boolean lastA = false;
    boolean atTarget = false;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;

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

        telemetry.addData("Status", "Initialized - keep robot still & place it at start (0,0)");
    }

    @Override
    public void loop() {
        // ----- A button toggles auto / driver control (edge-detected) -----
        if (gamepad1.a && !lastA) {
            autoMode = !autoMode;
            atTarget = false;
        }
        lastA = gamepad1.a;

        // Y re-zeros the position to wherever the robot is right now
        if (gamepad1.y) {
            otos.resetTracking();
        }

        SparkFunOTOS.Pose2D pos = otos.getPosition();

        if (autoMode) {
            driveToTarget(pos);
        } else {
            mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // ----- Intake + launcher (unchanged, works in both modes) -----
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        launch();
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // ----- Telemetry -----
        telemetry.addData("Mode", autoMode ? "AUTO (press A to cancel)" : "DRIVER (press A for auto)");
        if (autoMode) {
            telemetry.addData("At target", atTarget);
        }
        telemetry.addData("X (in)", "%.2f", pos.x);
        telemetry.addData("Y (in)", "%.2f", pos.y);
        telemetry.addData("Heading (deg)", "%.1f", pos.h);
    }

    /** Slowly drives toward (TARGET_X_IN, TARGET_Y_IN, TARGET_HEADING_DEG). */
    void driveToTarget(SparkFunOTOS.Pose2D pos) {
        double errX = TARGET_X_IN - pos.x;            // field-frame error, inches
        double errY = TARGET_Y_IN - pos.y;
        double errH = wrapDegrees(TARGET_HEADING_DEG - pos.h);

        double distance = Math.hypot(errX, errY);

        if (distance < POSITION_TOLERANCE_IN && Math.abs(errH) < HEADING_TOLERANCE_DEG) {
            atTarget = true;
            mecanumDrive(0, 0, 0);
            return;
        }
        atTarget = false;

        // Rotate the field-frame error into the robot's frame
        double h = Math.toRadians(pos.h);
        double strafeErr = errX * Math.cos(h) + errY * Math.sin(h);   // + = robot's right
        double forwardErr = -errX * Math.sin(h) + errY * Math.cos(h); // + = robot's front

        double forward = FORWARD_SIGN * clampWithMin(forwardErr * DRIVE_GAIN, POSITION_TOLERANCE_IN * DRIVE_GAIN);
        double strafe = STRAFE_SIGN * clampWithMin(strafeErr * DRIVE_GAIN, POSITION_TOLERANCE_IN * DRIVE_GAIN);

        // Positive "rotate" in mecanumDrive turns the robot clockwise,
        // but positive OTOS heading is counter-clockwise, so negate.
        double rotate = -errH * TURN_GAIN;
        if (Math.abs(errH) < HEADING_TOLERANCE_DEG) rotate = 0;
        rotate = Math.max(-MAX_AUTO_POWER, Math.min(MAX_AUTO_POWER, rotate));

        mecanumDrive(forward, strafe, rotate);
    }

    /** Limits to MAX_AUTO_POWER and bumps tiny commands up to MIN_AUTO_POWER so the robot actually moves. */
    double clampWithMin(double power, double deadband) {
        if (Math.abs(power) < deadband) return 0;
        double sign = Math.signum(power);
        double mag = Math.max(MIN_AUTO_POWER, Math.min(MAX_AUTO_POWER, Math.abs(power)));
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

    void launch() {
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // If the sensor isn't centered on the robot, put its offset here
        // (x = left(-)/right(+), y = back(-)/front(+), heading in degrees).
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();   // robot must be still during this (~0.6 s)
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }
}