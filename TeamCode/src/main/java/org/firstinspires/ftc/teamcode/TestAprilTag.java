package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * Your teleop + a "go to (0, 0)" auto mode + camera on the dashboard + keyboard control.
 *
 * KEYBOARD CONTROL
 *   1. Press INIT on the Driver Hub.
 *   2. On your laptop (on the robot's Wi-Fi) open  http://192.168.43.1:8090
 *   3. Keep that tab focused and use the keys:
 *        W/S forward/back   A/D strafe   Q/E turn
 *        I/K intake in/out  Space = feed launcher
 *        F = flywheel on/off   G = auto go-to-(0,0) on/off   R = re-zero position
 *   The physical gamepad still works too (inputs are added together).
 *   If the page stops sending (tab closed, Wi-Fi drop, tab unfocused), the robot stops.
 *
 * GAMEPAD
 *   Press A  -> robot slowly drives itself to X = 0 in, Y = 0 in (and heading 0).
 *   Press A again -> back to normal driver control.
 *   Right bumper -> feeds balls (windmill) once the flywheel is up to speed.
 *   Press B      -> toggle the flywheel off/on mid-match.
 *   Press Y      -> re-zero the position to where the robot is now.
 *
 * The flywheel/launcher spins up automatically when you press PLAY and
 * stays on until you press STOP on the Driver Hub.
 *
 * The robot's position comes from the SparkFun OTOS sensor, which must be
 * named "sensor_otos" in your robot configuration.
 * (0, 0) is wherever the robot was sitting when you pressed INIT.
 *
 * Camera: the webcam (named CAMERA_NAME in your robot configuration) is streamed
 * to FTC Dashboard at http://192.168.43.1:8080/dash
 */
@Config
@TeleOp(name = "NEWEST", group = "MainTeleOps") public class TestAprilTag extends OpMode {
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

    // ---------- Camera ----------
    private VisionPortal visionPortal = null;
    private CameraStreamProcessor cameraProcessor = null;
    static final String CAMERA_NAME = "Webcam 1";   // must match your robot configuration!
    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 480;
    static final int CAMERA_DASHBOARD_FPS = 15;     // lower this if the stream lags

    // ---------- Keyboard control ----------
    private KeyboardServer keyboard = null;
    private final ElapsedTime loopTimer = new ElapsedTime();
    public static double KB_RAMP_PER_SEC = 3.0;     // power per second when speeding up (3.0 = full power in ~0.33 s)
    public static double KB_BRAKE_MULTIPLIER = 3.0; // slowing down is this many times faster than speeding up
    private double kbForward = 0;
    private double kbStrafe = 0;
    private double kbRotate = 0;

    // ---------- Launcher ----------
    public static int LAUNCHER_TARGET_VELOCITY = 1400;
    public static int LAUNCHER_MIN_VELOCITY = 1350;
    public static boolean TEST = false;

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

    boolean flywheelOn = true;     // flywheel runs from Play until Stop (B toggles it)
    boolean lastB = false;

    boolean isTargetReset = false;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        configureCamera();

        keyboard = new KeyboardServer();
        keyboard.start();

        telemetry.addData("Status", "Initialized - keep robot still & place it at start (0,0)");
        telemetry.addData("Keyboard page", "http://192.168.43.1:" + KeyboardServer.PORT);
        telemetry.update();
    }

    /**
     * Starts the webcam and sends its frames to FTC Dashboard.
     */
    private void configureCamera() {
        cameraProcessor = new CameraStreamProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new android.util.Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(cameraProcessor)
                .enableLiveView(false)   // skip the Driver Hub preview; the dashboard gets the stream
                .build();

        FtcDashboard.getInstance().startCameraStream(cameraProcessor, CAMERA_DASHBOARD_FPS);
    }

    /**
     * Runs once when the driver presses PLAY: spin up the flywheel.
     */
    @Override
    public void start() {
        flywheelOn = true;
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        // ----- Read the keyboard (page must be open and sending, otherwise everything is 0) -----
        boolean kbActive = keyboard.isActive();
        int keys = kbActive ? keyboard.getMask() : 0;
        double kbSpeed = keyboard.getSpeed();

        boolean kbAuto = (keys & KeyboardServer.KEY_AUTO) != 0;
        boolean kbFlywheel = (keys & KeyboardServer.KEY_FLYWHEEL) != 0;
        boolean kbRezero = (keys & KeyboardServer.KEY_REZERO) != 0;
        boolean kbFeed = (keys & KeyboardServer.KEY_FEED) != 0;

        // ----- A button / G key toggles auto / driver control (edge-detected) -----
        boolean aNow = gamepad1.a || kbAuto;
        if (aNow && !lastA) {
            autoMode = !autoMode;
            atTarget = false;
        }
        lastA = aNow;

        // Y button / R key re-zeros the position to wherever the robot is right now
        if (gamepad1.y || kbRezero) {
            otos.resetTracking();
            isTargetReset = true;
        }

        SparkFunOTOS.Pose2D pos = otos.getPosition();

        // ----- Keyboard drive values, ramped so the robot doesn't jerk -----
        if (kbActive) {
            kbForward = ramp(kbForward, axis(keys, KeyboardServer.KEY_FORWARD, KeyboardServer.KEY_BACK) * kbSpeed, dt);
            kbStrafe = ramp(kbStrafe, axis(keys, KeyboardServer.KEY_STRAFE_R, KeyboardServer.KEY_STRAFE_L) * kbSpeed, dt);
            kbRotate = ramp(kbRotate, axis(keys, KeyboardServer.KEY_TURN_R, KeyboardServer.KEY_TURN_L) * kbSpeed, dt);
        } else {
            // Page closed / connection lost: stop immediately.
            kbForward = 0;
            kbStrafe = 0;
            kbRotate = 0;
        }

        if (autoMode) {
            driveToTarget(pos);
        } else {
            mecanumDrive(
                    -gamepad1.left_stick_y + kbForward,
                    gamepad1.left_stick_x + kbStrafe,
                    gamepad1.right_stick_x + kbRotate);
        }

        // ----- B button / F key toggles the flywheel on/off (edge-detected, optional) -----
        boolean bNow = gamepad1.b || kbFlywheel;
        if (bNow && !lastB) {
            flywheelOn = !flywheelOn;
        }
        lastB = bNow;

        // ----- Intake + launcher (works in both modes) -----
        double kbIntake = axis(keys, KeyboardServer.KEY_INTAKE_IN, KeyboardServer.KEY_INTAKE_OUT);
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger + kbIntake;
        launch(gamepad1.right_bumper || kbFeed);
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // ----- Telemetry -----
        telemetry.addData("Mode", autoMode ? "AUTO (press A or G to cancel)" : "DRIVER (press A or G for auto)");
        if (keyboard.getError() != null) {
            telemetry.addData("KEYBOARD SERVER ERROR", keyboard.getError());
        }
        telemetry.addData("Keyboard", kbActive ? "connected, speed " + Math.round(kbSpeed * 100) + "%" : "no page connected");
        if (autoMode) {
            telemetry.addData("At target", atTarget);
        }
        telemetry.addData("Flywheel on", flywheelOn);
        telemetry.addData("Flywheel velocity", "%.0f", launcher.getVelocity());
        telemetry.addData("X (in)", "%.2f", pos.x);
        telemetry.addData("Y (in)", "%.2f", pos.y);
        telemetry.addData("Heading (deg)", "%.1f", pos.h);
        telemetry.addData("is target set?", isTargetReset);
        telemetry.addData("Camera", visionPortal != null ? visionPortal.getCameraState() : "not started");
        telemetry.update();   // needed because telemetry is now a MultipleTelemetry wrapper
    }

    /**
     * Runs once when the driver presses STOP.
     */
    @Override
    public void stop() {
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        mecanumDrive(0, 0, 0);

        if (keyboard != null) {
            keyboard.stop();
        }
        FtcDashboard.getInstance().stopCameraStream();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /** +1 if the "positive" key is held, -1 if the "negative" key is held, 0 if both or neither. */
    private double axis(int keys, int positiveBit, int negativeBit) {
        double v = 0;
        if ((keys & positiveBit) != 0) v += 1;
        if ((keys & negativeBit) != 0) v -= 1;
        return v;
    }

    /** Moves "current" toward "target" no faster than the ramp rate (and brakes faster than it accelerates). */
    private double ramp(double current, double target, double dt) {
        boolean slowingDown = Math.abs(target) < Math.abs(current);
        double step = KB_RAMP_PER_SEC * dt * (slowingDown ? KB_BRAKE_MULTIPLIER : 1.0);
        if (target > current) return Math.min(target, current + step);
        return Math.max(target, current - step);
    }

    /**
     * Slowly drives toward (TARGET_X_IN, TARGET_Y_IN, TARGET_HEADING_DEG).
     */
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

    /**
     * Limits to MAX_AUTO_POWER and bumps tiny commands up to MIN_AUTO_POWER so the robot actually moves.
     */
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

    /**
     * Flywheel stays spinning (started in start()). The feed button (right bumper or Space)
     * only runs the windmill feeder once the flywheel is up to speed.
     */
    void launch(boolean feedPressed) {
        launcher.setVelocity(flywheelOn ? LAUNCHER_TARGET_VELOCITY : 0);

        if (flywheelOn && feedPressed && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
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