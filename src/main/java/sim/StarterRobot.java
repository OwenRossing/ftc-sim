package sim;

import java.util.Arrays;
import java.util.List;

/**
 * The goBILDA FTC StarterBot for 2026-27, in both versions goBILDA sells. Its hardware config matches goBILDA's
 * example code for each exactly, so their OpModes run here unchanged.
 *
 * "starter": the StarterBot with mecanum wheels (3200-2627-0004), on a Strafer chassis:
 *   left_front_drive, right_front_drive,   4 mecanum wheels (104 mm), a 312 rpm motor each, left side reversed
 *   left_back_drive, right_back_drive
 *   windmillServo                          the windmill (named differently from the tank version's)
 *
 * "starter-tank": the standard StarterBot the FTC Starter Kit builds (3200-2627-0003):
 *   left_drive, right_drive   6 wheel tank drive, one 312 rpm motor per side belted to 96 mm wheels (omnis in front)
 *   windmill
 *
 * Both:
 *   intake                    a 117 rpm motor on the 48 mm Gecko wheels across the front
 *   left_intake_servo,        continuous 2000 series servos with flat 72 mm wheels on the front corners, they pull
 *   right_intake_servo        pollen out of corners and out of the bottom of a flower
 *   launcher                  a 312 rpm motor straight on a 96 mm wheel, run with setVelocity (ticks per second)
 *   windmill                  a continuous servo that feeds the hopper's balls into the launcher
 *
 * It holds 4 balls and lobs them up into a Hive cell, so it starts facing the red Hive's upper cell, where
 * goBILDA's auto (shoot from the start, back up 12 cm) scores. Sizes and parts from goBILDA's CAD and models
 * and their code (537.7 ticks/rev, 402 mm track). Speeds and the shot are guesses (TUNE against the real robot).
 */
public final class StarterRobot extends SimRobot {

    public static final String LEFT_DRIVE = "left_drive";
    public static final String RIGHT_DRIVE = "right_drive";
    public static final String LEFT_FRONT = "left_front_drive";
    public static final String RIGHT_FRONT = "right_front_drive";
    public static final String LEFT_BACK = "left_back_drive";
    public static final String RIGHT_BACK = "right_back_drive";
    public static final String INTAKE = "intake";
    public static final String LEFT_INTAKE_SERVO = "left_intake_servo";
    public static final String RIGHT_INTAKE_SERVO = "right_intake_servo";
    public static final String LAUNCHER = "launcher";

    final boolean mecanum;
    final String windmill; // the two versions' example code name it differently

    // intake: 117 rpm 5203 (50.9:1) driving the Gecko shaft
    static final double INTAKE_RPM = 117;
    static final double INTAKE_TAU = 0.08;
    static final double MOUTH_X_MIN = 9; // where it grabs a ball, inches in front of center (the Gecko wheels are at 7.4)
    static final double MOUTH_X_MAX = 13;
    static final double MOUTH_HALF_WIDTH = 6.5;
    static final double MOUTH_Z_MAX = 5;
    static final double CORNER_REACH_X = 14.5; // the corner wheels stick out to 10.2, a bit past that into a flower
    static final double CAPTURE_COOLDOWN_SEC = 0.15;

    // launcher: a 312 rpm 5203 (19.2:1, 537.7 ticks/rev) straight on a 96 mm wheel, from the CAD. 1250 ticks/s is
    // 139 rpm, a 0.7 m/s rim, which can't throw a ball into a Hive by itself, so the real robot must do something
    // the CAD doesn't show. Until it's measured, the ball's speed scales with the encoder so goBILDA's 1250 ticks/s
    // throws about 6 m/s, which scores from the start pose (at 80 deg, 5.5-6.5 m/s lands in the upper cell from a
    // launch point anywhere from x 24 to 32). TUNE: film a shot
    static final double LAUNCHER_LOADED_RPM = 290; // at full power with the wheel on
    static final double LAUNCHER_TICKS_PER_REV = 537.7;
    static final double LAUNCHER_TAU = 0.25;
    static final double SHOT_RPM_LOSS = 20; // what one ball takes out of it
    static final double SPEED_AT_1250 = 6.0; // m/s
    static final double SHOT_ELEVATION_DEG = 80; // a steep lob, it has to drop into the cell from above. TUNE
    final double barrelForward; // the top of the launcher wheel, robot frame, from the CAD
    static final double BARREL_HEIGHT = 10;
    static final double FEED_SECONDS_PER_BALL = 0.3; // the windmill at full power
    static final double MIN_LAUNCH_SPEED = 3; // m/s, under this it just falls out

    double intakeRpm = 0;
    double launcherRpm = 0;
    private double feedTimer = 0;
    private double outtakeTimer = 0;
    private double captureCooldown = 0;

    /** The mecanum StarterBot, robot=starter */
    public StarterRobot() {
        this(true);
    }

    /** true: the mecanum StarterBot (robot=starter). false: the 6 wheel tank one (robot=starter-tank) */
    public StarterRobot(boolean mecanum) {
        this.mecanum = mecanum;
        windmill = mecanum ? "windmillServo" : "windmill";
        add(Kind.MOTOR, wheels());
        add(Kind.MOTOR, INTAKE, LAUNCHER);
        add(Kind.CRSERVO, LEFT_INTAKE_SERVO, RIGHT_INTAKE_SERVO, windmill);
        if (mecanum) {
            // a Strafer: 104 mm GripForce mecanum wheels, a 312 rpm motor on each, from goBILDA's model
            massKg = 10.4; // 23 lb, TUNE: weigh it
            wheelRadiusM = 0.052;
            trackIn = 16.3;
            wheelbaseIn = 10.4;
            barrelForward = -2.05;
        } else {
            tankDrive = true;
            massKg = 9.5; // 21 lb, TUNE: weigh it
            wheelRadiusM = 0.048; // 96 mm
            trackIn = 402 / 25.4;
            wheelbaseIn = 11.3; // front to back wheel, from the CAD
            // one motor per side drives 3 wheels, and each side is listed twice in wheels(): half each
            motorStallNm = 2.38 / 2;
            motorStallAmps = 9.2 / 2;
            barrelForward = -1.6;
        }
        // facing the red Hive's upper cell, with the launcher about 30 in back from its frame: where a lob at
        // goBILDA's launcher speed drops in (right against the frame, it can't get the arc over the cell's rim)
        startX = 28.4 - barrelForward; // the launch point at x 28.4, where the shot was tuned
        startY = 54;
        startHeading = 0;
    }

    @Override
    public String displayName() {
        return mecanum ? "goBILDA StarterBot (mecanum)" : "goBILDA StarterBot (tank)";
    }

    @Override
    public String modelFile() {
        return mecanum ? "robots/starter.json" : "robots/starter-tank.json";
    }

    @Override
    public String[] wheels() {
        return mecanum ? new String[]{LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK}
                : new String[]{LEFT_DRIVE, RIGHT_DRIVE, LEFT_DRIVE, RIGHT_DRIVE};
    }

    @Override
    public List<double[]> outline() {
        if (mecanum) {
            return Arrays.asList(
                    // from goBILDA's model: 17.8 in square with the wheels, corner wheels out to 9.8 in
                    new double[]{0.4, 0, 4, 8.6, 8.9, 3.5},
                    new double[]{2.1, 0, 9.5, 1.9, 4, 2.5}, // the hopper on top, up to 12 in, in front of the launch point
                    new double[]{9.4, 0, 1.6, 0.4, 7.5, 1.3});
        }
        return Arrays.asList(
                // from the CAD: the frame runs 7.6 in back to 9.5 in front and 16.3 in wide
                new double[]{0.95, 0, 4, 8.55, 8.15, 3.5},
                new double[]{2.1, 0, 9.5, 1.9, 4, 2.5}, // the hopper on top, up to 12 in, in front of the launch point
                new double[]{9.85, 0, 1.9, 0.35, 7.5, 1.4}); // the corner wheels, out to 10.2 in
    }

    @Override
    public int mount(String device) {
        // how goBILDA's code sets directions is how they're mounted: the mecanum code reverses the left side,
        // the tank code the right side, and both reverse right_intake_servo and the windmill
        if (device.equals(RIGHT_INTAKE_SERVO) || device.equals(windmill)) return -1;
        if (mecanum) return device.equals(LEFT_FRONT) || device.equals(LEFT_BACK) ? -1 : 1;
        return device.equals(RIGHT_DRIVE) ? -1 : 1;
    }

    @Override
    public double ticksPerRev(String motor) {
        return motor.equals(INTAKE) ? 1425.1 : 537.7; // 117 rpm and 312 rpm 5203s
    }

    @Override
    public double freeRpm(String motor) {
        return motor.equals(INTAKE) ? 117 : 312;
    }

    @Override
    protected void reset() {
        intakeRpm = 0;
        launcherRpm = 0;
        feedTimer = 0;
        outtakeTimer = 0;
    }

    @Override
    public void step(double dt) {
        double volts = physics.voltScale();

        SimMotor intake = motor(INTAKE);
        double intakePower = intake.mechanismPower();
        intakeRpm = approach(intakeRpm, INTAKE_RPM * intakePower * volts, INTAKE_TAU, dt);
        intake.mechanismRpm = intakeRpm;
        intake.currentAmps = 5.0 * Math.abs(intakePower);

        SimMotor launcher = motor(LAUNCHER);
        double launcherPower = launcher.mechanismPower();
        launcherRpm = approach(launcherRpm, LAUNCHER_LOADED_RPM * launcherPower * physics.batteryVolts() / 12.6,
                LAUNCHER_TAU, dt);
        launcher.mechanismRpm = launcherRpm;
        launcher.currentAmps = 4.0 * Math.abs(launcherPower);

        double feed = intakeRpm / INTAKE_RPM; // -1 to 1
        double corners = (crServo(LEFT_INTAKE_SERVO).mechanismPower() + crServo(RIGHT_INTAKE_SERVO).mechanismPower()) / 2;
        double windmillPower = crServo(windmill).mechanismPower();

        // pick up off the floor, and with the corner servos pulling, out of the bottom of a flower
        captureCooldown = Math.max(0, captureCooldown - dt);
        if (feed > 0.3 && captureCooldown == 0) {
            if (physics.capture(MOUTH_X_MIN, MOUTH_X_MAX, MOUTH_HALF_WIDTH, MOUTH_Z_MAX)) {
                captureCooldown = CAPTURE_COOLDOWN_SEC;
            } else if (corners > 0.3 && physics.takeFromFlower(MOUTH_X_MIN, CORNER_REACH_X, MOUTH_HALF_WIDTH)) {
                captureCooldown = 0.5; // the stack has to drop down and the next one roll out
            }
        }

        // the windmill feeds the hopper into the launcher
        if (windmillPower > 0.3 && physics.nextBall() >= 0) {
            feedTimer += dt * windmillPower;
            if (feedTimer >= FEED_SECONDS_PER_BALL) {
                feedTimer = 0;
                shoot();
            }
        } else {
            feedTimer = 0;
        }

        // the intake run backward spits the last one out the front
        if (feed < -0.3 && windmillPower <= 0.3 && physics.nextBall() >= 0) {
            outtakeTimer += dt;
            if (outtakeTimer >= 0.4) {
                outtakeTimer = 0;
                physics.eject(MOUTH_X_MAX + 2);
            }
        } else {
            outtakeTimer = 0;
        }
    }

    /** The launcher's encoder speed the code sees, ticks per second */
    static double ticksPerSec(double rpm) {
        return Math.abs(rpm) / 60 * LAUNCHER_TICKS_PER_REV;
    }

    /** How fast a ball leaves at this launcher speed, m/s (see SPEED_AT_1250) */
    static double launchSpeed(double rpm) {
        return SPEED_AT_1250 * ticksPerSec(rpm) / 1250;
    }

    private void shoot() {
        double speed = launchSpeed(launcherRpm);
        physics.launch(barrelForward, 0, BARREL_HEIGHT, physics.heading(), Math.toRadians(SHOT_ELEVATION_DEG),
                Math.max(0.6, speed)); // a dead launcher still lets it drop out
        physics.shotsLaunched++;
        launcherRpm = Math.copySign(Math.max(0, Math.abs(launcherRpm) - SHOT_RPM_LOSS), launcherRpm);
        if (speed < MIN_LAUNCH_SPEED) { // warnings show once each
            world.warn("A ball went into the launcher before it was up to speed, so it just fell out. "
                    + "Spin the launcher up first (goBILDA's code waits for getVelocity() over 1200)");
        }
    }

    @Override
    protected double sagVolts() {
        return 0.8 * Math.abs(motor(INTAKE).mechanismPower()) + 0.6 * Math.abs(motor(LAUNCHER).mechanismPower());
    }

    @Override
    protected void writeState(Json j) {
        j.field("flywheelRpm", launcherRpm * 19.2); // the motor's own speed, for the sound
        j.field("intakeRpm", intakeRpm);
    }

    @Override
    protected List<String[]> stats() {
        return Arrays.asList(
                new String[]{"Launcher", Math.round(ticksPerSec(launcherRpm)) + " ticks/s"},
                new String[]{"Intake", Math.round(intakeRpm) + " rpm"},
                new String[]{"Windmill", String.format("%.1f", crServo(windmill).mechanismPower())});
    }
}
