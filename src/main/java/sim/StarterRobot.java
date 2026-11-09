package sim;

import java.util.Arrays;
import java.util.List;

/**
 * The goBILDA FTC StarterBot for 2026-27 (3200-2627-0003), what the FTC Starter Kit builds. Its hardware
 * config matches goBILDA's example code exactly, so their TeleOp and Auto run here unchanged:
 *
 *   left_drive, right_drive   6 wheel tank drive, one 312 rpm motor per side belted to 96 mm wheels (omnis in front)
 *   intake                    a 117 rpm motor on the 48 mm Gecko wheels across the front
 *   left_intake_servo,        continuous 2000 series servos with flat 72 mm wheels on the front corners, they pull
 *   right_intake_servo        pollen out of corners and out of the bottom of a flower
 *   launcher                  a 312 rpm motor straight on a 96 mm wheel, run with setVelocity (ticks per second)
 *   windmill                  a continuous servo that feeds the hopper's balls into the launcher
 *
 * It holds 4 balls and lobs them up into a Hive cell, so it starts facing the red Hive's upper cell, where
 * goBILDA's auto (shoot from the start, back up 12 cm) scores. Sizes and parts from goBILDA's CAD
 * (3200-2627-0003.step) and code (537.7 ticks/rev, 402 mm track). Speeds and the shot are guesses
 * (TUNE against the real robot).
 */
public final class StarterRobot extends SimRobot {

    public static final String LEFT_DRIVE = "left_drive";
    public static final String RIGHT_DRIVE = "right_drive";
    public static final String INTAKE = "intake";
    public static final String LEFT_INTAKE_SERVO = "left_intake_servo";
    public static final String RIGHT_INTAKE_SERVO = "right_intake_servo";
    public static final String LAUNCHER = "launcher";
    public static final String WINDMILL = "windmill";

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
    // throws about 6.5 m/s, which scores from the start pose (6.25-6.75 m/s at 80 deg lands in the upper cell).
    // TUNE: film a shot
    static final double LAUNCHER_LOADED_RPM = 290; // at full power with the wheel on
    static final double LAUNCHER_TICKS_PER_REV = 537.7;
    static final double LAUNCHER_TAU = 0.25;
    static final double SHOT_RPM_LOSS = 20; // what one ball takes out of it
    static final double SPEED_AT_1250 = 6.5; // m/s
    static final double SHOT_ELEVATION_DEG = 80; // a steep lob, it has to drop into the cell from above. TUNE
    static final double BARREL_FORWARD = -1.6; // the top of the launcher wheel, robot frame, from the CAD
    static final double BARREL_HEIGHT = 10;
    static final double FEED_SECONDS_PER_BALL = 0.3; // the windmill at full power
    static final double MIN_LAUNCH_SPEED = 3; // m/s, under this it just falls out

    double intakeRpm = 0;
    double launcherRpm = 0;
    private double feedTimer = 0;
    private double outtakeTimer = 0;
    private double captureCooldown = 0;

    public StarterRobot() {
        add(Kind.MOTOR, LEFT_DRIVE, RIGHT_DRIVE, INTAKE, LAUNCHER);
        add(Kind.CRSERVO, LEFT_INTAKE_SERVO, RIGHT_INTAKE_SERVO, WINDMILL);
        tankDrive = true;
        massKg = 9.5; // 21 lb, TUNE: weigh it
        wheelRadiusM = 0.048; // 96 mm
        trackIn = 402 / 25.4;
        wheelbaseIn = 11.3; // front to back wheel, from the CAD
        // one motor per side drives 3 wheels, and each side is listed twice in wheels(): half each
        motorStallNm = 2.38 / 2;
        motorStallAmps = 9.2 / 2;
        // facing the red Hive's upper cell, about 30 in back from its frame: where a lob at goBILDA's launcher
        // speed drops in (parked right against the frame, it can't get the arc over the cell's rim)
        startX = 30;
        startY = 54;
        startHeading = 0;
    }

    @Override
    public String displayName() {
        return "goBILDA StarterBot";
    }

    @Override
    public String modelFile() {
        return "robots/starter.json";
    }

    @Override
    public String[] wheels() {
        return new String[]{LEFT_DRIVE, RIGHT_DRIVE, LEFT_DRIVE, RIGHT_DRIVE};
    }

    @Override
    public List<double[]> outline() {
        return Arrays.asList(
                // from the CAD: the frame runs 7.6 in back to 9.5 in front and 16.3 in wide
                new double[]{0.95, 0, 4, 8.55, 8.15, 3.5},
                new double[]{0, 0, 9.5, 4, 4, 2.5}, // the hopper and launcher on top, up to 12 in
                new double[]{9.85, 0, 1.9, 0.35, 7.5, 1.4}); // the corner wheels, out to 10.2 in
    }

    @Override
    public int mount(String device) {
        // goBILDA's code reverses right_drive, right_intake_servo and the windmill, so that's how they're mounted
        return device.equals(RIGHT_DRIVE) || device.equals(RIGHT_INTAKE_SERVO) || device.equals(WINDMILL) ? -1 : 1;
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
        double windmill = crServo(WINDMILL).mechanismPower();

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
        if (windmill > 0.3 && physics.nextBall() >= 0) {
            feedTimer += dt * windmill;
            if (feedTimer >= FEED_SECONDS_PER_BALL) {
                feedTimer = 0;
                shoot();
            }
        } else {
            feedTimer = 0;
        }

        // the intake run backward spits the last one out the front
        if (feed < -0.3 && windmill <= 0.3 && physics.nextBall() >= 0) {
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
        physics.launch(BARREL_FORWARD, 0, BARREL_HEIGHT, physics.heading(), Math.toRadians(SHOT_ELEVATION_DEG),
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
                new String[]{"Windmill", String.format("%.1f", crServo(WINDMILL).mechanismPower())});
    }
}
