package sim;

import java.util.Arrays;
import java.util.List;

/**
 * The starter bot: what a rookie gets to learn on. A mecanum drivetrain, an intake roller across the
 * front, a flywheel shooter on top that fires forward, and a gate servo between them.
 *
 * How the balls go: run the intake forward to pick balls up off the floor (it holds 4). Open the gate and
 * keep the intake running to feed them up into the shooter, one about every 0.3 s. Spin the shooter first
 * or they dribble out. Run the intake backward with the gate shut to spit the last one out the front.
 *
 * Like a real robot, the left drive motors are mounted mirrored, so with no setDirection the robot spins
 * instead of driving straight. See STARTER_BOT.md for the config.
 */
public final class StarterRobot extends SimRobot {

    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft";
    public static final String BACK_RIGHT = "backRight";
    public static final String INTAKE = "intake";
    public static final String SHOOTER = "shooter";
    public static final String GATE = "gate";

    // intake: 312 rpm 5203 on the roller
    static final double INTAKE_RPM = 312;
    static final double INTAKE_TAU = 0.08;
    static final double MOUTH_X_MIN = 9; // where it grabs a ball, inches in front of center
    static final double MOUTH_X_MAX = 13.5;
    static final double MOUTH_HALF_WIDTH = 6.5;
    static final double MOUTH_Z_MAX = 5;
    static final double CAPTURE_COOLDOWN_SEC = 0.15;

    // shooter: 6000 rpm 5203, it tops out around 4150 rpm under the flywheel's drag. 16:24 to 3 in wheels
    static final double SHOOTER_TOP_RPM = 4150; // at full power on a 12.6V battery
    static final double SHOOTER_TAU = 0.45;
    static final double SHOT_RPM_LOSS = 350; // what one ball takes out of it
    static final double SHOOTER_RATIO = 16.0 / 24.0;
    static final double WHEEL_DIAMETER_M = 3 * 0.0254;
    static final double BALL_EXIT_FRACTION = 0.65; // ball speed vs wheel rim speed. TUNE
    static final double HOOD_DEG = 70; // fixed hood
    static final double BARREL_FORWARD = 1.5; // where the ball leaves, robot frame
    static final double BARREL_HEIGHT = 16;
    static final double FEED_SECONDS_PER_BALL = 0.3;
    static final double GATE_OPEN_ABOVE = 0.5; // servo position

    double intakeRpm = 0;
    double shooterRpm = 0;
    private double feedTimer = 0;
    private double outtakeTimer = 0;
    private double captureCooldown = 0;

    public StarterRobot() {
        add(Kind.MOTOR, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, INTAKE, SHOOTER);
        add(Kind.SERVO, GATE);
        massKg = 12.7; // 28 lb
        trackIn = 14.0;
        wheelbaseIn = 11.0;
    }

    @Override
    public String displayName() {
        return "starter bot";
    }

    @Override
    public String modelFile() {
        return "robots/starter.json";
    }

    @Override
    public String[] wheels() {
        return new String[]{FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
    }

    @Override
    public List<double[]> outline() {
        return Arrays.asList(
                new double[]{0, 0, 3.5, 8.5, 8, 3.5}, // frame and bumpers, 17 x 16 in
                new double[]{-3, 0, 11, 4, 4.5, 4}, // the shooter tower
                new double[]{9.5, 0, 2.5, 1, 7, 2.5}); // the intake across the front
    }

    @Override
    public int mount(String motor) {
        // the left side is bolted in mirrored, like every real drivetrain
        return motor.equals(FRONT_LEFT) || motor.equals(BACK_LEFT) ? -1 : 1;
    }

    @Override
    public double ticksPerRev(String motor) {
        return motor.equals(SHOOTER) ? 28 : 537.7;
    }

    @Override
    public double servoRate(String servo) {
        return 4.0;
    }

    @Override
    public double servoStart(String servo) {
        return 0; // the gate sits shut
    }

    @Override
    protected void reset() {
        intakeRpm = 0;
        shooterRpm = 0;
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
        intake.currentAmps = 6.0 * Math.abs(intakePower);

        SimMotor shooter = motor(SHOOTER);
        double shooterPower = shooter.mechanismPower();
        shooterRpm = approach(shooterRpm, SHOOTER_TOP_RPM * shooterPower * physics.batteryVolts() / 12.6, SHOOTER_TAU, dt);
        shooter.mechanismRpm = shooterRpm;
        shooter.currentAmps = 4.0 * Math.abs(shooterPower);

        double feed = intakeRpm / INTAKE_RPM; // -1 to 1
        boolean gateOpen = servo(GATE).physical > GATE_OPEN_ABOVE;

        // pick up off the floor
        captureCooldown = Math.max(0, captureCooldown - dt);
        if (feed > 0.3 && captureCooldown == 0
                && physics.capture(MOUTH_X_MIN, MOUTH_X_MAX, MOUTH_HALF_WIDTH, MOUTH_Z_MAX)) {
            captureCooldown = CAPTURE_COOLDOWN_SEC;
        }

        // feed up through the open gate into the shooter
        if (gateOpen && feed > 0.3 && physics.nextBall() >= 0) {
            feedTimer += dt * feed;
            if (feedTimer >= FEED_SECONDS_PER_BALL) {
                feedTimer = 0;
                shoot();
            }
        } else {
            feedTimer = 0;
        }

        // spit one out the front
        if (!gateOpen && feed < -0.3 && physics.nextBall() >= 0) {
            outtakeTimer += dt;
            if (outtakeTimer >= 0.4) {
                outtakeTimer = 0;
                physics.eject(MOUTH_X_MAX + 2);
            }
        } else {
            outtakeTimer = 0;
        }
    }

    private void shoot() {
        double rim = Math.abs(shooterRpm) * SHOOTER_RATIO / 60 * Math.PI * WHEEL_DIAMETER_M;
        double speed = Math.max(0.6, rim * BALL_EXIT_FRACTION); // a dead flywheel still lets it drop out
        physics.launch(BARREL_FORWARD, 0, BARREL_HEIGHT, physics.heading(), Math.toRadians(HOOD_DEG), speed);
        physics.shotsLaunched++;
        shooterRpm = Math.copySign(Math.max(0, Math.abs(shooterRpm) - SHOT_RPM_LOSS), shooterRpm);
        if (rim * BALL_EXIT_FRACTION < 3) { // warnings show once each
            world.warn("A ball went into the shooter before it was spinning, so it just fell out. Spin the shooter up first");
        }
    }

    @Override
    protected double sagVolts() {
        return 1.0 * Math.abs(motor(INTAKE).power) + 0.6 * Math.abs(motor(SHOOTER).power);
    }

    @Override
    protected void writeState(Json j) {
        j.field("flywheelRpm", shooterRpm);
        j.field("intakeRpm", intakeRpm);
    }

    @Override
    protected List<String[]> stats() {
        return Arrays.asList(
                new String[]{"Shooter", Math.round(shooterRpm) + " rpm"},
                new String[]{"Intake", Math.round(intakeRpm) + " rpm"},
                new String[]{"Gate", servo(GATE).physical > GATE_OPEN_ABOVE ? "open" : "shut"});
    }
}
