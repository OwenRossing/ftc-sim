package sim;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * One robot the sim can drive: its hardware config, its drivetrain, its outline and what its
 * mechanisms do. The engine does everything else (the field, the balls, the OpMode, the page).
 *
 * To add a robot, extend this, set the numbers in the constructor, fill in step(), and give it a
 * model in web/robots/. Pick it with robot= in the team repo's sim.properties: "starter" for the
 * built-in one, or the full class name of your own (it can live in the team repo, see README).
 *
 * Units: inches and the robot frame (x forward, y left, z up) unless it says otherwise.
 */
public abstract class SimRobot {

    /** What a device is, like the type you pick on the Driver Hub's configure screen */
    public enum Kind {
        MOTOR(DcMotorEx.class), SERVO(Servo.class), OTOS(SparkFunOTOS.class),
        ANALOG(AnalogInput.class), LIMELIGHT(Limelight3A.class);

        final Class<?> type;

        Kind(Class<?> type) {
            this.type = type;
        }

        /** Can hardwareMap.get(requested, name) hand this device out */
        boolean fits(Class<?> requested) {
            return requested.isAssignableFrom(type) || type.isAssignableFrom(requested);
        }
    }

    protected SimWorld world;
    protected SimPhysics physics;

    private final Map<String, Kind> config = new LinkedHashMap<>();

    // ---- the drivetrain. Defaults are a goBILDA strafer: 4x 312 rpm 5203, 104 mm mecanum ----
    protected double massKg = 13.6; // 30 lb
    protected double motorStallNm = 2.38;
    protected double motorStallAmps = 9.2;
    protected double motorFreeRpm = 312;
    protected double wheelRadiusM = 0.052;
    protected double trackIn = 14.0; // left wheels to right wheels
    protected double wheelbaseIn = 11.0; // front wheels to back wheels
    protected double tractionMu = 0.7; // mecanum on foam tile, wheels slip past this
    protected double rollingResist = 0.05; // fraction of the weight lost to rollers and carpet
    protected double strafeEfficiency = 0.8; // rollers waste some going sideways

    // where it starts on the field, Pedro's coordinates (inches, heading radians counterclockwise)
    protected double startX = 24;
    protected double startY = 72;
    protected double startHeading = 0;

    /** Name on the page, like "Starter bot" */
    public abstract String displayName();

    /** The page's 3D model, a file under web/ */
    public abstract String modelFile();

    /** Front left, front right, back left, back right drive motor names */
    public abstract String[] wheels();

    /**
     * Collision boxes, {centerX, centerY, centerZ, halfX, halfY, halfZ}. A box around the frame and
     * bumpers is enough. Moving parts can add their own in attached() with physics.collisions
     */
    public abstract List<double[]> outline();

    /** Mechanisms for one physics step, after the drivetrain. Read motor power, set rpm, move balls */
    public abstract void step(double dt);

    /** Called once the world exists */
    protected void attached() {}

    /** Reset field: put mechanisms back the way they start */
    protected void reset() {}

    /** Volts the mechanisms pull the battery down by right now (the drivetrain is already counted) */
    protected double sagVolts() {
        return 0;
    }

    /** Extra numbers for the page's state, like flywheelRpm (sounds.js listens for some) */
    protected void writeState(Json j) {}

    /** Lines for the page's Driver Station panel: {label, value} */
    protected List<String[]> stats() {
        return Collections.emptyList();
    }

    /** One line about this robot's controls for the page, or null. The keyboard help is already there */
    public String controlsHelp() {
        return null;
    }

    /**
     * True: like a real robot, a name that isn't in the config (or is the wrong type) crashes the OpMode.
     * False: any name gets a working device (handy while a robot's config is still changing)
     */
    public boolean strictNames() {
        return true;
    }

    // ---- per device. Mount is which way it's bolted in: -1 means positive power runs the mechanism backwards ----

    public int mount(String motor) {
        return 1;
    }

    public double ticksPerRev(String motor) {
        return 537.7; // 312 rpm 5203
    }

    /** Servo travel speed, position units per second (1.0 = full range in a second) */
    public double servoRate(String servo) {
        return 2.0;
    }

    /** Where a servo sits before anything commands it */
    public double servoStart(String servo) {
        return 0.5;
    }

    /** An analog input's voltage */
    public double analogVolts(String name) {
        return 0;
    }

    /**
     * The Limelight's pose right now: {x, y, z (field inches), aim heading (rad), tilt up (rad)}, or null
     * if there's no camera
     */
    public double[] cameraPose() {
        return null;
    }

    /** The alliance picked on the page or the gamepad, "RED", "BLUE" or "" */
    public void onAlliance(String side) {}

    public String alliance() {
        return null; // null hides the page's alliance menu
    }

    // ---- config ----

    /** Adds a device to the config, like the Driver Hub's configure screen */
    protected final void add(Kind kind, String... names) {
        for (String name : names) config.put(name, kind);
    }

    public final Map<String, Kind> config() {
        return Collections.unmodifiableMap(config);
    }

    final void attach(SimWorld world) {
        this.world = world;
        this.physics = world.physics;
        attached();
    }

    /** Device names of one kind, for error messages */
    final List<String> names(Kind kind) {
        List<String> out = new ArrayList<>();
        for (Map.Entry<String, Kind> e : config.entrySet()) if (e.getValue() == kind) out.add(e.getKey());
        return out;
    }

    // ---- helpers for step() ----

    protected final SimMotor motor(String name) {
        return world.motor(name);
    }

    protected final SimServo servo(String name) {
        return world.servo(name);
    }

    /** A first order lag: moves value toward target with time constant tau seconds */
    protected static double approach(double value, double target, double tau, double dt) {
        return value + (target - value) * (1 - Math.exp(-dt / tau));
    }

    /**
     * Loads a robot by its sim.properties name: "starter", or a full class name with a no-argument constructor
     */
    public static SimRobot load(String name) {
        if (name == null || name.isEmpty() || name.equals("starter")) return new StarterRobot();
        try {
            return (SimRobot) Class.forName(name).getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException | ClassCastException e) {
            throw new IllegalArgumentException("Can't load the sim robot \"" + name
                    + "\". Use robot=starter or the full name of a class that extends sim.SimRobot", e);
        }
    }
}
