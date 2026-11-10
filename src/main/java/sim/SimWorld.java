package sim;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * All the fake devices plus the physics. Devices live as long as the sim does, so the robot
 * stays where it is between OpModes like a real one.
 */
public final class SimWorld {

    public enum Phase { IDLE, INIT, RUNNING, STOPPED }

    volatile Phase phase = Phase.IDLE;
    volatile boolean autonomous = false;

    final SimRobot robot;
    final SimPhysics physics;

    private final Map<String, SimMotor> motors = new LinkedHashMap<>();
    private final Map<String, SimServo> servos = new LinkedHashMap<>();
    private final Map<String, SimCRServo> crServos = new LinkedHashMap<>();
    private final Map<String, AnalogInput> analogs = new LinkedHashMap<>();
    private final SimOtos otos = new SimOtos(this);
    private final VoltageSensor batterySensor;

    private final Set<String> requested = Collections.synchronizedSet(new LinkedHashSet<>());
    // fault injection: names in here act like they're missing from the Driver Hub config
    private final Set<String> unplugged = Collections.synchronizedSet(new LinkedHashSet<>());
    private final Set<String> warnings = Collections.synchronizedSet(new LinkedHashSet<>());

    public SimWorld(SimRobot robot) {
        this.robot = robot;
        this.physics = new SimPhysics(this, robot);
        batterySensor = Fakes.proxy(VoltageSensor.class, "battery", (method, a) ->
                method.equals("getVoltage") ? physics.batteryVolts : Fakes.NOT_HANDLED);
        robot.attach(this);
        // the config's devices exist whether or not the code asked for them yet (the page draws them)
        for (String wheel : robot.wheels()) motor(wheel);
        for (Map.Entry<String, SimRobot.Kind> e : robot.config().entrySet()) {
            if (e.getValue() == SimRobot.Kind.MOTOR) motor(e.getKey());
            if (e.getValue() == SimRobot.Kind.CRSERVO) crServo(e.getKey());
        }
    }

    /**
     * Fault injection: the next OpMode's hardwareMap.get(name) fails like it's missing from the config
     */
    public void unplug(String... names) {
        unplugged.addAll(java.util.Arrays.asList(names));
    }

    public void plugEverythingIn() {
        unplugged.clear();
    }

    public SimRobot robot() {
        return robot;
    }

    public SimPhysics physics() {
        return physics;
    }

    public Phase phase() {
        return phase;
    }

    public SimServo servoState(String name) {
        return servo(name);
    }

    public SimMotor motorState(String name) {
        return motor(name);
    }

    VoltageSensor batterySensor() {
        return batterySensor;
    }

    SimOtos otos() {
        return otos;
    }

    /**
     * Hands out a fake device for hardwareMap.get(type, name), or null if the config has no such device
     */
    Object device(Class<?> type, String name) {
        if (unplugged.contains(name)) {
            requested.add(type.getSimpleName() + " \"" + name + "\" (UNPLUGGED)");
            return null;
        }
        if (robot.strictNames()) {
            SimRobot.Kind kind = robot.config().get(name);
            if (kind == null || !kind.fits(type)) {
                requested.add(type.getSimpleName() + " \"" + name + "\" (NOT IN CONFIG)");
                return null;
            }
        }
        requested.add(type.getSimpleName() + " \"" + name + "\"");
        // a CRServo is a DcMotorSimple too, so it has to come first
        if (CRServo.class.isAssignableFrom(type) || robot.config().get(name) == SimRobot.Kind.CRSERVO) {
            return crServo(name).device;
        }
        if (DcMotorSimple.class.isAssignableFrom(type)) return motor(name).device;
        if (Servo.class.isAssignableFrom(type)) return servo(name).device;
        if (SparkFunOTOS.class.isAssignableFrom(type)) return otos;
        if (type == AnalogInput.class) return analog(name);
        if (VoltageSensor.class.isAssignableFrom(type)) return batterySensor;
        if (com.qualcomm.hardware.limelightvision.Limelight3A.class.isAssignableFrom(type)) {
            return new SimLimelight(physics);
        }
        warn("The sim has no fake for " + type.getSimpleName() + " \"" + name + "\"");
        return null;
    }

    /** What the config says about a name that didn't work, for the crash message */
    String configHelp(Class<?> type, String name) {
        if (unplugged.contains(name)) return " (the sim unplugged it on purpose)";
        if (!robot.strictNames()) return " (the sim doesn't fake this type)";
        SimRobot.Kind kind = robot.config().get(name);
        if (kind != null) {
            return ". \"" + name + "\" is a " + kind.type.getSimpleName() + " in the " + robot.displayName()
                    + "'s config, not a " + type.getSimpleName();
        }
        List<String> same = new ArrayList<>();
        for (SimRobot.Kind k : SimRobot.Kind.values()) if (k.fits(type)) same.addAll(robot.names(k));
        String close = closest(name, same);
        String hint = close != null ? " Did you mean \"" + close + "\"?" : "";
        return ". The " + robot.displayName() + "'s config has " + (same.isEmpty() ? "none of those"
                : String.join(", ", same)) + "." + hint + " Names have to match exactly, capitals too";
    }

    // the nearest name, if it's only a typo or a capital away
    private static String closest(String name, List<String> names) {
        String best = null;
        int bestDistance = Integer.MAX_VALUE;
        for (String candidate : names) {
            String a = candidate.toLowerCase(), b = name.toLowerCase();
            // a capital off, or one is the start of the other ("windmill" for "windmillServo")
            int d = a.equals(b) ? 0 : a.startsWith(b) || b.startsWith(a) ? 1 : editDistance(a, b);
            if (d < bestDistance) {
                bestDistance = d;
                best = candidate;
            }
        }
        return bestDistance <= Math.max(2, name.length() / 3) ? best : null;
    }

    private static int editDistance(String a, String b) {
        int[] prev = new int[b.length() + 1];
        int[] cur = new int[b.length() + 1];
        for (int j = 0; j <= b.length(); j++) prev[j] = j;
        for (int i = 1; i <= a.length(); i++) {
            cur[0] = i;
            for (int j = 1; j <= b.length(); j++) {
                int cost = a.charAt(i - 1) == b.charAt(j - 1) ? 0 : 1;
                cur[j] = Math.min(Math.min(cur[j - 1] + 1, prev[j] + 1), prev[j - 1] + cost);
            }
            int[] t = prev;
            prev = cur;
            cur = t;
        }
        return prev[b.length()];
    }

    synchronized SimMotor motor(String name) {
        return motors.computeIfAbsent(name, n -> new SimMotor(n, robot.ticksPerRev(n), robot.mount(n), robot.freeRpm(n), this));
    }

    synchronized SimCRServo crServo(String name) {
        return crServos.computeIfAbsent(name, n -> new SimCRServo(n, robot.mount(n), this));
    }

    synchronized List<SimCRServo> crServos() {
        return new ArrayList<>(crServos.values());
    }

    synchronized SimServo servo(String name) {
        return servos.computeIfAbsent(name, n -> new SimServo(n, robot.servoRate(n), robot.servoStart(n), this));
    }

    private synchronized AnalogInput analog(String name) {
        return analogs.computeIfAbsent(name, n -> {
            AnalogInputController controller = Fakes.proxy(AnalogInputController.class, "analog " + n, (method, a) -> {
                if (method.equals("getAnalogInputVoltage")) return robot.analogVolts(n);
                if (method.equals("getMaxAnalogInputVoltage")) return 3.3;
                return Fakes.NOT_HANDLED;
            });
            return new AnalogInput(controller, 0);
        });
    }

    synchronized List<SimMotor> motors() {
        return new ArrayList<>(motors.values());
    }

    synchronized List<SimServo> servos() {
        return new ArrayList<>(servos.values());
    }

    // ---- hooks from the fake devices ----

    void onMotorWrite(SimMotor motor, double power) {
        if (phase == Phase.INIT && power != 0) {
            warn(motor.name + " got power " + String.format("%.2f", power) + " during INIT" + initNote());
        }
    }

    void onCrServoWrite(SimCRServo servo, double power) {
        if (phase == Phase.INIT) {
            warn(servo.name + " got power " + String.format("%.2f", power) + " during INIT" + initNote());
        }
    }

    void onServoWrite(SimServo servo) {
        if (phase == Phase.INIT) {
            warn(servo.name + " was moved during INIT" + initNote());
        }
    }

    private String initNote() {
        return autonomous ? " (allowed in auto)" : " (PENALTY in teleop: nothing may move before START)";
    }

    /**
     * The code set the localizer's pose during init. On a real robot that's where the drive team
     * put it, so the sim robot gets placed there too (unless it's the field origin, which is just
     * teleop zeroing its localizer).
     */
    void onOtosSetPosition(double x, double y, double heading) {
        if (phase != Phase.INIT) return;
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) return;
        physics.placeRobot(x, y, heading);
    }

    /**
     * Fake hardware latency, only on the OpMode thread
     */
    void hardwareDelay(long ms) {
        if (!Thread.currentThread().getName().equals("OpModeThread")) return;
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // OpMode threads that kept running after STOP: their writes to the hardware don't count any more
    private final Set<Thread> ignored = Collections.synchronizedSet(Collections.newSetFromMap(new java.util.WeakHashMap<>()));

    void ignoreThread(Thread t) {
        if (t != null) ignored.add(t);
    }

    boolean ignoring(Thread t) {
        return !ignored.isEmpty() && ignored.contains(t);
    }

    public void warn(String message) {
        if (warnings.add(message)) System.out.println("[sim] WARNING: " + message);
    }

    public List<String> warnings() {
        synchronized (warnings) {
            return new ArrayList<>(warnings);
        }
    }

    List<String> requestedDevices() {
        synchronized (requested) {
            return new ArrayList<>(requested);
        }
    }

    /**
     * New OpMode: motors stop and reset, servos go limp, warnings clear. Positions stay.
     */
    synchronized void resetForNewOpMode(boolean isAutonomous) {
        autonomous = isAutonomous;
        for (SimMotor m : motors.values()) m.reset();
        for (SimServo s : servos.values()) s.reset();
        for (SimCRServo s : crServos.values()) s.reset();
        warnings.clear();
        requested.clear();
    }
}
