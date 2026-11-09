package sim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.SimOpModeRunner;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.File;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

/**
 * The sim without the web page: fake robot world, physics loop, and the OpMode lifecycle
 * (INIT / START / STOP) the way the Robot Controller app does it. SimMain puts a web server on
 * top of this; the tests drive it directly.
 */
public final class SimEngine {

    private static final int PHYSICS_HZ = 200;
    /** How long an OpMode (not a LinearOpMode) can sit in one call before the sim says it's stuck */
    static final double STUCK_SEC = 3;
    private static final List<String> LIFECYCLE = Arrays.asList("init", "init_loop", "start", "loop", "stop");

    private final SimWorld world;
    private final SimTelemetry telemetry = new SimTelemetry();
    private final Gamepad input1 = new Gamepad();
    private final Gamepad input2 = new Gamepad();
    private final Map<String, Class<? extends OpMode>> opModes = new LinkedHashMap<>();
    private final Map<String, Boolean> opModeIsAuto = new LinkedHashMap<>();

    private volatile SimOpModeRunner runner;
    private volatile String opModeName = null;
    private volatile String errorText = null;
    private Thread physicsThread;

    // stuck detection: the SDK calls telemetry.update() after every init_loop() and loop()
    private long lastUpdates = -1;
    private long lastUpdateNanos = 0;

    /** The robot from the sim.robot system property (sim.properties' robot=), the starter bot if none */
    public SimEngine() {
        this(SimRobot.load(System.getProperty("sim.robot", "starter")));
    }

    public SimEngine(SimRobot robot) {
        world = new SimWorld(robot);
        findOpModes();
    }

    public SimWorld world() {
        return world;
    }

    public Map<String, Boolean> opModes() {
        return opModeIsAuto;
    }

    public String opModeName() {
        return opModeName;
    }

    public String error() {
        return errorText;
    }

    public List<String> telemetryLines() {
        return telemetry.lines();
    }

    public void start() {
        physicsThread = new Thread(this::physicsLoop, "sim-physics");
        physicsThread.setDaemon(true);
        physicsThread.start();
    }

    public void shutdown() {
        stopOpMode();
        if (physicsThread != null) physicsThread.interrupt();
    }

    // ---------------------------------------------------------------- OpModes

    // every OpMode in the robot code (any classes folder on the classpath with org/firstinspires/ftc/teamcode in it)
    private void findOpModes() {
        try {
            List<String> names = new ArrayList<>();
            java.util.Enumeration<java.net.URL> dirs =
                    SimEngine.class.getClassLoader().getResources("org/firstinspires/ftc/teamcode");
            while (dirs.hasMoreElements()) {
                java.net.URL url = dirs.nextElement();
                if (!url.getProtocol().equals("file")) continue;
                Path teamCode = Paths.get(url.toURI());
                Path classes = teamCode.getParent().getParent().getParent().getParent();
                try (Stream<Path> files = Files.walk(teamCode)) {
                    files.filter(p -> p.toString().endsWith(".class") && !p.getFileName().toString().contains("$"))
                            .forEach(p -> names.add(classes.relativize(p).toString()
                                    .replace(File.separatorChar, '.').replaceAll("\\.class$", "")));
                }
            }
            names.sort(String::compareTo);
            for (String className : names) {
                try {
                    addOpMode(Class.forName(className, false, SimEngine.class.getClassLoader()));
                } catch (Throwable t) {
                    System.out.println("[sim] skipped " + className + ": " + t);
                }
            }
        } catch (Exception e) {
            throw new IllegalStateException("couldn't find the OpModes", e);
        }
    }

    /**
     * Adds an OpMode the way the Driver Hub lists it: needs @TeleOp or @Autonomous and no @Disabled.
     * Returns its name, or null if it isn't one
     */
    @SuppressWarnings("unchecked")
    public String addOpMode(Class<?> c) {
        if (!OpMode.class.isAssignableFrom(c) || Modifier.isAbstract(c.getModifiers())) return null;
        if (c.isAnnotationPresent(Disabled.class)) return null;
        TeleOp teleOp = c.getAnnotation(TeleOp.class);
        Autonomous auto = c.getAnnotation(Autonomous.class);
        if (teleOp == null && auto == null) return null;
        String name = teleOp != null ? teleOp.name() : auto.name();
        if (name.isEmpty()) name = c.getSimpleName();
        opModes.put(name, (Class<? extends OpMode>) c);
        opModeIsAuto.put(name, auto != null);
        return name;
    }

    public synchronized void initOpMode(String name) throws Exception {
        stopOpMode();
        Class<? extends OpMode> c = opModes.get(name);
        if (c == null) throw new IllegalArgumentException("No OpMode named " + name);
        boolean auto = opModeIsAuto.get(name);
        errorText = null;
        world.resetForNewOpMode(auto);
        world.phase = SimWorld.Phase.INIT;
        opModeName = name;
        lastUpdates = -1;
        try {
            OpMode opMode = c.getDeclaredConstructor().newInstance();
            runner = new SimOpModeRunner(opMode, new SimHardwareMap(world), telemetry.telemetry, input1, input2);
            runner.init();
        } catch (Throwable t) {
            world.phase = SimWorld.Phase.STOPPED;
            errorText = explain(t);
            throw t;
        }
    }

    public synchronized void startOpMode() {
        if (runner == null || world.phase != SimWorld.Phase.INIT) return;
        runner.start();
        world.phase = SimWorld.Phase.RUNNING;
    }

    public synchronized void stopOpMode() {
        if (runner == null) return;
        SimOpModeRunner r = runner;
        r.stop();
        if (r.getError() != null && errorText == null) errorText = explain(r.getError());
        if (r.isStillRunning()) {
            // a real robot restarts its app here. We can't kill the thread, so we stop listening to it
            world.ignoreThread(r.thread());
            world.warn("Your OpMode kept running after STOP" + where(r) + ". In a LinearOpMode, every while loop needs "
                    + "opModeIsActive() in its condition, so it ends when STOP is pressed. A real robot would restart its app");
        }
        runner = null;
        world.phase = SimWorld.Phase.STOPPED;
        disableOutputs();
    }

    // when an OpMode ends the SDK cuts the motors and the servos go limp
    private void disableOutputs() {
        for (SimMotor m : world.motors()) m.power = 0;
        for (SimServo s : world.servos()) s.commanded = Double.NaN;
    }

    /** Is the OpMode (not a LinearOpMode) stuck inside one of its methods? Warns once if so */
    private void checkStuck(SimOpModeRunner r) {
        if (r.opMode() instanceof LinearOpMode || r.isFinished()) return;
        long updates = telemetry.updates();
        long now = System.nanoTime();
        if (updates != lastUpdates) {
            lastUpdates = updates;
            lastUpdateNanos = now;
            return;
        }
        if ((now - lastUpdateNanos) / 1e9 < STUCK_SEC) return;
        String method = lifecycleMethod(r);
        if (method == null) return; // it's waiting on the SDK, not stuck in user code
        world.warn("Your code has been stuck inside " + method + "() for " + (int) STUCK_SEC + " seconds" + where(r)
                + ". " + method + "() has to finish quickly, because the SDK calls it over and over. A while loop in there"
                + " never lets it finish: use an if instead. The robot keeps doing the last thing it was told."
                + " A real robot would stop with \"Stuck in " + method + "()\"");
    }

    // which of init/init_loop/start/loop/stop the OpMode thread is in, or null
    private static String lifecycleMethod(SimOpModeRunner r) {
        Thread t = r.thread();
        if (t == null) return null;
        String found = null;
        for (StackTraceElement frame : t.getStackTrace()) {
            if (isUserClass(frame.getClassName(), r.opMode()) && LIFECYCLE.contains(frame.getMethodName())) {
                found = frame.getMethodName(); // keep going, the outermost one is the lifecycle call
            }
        }
        return found;
    }

    // " (at TeleOp.java line 42)" for the deepest line of the student's code, or ""
    private static String where(SimOpModeRunner r) {
        Thread t = r.thread();
        if (t == null) return "";
        for (StackTraceElement frame : t.getStackTrace()) {
            if (isUserClass(frame.getClassName(), r.opMode()) && frame.getLineNumber() > 0) {
                return " (at " + frame.getFileName() + " line " + frame.getLineNumber() + ")";
            }
        }
        return "";
    }

    private static boolean isUserClass(String className, OpMode opMode) {
        for (Class<?> c = opMode.getClass(); c != null && c != OpMode.class && c != LinearOpMode.class; c = c.getSuperclass()) {
            if (className.equals(c.getName()) || className.startsWith(c.getName() + "$")) return true;
        }
        return className.startsWith("org.firstinspires.ftc.teamcode.");
    }

    // ---------------------------------------------------------------- gamepad

    /**
     * Sets gamepad 1 from short keys: lx ly rx ry lt rt (numbers), a b x y lb rb up down left
     * right start back ls rs (booleans). Anything not given is released.
     */
    public void setGamepad(Map<String, ?> v) {
        synchronized (input1) {
            Gamepad g = input1;
            g.left_stick_x = f(v, "lx");
            g.left_stick_y = f(v, "ly");
            g.right_stick_x = f(v, "rx");
            g.right_stick_y = f(v, "ry");
            g.left_trigger = f(v, "lt");
            g.right_trigger = f(v, "rt");
            g.a = b(v, "a");
            g.b = b(v, "b");
            g.x = b(v, "x");
            g.y = b(v, "y");
            g.left_bumper = b(v, "lb");
            g.right_bumper = b(v, "rb");
            g.dpad_up = b(v, "up");
            g.dpad_down = b(v, "down");
            g.dpad_left = b(v, "left");
            g.dpad_right = b(v, "right");
            g.start = b(v, "start");
            g.back = b(v, "back");
            g.left_stick_button = b(v, "ls");
            g.right_stick_button = b(v, "rs");
        }
    }

    private static float f(Map<String, ?> v, String key) {
        Object value = v.get(key);
        if (value instanceof Number) return ((Number) value).floatValue();
        try {
            return value == null ? 0f : Float.parseFloat(value.toString());
        } catch (NumberFormatException e) {
            return 0f;
        }
    }

    private static boolean b(Map<String, ?> v, String key) {
        Object value = v.get(key);
        return Boolean.TRUE.equals(value) || "true".equals(value);
    }

    // ---------------------------------------------------------------- physics loop

    private void physicsLoop() {
        long periodNanos = 1_000_000_000L / PHYSICS_HZ;
        long last = System.nanoTime();
        int count = 0;
        while (!Thread.currentThread().isInterrupted()) {
            long now = System.nanoTime();
            double dt = Math.min((now - last) / 1e9, 0.05);
            last = now;
            try {
                world.physics.step(dt);
                SimOpModeRunner r = runner;
                if (r != null && count++ % 2 == 0) {
                    r.tick();
                    if (count % 50 == 1) checkStuck(r);
                    if (r.isFinished()) {
                        synchronized (this) {
                            if (runner == r) {
                                if (r.getError() != null) errorText = explain(r.getError());
                                runner = null;
                                world.phase = SimWorld.Phase.STOPPED;
                                disableOutputs();
                            }
                        }
                    }
                }
            } catch (Throwable t) {
                System.out.println("[sim] physics error: " + t);
            }
            long sleepNanos = periodNanos - (System.nanoTime() - now);
            if (sleepNanos > 0) {
                try {
                    Thread.sleep(sleepNanos / 1_000_000L, (int) (sleepNanos % 1_000_000L));
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }

    // ---------------------------------------------------------------- state for the browser

    public String stateJson() {
        SimPhysics p = world.physics;
        SimRobot robot = world.robot;
        double[] otos = world.otos().reportedPose();
        Json j = new Json().begin();
        j.field("phase", world.phase.name());
        j.field("opmode", opModeName);
        j.field("autonomous", world.autonomous);
        j.field("error", errorText);
        j.field("robotName", robot.displayName());
        j.field("alliance", robot.alliance());
        j.key("robot").begin()
                .field("x", p.x).field("y", p.y).field("h", p.heading)
                .field("vForward", p.vForward).field("vLeft", p.vLeft).field("vTurn", p.vTurn)
                .end();
        j.key("otos").begin().field("x", otos[0]).field("y", otos[1]).field("h", otos[2]).end();
        j.field("battery", p.batteryVolts);
        j.field("balls", p.balls);
        j.field("shots", p.shotsLaunched);
        robot.writeState(j);
        j.numberArray("pieces", p.collisions.pieces()); // type, x, y, z in inches, repeating
        j.key("hives").begin()
                .field("redAngle", p.collisions.redHive.angle).field("redTips", p.collisions.redHive.tips)
                .field("redInCell", p.collisions.redHive.inCell)
                .field("blueAngle", p.collisions.blueHive.angle).field("blueTips", p.collisions.blueHive.tips)
                .field("blueInCell", p.collisions.blueHive.inCell)
                .end();
        j.key("servos").begin();
        for (SimServo s : world.servos()) {
            j.key(s.name).begin().field("cmd", s.commanded).field("pos", s.physical).end();
        }
        j.end();
        j.key("motors").begin();
        for (SimMotor m : world.motors()) {
            j.key(m.name).begin().field("power", m.power).field("rpm", m.mechanismRpm).end();
        }
        j.end();
        List<String> stats = new ArrayList<>();
        for (String[] s : robot.stats()) stats.add(s[0] + "\t" + s[1]);
        j.stringArray("stats", stats);
        j.stringArray("telemetry", telemetry.lines());
        j.stringArray("warnings", world.warnings());
        j.stringArray("devices", world.requestedDevices());
        j.key("opmodes").beginArray();
        for (Map.Entry<String, Boolean> e : opModeIsAuto.entrySet()) {
            j.begin().field("name", e.getKey()).field("auto", e.getValue()).end();
        }
        j.endArray();
        return j.end().toString();
    }

    /** The stack trace, with a plain English line on top for the crashes beginners hit most */
    static String explain(Throwable t) {
        String hint = Hints.forCrash(t);
        return hint == null ? stackTrace(t) : hint + "\n\n" + stackTrace(t);
    }

    static String stackTrace(Throwable t) {
        StringWriter sw = new StringWriter();
        t.printStackTrace(new PrintWriter(sw));
        return sw.toString();
    }
}
