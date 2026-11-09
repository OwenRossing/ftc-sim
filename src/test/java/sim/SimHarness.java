package sim;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Drives the sim from a test like a drive team would: INIT, START, hold buttons, wait, read the
 * Driver Hub telemetry. Runs in real time (the robot code uses the real clock).
 */
final class SimHarness implements AutoCloseable {

    final SimEngine engine;
    final SimWorld world;
    final SimPhysics physics;
    private final Map<String, Object> pad = new HashMap<>();

    SimHarness() {
        this(new StarterRobot());
    }

    SimHarness(SimRobot robot) {
        engine = new SimEngine(robot);
        world = engine.world();
        physics = world.physics();
        engine.start();
    }

    SimHarness init(String opMode) throws Exception {
        engine.initOpMode(opMode);
        sleep(800); // init + a few init_loops
        assertNoCrash();
        return this;
    }

    SimHarness start() {
        engine.startOpMode();
        sleep(300);
        assertNoCrash();
        return this;
    }

    SimHarness stop() {
        engine.stopOpMode();
        return this;
    }

    SimHarness hold(String key, Object value) {
        pad.put(key, value);
        engine.setGamepad(pad);
        return this;
    }

    SimHarness release(String key) {
        pad.remove(key);
        engine.setGamepad(pad);
        return this;
    }

    /** Waits until cond is true or the time runs out, returns whether it happened */
    boolean waitUntil(BooleanSupplier cond, double seconds) {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            if (cond.getAsBoolean()) return true;
            sleep(20);
        }
        return cond.getAsBoolean();
    }

    /** True once a warning containing this text shows up */
    boolean waitForWarning(String text, double seconds) {
        return waitUntil(() -> world.warnings().stream().anyMatch(w -> w.contains(text)), seconds);
    }

    List<String> telemetryLines() {
        return engine.telemetryLines();
    }

    void assertNoCrash() {
        if (engine.error() != null) throw new AssertionError("OpMode crashed:\n" + engine.error());
    }

    static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void close() {
        engine.shutdown();
    }
}
