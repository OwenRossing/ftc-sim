package sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.MistakeOpModes;
import org.junit.jupiter.api.Test;

/**
 * The goBILDA StarterBot, driven by goBILDA's own example TeleOp and Auto (unchanged), plus the mistakes
 * students make. Real time: each test takes a few seconds.
 */
class StarterBotTest {

    private static final String TELEOP = "BioBuzz StarterBot Teleop";
    private static final String AUTO = "StarterBotAuto";

    @Test
    void goBildasOpModesAreListed() {
        try (SimHarness sim = new SimHarness()) {
            assertTrue(sim.engine.opModes().containsKey(TELEOP), "OpModes: " + sim.engine.opModes());
            assertTrue(sim.engine.opModes().containsKey(AUTO), "OpModes: " + sim.engine.opModes());
        }
    }

    @Test
    void drivesStraightForward() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(15, 30, 0); // open floor all the way to x 90
            sim.init(TELEOP).start();
            double startX = sim.physics.x;
            sim.hold("ly", -1.0);
            SimHarness.sleep(1000);
            sim.release("ly");
            double moved = sim.physics.x - startX;
            assertTrue(moved > 30, "only went " + moved + " in");
            assertTrue(Math.abs(sim.physics.y - 30) < 2, "drifted sideways to y " + sim.physics.y);
            assertTrue(Math.abs(Math.toDegrees(sim.physics.heading())) < 5, "turned to " + Math.toDegrees(sim.physics.heading()));
            assertTrue(sim.world.warnings().isEmpty(), "warnings: " + sim.world.warnings());
        }
    }

    @Test
    void turnsButCantStrafe() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(40, 30, 0);
            sim.init(TELEOP).start();
            sim.hold("lx", -1.0); // a tank drive ignores this
            SimHarness.sleep(600);
            sim.release("lx");
            assertTrue(Math.abs(sim.physics.y - 30) < 1, "it strafed to y " + sim.physics.y);
            sim.hold("rx", 1.0); // turn right = clockwise = heading goes negative
            SimHarness.sleep(600);
            sim.release("rx");
            assertTrue(Math.toDegrees(sim.physics.heading()) < -30, "heading " + Math.toDegrees(sim.physics.heading()));
        }
    }

    @Test
    void withoutReversingTheRightSideItSpins() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(40, 30, 0);
            String name = sim.engine.addOpMode(MistakeOpModes.NoReverse.class);
            sim.init(name).start();
            double startX = sim.physics.x;
            sim.hold("ly", -1.0);
            SimHarness.sleep(800);
            assertTrue(Math.abs(sim.physics.x - startX) < 4, "it drove " + (sim.physics.x - startX) + " in");
            assertTrue(Math.abs(Math.toDegrees(sim.physics.heading())) > 30, "it only turned "
                    + Math.toDegrees(sim.physics.heading()));
        }
    }

    @Test
    void intakesABallThenLaunchesAllFour() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(40, 30, 0);
            sim.physics.emptyRobot();
            sim.physics.spawnBall(SimCollisions.POLLEN, sim.physics.x + 12, sim.physics.y, 1.4); // touching the intake
            sim.init(TELEOP).start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 1, 2), "never picked it up");
            sim.release("rt");

            sim.physics.loadPreloads(); // 4 to shoot
            sim.hold("rb", true); // spins up, then the windmill feeds once it's over 1200 ticks/s
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 6), "still holding " + sim.physics.balls);
            assertEquals(4, sim.physics.shotsLaunched);
            assertTrue(sim.world.warnings().isEmpty(), "warnings: " + sim.world.warnings());
        }
    }

    @Test
    void launcherHoldsGoBildasVelocity() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init(TELEOP).start();
            sim.physics.emptyRobot(); // so nothing gets fed and slows it down
            sim.hold("rb", true);
            SimHarness.sleep(2000);
            double ticksPerSec = sim.world.motorState("launcher").mechanismRpm / 60 * 537.7;
            assertTrue(Math.abs(ticksPerSec - 1250) < 60, "launcher at " + ticksPerSec + " ticks/s");
        }
    }

    @Test
    void goBildasAutoScoresThenBacksUp() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            int before = sim.physics.collisions.redHive.inCell;
            sim.init(AUTO).start();
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 8), "still holding " + sim.physics.balls);
            assertTrue(sim.waitUntil(() -> sim.physics.collisions.redHive.inCell >= before + 2, 4),
                    "only " + (sim.physics.collisions.redHive.inCell - before) + " landed in the cell");
            // at 10 s it drives back 120 mm with RUN_TO_POSITION, then holds there 2 s
            double x = sim.physics.x;
            assertTrue(sim.waitUntil(() -> sim.telemetryLines().contains("Auto Complete!"), 14),
                    "telemetry: " + sim.telemetryLines());
            assertTrue(Math.abs(x - sim.physics.x - 120 / 25.4) < 1.5, "backed up " + (x - sim.physics.x) + " in");
        }
    }

    @Test
    void launchingWithoutSpinningUpWarns() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.NoSpinUp.class);
            sim.init(name).start();
            assertTrue(sim.waitForWarning("before it was up to speed", 2), "warnings: " + sim.world.warnings());
        }
    }

    @Test
    void typoInADeviceNameCrashesWithAHint() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.Typo.class);
            sim.engine.initOpMode(name);
            assertTrue(sim.waitUntil(() -> sim.engine.error() != null, 2), "didn't crash");
            String error = sim.engine.error();
            assertTrue(error.contains("Unable to find a hardware device with name \"intak\""), error);
            assertTrue(error.contains("Did you mean \"intake\"?"), error);
        }
    }

    @Test
    void forgettingHardwareMapExplainsTheNull() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.NeverGotMotor.class);
            sim.init(name);
            sim.engine.startOpMode();
            assertTrue(sim.waitUntil(() -> sim.engine.error() != null, 2), "didn't crash");
            assertTrue(sim.engine.error().contains("HINT: intake is null"), sim.engine.error());
        }
    }

    @Test
    void whileLoopInsideLoopIsCalledOut() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.WhileInLoop.class);
            sim.init(name).start();
            assertTrue(sim.waitForWarning("stuck inside loop()", SimEngine.STUCK_SEC + 2), "warnings: " + sim.world.warnings());
            String warning = sim.world.warnings().get(0);
            assertTrue(warning.contains("MistakeOpModes.java line"), warning);
        }
    }

    @Test
    void anOpModeThatIgnoresStopIsCalledOutAndCut() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.IgnoresStop.class);
            sim.init(name).start();
            assertTrue(sim.waitUntil(() -> sim.world.motorState("intake").power == 1, 1));
            sim.stop();
            assertTrue(sim.waitForWarning("kept running after STOP", 1), "warnings: " + sim.world.warnings());
            SimHarness.sleep(200);
            assertEquals(0, sim.world.motorState("intake").power, "its writes should be ignored now");
        }
    }

    @Test
    void movingInInitIsAPenalty() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.MovesInInit.class);
            sim.init(name);
            assertTrue(sim.waitForWarning("during INIT", 1), "warnings: " + sim.world.warnings());
            assertTrue(sim.world.warnings().get(0).contains("PENALTY"), sim.world.warnings().get(0));
        }
    }

    @Test
    void oldStyleHardwareMapWorks() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.OldStyle.class);
            assertNotNull(name);
            sim.init(name).start();
            assertTrue(sim.waitUntil(() -> sim.world.motorState("intake").power == 1, 1));
            sim.assertNoCrash();
        }
    }
}
