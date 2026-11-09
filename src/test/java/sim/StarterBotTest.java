package sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.MistakeOpModes;
import org.junit.jupiter.api.Test;

/**
 * The starter bot, driven by the answer key the way a student would, plus the mistakes students make.
 * Real time: each test takes a few seconds.
 */
class StarterBotTest {

    private static final String ANSWER_KEY = "Starter Bot (answer key)";

    @Test
    void answerKeyIsListed() {
        try (SimHarness sim = new SimHarness()) {
            assertTrue(sim.engine.opModes().containsKey(ANSWER_KEY), "OpModes: " + sim.engine.opModes());
        }
    }

    @Test
    void drivesStraightForward() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(15, 30, 0); // open floor all the way to x 90 (the start faces the Hives)
            sim.init(ANSWER_KEY).start();
            double startX = sim.physics.x;
            sim.hold("ly", -1.0);
            SimHarness.sleep(1000);
            sim.release("ly");
            double moved = sim.physics.x - startX;
            assertTrue(moved > 40, "only went " + moved + " in");
            assertTrue(Math.abs(sim.physics.y - 30) < 3, "drifted sideways to y " + sim.physics.y);
            assertTrue(Math.abs(Math.toDegrees(sim.physics.heading())) < 5, "turned to " + Math.toDegrees(sim.physics.heading()));
            assertTrue(sim.world.warnings().isEmpty(), "warnings: " + sim.world.warnings());
        }
    }

    @Test
    void strafesAndTurns() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init(ANSWER_KEY).start();
            double startY = sim.physics.y;
            sim.hold("lx", -1.0); // left
            SimHarness.sleep(800);
            sim.release("lx");
            assertTrue(sim.physics.y - startY > 12, "strafed only to y " + sim.physics.y);
            sim.hold("rx", 1.0); // turn right = clockwise = heading goes negative
            SimHarness.sleep(500);
            sim.release("rx");
            assertTrue(Math.toDegrees(sim.physics.heading()) < -30, "heading " + Math.toDegrees(sim.physics.heading()));
        }
    }

    @Test
    void withoutReversingTheLeftSideItSpins() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.NoReverse.class);
            sim.init(name).start();
            double startX = sim.physics.x;
            sim.hold("ly", -1.0);
            SimHarness.sleep(800);
            assertTrue(Math.abs(sim.physics.x - startX) < 4, "it drove " + (sim.physics.x - startX) + " in");
            assertTrue(Math.abs(Math.toDegrees(sim.physics.heading())) > 45, "it only turned "
                    + Math.toDegrees(sim.physics.heading()));
        }
    }

    @Test
    void intakePicksUpABallAndTheShooterFiresAll() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.emptyRobot();
            sim.physics.spawnBall(SimCollisions.POLLEN, sim.physics.x + 12, sim.physics.y, 1.4); // touching the roller
            sim.init(ANSWER_KEY).start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 1, 2), "never picked it up");
            sim.release("rt");

            sim.physics.loadPreloads(); // 4 to shoot
            sim.hold("lt", 1.0);
            SimHarness.sleep(1500); // spin up
            sim.hold("a", true);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 3), "still holding " + sim.physics.balls);
            assertEquals(4, sim.physics.shotsLaunched);
            assertTrue(sim.world.warnings().isEmpty(), "warnings: " + sim.world.warnings());
        }
    }

    @Test
    void shootingWithoutSpinningUpWarns() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            String name = sim.engine.addOpMode(MistakeOpModes.NoSpinUp.class);
            sim.init(name).start();
            assertTrue(sim.waitForWarning("before it was spinning", 2), "warnings: " + sim.world.warnings());
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
