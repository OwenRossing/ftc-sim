package sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The mecanum StarterBot (robot=starter), driven by goBILDA's own mecanum TeleOp, unchanged. Real time.
 */
class MecanumStarterBotTest {

    private static final String TELEOP = "Mec BioBuzz StarterBot Teleop";

    @Test
    void isTheDefaultRobot() {
        try (SimHarness sim = new SimHarness(SimRobot.load("starter"))) {
            assertEquals("goBILDA StarterBot (mecanum)", sim.world.robot().displayName());
            assertTrue(sim.engine.opModes().containsKey(TELEOP), "OpModes: " + sim.engine.opModes());
        }
    }

    @Test
    void drivesStrafesAndTurns() throws Exception {
        try (SimHarness sim = new SimHarness(new StarterRobot(true))) {
            sim.physics.placeRobot(15, 30, 0); // open floor all the way to x 90
            sim.init(TELEOP).start();
            sim.hold("ly", -1.0);
            SimHarness.sleep(900);
            sim.release("ly");
            assertTrue(sim.physics.x - 15 > 30, "only went " + (sim.physics.x - 15) + " in");
            assertTrue(Math.abs(sim.physics.y - 30) < 2, "drifted sideways to y " + sim.physics.y);
            assertTrue(Math.abs(Math.toDegrees(sim.physics.heading())) < 5, "turned to " + Math.toDegrees(sim.physics.heading()));

            SimHarness.sleep(500);
            double y = sim.physics.y;
            sim.hold("lx", -1.0); // strafe left = +y
            SimHarness.sleep(700);
            sim.release("lx");
            assertTrue(sim.physics.y - y > 10, "strafed only " + (sim.physics.y - y) + " in");

            sim.hold("rx", 1.0); // turn right = clockwise = heading goes negative
            SimHarness.sleep(500);
            sim.release("rx");
            assertTrue(Math.toDegrees(sim.physics.heading()) < -30, "heading " + Math.toDegrees(sim.physics.heading()));
            assertTrue(sim.world.warnings().isEmpty(), "warnings: " + sim.world.warnings());
        }
    }

    @Test
    void intakesABallThenLaunchesAllFourIntoTheHive() throws Exception {
        try (SimHarness sim = new SimHarness(new StarterRobot(true))) {
            int before = sim.physics.collisions.redHive.inCell;
            double x0 = sim.physics.x, y0 = sim.physics.y;
            sim.physics.placeRobot(40, 30, 0);
            sim.physics.emptyRobot();
            sim.physics.spawnBall(SimCollisions.POLLEN, 52, 30, 1.4); // touching the corner wheels
            sim.init(TELEOP).start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 1, 2), "never picked it up");
            sim.release("rt");

            // back to the start, facing the Hive, and fire 4
            sim.physics.placeRobot(x0, y0, 0);
            sim.physics.loadPreloads();
            sim.hold("rb", true);
            assertTrue(sim.waitUntil(() -> sim.physics.shotsLaunched >= 4, 6), "shot " + sim.physics.shotsLaunched);
            // more is fine: goBILDA's code runs the intake while launching, so it picks up loose pollen and fires that too
            // scoring: balls land in the cell, and enough of them tip the Hive (which empties the cell)
            assertTrue(sim.waitUntil(() -> sim.physics.collisions.redHive.inCell >= before + 2
                            || sim.physics.collisions.redHive.tips > 0, 4),
                    "only " + (sim.physics.collisions.redHive.inCell - before) + " landed in the cell, no tip");
        }
    }

    @Test
    void theTankVersionsWindmillNameGetsAHint() throws Exception {
        try (SimHarness sim = new SimHarness(new StarterRobot(true))) {
            String name = sim.engine.addOpMode(TankNameOnMecanum.class);
            sim.engine.initOpMode(name);
            assertTrue(sim.waitUntil(() -> sim.engine.error() != null, 2), "didn't crash");
            assertTrue(sim.engine.error().contains("Did you mean \"windmillServo\"?"), sim.engine.error());
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tank name on mecanum", group = "test")
    public static class TankNameOnMecanum extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
        @Override
        public void init() {
            hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "windmill");
        }

        @Override
        public void loop() {}
    }
}
