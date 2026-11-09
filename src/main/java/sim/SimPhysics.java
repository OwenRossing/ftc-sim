package sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * The physics every robot shares: a mecanum drivetrain pushing on the floor, the battery, the balls
 * the robot is holding, and what the Limelight can see. What a robot's own mechanisms do lives in
 * its SimRobot.
 *
 * Field coordinates are Pedro's: inches, 0-144, heading radians counterclockwise.
 */
public final class SimPhysics {

    static final double GRAVITY = 9.81;
    static final double IN_PER_M = 39.37;
    static final double HIVE_AXLE_IN = 43.95; // manual 9.6.1
    static final double BATTERY_REST_VOLTS = 12.7;
    static final double BATTERY_OHMS = 0.06; // battery + wiring, sets the sag when the wheels pull hard. TUNE
    public static final int MAX_BALLS = 4; // G407, no more than 4 at a time

    // FL FR BL BR: how strafing and turning spin each wheel (Pedro's mix)
    private static final double[] STRAFE_SIGN = {-1, 1, 1, -1};
    private static final double[] TURN_SIGN = {-1, 1, -1, 1};

    // the AprilTag clusters, on the bottom face of each cell (field_info.json, inches: x, y, z) with the
    // Hives as staged, 1 = red, then the first tag ID (manual 9.9: audience is low y). They all face the way
    // the Hive's underside does, and turn with it. Each is 4 tags in a row (manual figure 9-15)
    private static final double[][] CELL_TAGS = {
            {59.26, 59.11, 49.67, 1, 34}, {59.26, 83.39, 35.65, 1, 30},
            {84.76, 60.61, 35.65, 0, 38}, {84.76, 84.89, 49.67, 0, 42}};
    private static final double[] TAG_OFFSETS_X = {-6.5, -2.75, 2.75, 6.5};
    private static final double TAG_FACING_MIN = 0.25; // a tag edge-on past about 75 deg can't be read
    static final double CAMERA_HALF_FOV_X = 27.25; // Limelight 3A
    static final double CAMERA_HALF_FOV_Y = 21;
    static final double CAMERA_MAX_RANGE = 130; // 3.25 in tags get too small past about here. TUNE

    private final SimWorld world;
    final SimRobot robot;

    // drivetrain numbers worked out from the robot's
    private final double wheelFreeSpeed;
    private final double turnRadiusM;
    private final double yawInertia;

    // true robot pose on the field
    public volatile double x;
    public volatile double y;
    volatile double heading;
    // body velocity: forward, left (in/s), counterclockwise (rad/s)
    volatile double vForward = 0;
    volatile double vLeft = 0;
    volatile double vTurn = 0;

    volatile double batteryVolts = BATTERY_REST_VOLTS;
    /** Fault injection: a tired battery rests lower */
    public volatile double batteryRestVolts = BATTERY_REST_VOLTS;
    /** Fault injection: the Limelight stops sending frames */
    public volatile boolean cameraDead = false;
    /** For tests: Limelight frames keep coming but there's never a tag in them */
    public volatile boolean limelightHidden = false;

    /** Balls inside the robot, first in is first out (SimCollisions.POLLEN etc.) */
    final java.util.ArrayDeque<Integer> held = new java.util.ArrayDeque<>();
    public volatile int balls = 0;
    public volatile int shotsLaunched = 0;
    final SimCollisions collisions;
    private double stepBacklog = 0;
    private double driveAmps = 0;
    private double publishTimer = 0;

    SimPhysics(SimWorld world, SimRobot robot) {
        this.world = world;
        this.robot = robot;
        wheelFreeSpeed = robot.motorFreeRpm / 60 * 2 * Math.PI * robot.wheelRadiusM;
        turnRadiusM = (robot.trackIn + robot.wheelbaseIn) / 2 * 0.0254;
        yawInertia = robot.massKg * (0.457 * 0.457 * 2) / 12; // an 18 in square box
        x = robot.startX;
        y = robot.startY;
        heading = robot.startHeading;
        collisions = new SimCollisions(robot.massKg, robot.outline(),
                robot.startX / IN_PER_M, robot.startY / IN_PER_M, robot.startHeading);
        loadPreloads();
    }

    /** Fills the robot back up with the 4 pollen every robot starts with (the manual) */
    public void loadPreloads() {
        held.clear();
        for (int i = 0; i < MAX_BALLS; i++) held.add(SimCollisions.POLLEN);
        balls = held.size();
    }

    /** Empties the robot. For tests */
    public void emptyRobot() {
        held.clear();
        balls = 0;
    }

    public double heading() {
        return heading;
    }

    public double batteryVolts() {
        return batteryVolts;
    }

    void placeRobot(double newX, double newY, double newHeading) {
        collisions.setPose(newX / IN_PER_M, newY / IN_PER_M, newHeading);
        x = newX;
        y = newY;
        heading = newHeading;
        vForward = 0;
        vLeft = 0;
        vTurn = 0;
    }

    void resetRobot() {
        placeRobot(robot.startX, robot.startY, robot.startHeading);
        collisions.resetPieces();
        loadPreloads();
        shotsLaunched = 0;
        batteryRestVolts = BATTERY_REST_VOLTS;
        robot.reset();
        world.otos().resetTracking();
    }

    // ---- the step ----

    void step(double dt) {
        // drivetrain + collisions, in the fixed steps the physics engine needs
        stepBacklog = Math.min(stepBacklog + dt, 0.05);
        while (stepBacklog >= SimCollisions.STEP_SEC) {
            stepDrive(SimCollisions.STEP_SEC);
            stepBacklog -= SimCollisions.STEP_SEC;
        }

        for (SimServo servo : world.servos()) servo.step(dt);
        robot.step(dt);
        // encoders count whatever the mechanism did
        for (SimMotor m : world.motors()) m.positionTicks += m.mechanismRpm / 60 * m.ticksPerRev * dt;

        double sag = robot.sagVolts() + BATTERY_OHMS * driveAmps;
        batteryVolts += (batteryRestVolts - sag - batteryVolts) * (1 - Math.exp(-dt / 0.05));

        publishTimer += dt;
        if (publishTimer >= 0.015) {
            publishTimer = 0;
            collisions.publish();
        }
    }

    /** Battery volts over 12, how much a motor slows on a tired battery */
    public double voltScale() {
        return batteryVolts / 12.0;
    }

    // wheel speed from the robot's motion -> motor force -> traction limit -> the physics world
    private void stepDrive(double h) {
        String[] names = robot.wheels();
        SimMotor[] wheels = {world.motor(names[0]), world.motor(names[1]), world.motor(names[2]), world.motor(names[3])};
        double yaw = collisions.heading();
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double forwardMs = collisions.vx() * cos + collisions.vy() * sin;
        double leftMs = -collisions.vx() * sin + collisions.vy() * cos;
        double spin = collisions.vTurn();

        double stallForce = robot.motorStallNm / robot.wheelRadiusM;
        double maxTraction = robot.tractionMu * robot.massKg * GRAVITY / 4;
        double forceForward = 0, forceLeft = 0, torque = 0, amps = 0;
        for (int i = 0; i < 4; i++) {
            SimMotor wheel = wheels[i];
            double duty = wheel.mechanismPower();
            double wheelSpeed = forwardMs + STRAFE_SIGN[i] * leftMs / robot.strafeEfficiency
                    + TURN_SIGN[i] * spin * turnRadiusM;
            double speedRatio = wheelSpeed / wheelFreeSpeed;
            wheel.mechanismRpm = speedRatio * robot.motorFreeRpm;
            // a DC motor: full force at stall, none at free speed. At zero power BRAKE shorts the motor
            // (it fights the motion), FLOAT doesn't
            boolean floating = duty == 0 && wheel.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT;
            double drive = duty * batteryVolts / 12.0 - speedRatio;
            double force = floating ? 0 : Range.clip(stallForce * drive, -maxTraction, maxTraction);
            forceForward += force;
            forceLeft += STRAFE_SIGN[i] * force * robot.strafeEfficiency;
            torque += TURN_SIGN[i] * force * turnRadiusM;
            wheel.currentAmps = floating ? 0 : robot.motorStallAmps * Math.abs(drive);
            amps += Math.abs(duty) * wheel.currentAmps;
        }
        driveAmps = amps;

        // rollers and carpet: opposes motion, can stop the robot, can't push it backwards
        double rollForce = robot.rollingResist * robot.massKg * GRAVITY;
        forceForward = withDrag(forceForward, forwardMs, rollForce, robot.massKg / h);
        forceLeft = withDrag(forceLeft, leftMs, rollForce, robot.massKg / h);
        torque = withDrag(torque, spin, rollForce * turnRadiusM, yawInertia / h);

        collisions.step(forceForward * cos - forceLeft * sin, forceForward * sin + forceLeft * cos, torque);

        // what actually happened after contacts. The OTOS only sees real motion, so pushing a wall doesn't count
        double newYaw = collisions.heading();
        double c = Math.cos(newYaw);
        double s = Math.sin(newYaw);
        double actualForward = collisions.vx() * c + collisions.vy() * s;
        double actualLeft = -collisions.vx() * s + collisions.vy() * c;
        vForward = actualForward * IN_PER_M;
        vLeft = actualLeft * IN_PER_M;
        vTurn = collisions.vTurn();
        x = collisions.x() * IN_PER_M;
        y = collisions.y() * IN_PER_M;
        heading = newYaw;
        world.otos().integrate(vForward, vLeft, vTurn, h);
    }

    // friction force on an axis: never bigger than what it takes to stop it this step
    private static double withDrag(double force, double v, double drag, double massPerStep) {
        double toStop = force + v * massPerStep;
        if (Math.abs(toStop) <= drag) return -v * massPerStep;
        return force - Math.copySign(drag, toStop);
    }

    // ---- balls, for the robots' mechanisms ----

    /**
     * Picks up a ball lying in this box in front of the robot (robot frame, inches), if there's room.
     * Returns true if it got one
     */
    public boolean capture(double xMin, double xMax, double halfWidth, double zMax) {
        if (held.size() >= MAX_BALLS) return false;
        int type = collisions.capture(xMin, xMax, halfWidth, zMax);
        if (type < 0) return false;
        held.addLast(type);
        balls = held.size();
        return true;
    }

    /** The flower bar: takes the bottom pollen of a flower stack in this box. True if it got one */
    public boolean takeFromFlower(double xMin, double xMax, double halfWidth) {
        if (held.size() >= MAX_BALLS || !collisions.takeFromFlower(xMin, xMax, halfWidth)) return false;
        held.addLast(SimCollisions.POLLEN);
        balls = held.size();
        return true;
    }

    /** The next ball to shoot, or -1 if it's empty */
    public int nextBall() {
        Integer next = held.peekFirst();
        return next == null ? -1 : next;
    }

    /**
     * Shoots the next ball. The barrel is at (forward, left, height) inches from the robot center, aimed along
     * aimHeading (field radians) and tilted up by elevation (radians), leaving at speed m/s.
     * Returns the type it shot, or -1 if it was empty
     */
    public int launch(double forward, double left, double height, double aimHeading, double elevation, double speed) {
        Integer type = held.pollFirst();
        if (type == null) return -1;
        balls = held.size();
        double[] p = toField(forward, left);
        double flat = speed * Math.cos(elevation);
        collisions.spawn(type, p[0] / IN_PER_M, p[1] / IN_PER_M, height / IN_PER_M,
                flat * Math.cos(aimHeading) + collisions.vx(), flat * Math.sin(aimHeading) + collisions.vy(),
                speed * Math.sin(elevation));
        return type;
    }

    /** Spits the last ball back out the front, rolling. Returns false if it was empty */
    public boolean eject(double frontX) {
        Integer type = held.pollLast();
        if (type == null) return false;
        balls = held.size();
        double c = Math.cos(heading), s = Math.sin(heading);
        double out = 1.5;
        collisions.spawn(type, (x + frontX * c) / IN_PER_M, (y + frontX * s) / IN_PER_M, 3 / IN_PER_M,
                out * c + collisions.vx(), out * s + collisions.vy(), 0);
        return true;
    }

    /** Drops a ball on the field, inches. For tests */
    public void spawnBall(int type, double xIn, double yIn, double zIn) {
        collisions.spawn(type, xIn / IN_PER_M, yIn / IN_PER_M, zIn / IN_PER_M, 0, 0, 0);
    }

    /** Where a point in the robot frame (inches) is on the field */
    public double[] toField(double forward, double left) {
        return new double[]{x + forward * Math.cos(heading) - left * Math.sin(heading),
                y + forward * Math.sin(heading) + left * Math.cos(heading)};
    }

    /** The robot's velocity on the field, m/s */
    public double[] fieldVelocity() {
        return new double[]{collisions.vx(), collisions.vy()};
    }

    // ---- the Limelight ----

    /**
     * What the Limelight sees right now, each tag it can read as {id, tx, ty, ta}, biggest first.
     * A tag has to be in view, in range and facing the camera. A cell tipped down toward you has its
     * bottom face turned away, which is how you can tell not to shoot there. Things in the way aren't checked.
     */
    java.util.List<double[]> limelightTags() {
        java.util.List<double[]> seen = new java.util.ArrayList<>();
        double[] cam = robot.cameraPose();
        if (cam == null || limelightHidden) return seen;
        double camX = cam[0], camY = cam[1], camZ = cam[2], aim = cam[3], tilt = cam[4];
        for (double[] tag : CELL_TAGS) for (int n = 0; n < TAG_OFFSETS_X.length; n++) {
            boolean red = tag[3] == 1;
            double[] m = hiveMirror(red);
            double hive = red ? collisions.redHive.angle : collisions.blueHive.angle;
            double c = Math.cos(hive), s = Math.sin(hive);
            double dy = tag[1] - 72, dz = tag[2] - HIVE_AXLE_IN;
            double tagY = 72 + dy * c - dz * s;
            double tagZ = HIVE_AXLE_IN + dy * s + dz * c;
            double normalY = m[0] * c - m[1] * s;
            double normalZ = m[0] * s + m[1] * c;

            double toX = tag[0] + TAG_OFFSETS_X[n] - camX, toY = tagY - camY, toZ = tagZ - camZ;
            double range = Math.sqrt(toX * toX + toY * toY + toZ * toZ);
            double facing = -(toY * normalY + toZ * normalZ) / range;
            if (range > CAMERA_MAX_RANGE || facing < TAG_FACING_MIN) continue;

            double flat = Math.hypot(toX, toY);
            double tx = Math.toDegrees(wrap(aim - Math.atan2(toY, toX))); // right of center is positive
            double ty = Math.toDegrees(Math.atan2(toZ, flat) - tilt);
            if (Math.abs(tx) > CAMERA_HALF_FOV_X || Math.abs(ty) > CAMERA_HALF_FOV_Y) continue;

            double area = 100 * (3.25 * 3.25 * facing) / (range * range);
            seen.add(new double[]{tag[4] + n, tx, ty, area});
        }
        seen.sort((a, b) -> Double.compare(b[3], a[3]));
        return seen;
    }

    // which way the bottom of a Hive faces as staged (y, z): 30 deg off straight down, the red one toward
    // low y and the blue one toward high y, since they're tilted opposite ways
    private static double[] hiveMirror(boolean red) {
        return new double[]{red ? -0.5 : 0.5, -0.866};
    }

    static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
