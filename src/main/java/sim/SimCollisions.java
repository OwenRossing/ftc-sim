package sim;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The physical world, using ode4j. SI units, Pedro's field axes: x,y on the floor (0 to 3.6576 m),
 * z up. The shapes come from the CAD (sim/cad/field_colliders.py).
 *
 * Robot: a body that only slides and spins on the floor. SimPhysics works out the wheel forces,
 * this applies them and resolves contacts. It can't tip in this model.
 * Balls: real spheres, with air drag and rolling loss.
 * Hives: real hinged bodies with hollow cells. The manual says they're bi-stable (they hold either
 * end until enough weight lands in the upward cell), modeled as a torque toward the nearer end.
 *
 * Everything that isn't from the CAD or the manual is marked TUNE.
 */
public final class SimCollisions {

    static final double STEP_SEC = 1.0 / 400; // ode needs fixed steps to stay stable
    public static final int POLLEN = 0;
    public static final int RED_NECTAR = 1;
    public static final int BLUE_NECTAR = 2;

    private static final int MAX_CONTACTS = 8;
    private static final double IN = 0.0254;
    private static final double FIELD_SIZE = 3.6576;
    private static final double WALL_INSET = 0.025; // the inside face of the rails is about an inch in from 72 in

    // balls, sizes from the CAD and manual. Masses aren't in the manual, TUNE on a scale
    private static final double POLLEN_RADIUS = 1.4 * IN;
    private static final double NECTAR_RADIUS = 1.81 * IN;
    private static final double POLLEN_MASS = 0.015;
    private static final double NECTAR_MASS = 0.027;
    private static final double AIR_DRAG = 0.5 * 1.2 * 0.5; // 1/2 rho Cd, times area and v squared. TUNE
    private static final double ROLL_LOSS_PER_SEC = 1.5; // how fast a ball rolling on tile slows. TUNE
    private static final double FLOWER_STACK_SPACING = 2.9 * IN; // from the CAD's stacked pollen
    private static final double MAX_BALL_SPEED = 15; // a robot squeezing a ball can't fire it off at silly speeds

    // hives: the axle is 43.95 in above the tiles (manual 9.6.1), which is also the height that makes a
    // 60 deg swing land the cell on its mirror position. 30 deg each way is manual figure 9-10
    private static final double AXLE_Z = 43.95 * IN;
    private static final double TILT_TOTAL = Math.toRadians(60);
    private static final double HIVE_MASS = 3.0; // TUNE
    // holds it at either end. The 3 staged nectar must not tip it, about 5 more pollen should. TUNE
    // what holds a Hive at its end. The manual gives no tip count: this tips after 5 to 8 pollen on top of the
    // 3 staged nectar, measured by dropping them in. Under about 0.20 the nectar tips it alone. TUNE to a real count
    static double bistableTorque = 0.22; // the settings page changes it
    private static final double DAMPER = 0.5; // TUNE, N*m per rad/s

    private enum Kind { FLOOR, WALL, STATIC, ROBOT, HIVE, BALL }

    /** One game piece. Frozen ones (in flowers, off the field) have no body, they're just staged. */
    private static final class Ball {
        final int type;
        final double radius;
        final double mass;
        DBody body;
        DGeom geom;
        double x, y, z;

        Ball(int type, double x, double y, double z) {
            this.type = type;
            this.radius = type == POLLEN ? POLLEN_RADIUS : NECTAR_RADIUS;
            this.mass = type == POLLEN ? POLLEN_MASS : NECTAR_MASS;
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    /** A hive: one hinged body, a cell either side of the axle */
    final class Hive {
        final DBody body;
        final double axleX;
        final double loStop;
        final double hiStop;
        final double[][] cells = {{1e9, 1e9, 1e9, -1e9, -1e9, -1e9}, {1e9, 1e9, 1e9, -1e9, -1e9, -1e9}};
        volatile double angle = 0;
        volatile int tips = 0;
        volatile int inCell = 0;
        private double settledEnd; // the end it's resting at, a tip is reaching the other one

        Hive(List<double[]> plates, double loStop, double hiStop) {
            this.loStop = loStop;
            this.hiStop = hiStop;
            double sum = 0;
            for (double[] p : plates) sum += p[0];
            axleX = sum / plates.size();

            body = OdeHelper.createBody(world);
            DMass mass = OdeHelper.createMass();
            mass.setBoxTotal(HIVE_MASS, 0.6, 0.9, 0.3);
            body.setMass(mass);
            body.setPosition(axleX, FIELD_SIZE / 2, AXLE_Z);
            body.setGravityMode(false); // balanced on the axle, the bi-stable torque is what holds it
            body.setAutoDisableFlag(false);

            for (double[] p : plates) {
                DBox box = OdeHelper.createBox(space, 2 * p[12], 2 * p[13], 2 * p[14]);
                box.setData(Kind.HIVE);
                box.setBody(body);
                box.setOffsetPosition(p[0] - axleX, p[1] - FIELD_SIZE / 2, p[2] - AXLE_Z);
                box.setOffsetRotation(rotationOf(p));
                // which cell this plate belongs to, for counting what's in them
                int cell = p[1] > FIELD_SIZE / 2 ? 1 : 0;
                for (int i = 0; i < 3; i++) {
                    double reach = Math.abs(p[3 + i]) * p[12] + Math.abs(p[6 + i]) * p[13] + Math.abs(p[9 + i]) * p[14];
                    cells[cell][i] = Math.min(cells[cell][i], p[i] - reach);
                    cells[cell][3 + i] = Math.max(cells[cell][3 + i], p[i] + reach);
                }
            }
            DHingeJoint hinge = OdeHelper.createHingeJoint(world);
            hinge.attach(body, null);
            hinge.setAnchor(axleX, FIELD_SIZE / 2, AXLE_Z);
            hinge.setAxis(1, 0, 0);
        }

        private void step() {
            double[] r = {body.getRotation().get11(), body.getRotation().get21()};
            angle = Math.atan2(r[1], r[0]);
            double spin = body.getAngularVel().get0();
            double mid = (loStop + hiStop) / 2;
            double torque = (angle > mid ? bistableTorque : -bistableTorque) - DAMPER * spin;
            body.addTorque(torque, 0, 0);
            // the ends are the frame: a stiff stop with no bounce (that's what the real damper is for)
            if (angle <= loStop && spin < 0 || angle >= hiStop && spin > 0) {
                body.setAngularVel(0, 0, 0);
                setAngle(angle <= loStop ? loStop : hiStop);
            }
            double near = Math.abs(angle - loStop) < Math.toRadians(3) ? loStop
                    : Math.abs(angle - hiStop) < Math.toRadians(3) ? hiStop : Double.NaN;
            if (!Double.isNaN(near) && near != settledEnd) {
                settledEnd = near;
                tips++; // the manual's HIVE TIP: moved to the other stable end
            }
        }

        private void setAngle(double a) {
            body.setQuaternion(new DQuaternion(Math.cos(a / 2), Math.sin(a / 2), 0, 0));
        }

        private void reset() {
            angle = 0;
            settledEnd = 0;
            tips = 0;
            inCell = 0;
            setAngle(0);
            body.setAngularVel(0, 0, 0);
            body.setLinearVel(0, 0, 0);
            body.setPosition(axleX, FIELD_SIZE / 2, AXLE_Z);
        }

        /** Is this point inside either cell? Rotates it back to the pose the cell boxes were measured in */
        boolean holds(double px, double py, double pz) {
            double dy = py - FIELD_SIZE / 2, dz = pz - AXLE_Z;
            double c = Math.cos(-angle), s = Math.sin(-angle);
            double y0 = dy * c - dz * s + FIELD_SIZE / 2, z0 = dy * s + dz * c + AXLE_Z;
            for (double[] b : cells) {
                if (px > b[0] && px < b[3] && y0 > b[1] && y0 < b[4] && z0 > b[2] && z0 < b[5]) return true;
            }
            return false;
        }
    }

    private final DWorld world = OdeHelper.createWorld();
    private final DSpace space = OdeHelper.createHashSpace();
    private final DJointGroup contactGroup = OdeHelper.createJointGroup();
    private final DBody robot;
    private final DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
    final Hive redHive;
    final Hive blueHive;

    private final List<Ball> balls = new ArrayList<>();
    private final List<double[]> staging = new ArrayList<>(); // type, x, y, z, frozen(0/1)
    private volatile double[] pieceState = new double[0];

    private volatile double x, y, heading, vx, vy, vTurn;
    private int wakeSteps = 0;

    static {
        OdeHelper.initODE2(0);
    }

    /**
     * @param outline the robot's boxes, {centerX, centerY, centerZ, halfX, halfY, halfZ} inches in the robot frame
     */
    SimCollisions(double massKg, List<double[]> outline, double startX, double startY, double startHeading) {
        world.setGravity(0, 0, -9.81);
        world.setQuickStepNumIterations(20);
        world.setContactSurfaceLayer(0.0005);

        OdeHelper.createPlane(space, 0, 0, 1, 0).setData(Kind.FLOOR);
        OdeHelper.createPlane(space, 1, 0, 0, WALL_INSET).setData(Kind.WALL);
        OdeHelper.createPlane(space, -1, 0, 0, -(FIELD_SIZE - WALL_INSET)).setData(Kind.WALL);
        OdeHelper.createPlane(space, 0, 1, 0, WALL_INSET).setData(Kind.WALL);
        OdeHelper.createPlane(space, 0, -1, 0, -(FIELD_SIZE - WALL_INSET)).setData(Kind.WALL);

        String json = resource("/field_colliders.json");
        for (double[] v : boxes(json, "\"static\":", "\"red_hive\":")) {
            DBox box = OdeHelper.createBox(space, 2 * v[12], 2 * v[13], 2 * v[14]);
            box.setData(Kind.STATIC);
            box.setPosition(v[0], v[1], v[2]);
            box.setRotation(rotationOf(v));
        }
        // from their CAD pose red can only rotate one way and blue the other, see the tag heights
        redHive = new Hive(boxes(json, "\"red_hive\":", "\"blue_hive\":"), 0, TILT_TOTAL);
        blueHive = new Hive(boxes(json, "\"blue_hive\":", "\"axle\":"), -TILT_TOTAL, 0);
        loadStaging(json);

        robot = OdeHelper.createBody(world);
        DMass mass = OdeHelper.createMass();
        mass.setBoxTotal(massKg, 18 * IN, 18 * IN, 18 * IN);
        robot.setMass(mass);
        robot.setGravityMode(false); // it just stays on the floor
        robot.setAutoDisableFlag(false);
        for (double[] b : outline) addRobotBox(b);
        setPose(startX, startY, startHeading);
        resetPieces();
    }

    /**
     * Adds a box to the robot, {centerX, centerY, centerZ, halfX, halfY, halfZ} inches in the robot frame.
     * Keep it to swing it later with swingBox
     */
    public synchronized Object addRobotBox(double[] b) {
        DBox box = OdeHelper.createBox(space, 2 * b[3] * IN, 2 * b[4] * IN, 2 * b[5] * IN);
        box.setData(Kind.ROBOT);
        box.setBody(robot);
        box.setOffsetPosition(b[0] * IN, b[1] * IN, b[2] * IN);
        return box;
    }

    /**
     * Swings a robot box on a sideways hinge, like an intake on a pivot. degUp lifts the front.
     * box and pivot are {x, y, z} inches in the robot frame, with the box where it is at 0 degrees
     */
    public synchronized void swingBox(Object handle, double[] pivot, double[] center, double degUp) {
        DBox box = (DBox) handle;
        double t = Math.toRadians(-degUp); // rotating about +y by a negative angle lifts the front
        double c = Math.cos(t), s = Math.sin(t);
        double dx = center[0] - pivot[0], dz = center[2] - pivot[2];
        box.setOffsetPosition((pivot[0] + c * dx + s * dz) * IN, center[1] * IN,
                (pivot[2] - s * dx + c * dz) * IN);
        box.setOffsetRotation(new DMatrix3(c, 0, s, 0, 1, 0, -s, 0, c));
    }

    synchronized void setPose(double newX, double newY, double newHeading) {
        robot.setPosition(newX, newY, 0);
        robot.setQuaternion(new DQuaternion(Math.cos(newHeading / 2), 0, 0, Math.sin(newHeading / 2)));
        robot.setLinearVel(0, 0, 0);
        robot.setAngularVel(0, 0, 0);
        read();
    }

    /**
     * One fixed step with these forces on the robot (world frame, N and N*m)
     */
    synchronized void step(double forceX, double forceY, double torqueZ) {
        robot.addForce(forceX, forceY, 0);
        robot.addTorque(0, 0, torqueZ);
        redHive.step();
        blueHive.step();
        // resting balls go to sleep to save CPU, and nothing wakes one when its floor swings away
        boolean hiveMoving = Math.abs(redHive.body.getAngularVel().get0()) > 0.01
                || Math.abs(blueHive.body.getAngularVel().get0()) > 0.01;
        // and a ball that's only just started rolling is slow enough to doze off again, give it a few seconds
        wakeSteps = hiveMoving ? (int) (3 / STEP_SEC) : Math.max(0, wakeSteps - 1);
        for (Ball ball : balls) {
            if (wakeSteps > 0 && ball.body != null && !ball.body.isEnabled()) ball.body.enable();
            stepBall(ball);
        }

        OdeHelper.spaceCollide(space, null, this::nearCallback);
        world.quickStep(STEP_SEC);
        contactGroup.empty();

        // keep the robot flat on the floor
        DVector3C p = robot.getPosition();
        DVector3C v = robot.getLinearVel();
        double yaw = yawOf();
        double spin = robot.getAngularVel().get2();
        robot.setPosition(p.get0(), p.get1(), 0);
        robot.setLinearVel(v.get0(), v.get1(), 0);
        robot.setAngularVel(0, 0, spin);
        robot.setQuaternion(new DQuaternion(Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)));
        read();
    }

    private void stepBall(Ball ball) {
        if (ball.body == null || !ball.body.isEnabled()) return;
        DVector3C v = ball.body.getLinearVel();
        double vx0 = v.get0(), vy0 = v.get1(), vz0 = v.get2();
        double speed = Math.sqrt(vx0 * vx0 + vy0 * vy0 + vz0 * vz0);
        if (speed > MAX_BALL_SPEED) {
            double k = MAX_BALL_SPEED / speed;
            ball.body.setLinearVel(vx0 * k, vy0 * k, vz0 * k);
        }
        double area = Math.PI * ball.radius * ball.radius;
        double drag = AIR_DRAG * area * speed;
        double fx = -drag * vx0, fy = -drag * vy0, fz = -drag * vz0;
        DVector3C p = ball.body.getPosition();
        if (p.get2() < ball.radius + 0.003) {
            // rolling on the tile. Wiffle balls with holes in foam stop quick
            fx -= ball.mass * ROLL_LOSS_PER_SEC * vx0;
            fy -= ball.mass * ROLL_LOSS_PER_SEC * vy0;
        }
        ball.body.addForce(fx, fy, fz);
    }

    private void nearCallback(Object data, DGeom a, DGeom b) {
        DBody bodyA = a.getBody();
        DBody bodyB = b.getBody();
        if (bodyA == bodyB) return; // same robot or hive, or two field parts
        Kind kindA = (Kind) a.getData();
        Kind kindB = (Kind) b.getData();
        boolean anyBall = kindA == Kind.BALL || kindB == Kind.BALL;
        if (!anyBall) {
            // only the robot hits things that aren't balls: hives go through their frame (it holds them),
            // and the robot's bottom doesn't touch the floor (it's kept on it)
            boolean robotInvolved = kindA == Kind.ROBOT || kindB == Kind.ROBOT;
            boolean hive = kindA == Kind.HIVE || kindB == Kind.HIVE;
            boolean floor = kindA == Kind.FLOOR || kindB == Kind.FLOOR;
            if (!robotInvolved || hive || floor) return;
        }
        int n = OdeHelper.collide(a, b, MAX_CONTACTS, contacts.getGeomBuffer());
        for (int i = 0; i < n; i++) {
            DContact contact = contacts.get(i);
            // Approx1 makes mu a real friction coefficient. Without it mu is a fixed force in newtons,
            // which is more than a ball weighs: balls would cling to slopes and creep
            contact.surface.mode = OdeConstants.dContactBounce | OdeConstants.dContactSoftCFM | OdeConstants.dContactApprox1;
            if (anyBall) {
                // polyethylene on polycarb is slippery, on foam tile it grips. TUNE
                boolean onHive = kindA == Kind.HIVE || kindB == Kind.HIVE;
                contact.surface.mu = onHive ? 0.2 : 0.5;
                contact.surface.bounce = 0.4; // wiffle ball on foam, TUNE with a drop test
                contact.surface.bounce_vel = 0.2;
                contact.surface.soft_cfm = 1e-4;
            } else {
                contact.surface.mu = 0.15; // bumpers slide along walls instead of grabbing them
                contact.surface.bounce = 0.05;
                contact.surface.bounce_vel = 0.1;
                contact.surface.soft_cfm = 1e-5;
            }
            DJoint joint = OdeHelper.createContactJoint(world, contactGroup, contact);
            joint.attach(bodyA, bodyB);
        }
    }

    private double yawOf() {
        return Math.atan2(robot.getRotation().get10(), robot.getRotation().get00());
    }

    private void read() {
        DVector3C p = robot.getPosition();
        DVector3C v = robot.getLinearVel();
        x = p.get0();
        y = p.get1();
        heading = yawOf();
        vx = v.get0();
        vy = v.get1();
        vTurn = robot.getAngularVel().get2();
    }

    double x() { return x; }
    double y() { return y; }
    double heading() { return heading; }
    double vx() { return vx; }
    double vy() { return vy; }
    double vTurn() { return vTurn; }

    // ---------------------------------------------------------------- balls

    /**
     * Adds a live ball (meters, m/s)
     */
    synchronized void spawn(int type, double px, double py, double pz, double vxBall, double vyBall, double vzBall) {
        Ball ball = new Ball(type, px, py, pz);
        ball.body = OdeHelper.createBody(world);
        DMass mass = OdeHelper.createMass();
        mass.setSphereTotal(ball.mass, ball.radius);
        ball.body.setMass(mass);
        ball.body.setPosition(px, py, pz);
        ball.body.setLinearVel(vxBall, vyBall, vzBall);
        ball.body.setAngularDamping(0.002);
        ball.body.setAutoDisableFlag(true);
        DSphere sphere = OdeHelper.createSphere(space, ball.radius);
        sphere.setData(Kind.BALL);
        sphere.setBody(ball.body);
        ball.geom = sphere;
        balls.add(ball);
    }

    /**
     * Takes a live ball out of a box in the robot's frame (inches), returns its type or -1
     */
    synchronized int capture(double xMin, double xMax, double halfWidth, double zMax) {
        double c = Math.cos(heading), s = Math.sin(heading);
        for (Iterator<Ball> it = balls.iterator(); it.hasNext(); ) {
            Ball ball = it.next();
            if (ball.body == null) continue;
            DVector3C p = ball.body.getPosition();
            double dx = p.get0() - x, dy = p.get1() - y;
            double fwd = (dx * c + dy * s) / IN;
            double left = (-dx * s + dy * c) / IN;
            if (fwd > xMin && fwd < xMax && Math.abs(left) < halfWidth && p.get2() / IN < zMax) {
                remove(ball);
                it.remove();
                return ball.type;
            }
        }
        return -1;
    }

    /**
     * The flower bar pulling pollen out of a flower's retrieval opening: takes the bottom ball of a stack
     * in this box in the robot's frame (inches) and the rest drop down one. True if it got one
     */
    synchronized boolean takeFromFlower(double xMin, double xMax, double halfWidth) {
        double c = Math.cos(heading), s = Math.sin(heading);
        for (Ball ball : balls) {
            if (ball.body != null || ball.type != POLLEN || ball.z > 3 * IN) continue;
            double dx = ball.x - x, dy = ball.y - y;
            double fwd = (dx * c + dy * s) / IN;
            double left = (-dx * s + dy * c) / IN;
            if (fwd > xMin && fwd < xMax && Math.abs(left) < halfWidth) {
                balls.remove(ball);
                for (Ball other : balls) {
                    if (other.body == null && Math.hypot(other.x - ball.x, other.y - ball.y) < 2 * IN) {
                        other.z -= FLOWER_STACK_SPACING;
                    }
                }
                return true;
            }
        }
        return false;
    }

    private void remove(Ball ball) {
        if (ball.geom != null) ball.geom.destroy();
        if (ball.body != null) ball.body.destroy();
    }

    /**
     * Back to the match setup in the manual
     */
    synchronized void resetPieces() {
        for (Ball ball : balls) remove(ball);
        balls.clear();
        redHive.reset();
        blueHive.reset();
        for (double[] s : staging) {
            if (s[4] == 0) {
                spawn((int) s[0], s[1], s[2], s[3], 0, 0, 0);
            } else {
                balls.add(new Ball((int) s[0], s[1], s[2], s[3]));
            }
        }
        publish();
    }

    /**
     * Copies out what the browser needs and counts what's in the cells. Slower than a physics step needs
     */
    synchronized void publish() {
        double[] out = new double[balls.size() * 4];
        int red = 0, blue = 0, i = 0;
        for (Iterator<Ball> it = balls.iterator(); it.hasNext(); ) {
            Ball ball = it.next();
            if (ball.body != null) {
                DVector3C p = ball.body.getPosition();
                ball.x = p.get0();
                ball.y = p.get1();
                ball.z = p.get2();
                if (ball.z < -0.5) { // fell through something, shouldn't happen but don't keep simulating it
                    remove(ball);
                    it.remove();
                    continue;
                }
                // settled balls only, one flying through doesn't count as in the cell
                DVector3C v = ball.body.getLinearVel();
                boolean settled = v.get0() * v.get0() + v.get1() * v.get1() + v.get2() * v.get2() < 0.25;
                if (settled && redHive.holds(ball.x, ball.y, ball.z)) red++;
                else if (settled && blueHive.holds(ball.x, ball.y, ball.z)) blue++;
            }
            out[i++] = ball.type;
            out[i++] = ball.x / IN;
            out[i++] = ball.y / IN;
            out[i++] = ball.z / IN;
        }
        redHive.inCell = red;
        blueHive.inCell = blue;
        pieceState = i == out.length ? out : java.util.Arrays.copyOf(out, i);
    }

    double[] pieces() {
        return pieceState;
    }

    // ---------------------------------------------------------------- field shapes

    private static final String NUM = "(-?[0-9.e-]+), (-?[0-9.e-]+), (-?[0-9.e-]+)";
    private static final Pattern BOX = Pattern.compile("\"c\": \\[" + NUM + "\\], \"axes\": \\[\\[" + NUM + "\\], \\["
            + NUM + "\\], \\[" + NUM + "\\]\\], \"h\": \\[" + NUM + "\\]");
    private static final Pattern PIECE = Pattern.compile("\"type\": \"(\\w+)\", \"alliance\": (\"\\w+\"|null), \"pos\": \\["
            + NUM + "\\]");

    private static List<double[]> boxes(String json, String from, String to) {
        String section = json.substring(json.indexOf(from), json.indexOf(to));
        List<double[]> out = new ArrayList<>();
        Matcher m = BOX.matcher(section);
        while (m.find()) {
            double[] v = new double[15];
            for (int i = 0; i < 15; i++) v[i] = Double.parseDouble(m.group(i + 1));
            out.add(v);
        }
        return out;
    }

    // the axes are the box's own x, y, z. A rotation needs them right handed, a box doesn't care
    private static DMatrix3 rotationOf(double[] v) {
        double[] a = {v[3], v[4], v[5]}, b = {v[6], v[7], v[8]}, c = {v[9], v[10], v[11]};
        double det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]);
        if (det < 0) c = new double[]{-c[0], -c[1], -c[2]};
        return new DMatrix3(a[0], b[0], c[0], a[1], b[1], c[1], a[2], b[2], c[2]);
    }

    /**
     * The manual's setup, positions from the CAD. Live: pollen in the gardens, nectar in the cells.
     * Staged only for now: pollen in flowers (needs the flower mech) and nectar in the alliance areas
     * (needs the human player). The 16 pollen off the field are robot preloads, they aren't shown.
     */
    private void loadStaging(String json) {
        double[][] flowers = {{140, 95.4}, {48.6, 140}, {95.4, 4}, {4, 48.6}};
        Matcher m = PIECE.matcher(json);
        while (m.find()) {
            boolean pollen = m.group(1).equals("pollen");
            int type = pollen ? POLLEN : m.group(2).contains("red") ? RED_NECTAR : BLUE_NECTAR;
            double px = Double.parseDouble(m.group(3)), py = Double.parseDouble(m.group(4)), pz = Double.parseDouble(m.group(5));
            double xIn = px / IN, yIn = py / IN;
            boolean onField = xIn > 0 && xIn < 144 && yIn > 0 && yIn < 144;
            if (pollen && !onField) continue;
            boolean inFlower = false;
            for (double[] f : flowers) inFlower |= Math.hypot(xIn - f[0], yIn - f[1]) < 5;
            staging.add(new double[]{type, px, py, pz, !onField || inFlower ? 1 : 0});
        }
    }

    static String resource(String name) {
        try (InputStream in = SimCollisions.class.getResourceAsStream(name)) {
            if (in == null) throw new IllegalStateException("missing resource " + name);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            in.transferTo(out);
            return out.toString(StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }
}
