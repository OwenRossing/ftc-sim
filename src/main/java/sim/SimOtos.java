package sim;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Fake SparkFun OTOS. Pedro's real OTOSLocalizer talks to it. Like the real chip it dead reckons
 * its own pose from how the robot moves, so setPosition() changes what it reports, not where the
 * robot is.
 *
 * Internally everything is inches and radians in the chip's frame; the units the code sets are
 * applied on the way in and out.
 */
public final class SimOtos extends SparkFunOTOS {

    private final SimWorld world;

    private volatile DistanceUnit linearUnit = DistanceUnit.INCH;
    private volatile AngleUnit angularUnit = AngleUnit.DEGREES;

    // chip frame, inches / radians
    private volatile double x = 0;
    private volatile double y = 0;
    private volatile double h = 0;
    private volatile double vx = 0;
    private volatile double vy = 0;
    private volatile double vh = 0;

    SimOtos(SimWorld world) {
        super(Fakes.proxy(I2cDeviceSynch.class, "otos i2c", null));
        this.world = world;
    }

    /**
     * Called by the physics with the robot's body velocity (in/s forward, in/s left, rad/s ccw)
     */
    void integrate(double forward, double left, double turn, double dt) {
        // Pedro adds 90 deg going into the chip, so the robot's heading is h - 90
        double heading = h - Math.PI / 2;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        vx = forward * cos - left * sin;
        vy = forward * sin + left * cos;
        vh = turn;
        x += vx * dt;
        y += vy * dt;
        h = wrap(h + turn * dt);
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double outLinear(double inches) {
        return linearUnit.fromInches(inches);
    }

    private double outAngle(double radians) {
        return angularUnit == AngleUnit.DEGREES ? Math.toDegrees(radians) : radians;
    }

    private double inLinear(double value) {
        return linearUnit.toInches(value);
    }

    private double inAngle(double value) {
        return angularUnit == AngleUnit.DEGREES ? Math.toRadians(value) : value;
    }

    @Override
    public boolean begin() {
        return true;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public boolean calibrateImu() {
        return true;
    }

    @Override
    public boolean calibrateImu(int numSamples, boolean waitUntilDone) {
        return true;
    }

    @Override
    public void setLinearUnit(DistanceUnit unit) {
        linearUnit = unit;
    }

    @Override
    public void setAngularUnit(AngleUnit unit) {
        angularUnit = unit;
    }

    @Override
    public AngleUnit getAngularUnit() {
        return angularUnit;
    }

    @Override
    public void setOffset(Pose2D pose) {
        // the real chip uses this to report the robot center instead of the sensor, we already do
    }

    @Override
    public boolean setLinearScalar(double scalar) {
        return true;
    }

    @Override
    public boolean setAngularScalar(double scalar) {
        return true;
    }

    @Override
    public void resetTracking() {
        x = 0;
        y = 0;
        h = 0;
    }

    @Override
    public void setPosition(Pose2D pose) {
        x = inLinear(pose.x);
        y = inLinear(pose.y);
        h = wrap(inAngle(pose.h));
        world.onOtosSetPosition(x, y, h - Math.PI / 2);
    }

    @Override
    public Pose2D getPosition() {
        world.hardwareDelay(3); // an I2C read on the real robot takes a few ms
        return new Pose2D(outLinear(x), outLinear(y), outAngle(h));
    }

    @Override
    public Pose2D getVelocity() {
        return new Pose2D(outLinear(vx), outLinear(vy), outAngle(vh));
    }

    @Override
    public Pose2D getAcceleration() {
        return new Pose2D(0, 0, 0);
    }

    /**
     * What the robot thinks its pose is, in Pedro's frame (x, y in, heading rad)
     */
    double[] reportedPose() {
        return new double[]{x, y, wrap(h - Math.PI / 2)};
    }
}
