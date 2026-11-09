package sim;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * A fake servo. It moves toward the commanded position at a fixed rate, and stays limp
 * (wherever it physically is) until something commands it, like a real one.
 */
public final class SimServo {

    public final String name;
    final double ratePerSec; // position units per second, 1.0 = full range in a second
    final Servo device;

    volatile double lastSet = Double.NaN; // what the code asked for, before direction/scaling
    public volatile double commanded = Double.NaN; // NaN = limp
    public volatile double physical;
    volatile Servo.Direction direction = Servo.Direction.FORWARD;
    private volatile double scaleMin = 0;
    private volatile double scaleMax = 1;

    SimServo(String name, double ratePerSec, double startPosition, SimWorld world) {
        this.name = name;
        this.ratePerSec = ratePerSec;
        this.physical = startPosition;
        this.device = Fakes.proxy(Servo.class, name, (method, a) -> {
            if (world.ignoring(Thread.currentThread())) return Fakes.NOT_HANDLED; // an OpMode that ignored STOP
            switch (method) {
                case "setPosition":
                    double p = Range.clip((double) a[0], 0, 1);
                    lastSet = p;
                    if (direction == Servo.Direction.REVERSE) p = 1 - p;
                    commanded = scaleMin + p * (scaleMax - scaleMin);
                    world.onServoWrite(this);
                    return null;
                case "getPosition":
                    return Double.isNaN(lastSet) ? 0.0 : lastSet;
                case "setDirection":
                    direction = (Servo.Direction) a[0];
                    return null;
                case "getDirection":
                    return direction;
                case "scaleRange":
                    scaleMin = (double) a[0];
                    scaleMax = (double) a[1];
                    return null;
                case "getPortNumber":
                    return 0;
                default:
                    return Fakes.NOT_HANDLED;
            }
        });
    }

    void step(double dt) {
        double target = commanded;
        if (Double.isNaN(target)) return;
        double maxStep = ratePerSec * dt;
        physical += Range.clip(target - physical, -maxStep, maxStep);
    }

    /**
     * New OpMode: the SDK disables servos when an OpMode stops, so they go limp where they are
     */
    void reset() {
        lastSet = Double.NaN;
        commanded = Double.NaN;
        direction = Servo.Direction.FORWARD;
        scaleMin = 0;
        scaleMax = 1;
    }
}
