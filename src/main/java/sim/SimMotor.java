package sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A fake motor. The code sees a DcMotorEx, the physics sees the power and sets the speed.
 *
 * Direction is modeled like the real thing: setDirection(REVERSE) flips both the power and the
 * encoder. "mount" is which way the motor is physically bolted in (from the SimRobot), so if
 * someone flips a Direction in the code, the mechanism runs backwards in the sim too.
 *
 * The run modes work like the hub's: RUN_USING_ENCODER holds a velocity (setVelocity, or power as a
 * fraction of top speed) and RUN_TO_POSITION drives to setTargetPosition at up to the set power.
 */
public final class SimMotor {

    // the hub's controllers, close enough: power per tick/s of velocity error, and per tick of position error
    private static final double VELOCITY_KP = 0.002;
    private static final double POSITION_KP = 0.01;
    private static final int POSITION_TOLERANCE = 10; // ticks, the SDK's default

    public final String name;
    final double ticksPerRev;
    final int mount;
    final double maxTicksPerSec; // free speed
    final DcMotorEx device;

    public volatile double power = 0;
    volatile DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    volatile DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public volatile DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    private volatile double velocityTarget = Double.NaN; // ticks/s in the code's frame, NaN = use power
    private volatile int targetPosition = 0;

    // set by the physics, in the mechanism's frame (+ = the way the mechanism is supposed to go)
    public volatile double mechanismRpm = 0;
    public volatile double positionTicks = 0;
    public volatile double currentAmps = 0;

    SimMotor(String name, double ticksPerRev, int mount, double freeRpm, SimWorld world) {
        this.name = name;
        this.ticksPerRev = ticksPerRev;
        this.mount = mount;
        this.maxTicksPerSec = freeRpm / 60 * ticksPerRev;
        this.device = Fakes.proxy(DcMotorEx.class, name, (method, a) -> {
            if (world.ignoring(Thread.currentThread())) return Fakes.NOT_HANDLED; // an OpMode that ignored STOP
            switch (method) {
                case "setPower":
                    power = Range.clip((double) a[0], -1, 1);
                    velocityTarget = Double.NaN;
                    world.onMotorWrite(this, power);
                    return null;
                case "getPower":
                    return power;
                case "setVelocity": {
                    double v = (double) a[0];
                    if (a.length > 1) { // angular units: turn it into ticks/s
                        double revPerSec = a[1] == AngleUnit.DEGREES ? v / 360 : v / (2 * Math.PI);
                        v = revPerSec * ticksPerRev;
                    }
                    velocityTarget = v;
                    power = Range.clip(v / maxTicksPerSec, -1, 1);
                    world.onMotorWrite(this, power);
                    return null;
                }
                case "setDirection":
                    direction = (DcMotorSimple.Direction) a[0];
                    return null;
                case "getDirection":
                    return direction;
                case "setMode":
                    mode = (DcMotor.RunMode) a[0];
                    if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                        positionTicks = 0;
                        power = 0;
                    }
                    return null;
                case "getMode":
                    return mode;
                case "setTargetPosition":
                    targetPosition = (int) a[0];
                    return null;
                case "getTargetPosition":
                    return targetPosition;
                case "isBusy":
                    return mode == DcMotor.RunMode.RUN_TO_POSITION
                            && Math.abs(targetPosition - currentPosition()) > POSITION_TOLERANCE;
                case "setZeroPowerBehavior":
                    zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) a[0];
                    return null;
                case "getZeroPowerBehavior":
                    return zeroPowerBehavior;
                case "getCurrentPosition":
                    return currentPosition();
                case "getVelocity":
                    if (a.length == 0) return velocity();
                    double revPerSec = velocity() / ticksPerRev;
                    return a[0] == AngleUnit.DEGREES ? revPerSec * 360 : revPerSec * 2 * Math.PI;
                case "getCurrent":
                    return a[0] == CurrentUnit.MILLIAMPS ? currentAmps * 1000 : currentAmps;
                case "isMotorEnabled":
                    return true;
                case "getPortNumber":
                    return 0;
                default:
                    return Fakes.NOT_HANDLED;
            }
        });
    }

    double dirSign() {
        return direction == DcMotorSimple.Direction.REVERSE ? -1 : 1;
    }

    /** The encoder count the code sees */
    int currentPosition() {
        return (int) Math.round(positionTicks * dirSign() * mount);
    }

    /** Ticks per second the code sees */
    double velocity() {
        return mechanismRpm * mount * dirSign() / 60.0 * ticksPerRev;
    }

    /**
     * What the motor driver actually puts out after the run mode's controller, in the code's frame, -1 to 1
     */
    private double appliedPower() {
        if (mode == DcMotor.RunMode.RUN_TO_POSITION) {
            double limit = Math.abs(power);
            return Range.clip((targetPosition - currentPosition()) * POSITION_KP, -limit, limit);
        }
        if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            // hold a speed: setVelocity's, or the power as a fraction of top speed
            double target = Double.isNaN(velocityTarget) ? power * maxTicksPerSec : velocityTarget;
            if (target == 0) return 0;
            return Range.clip(target / maxTicksPerSec + (target - velocity()) * VELOCITY_KP, -1, 1);
        }
        if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) return 0;
        return power;
    }

    /**
     * Power in the mechanism's frame after the run mode, direction and mounting, -1 to 1
     */
    public double mechanismPower() {
        return appliedPower() * dirSign() * mount;
    }

    /** The OpMode ended: the SDK cuts the motor, whatever mode it was in */
    void cut() {
        power = 0;
        velocityTarget = Double.NaN;
    }

    void reset() {
        power = 0;
        velocityTarget = Double.NaN;
        targetPosition = 0;
        direction = DcMotorSimple.Direction.FORWARD;
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    }
}
