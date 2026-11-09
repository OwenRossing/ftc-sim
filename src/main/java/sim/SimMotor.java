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
 * encoder. "mount" is which way the motor is physically bolted in (from SimPhysics), so if
 * someone flips a Direction in the code, the mechanism runs backwards in the sim too.
 */
public final class SimMotor {

    public final String name;
    final double ticksPerRev;
    final int mount;
    final DcMotorEx device;

    public volatile double power = 0;
    volatile DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    volatile DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public volatile DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    // set by the physics, in the mechanism's frame (+ = the way the mechanism is supposed to go)
    public volatile double mechanismRpm = 0;
    public volatile double positionTicks = 0;
    public volatile double currentAmps = 0;

    SimMotor(String name, double ticksPerRev, int mount, SimWorld world) {
        this.name = name;
        this.ticksPerRev = ticksPerRev;
        this.mount = mount;
        this.device = Fakes.proxy(DcMotorEx.class, name, (method, a) -> {
            if (world.ignoring(Thread.currentThread())) return Fakes.NOT_HANDLED; // an OpMode that ignored STOP
            switch (method) {
                case "setPower":
                    power = Range.clip((double) a[0], -1, 1);
                    world.onMotorWrite(this, power);
                    return null;
                case "getPower":
                    return power;
                case "setDirection":
                    direction = (DcMotorSimple.Direction) a[0];
                    return null;
                case "getDirection":
                    return direction;
                case "setMode":
                    mode = (DcMotor.RunMode) a[0];
                    if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) positionTicks = 0;
                    return null;
                case "getMode":
                    return mode;
                case "setZeroPowerBehavior":
                    zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) a[0];
                    return null;
                case "getZeroPowerBehavior":
                    return zeroPowerBehavior;
                case "getCurrentPosition":
                    return (int) Math.round(positionTicks * dirSign() * mount);
                case "getVelocity":
                    double revPerSec = mechanismRpm * mount * dirSign() / 60.0;
                    if (a.length == 0) return revPerSec * ticksPerRev;
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

    /**
     * Power in the mechanism's frame after direction and mounting, -1 to 1
     */
    public double mechanismPower() {
        return power * dirSign() * mount;
    }

    void reset() {
        power = 0;
        direction = DcMotorSimple.Direction.FORWARD;
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    }
}
