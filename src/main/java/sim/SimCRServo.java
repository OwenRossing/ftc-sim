package sim;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * A fake continuous rotation servo: it takes a power like a motor and just spins. The robot's step()
 * reads mechanismPower() to see what it's doing.
 */
public final class SimCRServo {

    public final String name;
    final int mount;
    final CRServo device;

    public volatile double power = 0;
    volatile DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    SimCRServo(String name, int mount, SimWorld world) {
        this.name = name;
        this.mount = mount;
        this.device = Fakes.proxy(CRServo.class, name, (method, a) -> {
            if (world.ignoring(Thread.currentThread())) return Fakes.NOT_HANDLED; // an OpMode that ignored STOP
            switch (method) {
                case "setPower":
                    power = Range.clip((double) a[0], -1, 1);
                    if (power != 0) world.onCrServoWrite(this, power);
                    return null;
                case "getPower":
                    return power;
                case "setDirection":
                    direction = (DcMotorSimple.Direction) a[0];
                    return null;
                case "getDirection":
                    return direction;
                case "getPortNumber":
                    return 0;
                default:
                    return Fakes.NOT_HANDLED;
            }
        });
    }

    /** Power in the mechanism's frame after direction and mounting, -1 to 1 */
    public double mechanismPower() {
        return power * (direction == DcMotorSimple.Direction.REVERSE ? -1 : 1) * mount;
    }

    void reset() {
        power = 0;
        direction = DcMotorSimple.Direction.FORWARD;
    }
}
