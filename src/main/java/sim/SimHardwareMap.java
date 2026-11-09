package sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The HardwareMap the OpMode gets. Every get() comes from the sim world instead of the hubs.
 * getAll() is empty, so there are no Lynx hubs and bulk reads are a no-op.
 *
 * Like the real one, a name that isn't in the robot's config throws, with the same message the
 * Driver Hub shows, plus a hint for the usual typo.
 */
public final class SimHardwareMap extends HardwareMap {

    private final SimWorld world;

    public SimHardwareMap(SimWorld world) {
        super(null, null);
        this.world = world;
        voltageSensor.put("Control Hub", world.batterySensor());
        // the old style, hardwareMap.dcMotor.get("name"), needs the config filled in ahead of time
        if (world.robot.strictNames()) {
            for (Map.Entry<String, SimRobot.Kind> e : world.robot.config().entrySet()) {
                if (e.getValue() == SimRobot.Kind.MOTOR) dcMotor.put(e.getKey(), world.motor(e.getKey()).device);
                if (e.getValue() == SimRobot.Kind.SERVO) servo.put(e.getKey(), world.servo(e.getKey()).device);
                if (e.getValue() == SimRobot.Kind.CRSERVO) crservo.put(e.getKey(), world.crServo(e.getKey()).device);
            }
        }
    }

    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) {
            // the SDK's own wording, so it reads the same as on the Driver Hub
            throw new IllegalArgumentException(String.format(
                    "Unable to find a hardware device with name \"%s\" and type %s%s",
                    deviceName, classOrInterface.getSimpleName(), world.configHelp(classOrInterface, deviceName.trim())));
        }
        return result;
    }

    @Override
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        Object device = world.device(classOrInterface, deviceName.trim());
        return device == null ? null : classOrInterface.cast(device);
    }

    @Override
    public HardwareDevice get(String deviceName) {
        SimRobot.Kind kind = world.robot.config().get(deviceName.trim());
        Class<?> type = kind == SimRobot.Kind.SERVO ? Servo.class : kind == SimRobot.Kind.MOTOR || kind == null ? DcMotor.class : kind.type;
        return (HardwareDevice) get(type, deviceName);
    }

    @Override
    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        return new ArrayList<>();
    }
}
