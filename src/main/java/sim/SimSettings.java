package sim;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/**
 * What the viewer's settings page can change while the sim runs. A robot adds the robot code's own constants
 * (they just can't be final) with slider() and toggle() in its constructor, so the running OpMode uses a new
 * value on its next loop. It only changes this sim: the page lists what's different so it can be copied into
 * the code for the real robot.
 *
 * Only list something the code reads every loop. Things read once at INIT would look broken.
 */
public final class SimSettings {

    private static final class Setting {
        final String group, label, help, name;
        final Field field;
        final boolean simOnly;
        final double min, max, step, defaultValue;

        Setting(String group, String label, String help, Class<?> owner, String fieldName,
                double min, double max, double step, boolean simOnly) {
            this.group = group;
            this.label = label;
            this.help = help;
            this.name = owner.getSimpleName() + "." + fieldName;
            this.min = min;
            this.max = max;
            this.step = step;
            this.simOnly = simOnly;
            try {
                field = owner.getDeclaredField(fieldName);
                field.setAccessible(true);
                defaultValue = read();
            } catch (ReflectiveOperationException e) {
                throw new IllegalStateException("No setting " + name, e);
            }
        }

        boolean isToggle() {
            return field.getType() == boolean.class;
        }

        double read() throws IllegalAccessException {
            return isToggle() ? (field.getBoolean(null) ? 1 : 0) : field.getDouble(null);
        }

        void write(double value) throws IllegalAccessException {
            if (isToggle()) field.setBoolean(null, value != 0);
            else field.setDouble(null, Math.max(min, Math.min(max, value)));
        }
    }

    private static final List<Setting> ALL = new ArrayList<>();

    static {
        add(new Setting("Field", "Hive stiffness", "Sim only. How hard a Hive holds at its end. Under about 0.20 the staged nectar tips it",
                SimCollisions.class, "bistableTorque", 0.20, 0.50, 0.01, true));
    }

    private SimSettings() {}

    private static synchronized void add(Setting s) {
        for (Setting existing : ALL) if (existing.name.equals(s.name)) return;
        ALL.add(s);
    }

    /** A static boolean in the robot code the page can flip */
    public static void toggle(String group, String label, String help, Class<?> owner, String field) {
        add(new Setting(group, label, help, owner, field, 0, 1, 1, false));
    }

    /** A static double in the robot code the page can slide */
    public static void slider(String group, String label, String help, Class<?> owner, String field,
                              double min, double max, double step) {
        add(new Setting(group, label, help, owner, field, min, max, step, false));
    }

    /**
     * Sets one by its name, like Drive.BIAS_kP. False if there's no such setting
     */
    static synchronized boolean set(String name, double value) {
        for (Setting s : ALL) {
            if (!s.name.equals(name)) continue;
            try {
                s.write(value);
                return true;
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(e);
            }
        }
        return false;
    }

    static synchronized double get(String name) {
        for (Setting s : ALL) {
            if (!s.name.equals(name)) continue;
            try {
                return s.read();
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(e);
            }
        }
        throw new IllegalArgumentException(name);
    }

    static synchronized String json() {
        Json j = new Json().beginArray();
        try {
            for (Setting s : ALL) {
                j.begin()
                        .field("name", s.name).field("group", s.group).field("label", s.label).field("help", s.help)
                        .field("toggle", s.isToggle()).field("simOnly", s.simOnly)
                        .field("min", s.min).field("max", s.max).field("step", s.step)
                        .field("default", s.defaultValue).field("value", s.read())
                        .end();
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
        return j.endArray().toString();
    }
}
