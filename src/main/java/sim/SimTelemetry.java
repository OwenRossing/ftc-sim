package sim;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Catches what the OpMode puts on telemetry so the browser can show it like the Driver Hub.
 */
final class SimTelemetry {

    private final List<String> pending = new ArrayList<>();
    private volatile List<String> shown = Collections.emptyList();
    final Telemetry telemetry;

    SimTelemetry() {
        telemetry = Fakes.proxy(Telemetry.class, "telemetry", (method, a) -> {
            switch (method) {
                case "addData":
                    add(formatData(a));
                    return chain(Telemetry.Item.class);
                case "addLine":
                    add(a.length == 0 ? "" : String.valueOf(a[0]));
                    return chain(Telemetry.Line.class);
                case "update":
                    publish();
                    return true;
                case "clear":
                case "clearAll":
                    synchronized (pending) {
                        pending.clear();
                        dirty = true;
                    }
                    return null;
                case "log":
                    return chain(Telemetry.Log.class);
                case "isAutoClear":
                    return true;
                case "getMsTransmissionInterval":
                    return 250;
                case "getItemSeparator":
                    return " | ";
                case "getCaptionValueSeparator":
                    return " : ";
                default:
                    return Fakes.NOT_HANDLED;
            }
        });
    }

    private static String formatData(Object[] a) {
        String caption = String.valueOf(a[0]);
        String value;
        if (a.length == 2) {
            value = a[1] instanceof Func ? String.valueOf(((Func<?>) a[1]).value()) : String.valueOf(a[1]);
        } else if (a.length == 3 && a[2] instanceof Object[]) {
            value = safeFormat(String.valueOf(a[1]), (Object[]) a[2]);
        } else if (a.length == 3 && a[2] instanceof Func) {
            value = safeFormat(String.valueOf(a[1]), new Object[]{((Func<?>) a[2]).value()});
        } else {
            value = a.length > 1 ? String.valueOf(a[1]) : "";
        }
        return value.isEmpty() ? caption : caption + " : " + value;
    }

    private static String safeFormat(String format, Object[] args) {
        try {
            return String.format(format, args);
        } catch (Exception e) {
            return format;
        }
    }

    private void add(String line) {
        synchronized (pending) {
            pending.add(line);
            dirty = true;
        }
    }

    private volatile long updates = 0;

    /** How many times update() has been called. The SDK calls it after every init_loop() and loop() */
    long updates() {
        return updates;
    }

    private boolean dirty = false;

    private void publish() {
        updates++;
        synchronized (pending) {
            // like the real one: an update with nothing new since the last (the SDK's own one, right after the
            // OpMode called update() itself) leaves the screen alone
            if (!dirty) return;
            shown = new ArrayList<>(pending);
            pending.clear();
            dirty = false;
        }
    }

    List<String> lines() {
        return shown;
    }

    // Item / Line / Log builders: ignore everything, keep chaining
    @SuppressWarnings("unchecked")
    private static <T> T chain(Class<T> type) {
        return (T) Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, (proxy, method, args) -> {
            if (method.getReturnType().isInstance(proxy)) return proxy;
            return Fakes.defaultValue(method);
        });
    }
}
