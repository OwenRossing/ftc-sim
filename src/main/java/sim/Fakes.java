package sim;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

/**
 * Builds fake implementations of SDK interfaces. A handler answers the methods it knows and
 * returns NOT_HANDLED for the rest, which get a harmless default (0, false, null).
 */
final class Fakes {

    static final Object NOT_HANDLED = new Object();

    interface Handler {
        Object handle(String method, Object[] args) throws Throwable;
    }

    private Fakes() {}

    @SuppressWarnings("unchecked")
    static <T> T proxy(Class<T> iface, String name, Handler handler) {
        InvocationHandler invocation = (proxy, method, args) -> {
            Object[] a = args == null ? new Object[0] : args;
            switch (method.getName()) {
                case "toString":
                    if (a.length == 0) return "Sim" + iface.getSimpleName() + "(" + name + ")";
                    break;
                case "hashCode":
                    if (a.length == 0) return System.identityHashCode(proxy);
                    break;
                case "equals":
                    if (a.length == 1) return proxy == a[0];
                    break;
                case "getDeviceName":
                case "getConnectionInfo":
                    if (a.length == 0) return "Sim " + name;
                    break;
                default:
                    break;
            }
            Object result = handler == null ? NOT_HANDLED : handler.handle(method.getName(), a);
            return result == NOT_HANDLED ? defaultValue(method) : result;
        };
        return (T) Proxy.newProxyInstance(iface.getClassLoader(), new Class<?>[]{iface}, invocation);
    }

    static Object defaultValue(Method method) {
        return defaultValue(method.getReturnType());
    }

    static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive() || type == void.class) return null;
        if (type == boolean.class) return false;
        if (type == char.class) return (char) 0;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0f;
        return 0.0;
    }
}
