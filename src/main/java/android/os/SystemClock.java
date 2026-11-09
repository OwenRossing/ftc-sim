package android.os;

/**
 * Stand-in for Android's SystemClock (android.jar only has stubs that throw).
 */
public final class SystemClock {

    private static final long START_NANOS = System.nanoTime();

    private SystemClock() {}

    public static long uptimeMillis() {
        return (System.nanoTime() - START_NANOS) / 1_000_000L;
    }

    public static long elapsedRealtime() {
        return uptimeMillis();
    }

    public static long elapsedRealtimeNanos() {
        return System.nanoTime() - START_NANOS;
    }

    public static long currentThreadTimeMillis() {
        return uptimeMillis();
    }

    public static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
