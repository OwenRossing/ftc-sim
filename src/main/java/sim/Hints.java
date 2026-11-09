package sim;

import java.lang.reflect.InvocationTargetException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Plain English for the crashes new programmers hit most. The page shows the hint above the stack trace.
 * Add to it when a student gets stuck on something it doesn't explain.
 */
final class Hints {

    // Java's own message: Cannot invoke "...setPower(double)" because "this.intake" is null
    private static final Pattern NULL_VARIABLE = Pattern.compile("because \"(?:this\\.)?([\\w.<>]+)\" is null");
    private static final Pattern NULL_CALL = Pattern.compile("Cannot invoke \"[\\w.$]+\\.(\\w+)\\(");

    private Hints() {}

    static String forCrash(Throwable t) {
        Throwable cause = unwrap(t);
        String message = cause.getMessage() == null ? "" : cause.getMessage();

        if (cause instanceof NullPointerException) {
            Matcher m = NULL_VARIABLE.matcher(message);
            String variable = m.find() ? m.group(1) : null;
            Matcher call = NULL_CALL.matcher(message);
            String method = call.find() ? call.group(1) : null;
            if ("hardwareMap".equals(variable)) {
                return "HINT: hardwareMap isn't ready yet when you used it. Get your devices inside init() "
                        + "(or at the top of runOpMode()), not up where you declare the variables.";
            }
            if (variable != null && variable.startsWith("<local")) {
                return "HINT: a variable was null (it was never given a value) when you called "
                        + (method != null ? method + "() on it" : "something on it") + ". Look at the line number below.";
            }
            if (variable != null) {
                return "HINT: " + variable + " is null, which means it was never set. "
                        + "If it's a motor or servo, did you forget " + variable
                        + " = hardwareMap.get(...) in init()? Check the spelling matches too.";
            }
            return "HINT: something was null (never given a value) when the code used it. Look at the line number below.";
        }
        if (cause instanceof IllegalArgumentException && message.startsWith("Unable to find a hardware device")) {
            return "HINT: the name in hardwareMap.get(...) has to match the robot's config exactly.";
        }
        if (cause instanceof ClassCastException && message.contains("com.qualcomm")) {
            return "HINT: the type in hardwareMap.get(...) doesn't match the device. A motor is a DcMotor, a servo is a Servo.";
        }
        if (cause instanceof ArithmeticException && message.contains("/ by zero")) {
            return "HINT: the code divided a whole number (int) by zero.";
        }
        if (cause instanceof StackOverflowError) {
            return "HINT: a method kept calling itself forever.";
        }
        return null;
    }

    private static Throwable unwrap(Throwable t) {
        Throwable cause = t;
        while ((cause instanceof InvocationTargetException || cause instanceof ExceptionInInitializerError
                || (cause instanceof RuntimeException && cause.getClass() == RuntimeException.class))
                && cause.getCause() != null && cause.getCause() != cause) {
            cause = cause.getCause();
        }
        return cause;
    }
}
