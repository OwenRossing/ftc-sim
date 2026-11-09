package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;

import java.lang.reflect.Proxy;

/**
 * Runs a real OpMode the way the robot controller app does: its own thread, INIT, then START,
 * then stop. It lives in the SDK's package so it can use the same package-private hooks the
 * app uses (isStarted, stopRequested, internalRunOpMode...).
 */
public final class SimOpModeRunner {

    private final OpMode opMode;
    private final Gamepad input1;
    private final Gamepad input2;
    private Thread thread;
    private volatile Throwable error;
    private volatile boolean finished = false;
    private volatile boolean started = false;

    /**
     * @param input1 gamepad the sim writes controller input into, copied into the OpMode each tick
     */
    public SimOpModeRunner(OpMode opMode, HardwareMap hardwareMap, Telemetry telemetry,
                           Gamepad input1, Gamepad input2) {
        this.opMode = opMode;
        this.input1 = input1;
        this.input2 = input2;
        opMode.hardwareMap = hardwareMap;
        opMode.telemetry = telemetry;
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.internalOpModeServices = (OpModeServices) Proxy.newProxyInstance(
                OpModeServices.class.getClassLoader(),
                new Class<?>[]{OpModeServices.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("requestOpModeStop")) opMode.stopRequested = true;
                    Class<?> type = method.getReturnType();
                    if (type == boolean.class) return false;
                    if (type.isPrimitive() && type != void.class) return 0;
                    return null;
                });
    }

    /**
     * Press INIT: runs the OpMode's init on its own thread
     */
    public void init() {
        opMode.isStarted = false;
        opMode.stopRequested = false;
        thread = new Thread(() -> {
            try {
                opMode.internalRunOpMode();
            } catch (InterruptedException ignored) {
                // stopped
            } catch (Throwable t) {
                error = t;
            } finally {
                finished = true;
            }
        }, "OpModeThread");
        thread.setDaemon(true);
        thread.start();
    }

    /**
     * Press START
     */
    public void start() {
        synchronized (input1) {
            opMode.gamepad1.resetEdgeDetection();
            opMode.gamepad2.resetEdgeDetection();
        }
        started = true;
        opMode.isStarted = true;
        opMode.internalOnStart();
    }

    /**
     * Press STOP: asks nicely, then interrupts if the OpMode doesn't notice
     */
    public void stop() {
        opMode.stopRequested = true;
        if (thread == null) return;
        try {
            thread.join(800);
            if (thread.isAlive()) {
                thread.interrupt();
                thread.join(1000);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * One event loop iteration: new gamepad data, wake up waitForStart, update time
     */
    public void tick() {
        if (finished) return;
        synchronized (input1) {
            opMode.newGamepadDataAvailable(input1, input2);
        }
        opMode.internalOnEventLoopIteration();
    }

    public boolean isStarted() {
        return started;
    }

    public boolean isFinished() {
        return finished;
    }

    public Throwable getError() {
        return error;
    }

    public OpMode opMode() {
        return opMode;
    }

    /** The OpMode's own thread, for working out where it's stuck */
    public Thread thread() {
        return thread;
    }

    /** True if STOP was pressed and the thread still didn't end */
    public boolean isStillRunning() {
        return thread != null && thread.isAlive() && opMode.stopRequested;
    }
}
