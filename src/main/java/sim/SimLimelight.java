package sim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.net.InetAddress;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Random;

/**
 * A fake Limelight 3A. The robot's real LimelightSubsystem and AimCommand read it. It reports the
 * tags SimPhysics says this camera can see, at about the real camera's frame rate, as old as a real
 * frame would be and with a bit of noise, so shooting on the move has to cope with what it really gets.
 */
final class SimLimelight extends Limelight3A {

    private static final long FRAME_MS = 33;
    private static final double CAPTURE_LATENCY_MS = 15;
    private static final double TARGETING_LATENCY_MS = 12;
    private static final double NOISE_DEG = 0.04; // TUNE from the real camera's telemetry

    private final SimPhysics physics;
    private final Random noise = new Random(1);
    private final ArrayDeque<Object[]> inFlight = new ArrayDeque<>();
    private LLResult latest;
    private long lastCaptureAt = 0;

    SimLimelight(SimPhysics physics) {
        super(null, "limelight", InetAddress.getLoopbackAddress());
        this.physics = physics;
    }

    private static final class Frame extends LLResult {
        private final double timestamp;

        Frame(JSONObject json, double timestamp) throws JSONException {
            super(json);
            this.timestamp = timestamp;
        }

        @Override
        public double getTimestamp() {
            return timestamp;
        }
    }

    @Override
    public synchronized LLResult getLatestResult() {
        long now = System.currentTimeMillis();
        if (physics.cameraDead) return latest;
        if (now - lastCaptureAt >= FRAME_MS) {
            lastCaptureAt = now;
            inFlight.addLast(new Object[]{now, physics.limelightTags()});
        }
        // a frame turns up once it's been through the camera
        while (!inFlight.isEmpty() && now - (long) inFlight.peekFirst()[0] >= CAPTURE_LATENCY_MS + TARGETING_LATENCY_MS) {
            @SuppressWarnings("unchecked")
            List<double[]> tags = (List<double[]>) inFlight.pollFirst()[1];
            latest = build(tags, now);
        }
        return latest;
    }

    private LLResult build(List<double[]> tags, long now) {
        try {
            JSONObject json = new JSONObject();
            json.put("v", tags.isEmpty() ? 0 : 1);
            json.put("cl", CAPTURE_LATENCY_MS);
            json.put("tl", TARGETING_LATENCY_MS);
            JSONArray fiducials = new JSONArray();
            for (double[] tag : tags) {
                JSONObject f = new JSONObject();
                f.put("fID", (int) tag[0]);
                f.put("tx", tag[1] + noise.nextGaussian() * NOISE_DEG);
                f.put("ty", tag[2] + noise.nextGaussian() * NOISE_DEG);
                f.put("ta", tag[3]);
                fiducials.put(f);
            }
            json.put("Fiducial", fiducials);
            if (!tags.isEmpty()) { // the top level ones are just the biggest tag, like the real thing
                json.put("tx", tags.get(0)[1]);
                json.put("ty", tags.get(0)[2]);
                json.put("ta", tags.get(0)[3]);
            }
            return new Frame(json, now);
        } catch (JSONException e) {
            throw new IllegalStateException(e);
        }
    }

    @Override
    public synchronized void start() {}

    @Override
    public synchronized void pause() {}

    @Override
    public synchronized void stop() {}

    @Override
    public boolean isRunning() {
        return true;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public boolean pipelineSwitch(int index) {
        return true;
    }
}
