package sim;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import java.awt.Desktop;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The sim with a web page on top: a driver station + 3D view at http://localhost:8080.
 *   .\gradlew -p sim run
 * All the actual sim lives in SimEngine.
 */
public final class SimMain {

    private final SimEngine engine = new SimEngine();
    // the team repo can add its own robot models in sim-web/, those win over the sim's own
    private final List<Path> webRoots = new ArrayList<>();
    private final java.util.concurrent.atomic.AtomicInteger openPages = new java.util.concurrent.atomic.AtomicInteger();

    public static void main(String[] args) throws Exception {
        int port = Integer.parseInt(System.getProperty("sim.port", System.getenv().getOrDefault("SIM_PORT", "8080")));
        new SimMain().run(port);
    }

    private SimMain() {
        String hostWeb = System.getProperty("sim.hostWeb");
        if (hostWeb != null && Files.isDirectory(Paths.get(hostWeb))) webRoots.add(Paths.get(hostWeb).toAbsolutePath().normalize());
        webRoots.add(Paths.get("web").toAbsolutePath().normalize());
    }

    /**
     * Asks a sim that's already on this port to quit (the last Run in Sim), and waits for it to go.
     * Returns how many pages it had open, -1 if nothing was there
     */
    private static int replaceOldSim(int port) {
        try {
            java.net.HttpURLConnection c = (java.net.HttpURLConnection)
                    new java.net.URL("http://127.0.0.1:" + port + "/cmd?action=quit").openConnection();
            c.setRequestMethod("POST");
            c.setConnectTimeout(1000);
            c.setReadTimeout(2000);
            int pages = 0;
            if (c.getResponseCode() == 200) {
                pages = Integer.parseInt(new String(c.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim());
            }
            c.disconnect();
            System.out.println("[sim] Closed the sim that was already running");
            return pages;
        } catch (Exception e) {
            return -1;
        }
    }

    private void run(int port) throws Exception {
        // started by the website gateway: don't outlive it
        String parent = System.getenv("SIM_PARENT_PID");
        if (parent != null) {
            ProcessHandle.of(Long.parseLong(parent)).ifPresentOrElse(
                    p -> p.onExit().thenRun(() -> System.exit(0)), () -> System.exit(0));
        }

        engine.start();

        // 0.0.0.0 = reachable from other computers on the network, SIM_HOST=localhost keeps it to this PC
        String host = System.getProperty("sim.host", System.getenv().getOrDefault("SIM_HOST", "0.0.0.0"));
        HttpServer server;
        int pagesWaiting = 0;
        try {
            server = HttpServer.create(new InetSocketAddress(host, port), 0);
        } catch (java.net.BindException busy) {
            // pressing Run in Sim again replaces the old one, and its open tab reconnects to this one
            pagesWaiting = replaceOldSim(port);
            server = null;
            for (int tries = 0; tries < 40 && server == null; tries++) {
                Thread.sleep(100);
                try {
                    server = HttpServer.create(new InetSocketAddress(host, port), 0);
                } catch (java.net.BindException stillBusy) {
                    // the old one is still letting go
                }
            }
            if (server == null) {
                System.out.println("[sim] Something else is using port " + port + ". Close it, or set SIM_PORT to another number");
                System.exit(1);
                return;
            }
        }
        server.setExecutor(Executors.newCachedThreadPool());
        server.createContext("/events", this::handleEvents);
        server.createContext("/gamepad", this::handleGamepad);
        server.createContext("/cmd", this::handleCommand);
        server.createContext("/field_colliders.json", ex -> reply(ex, 200, "application/json",
                SimCollisions.resource("/field_colliders.json").getBytes(StandardCharsets.UTF_8)));
        server.createContext("/", this::handleStatic);
        server.start();

        String url = "http://localhost:" + port;
        System.out.println("[sim] Robot: " + engine.world().robot().displayName());
        System.out.println("[sim] OpModes: " + engine.opModes().keySet());
        if (engine.opModes().isEmpty()) {
            System.out.println("[sim] No OpModes found. An OpMode needs @TeleOp or @Autonomous above its class to show up");
        }
        System.out.println("[sim] Running at " + url + "  (stop button or Ctrl+C to quit)");
        if (!host.equals("localhost") && !host.equals("127.0.0.1")) {
            for (String lanUrl : lanUrls(port)) System.out.println("[sim] On your network: " + lanUrl);
        }
        boolean pageWillReconnect = pagesWaiting > 0;
        if (!pageWillReconnect && !Boolean.getBoolean("sim.noBrowser") && System.getenv("SIM_NO_BROWSER") == null) {
            try {
                if (Desktop.isDesktopSupported()) Desktop.getDesktop().browse(new URI(url));
            } catch (Exception ignored) {
                // open it yourself
            }
        }
    }

    // addresses other computers can reach this one at (skips loopback, VPNs and virtual adapters)
    private static List<String> lanUrls(int port) {
        List<String> urls = new ArrayList<>();
        try {
            for (java.net.NetworkInterface nic : java.util.Collections.list(java.net.NetworkInterface.getNetworkInterfaces())) {
                if (!nic.isUp() || nic.isLoopback() || nic.isVirtual()) continue;
                String label = nic.getDisplayName().toLowerCase(java.util.Locale.ROOT);
                if (label.contains("virtual") || label.contains("vethernet") || label.contains("hyper-v")
                        || label.contains("vmware") || label.contains("wsl")) continue;
                for (java.net.InetAddress address : java.util.Collections.list(nic.getInetAddresses())) {
                    if (address instanceof java.net.Inet4Address && address.isSiteLocalAddress()) {
                        urls.add("http://" + address.getHostAddress() + ":" + port);
                    }
                }
            }
        } catch (Exception ignored) {
            // no network info, localhost still works
        }
        return urls;
    }

    // ---------------------------------------------------------------- web

    private void handleEvents(HttpExchange ex) throws IOException {
        ex.getResponseHeaders().add("Content-Type", "text/event-stream");
        ex.getResponseHeaders().add("Cache-Control", "no-cache");
        ex.sendResponseHeaders(200, 0);
        openPages.incrementAndGet();
        try (OutputStream out = ex.getResponseBody()) {
            while (true) {
                out.write(("data: " + engine.stateJson() + "\n\n").getBytes(StandardCharsets.UTF_8));
                out.flush();
                Thread.sleep(16); // ~60 updates a second, part of the control lag you can feel
            }
        } catch (IOException | InterruptedException ignored) {
            // browser went away
        } finally {
            openPages.decrementAndGet();
        }
    }

    private static final Pattern JSON_FIELD = Pattern.compile("\"(\\w+)\"\\s*:\\s*(true|false|-?[0-9.eE+-]+)");

    private void handleGamepad(HttpExchange ex) throws IOException {
        String body = new String(ex.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
        Map<String, String> v = new LinkedHashMap<>();
        Matcher m = JSON_FIELD.matcher(body);
        while (m.find()) v.put(m.group(1), m.group(2));
        engine.setGamepad(v);
        reply(ex, 204, null, null);
    }

    private void handleCommand(HttpExchange ex) throws IOException {
        Map<String, String> q = query(ex.getRequestURI().getRawQuery());
        String action = q.getOrDefault("action", "");
        try {
            switch (action) {
                case "init":
                    engine.initOpMode(q.getOrDefault("opmode", ""));
                    break;
                case "start":
                    engine.startOpMode();
                    break;
                case "stop":
                    engine.stopOpMode();
                    break;
                case "alliance":
                    engine.world().robot().onAlliance(q.getOrDefault("side", ""));
                    break;
                case "quit": {
                    // only from this computer: the next Run in Sim replacing this one
                    if (!ex.getRemoteAddress().getAddress().isLoopbackAddress()) {
                        reply(ex, 403, "text/plain", "only from this computer".getBytes(StandardCharsets.UTF_8));
                        return;
                    }
                    reply(ex, 200, "text/plain", String.valueOf(openPages.get()).getBytes(StandardCharsets.UTF_8));
                    new Thread(() -> {
                        engine.shutdown();
                        System.exit(0);
                    }).start();
                    return;
                }
                case "settings":
                    reply(ex, 200, "application/json", SimSettings.json().getBytes(StandardCharsets.UTF_8));
                    return;
                case "set":
                    if (!SimSettings.set(q.getOrDefault("name", ""), Double.parseDouble(q.getOrDefault("value", "")))) {
                        reply(ex, 400, "text/plain", "no such setting".getBytes(StandardCharsets.UTF_8));
                        return;
                    }
                    break;
                case "reset":
                    engine.world().physics.resetRobot();
                    break;
                default:
                    reply(ex, 400, "text/plain", "unknown action".getBytes(StandardCharsets.UTF_8));
                    return;
            }
            reply(ex, 204, null, null);
        } catch (Throwable t) {
            reply(ex, 500, "text/plain", SimEngine.stackTrace(t).getBytes(StandardCharsets.UTF_8));
        }
    }

    private void handleStatic(HttpExchange ex) throws IOException {
        String path = URLDecoder.decode(ex.getRequestURI().getPath(), StandardCharsets.UTF_8);
        if (path.equals("/")) path = "/index.html";
        if (path.equals("/robot.json")) path = "/" + engine.world().robot().modelFile();
        for (Path root : webRoots) {
            Path file = root.resolve(path.substring(1)).normalize();
            if (file.startsWith(root) && Files.isRegularFile(file)) {
                reply(ex, 200, contentType(file.toString()), Files.readAllBytes(file));
                return;
            }
        }
        reply(ex, 404, "text/plain", "not found".getBytes(StandardCharsets.UTF_8));
    }

    static String contentType(String name) {
        if (name.endsWith(".html")) return "text/html; charset=utf-8";
        if (name.endsWith(".js")) return "text/javascript; charset=utf-8";
        if (name.endsWith(".json")) return "application/json";
        if (name.endsWith(".css")) return "text/css";
        if (name.endsWith(".glb")) return "model/gltf-binary";
        if (name.endsWith(".gltf")) return "model/gltf+json";
        if (name.endsWith(".png")) return "image/png";
        return "application/octet-stream";
    }

    static void reply(HttpExchange ex, int code, String type, byte[] body) throws IOException {
        if (type != null) ex.getResponseHeaders().add("Content-Type", type);
        ex.getResponseHeaders().add("Cache-Control", "no-store");
        if (body == null) {
            ex.sendResponseHeaders(code, -1);
        } else {
            ex.sendResponseHeaders(code, body.length);
            try (OutputStream out = ex.getResponseBody()) {
                out.write(body);
            }
        }
        ex.close();
    }

    private static Map<String, String> query(String raw) {
        Map<String, String> map = new LinkedHashMap<>();
        if (raw == null) return map;
        for (String pair : raw.split("&")) {
            int i = pair.indexOf('=');
            if (i < 0) continue;
            map.put(URLDecoder.decode(pair.substring(0, i), StandardCharsets.UTF_8),
                    URLDecoder.decode(pair.substring(i + 1), StandardCharsets.UTF_8));
        }
        return map;
    }
}
