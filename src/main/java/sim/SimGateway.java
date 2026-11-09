package sim;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.URI;
import java.net.URLDecoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.time.Duration;
import java.util.Base64;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * The public front door for the sim (biobuzz.owenrossing.me, see sim/site.bat).
 *
 * Nothing gets through without an invite code, including the models. Each logged in person gets
 * their own copy of the sim in its own process, because the robot code uses static singletons
 * (one CommandScheduler per JVM). Everything listens on localhost only: the Cloudflare tunnel
 * is the only way in from outside.
 *
 * Codes live in sim/site/invite_codes.txt (not in git), one per line: CODE optional-name
 *   .\gradlew -p sim site
 */
public final class SimGateway {

    private static final int MAX_PLAYERS = envInt("SITE_MAX_PLAYERS", 6);
    private static final long IDLE_STOP_MS = 3 * 60 * 1000; // tab closed for this long: free the PC
    private static final long FULL_RECLAIM_MS = 20 * 1000; // when every field is taken, closed tabs lose theirs sooner
    private static final long SESSION_MS = 12 * 60 * 60 * 1000;
    private static final int MAX_BAD_CODES = 8;
    private static final long BAD_CODE_WINDOW_MS = 15 * 60 * 1000;
    private static final String COOKIE = "biobuzz_session";

    private static final class Player {
        final String id;
        final String label;
        final long expires = System.currentTimeMillis() + SESSION_MS;
        final AtomicInteger streams = new AtomicInteger();
        volatile long lastSeen = System.currentTimeMillis();
        volatile Process process;
        volatile int port;

        Player(String id, String label) {
            this.id = id;
            this.label = label;
        }

        boolean running() {
            return process != null && process.isAlive();
        }
    }

    private static final class Strikes {
        int count;
        long since = System.currentTimeMillis();
    }

    private final Map<String, Player> players = new ConcurrentHashMap<>();
    private final Map<String, Strikes> strikes = new ConcurrentHashMap<>();
    private final Map<String, String> codes = new LinkedHashMap<>();
    private final Path webRoot = Paths.get("web").toAbsolutePath().normalize();
    private final Path hostWeb = System.getProperty("sim.hostWeb", "").isEmpty() ? null
            : Paths.get(System.getProperty("sim.hostWeb")).toAbsolutePath().normalize();
    private final String robotModel = SimRobot.load(System.getProperty("sim.robot", "starter")).modelFile();
    private final Path siteDir = Paths.get("site").toAbsolutePath().normalize();
    private final SecureRandom random = new SecureRandom();
    private final HttpClient client = HttpClient.newBuilder().version(HttpClient.Version.HTTP_1_1)
            .connectTimeout(Duration.ofSeconds(3)).build();

    public static void main(String[] args) throws Exception {
        new SimGateway().run(envInt("SITE_PORT", 8090));
    }

    private static int envInt(String name, int fallback) {
        String value = System.getenv(name);
        return value == null ? fallback : Integer.parseInt(value);
    }

    private void run(int port) throws Exception {
        loadCodes();
        HttpServer server = HttpServer.create(new InetSocketAddress(InetAddress.getLoopbackAddress(), port), 0);
        server.setExecutor(Executors.newCachedThreadPool());
        server.createContext("/", this::handle);
        server.start();

        Thread janitor = new Thread(this::janitorLoop, "site-janitor");
        janitor.setDaemon(true);
        janitor.start();
        Runtime.getRuntime().addShutdownHook(new Thread(() -> players.values().forEach(this::stopSim)));
        System.out.println("[site] Gateway on http://localhost:" + port + ", " + codes.size()
                + " invite code(s) in " + siteDir.resolve("invite_codes.txt") + ", up to " + MAX_PLAYERS + " players");
    }

    private void loadCodes() throws IOException {
        Files.createDirectories(siteDir);
        Path file = siteDir.resolve("invite_codes.txt");
        if (!Files.exists(file)) {
            String alphabet = "ABCDEFGHJKMNPQRSTUVWXYZ23456789"; // nothing you could misread
            StringBuilder code = new StringBuilder();
            for (int i = 0; i < 12; i++) code.append(alphabet.charAt(random.nextInt(alphabet.length())));
            Files.writeString(file, code + " team\n");
            System.out.println("[site] Made a first invite code in " + file);
        }
        for (String line : Files.readAllLines(file)) {
            String[] parts = line.trim().split("\\s+", 2);
            if (parts[0].isEmpty() || parts[0].startsWith("#")) continue;
            codes.put(parts[0].toUpperCase(), parts.length > 1 ? parts[1] : "player");
        }
        if (codes.isEmpty()) throw new IllegalStateException("no invite codes in " + file);
    }

    // ---------------------------------------------------------------- routing

    private void handle(HttpExchange ex) throws IOException {
        try {
            String path = ex.getRequestURI().getPath();
            if (path.equals("/login")) {
                login(ex);
                return;
            }
            Player player = playerFor(ex);
            if (player == null) {
                if (ex.getRequestMethod().equals("GET") && (path.equals("/") || path.endsWith(".html"))) {
                    page(ex, 200, loginPage(""));
                } else {
                    SimMain.reply(ex, 401, "text/plain", "log in first".getBytes(StandardCharsets.UTF_8));
                }
                return;
            }
            player.lastSeen = System.currentTimeMillis();
            switch (path) {
                case "/events":
                    proxyStream(ex, player);
                    break;
                case "/gamepad":
                case "/cmd":
                case "/field_colliders.json":
                    proxy(ex, player);
                    break;
                default:
                    staticFile(ex, path);
            }
        } catch (IOException e) {
            throw e;
        } catch (Exception e) {
            // never show internals to the internet, they go in the local console
            System.out.println("[site] " + ex.getRequestURI().getPath() + ": " + e);
            SimMain.reply(ex, 500, "text/plain", "sim error".getBytes(StandardCharsets.UTF_8));
        } finally {
            ex.close();
        }
    }

    private Player playerFor(HttpExchange ex) {
        List<String> cookies = ex.getRequestHeaders().get("Cookie");
        if (cookies == null) return null;
        for (String header : cookies) {
            for (String part : header.split(";")) {
                String[] kv = part.trim().split("=", 2);
                if (kv.length == 2 && kv[0].equals(COOKIE)) {
                    Player player = players.get(kv[1]);
                    if (player != null && player.expires > System.currentTimeMillis()) return player;
                }
            }
        }
        return null;
    }

    // ---------------------------------------------------------------- login

    private String clientIp(HttpExchange ex) {
        // only Cloudflare can reach us, and it sets this. Local testing doesn't have it
        String ip = ex.getRequestHeaders().getFirst("CF-Connecting-IP");
        return ip != null ? ip : ex.getRemoteAddress().getAddress().getHostAddress();
    }

    private void login(HttpExchange ex) throws IOException {
        if (!ex.getRequestMethod().equals("POST")) {
            page(ex, 200, loginPage(""));
            return;
        }
        String ip = clientIp(ex);
        Strikes s = strikes.computeIfAbsent(ip, k -> new Strikes());
        synchronized (s) {
            if (System.currentTimeMillis() - s.since > BAD_CODE_WINDOW_MS) {
                s.count = 0;
                s.since = System.currentTimeMillis();
            }
            if (s.count >= MAX_BAD_CODES) {
                page(ex, 429, loginPage("Too many wrong codes. Try again in 15 minutes."));
                return;
            }
        }
        byte[] limited = ex.getRequestBody().readNBytes(512);
        String body = new String(limited, StandardCharsets.UTF_8);
        String entered = "";
        for (String pair : body.split("&")) {
            if (pair.startsWith("code=")) entered = URLDecoder.decode(pair.substring(5), StandardCharsets.UTF_8);
        }
        String label = matchCode(entered.trim().toUpperCase());
        if (label == null) {
            synchronized (s) {
                s.count++;
            }
            page(ex, 401, loginPage("That code didn't work."));
            return;
        }
        reclaimClosedTabs();
        if (activePlayers() >= MAX_PLAYERS) {
            page(ex, 503, loginPage("All " + MAX_PLAYERS + " fields are in use, try again in a bit."));
            return;
        }
        byte[] token = new byte[32];
        random.nextBytes(token);
        String id = Base64.getUrlEncoder().withoutPadding().encodeToString(token);
        players.put(id, new Player(id, label));
        boolean https = ex.getRequestHeaders().containsKey("CF-Connecting-IP");
        ex.getResponseHeaders().add("Set-Cookie", COOKIE + "=" + id + "; Path=/; HttpOnly; SameSite=Lax; Max-Age="
                + SESSION_MS / 1000 + (https ? "; Secure" : ""));
        ex.getResponseHeaders().add("Location", "/");
        System.out.println("[site] " + label + " logged in");
        ex.sendResponseHeaders(303, -1);
    }

    private String matchCode(String entered) {
        // compares against every code the same way so timing doesn't leak anything
        String found = null;
        byte[] attempt = entered.getBytes(StandardCharsets.UTF_8);
        for (Map.Entry<String, String> e : codes.entrySet()) {
            if (MessageDigest.isEqual(attempt, e.getKey().getBytes(StandardCharsets.UTF_8))) found = e.getValue();
        }
        return found;
    }

    // nobody connected to it for a bit means the tab closed, they get a fresh field if they come back
    private void reclaimClosedTabs() {
        if (activePlayers() < MAX_PLAYERS) return;
        long now = System.currentTimeMillis();
        for (Player player : players.values()) {
            if (player.running() && player.streams.get() == 0 && now - player.lastSeen > FULL_RECLAIM_MS) {
                stopSim(player);
            }
        }
    }

    private long activePlayers() {
        return players.values().stream().filter(Player::running).count();
    }

    private void page(HttpExchange ex, int code, String html) throws IOException {
        SimMain.reply(ex, code, "text/html; charset=utf-8", html.getBytes(StandardCharsets.UTF_8));
    }

    private static String loginPage(String message) {
        return "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
                + "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><title>BioBuzz Sim</title><style>"
                + ":root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;place-items:center;"
                + "padding:16px;background:#14171c;color:#e6e9ee;font:15px/1.5 system-ui,sans-serif}"
                + "form{width:100%;max-width:340px;background:#1b2027;border:1px solid #2c333d;border-radius:12px;padding:24px}"
                + "h1{margin:0 0 4px;font-size:20px}p{margin:0 0 16px;color:#8c96a3}"
                + "input,button{width:100%;font:inherit;padding:10px 12px;border-radius:8px;border:1px solid #2c333d}"
                + "input{background:#14171c;color:inherit;margin-bottom:10px;text-transform:uppercase;letter-spacing:.08em}"
                + "button{background:#f2b632;color:#14171c;border:0;font-weight:600;cursor:pointer}"
                + ".msg{color:#ff6b6b;margin:0 0 12px}</style></head><body><form method=\"post\" action=\"/login\">"
                + "<h1>BioBuzz Sim</h1><p>Enter your invite code to get your own field.</p>"
                + (message.isEmpty() ? "" : "<p class=\"msg\">" + message + "</p>")
                + "<input name=\"code\" aria-label=\"Invite code\" placeholder=\"Invite code\" autocomplete=\"off\" autofocus required maxlength=\"64\">"
                + "<button>Enter</button></form></body></html>";
    }

    // ---------------------------------------------------------------- each player's sim

    private synchronized void ensureSim(Player player) throws Exception {
        if (player.running()) return;
        reclaimClosedTabs();
        if (activePlayers() >= MAX_PLAYERS) throw new IllegalStateException("all fields in use");
        int port;
        try (ServerSocket probe = new ServerSocket(0, 1, InetAddress.getLoopbackAddress())) {
            port = probe.getLocalPort();
        }
        String java = ProcessHandle.current().info().command().orElse("java");
        ProcessBuilder builder = new ProcessBuilder(java, "-Xmx400m", "-cp", System.getProperty("java.class.path"),
                "-Dsim.robot=" + System.getProperty("sim.robot", "starter"),
                "-Dsim.hostWeb=" + System.getProperty("sim.hostWeb", ""), "sim.SimMain");
        builder.directory(new File(".").getAbsoluteFile());
        builder.environment().put("SIM_PORT", String.valueOf(port));
        builder.environment().put("SIM_HOST", "localhost");
        builder.environment().put("SIM_NO_BROWSER", "1");
        builder.environment().put("SIM_PARENT_PID", String.valueOf(ProcessHandle.current().pid()));
        builder.redirectErrorStream(true);
        builder.redirectOutput(siteDir.resolve("sim-" + port + ".log").toFile());
        player.process = builder.start();
        player.port = port;

        HttpRequest ping = HttpRequest.newBuilder(URI.create("http://localhost:" + port + "/robot.json"))
                .timeout(Duration.ofSeconds(1)).build();
        long deadline = System.currentTimeMillis() + 30_000;
        while (System.currentTimeMillis() < deadline) {
            try {
                if (client.send(ping, HttpResponse.BodyHandlers.discarding()).statusCode() == 200) {
                    System.out.println("[site] " + player.label + " got a field on port " + port);
                    return;
                }
            } catch (IOException e) {
                Thread.sleep(200); // still starting
            }
        }
        stopSim(player);
        throw new IllegalStateException("sim didn't start");
    }

    private void stopSim(Player player) {
        Process process = player.process;
        if (process == null) return;
        process.destroy();
        try {
            Files.deleteIfExists(siteDir.resolve("sim-" + player.port + ".log"));
        } catch (IOException ignored) {
            // still open, the next start cleans nothing up but it's small
        }
        player.process = null;
    }

    private void janitorLoop() {
        while (true) {
            try {
                Thread.sleep(30_000);
                long now = System.currentTimeMillis();
                for (Player player : players.values()) {
                    boolean idle = player.streams.get() == 0 && now - player.lastSeen > IDLE_STOP_MS;
                    if (player.running() && (idle || now > player.expires)) {
                        System.out.println("[site] stopping idle field for " + player.label);
                        stopSim(player);
                    }
                    if (now > player.expires) players.remove(player.id);
                }
                strikes.entrySet().removeIf(e -> now - e.getValue().since > BAD_CODE_WINDOW_MS);
            } catch (InterruptedException e) {
                return;
            } catch (RuntimeException e) {
                System.out.println("[site] janitor: " + e);
            }
        }
    }

    // ---------------------------------------------------------------- passing requests through

    private HttpRequest.Builder upstream(HttpExchange ex, Player player) {
        String query = ex.getRequestURI().getRawQuery();
        return HttpRequest.newBuilder(URI.create("http://localhost:" + player.port + ex.getRequestURI().getRawPath()
                + (query == null ? "" : "?" + query)));
    }

    private void proxy(HttpExchange ex, Player player) throws Exception {
        ensureSim(player);
        byte[] body = ex.getRequestBody().readNBytes(16 * 1024);
        HttpRequest request = upstream(ex, player).timeout(Duration.ofSeconds(20))
                .method(ex.getRequestMethod(), body.length == 0 ? HttpRequest.BodyPublishers.noBody()
                        : HttpRequest.BodyPublishers.ofByteArray(body))
                .build();
        HttpResponse<byte[]> response = client.send(request, HttpResponse.BodyHandlers.ofByteArray());
        byte[] out = response.body();
        SimMain.reply(ex, response.statusCode(), response.headers().firstValue("Content-Type").orElse(null),
                out.length == 0 ? null : out);
    }

    private void proxyStream(HttpExchange ex, Player player) throws Exception {
        ensureSim(player);
        player.streams.incrementAndGet();
        try {
            HttpResponse<InputStream> response = client.send(upstream(ex, player).GET().build(),
                    HttpResponse.BodyHandlers.ofInputStream());
            ex.getResponseHeaders().add("Content-Type", "text/event-stream");
            ex.getResponseHeaders().add("Cache-Control", "no-cache, no-transform");
            ex.getResponseHeaders().add("X-Accel-Buffering", "no");
            ex.sendResponseHeaders(200, 0);
            byte[] buffer = new byte[8192];
            try (InputStream in = response.body(); OutputStream out = ex.getResponseBody()) {
                int n;
                while ((n = in.read(buffer)) >= 0) {
                    out.write(buffer, 0, n);
                    out.flush();
                    player.lastSeen = System.currentTimeMillis();
                }
            } catch (IOException ignored) {
                // they closed the tab
            }
        } finally {
            player.streams.decrementAndGet();
        }
    }

    private void staticFile(HttpExchange ex, String rawPath) throws IOException {
        String path = URLDecoder.decode(rawPath, StandardCharsets.UTF_8);
        if (path.equals("/")) path = "/index.html";
        if (path.equals("/robot.json")) path = "/" + robotModel;
        Path root = webRoot;
        Path hostFile = hostWeb == null ? null : hostWeb.resolve(path.substring(1)).normalize();
        if (hostFile != null && hostFile.startsWith(hostWeb) && Files.isRegularFile(hostFile)) root = hostWeb;
        Path file = root.resolve(path.substring(1)).normalize();
        if (!file.startsWith(root) || !Files.isRegularFile(file)) {
            SimMain.reply(ex, 404, "text/plain", "not found".getBytes(StandardCharsets.UTF_8));
            return;
        }
        byte[] body = Files.readAllBytes(file);
        // models are big and rarely change: don't re-download them every visit. "private" keeps
        // them out of Cloudflare's shared cache, they're behind the invite code
        if (path.startsWith("/models/")) ex.getResponseHeaders().add("Cache-Control", "private, max-age=3600");
        else ex.getResponseHeaders().add("Cache-Control", "no-store");
        ex.getResponseHeaders().add("Content-Type", SimMain.contentType(file.toString()));
        ex.sendResponseHeaders(200, body.length);
        try (OutputStream out = ex.getResponseBody()) {
            out.write(body);
        }
    }
}
