package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Web server that lives for as long as the Robot Controller app is running (it is started by the
 * SDK through the @OnCreateEventLoop hook below), so the page works BETWEEN OpModes too.
 *
 *   GET /               the control page
 *   GET /in?m=..&s=..   key state from the page; the reply is the current state + telemetry (JSON)
 *   GET /ops            list of OpModes you can pick from (JSON)
 *   GET /cmd?c=init&n=NAME | c=start | c=stop    init / start / stop an OpMode
 *   GET /cam            live camera (MJPEG)
 *
 * If key packets stop arriving for TIMEOUT_MS, isActive() turns false and the OpMode stops the robot.
 */
public class KeyboardServer {
    public static final int PORT = 8090;             // 8081 is used by the FTC SDK itself
    public static final long TIMEOUT_MS = 300;
    public static final int CAM_FPS = 15;            // max frames per second sent to the page
    public static final int CAM_JPEG_QUALITY = 70;   // 1-100, lower = smaller/faster

    // Key bits. These must match the numbers in the web page below.
    public static final int KEY_FORWARD = 1;     // W
    public static final int KEY_BACK = 2;        // S
    public static final int KEY_STRAFE_L = 4;    // A
    public static final int KEY_STRAFE_R = 8;    // D
    public static final int KEY_TURN_L = 16;     // Q
    public static final int KEY_TURN_R = 32;     // E
    public static final int KEY_INTAKE_IN = 64;  // I
    public static final int KEY_INTAKE_OUT = 128;// K
    public static final int KEY_FEED = 256;      // Space  (right bumper)
    public static final int KEY_FLYWHEEL = 512;  // F      (B button)
    public static final int KEY_AUTO = 1024;     // G      (A button)
    public static final int KEY_REZERO = 2048;   // R      (Y button)

    private static final String STOP_ROBOT = "$Stop$Robot$";   // the SDK's "no OpMode running" name

    private static final KeyboardServer INSTANCE = new KeyboardServer();

    public static KeyboardServer getInstance() {
        return INSTANCE;
    }

    /** The SDK calls this when the robot's event loop is created, so the server starts with the app. */
    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        INSTANCE.eventLoop = eventLoop;
        INSTANCE.start();
    }

    private volatile FtcEventLoop eventLoop = null;
    private volatile int mask = 0;
    private volatile double speed = 0.6;
    private volatile long lastPacketMs = 0;
    private volatile boolean running = false;
    private volatile String error = null;
    private volatile CameraStreamProcessor camera = null;
    private volatile String phase = "";            // set by the OpMode: "INIT", "RUNNING" or ""
    private volatile String telemetryText = "";
    private ServerSocket serverSocket;

    private KeyboardServer() {
    }

    // ------------------------------------------------------------ used by the OpMode
    /** Safe to call repeatedly. */
    public synchronized void start() {
        if (running) return;
        running = true;
        error = null;
        Thread t = new Thread(this::serve, "KeyboardServer");
        t.setDaemon(true);
        t.start();
    }

    public synchronized void stop() {
        running = false;
        mask = 0;
        try {
            if (serverSocket != null) serverSocket.close();
        } catch (IOException ignored) {
        }
    }

    public void setCamera(CameraStreamProcessor camera) {
        this.camera = camera;
    }

    public void setPhase(String phase) {
        this.phase = phase == null ? "" : phase;
    }

    public void setTelemetry(String text) {
        this.telemetryText = text == null ? "" : text;
    }

    /** Forget any held keys (call when the OpMode stops). */
    public void clearInput() {
        mask = 0;
    }

    /** True while the page is open and sending key updates. */
    public boolean isActive() {
        return System.currentTimeMillis() - lastPacketMs < TIMEOUT_MS;
    }

    /** Null if the server is fine, otherwise why it couldn't start (e.g. port already in use). */
    public String getError() {
        return error;
    }

    /** Milliseconds since the page last sent an update (-1 if never). */
    public long getLastPacketAgeMs() {
        return lastPacketMs == 0 ? -1 : System.currentTimeMillis() - lastPacketMs;
    }

    public int getMask() {
        return mask;
    }

    /** 0.2 - 1.0 */
    public double getSpeed() {
        return speed;
    }

    // ------------------------------------------------------------ server
    private void serve() {
        try {
            serverSocket = new ServerSocket();
            serverSocket.setReuseAddress(true);
            serverSocket.bind(new InetSocketAddress(PORT));
            while (running) {
                final Socket client = serverSocket.accept();
                // One thread per connection: browsers open idle "spare" connections, and
                // handling them one at a time would stall the key updates.
                Thread t = new Thread(() -> {
                    try {
                        handle(client);
                    } catch (IOException ignored) {
                    } finally {
                        try {
                            client.close();
                        } catch (IOException ignored) {
                        }
                    }
                }, "KeyboardServer-client");
                t.setDaemon(true);
                t.start();
            }
        } catch (IOException e) {
            // A normal shutdown closes the socket; anything else (port in use) is reported.
            if (running) {
                error = "Could not start on port " + PORT + ": " + e.getMessage();
                running = false;
            }
        }
    }

    private void handle(Socket client) throws IOException {
        client.setSoTimeout(1000);
        BufferedReader in = new BufferedReader(
                new InputStreamReader(client.getInputStream(), StandardCharsets.UTF_8));
        String requestLine = in.readLine();
        if (requestLine == null) return;

        String header;
        while ((header = in.readLine()) != null && !header.isEmpty()) {
            // skip headers
        }

        String[] parts = requestLine.split(" ");
        String path = parts.length > 1 ? parts[1] : "/";
        OutputStream out = client.getOutputStream();

        if (path.startsWith("/cam")) {
            streamCamera(out);
        } else if (path.startsWith("/in?")) {
            parseInput(path);
            send(out, "200 OK", "application/json; charset=utf-8", stateJson());
        } else if (path.startsWith("/ops")) {
            send(out, "200 OK", "application/json; charset=utf-8", opsJson());
        } else if (path.startsWith("/cmd?")) {
            Map<String, String> q = parseQuery(path);
            send(out, "200 OK", "text/plain; charset=utf-8", runCommand(q.get("c"), q.get("n")));
        } else if (path.equals("/") || path.startsWith("/index")) {
            send(out, "200 OK", "text/html; charset=utf-8", PAGE);
        } else {
            send(out, "404 Not Found", "text/plain", "not found");
        }
    }

    // ------------------------------------------------------------ OpMode control
    private OpModeManagerImpl manager() {
        FtcEventLoop loop = eventLoop;
        return loop == null ? null : loop.getOpModeManager();
    }

    /**
     * Calls the first method in "names" that exists on target, by reflection. SDK versions name the
     * OpMode init/start/stop methods differently, so this avoids compile errors on a missing name.
     * Pass arg = null for a method with no parameters.
     */
    private static void invoke(Object target, String arg, String... names) throws Exception {
        for (String n : names) {
            try {
                if (arg == null) {
                    target.getClass().getMethod(n).invoke(target);
                } else {
                    target.getClass().getMethod(n, String.class).invoke(target, arg);
                }
                return;
            } catch (NoSuchMethodException ignored) {
                // try the next name
            } catch (java.lang.reflect.InvocationTargetException e) {
                Throwable cause = e.getCause() != null ? e.getCause() : e;
                throw new Exception(String.valueOf(cause.getMessage()), cause);
            }
        }
        throw new NoSuchMethodException("none of these exist in this SDK: " + java.util.Arrays.toString(names));
    }

    /** Name of the OpMode that is initialized/running, or STOP_ROBOT when none is. */
    private String activeName(OpModeManagerImpl m) {
        try {
            Object name = m.getClass().getMethod("getActiveOpModeName").invoke(m);
            if (name != null) return String.valueOf(name);
        } catch (Exception ignored) {
        }
        return STOP_ROBOT;
    }

    private String runCommand(String cmd, String name) {
        OpModeManagerImpl m = manager();
        if (m == null) return "Robot not ready yet";
        if (cmd == null) return "No command";
        try {
            String active = activeName(m);
            boolean idle = STOP_ROBOT.equals(active);
            switch (cmd) {
                case "init":
                    if (name == null || name.isEmpty()) return "Pick an OpMode first";
                    if (!idle) return "Stop the running OpMode (" + active + ") first";
                    invoke(m, name, "initActiveOpMode", "initOpMode");
                    return "Initializing " + name + "... keep the robot still";
                case "start":
                    if (idle) return "Nothing is initialized yet";
                    invoke(m, null, "startActiveOpMode", "startOpMode");
                    return "Started " + active;
                case "stop":
                    invoke(m, null, "stopActiveOpMode", "stopOpMode");
                    return "Stopped";
                default:
                    return "Unknown command";
            }
        } catch (Exception e) {
            return "Error: " + e.getMessage();
        }
    }

    private String opsJson() {
        StringBuilder sb = new StringBuilder("[");
        try {
            // Read each entry's "name" and "flavor" by reflection, so this compiles no matter which
            // package this SDK version keeps its OpModeMeta class in.
            List<?> list = RegisteredOpModes.getInstance().getOpModes();
            boolean first = true;
            for (Object meta : list) {
                try {
                    String name = String.valueOf(meta.getClass().getField("name").get(meta));
                    String flavor = String.valueOf(meta.getClass().getField("flavor").get(meta));
                    if (flavor.equals("SYSTEM")) continue;
                    if (!first) sb.append(',');
                    first = false;
                    sb.append("{\"n\":").append(json(name))
                            .append(",\"f\":").append(json(flavor)).append('}');
                } catch (Exception ignored) {
                }
            }
        } catch (Exception ignored) {
        }
        return sb.append(']').toString();
    }

    private String stateJson() {
        OpModeManagerImpl m = manager();
        String active = m == null ? "" : activeName(m);
        boolean idle = m == null || STOP_ROBOT.equals(active);
        String ph = idle ? "IDLE" : (phase.isEmpty() ? "ACTIVE" : phase);
        String tel = idle ? "" : telemetryText;
        return "{\"active\":" + json(idle ? "" : active)
                + ",\"phase\":" + json(ph)
                + ",\"telemetry\":" + json(tel) + "}";
    }

    private static String json(String s) {
        StringBuilder b = new StringBuilder("\"");
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"':
                    b.append("\\\"");
                    break;
                case '\\':
                    b.append("\\\\");
                    break;
                case '\n':
                    b.append("\\n");
                    break;
                case '\t':
                    b.append("\\t");
                    break;
                default:
                    if (c >= 0x20) b.append(c);
            }
        }
        return b.append('"').toString();
    }

    // ------------------------------------------------------------ camera + helpers
    /** Sends the camera as an MJPEG stream (multipart/x-mixed-replace) until the browser disconnects. */
    private void streamCamera(OutputStream out) throws IOException {
        CameraStreamProcessor cam = camera;
        if (cam == null) {
            send(out, "404 Not Found", "text/plain", "no camera");
            return;
        }
        String head = "HTTP/1.1 200 OK\r\n"
                + "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n";
        out.write(head.getBytes(StandardCharsets.UTF_8));
        out.flush();

        long lastFrame = -1;
        long frameDelayMs = 1000 / CAM_FPS;
        // Stop streaming if the OpMode ended (the camera object changed or was removed).
        while (running && camera == cam) {
            long id = cam.getFrameCount();
            if (id > 0 && id != lastFrame) {
                byte[] jpeg = cam.getJpeg(CAM_JPEG_QUALITY);
                String partHead = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + jpeg.length + "\r\n\r\n";
                out.write(partHead.getBytes(StandardCharsets.UTF_8));
                out.write(jpeg);
                out.write("\r\n".getBytes(StandardCharsets.UTF_8));
                out.flush();   // throws once the browser closes the connection, which ends the loop
                lastFrame = id;
            }
            try {
                Thread.sleep(frameDelayMs);
            } catch (InterruptedException e) {
                return;
            }
        }
    }

    private static Map<String, String> parseQuery(String path) {
        Map<String, String> map = new HashMap<>();
        int q = path.indexOf('?');
        if (q < 0) return map;
        for (String kv : path.substring(q + 1).split("&")) {
            int eq = kv.indexOf('=');
            if (eq <= 0) continue;
            try {
                map.put(kv.substring(0, eq), URLDecoder.decode(kv.substring(eq + 1), "UTF-8"));
            } catch (Exception ignored) {
            }
        }
        return map;
    }

    private void parseInput(String path) {
        Map<String, String> q = parseQuery(path);
        try {
            if (q.containsKey("m")) mask = Integer.parseInt(q.get("m"));
            if (q.containsKey("s")) {
                speed = Math.max(0.2, Math.min(1.0, Integer.parseInt(q.get("s")) / 100.0));
            }
        } catch (NumberFormatException ignored) {
        }
        lastPacketMs = System.currentTimeMillis();
    }

    private void send(OutputStream out, String status, String type, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        String head = "HTTP/1.1 " + status + "\r\n"
                + "Content-Type: " + type + "\r\n"
                + "Content-Length: " + bytes.length + "\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n";
        out.write(head.getBytes(StandardCharsets.UTF_8));
        out.write(bytes);
        out.flush();
    }

    // ---------------------------------------------------------------- the web page
    private static final String PAGE =
            "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Robot control</title>"
                    + "<style>"
                    + "body{font-family:sans-serif;background:#111827;color:#e5e7eb;text-align:center;margin:0;padding:24px}"
                    + "h1{margin:0 0 12px}"
                    + "#st{display:inline-block;padding:4px 14px;border-radius:12px;background:#7f1d1d}"
                    + "#st.ok{background:#166534}"
                    + ".row{margin:14px 0}"
                    + "select,button{font-size:16px;padding:6px 12px;margin:0 4px;border-radius:6px;border:1px solid #374151;background:#1f2937;color:#e5e7eb}"
                    + "button{cursor:pointer}button:hover{background:#2563eb}"
                    + ".grid{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin:24px auto;max-width:640px}"
                    + ".k{width:110px;padding:12px 4px;border:1px solid #374151;border-radius:8px;background:#1f2937}"
                    + ".k b{display:block;font-size:20px}"
                    + ".k span{font-size:12px;color:#9ca3af}"
                    + ".k.on{background:#2563eb;border-color:#60a5fa}"
                    + "p{color:#9ca3af}"
                    + "#camBox{position:relative;width:640px;max-width:100%;aspect-ratio:4/3;margin:12px auto;background:#000;"
                    + "border:1px solid #374151;border-radius:8px;overflow:hidden}"
                    + "#cam{display:none;width:100%;height:100%;object-fit:contain}"
                    + "#camMsg{position:absolute;top:0;left:0;right:0;bottom:0;display:flex;align-items:center;justify-content:center;color:#9ca3af}"
                    + "#tl{text-align:left;max-width:640px;margin:16px auto;padding:12px;background:#0b1220;border:1px solid #374151;"
                    + "border-radius:8px;white-space:pre-wrap;font-family:monospace;font-size:13px;height:260px;overflow:auto}"
                    + "</style></head><body>"
                    + "<h1>Robot control</h1><div id='st'>offline</div>"
                    + "<div class='row'><select id='ops'></select>"
                    + "<button id='bi'>Init</button><button id='bs'>Start</button><button id='bx'>Stop</button></div>"
                    + "<div class='row'>State: <b id='ph'>-</b> <span id='msg'></span></div>"
                    + "<p>Keep this tab focused to drive. Releasing all keys, or switching tabs, stops the robot.</p>"
                    + "<div id='camBox'><img id='cam' alt=''><div id='camMsg'>Camera starts when an OpMode is initialized</div></div>"
                    + "<div>Speed: <input id='sp' type='range' min='20' max='100' value='60'> <span id='spv'>60</span>%</div>"
                    + "<div class='grid' id='g'></div>"
                    + "<pre id='tl'></pre>"
                    + "<script>"
                    + "var K=[['KeyW',1,'W','Forward'],['KeyS',2,'S','Back'],['KeyA',4,'A','Strafe left'],"
                    + "['KeyD',8,'D','Strafe right'],['KeyQ',16,'Q','Turn left'],['KeyE',32,'E','Turn right'],"
                    + "['KeyI',64,'I','Intake in'],['KeyK',128,'K','Intake out'],['Space',256,'Space','Feed launcher'],"
                    + "['KeyF',512,'F','Flywheel on/off'],['KeyG',1024,'G','Auto go to 0,0'],['KeyR',2048,'R','Re-zero position']];"
                    + "var down={},latched=0,els={},g=document.getElementById('g');"
                    + "var cam=document.getElementById('cam'),ops=document.getElementById('ops'),msg=document.getElementById('msg');"
                    + "var sp=document.getElementById('sp'),spv=document.getElementById('spv'),st=document.getElementById('st');"
                    + "var ph=document.getElementById('ph'),tl=document.getElementById('tl');"
                    + "K.forEach(function(k){var d=document.createElement('div');d.className='k';"
                    + "d.innerHTML='<b>'+k[2]+'</b><span>'+k[3]+'</span>';g.appendChild(d);els[k[0]]=d;});"
                    + "function find(c){for(var i=0;i<K.length;i++){if(K[i][0]===c)return K[i];}return null;}"
                    + "addEventListener('keydown',function(e){var k=find(e.code);if(!k)return;e.preventDefault();"
                    + "down[k[0]]=1;latched|=k[1];els[k[0]].className='k on';});"
                    + "addEventListener('keyup',function(e){var k=find(e.code);if(!k)return;e.preventDefault();"
                    + "delete down[k[0]];els[k[0]].className='k';});"
                    + "function clearKeys(){down={};for(var c in els){els[c].className='k';}}"
                    + "addEventListener('blur',clearKeys);"
                    + "document.addEventListener('visibilitychange',function(){if(document.hidden){clearKeys();}});"
                    + "sp.oninput=function(){spv.textContent=sp.value;};"
                    + "sp.onchange=function(){sp.blur();};"
                    + "ops.onchange=function(){ops.blur();};"
                    + "function loadOps(){fetch('/ops',{cache:'no-store'}).then(function(r){return r.json();}).then(function(l){"
                    + "var cur=ops.value;ops.innerHTML='';l.forEach(function(o){var e=document.createElement('option');"
                    + "e.value=o.n;e.textContent=o.n+' ('+o.f+')';ops.appendChild(e);});"
                    + "if(cur){ops.value=cur;}else{for(var i=0;i<l.length;i++){if(l[i].n==='Prototype chassis + camera'){ops.value='Prototype chassis + camera';}}}"
                    + "}).catch(function(){});}"
                    + "function cmd(c){var u='/cmd?c='+c;if(c==='init'){u+='&n='+encodeURIComponent(ops.value);}"
                    + "fetch(u,{cache:'no-store'}).then(function(r){return r.text();})"
                    + ".then(function(t){msg.textContent=t;}).catch(function(){msg.textContent='Request failed';});}"
                    + "document.getElementById('bi').onclick=function(){cmd('init');this.blur();};"
                    + "document.getElementById('bs').onclick=function(){cmd('start');this.blur();};"
                    + "document.getElementById('bx').onclick=function(){cmd('stop');this.blur();};"
                    + "loadOps();setInterval(function(){if(!ops.options.length){loadOps();}},3000);"
                    + "var lastPh='',fails=0,lastTel=0,camMsg=document.getElementById('camMsg');"
                    + "var bi=document.getElementById('bi'),bs=document.getElementById('bs'),bx=document.getElementById('bx');"
                    + "function setButtons(p){var idle=(p==='IDLE');"
                    + "ops.style.display=idle?'':'none';bi.style.display=idle?'':'none';"
                    + "bs.style.display=(p==='INIT'||p==='ACTIVE')?'':'none';"
                    + "bx.style.display=(p==='RUNNING'||p==='ACTIVE')?'':'none';}"
                    + "function showCam(on){if(on){camMsg.style.display='none';cam.style.display='block';cam.src='/cam?'+Date.now();}"
                    + "else{cam.removeAttribute('src');cam.style.display='none';camMsg.style.display='flex';}}"
                    + "cam.onerror=function(){setTimeout(function(){if(lastPh!==''&&lastPh!=='IDLE'){cam.src='/cam?'+Date.now();}},2000);};"
                    + "function keyMask(){var m=latched;latched=0;K.forEach(function(k){if(down[k[0]]){m|=k[1];}});return m;}"
                    + "function tick(){"
                    + "fetch('/in?m='+keyMask()+'&s='+sp.value,{cache:'no-store'})"
                    + ".then(function(r){return r.json();})"
                    + ".then(function(s){fails=0;st.textContent='connected';st.className='ok';"
                    + "var p=s.phase+(s.phase!=='IDLE'&&s.active?' ('+s.active+')':'');"
                    + "if(ph.textContent!==p){ph.textContent=p;}"
                    + "if((s.phase!=='IDLE')!==(lastPh!=='IDLE')){showCam(s.phase!=='IDLE');}"
                    + "if(s.phase!==lastPh){setButtons(s.phase);}"
                    + "lastPh=s.phase;"
                    + "var now=Date.now();"
                    + "if(s.telemetry!==tl.textContent&&now-lastTel>250){tl.textContent=s.telemetry;lastTel=now;}})"
                    + ".catch(function(){fails++;if(fails>=3){st.textContent='offline';st.className='';}})"
                    + ".then(function(){setTimeout(tick,50);});}"
                    + "showCam(false);setButtons('IDLE');tick();"
                    + "</script></body></html>";
}