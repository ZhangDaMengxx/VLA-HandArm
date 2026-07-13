"""
Browser control panel for CPU-only (GPU-free) retargeting playback.

Runs two local servers:
  * MeshCat  (default :7000)  -- the WebGL 3D view (three.js in the browser)
  * a small control page (:7001) -- pick / switch the .pkl at runtime

Open http://localhost:7001/ in a browser. From there you can:
  - choose any .pkl found in the data/ folder from a dropdown (select by name),
  - or type an arbitrary .pkl path,
  - hot-swap robot + trajectory without restarting,
  - play / pause, change fps, scrub frames.

All rendering happens client-side in the browser (WebGL), so no local GPU/Vulkan is needed.
Reuses the proven Pinocchio + MeshCat path from webgl_replay.py.
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import meshcat
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

from webgl_replay import build_robot, make_qmap, set_q

DATA_DIR = (Path(__file__).absolute().parent / "data")


class Controller:
    def __init__(self, mc_viewer):
        self.mc = mc_viewer
        self.lock = threading.Lock()
        self.viz = None
        self.model = None
        self.qmap = None
        self.traj = None
        self.frame = 0
        self.playing = False
        self.fps = 30.0
        self.dirty = False
        self.name = None
        self.message = "No trajectory loaded. Pick one above."
        self._stop = False

    def list_pkls(self):
        if not DATA_DIR.is_dir():
            return []
        return sorted(p.name for p in DATA_DIR.glob("*.pkl"))

    def _resolve(self, name=None, path=None):
        if path:
            p = Path(path)
            return p if p.is_absolute() else (DATA_DIR / path)
        return DATA_DIR / name

    def load(self, name=None, path=None):
        target = self._resolve(name, path)
        try:
            if not target.exists():
                raise FileNotFoundError(f"{target} not found")
            # build_robot reads the pkl, resolves the URDF via its config_path, and
            # builds the Pinocchio model + visual/collision geometry.
            model, cmodel, vmodel, meta, data = build_robot(str(target))
            qmap = make_qmap(model, list(meta["joint_names"]))
            with self.lock:
                # Clear the previous robot subtree (keep the grid), then load the new one.
                try:
                    self.mc["pinocchio"].delete()
                except Exception:
                    pass
                viz = MeshcatVisualizer(model, cmodel, vmodel)
                viz.initViewer(viewer=self.mc)
                viz.loadViewerModel(rootNodeName="pinocchio")
                self.viz = viz
                self.model = model
                self.qmap = qmap
                self.traj = list(data)
                self.frame = 0
                self.playing = True
                self.dirty = True
                self.name = target.name
                self.message = (
                    f"Loaded {target.name}: {model.name}, "
                    f"{len(self.traj)} frames, {vmodel.ngeoms} visual geoms"
                )
            return True, self.message
        except Exception as e:
            with self.lock:
                self.message = f"ERROR loading {target.name}: {e!r}"
            return False, self.message

    def set_playing(self, val):
        with self.lock:
            self.playing = bool(val)

    def toggle(self):
        with self.lock:
            self.playing = not self.playing

    def set_fps(self, v):
        with self.lock:
            self.fps = max(0.5, float(v))

    def seek(self, f):
        with self.lock:
            if self.traj:
                self.frame = int(f) % len(self.traj)
                self.dirty = True

    def status(self):
        with self.lock:
            return {
                "name": self.name,
                "frame": self.frame,
                "nframes": len(self.traj) if self.traj else 0,
                "playing": self.playing,
                "fps": self.fps,
                "message": self.message,
            }

    def run_player(self):
        while not self._stop:
            sleep_t = 0.05
            with self.lock:
                if self.viz is not None and self.traj and (self.playing or self.dirty):
                    q = pin.neutral(self.model)
                    set_q(q, self.qmap, np.asarray(self.traj[self.frame]))
                    self.viz.display(q)
                    if self.playing:
                        self.frame = (self.frame + 1) % len(self.traj)
                        sleep_t = 1.0 / self.fps
                    self.dirty = False
            time.sleep(sleep_t)


CONTROLLER = None
MESHCAT_URL = ""

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Retargeting replay</title>
<style>
 body{margin:0;font-family:system-ui,Arial,sans-serif;background:#111;color:#ddd}
 #bar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:8px 12px;background:#1c1c22;border-bottom:1px solid #333}
 select,input,button{background:#2a2a33;color:#eee;border:1px solid #444;border-radius:5px;padding:5px 8px;font-size:13px}
 button{cursor:pointer} button:hover{background:#39394a}
 #status{color:#8fd; font-size:12px; margin-left:auto}
 #frame{width:280px}
 iframe{width:100%;height:calc(100vh - 52px);border:0;background:#000}
 label{font-size:12px;color:#aaa}
</style></head><body>
<div id="bar">
 <label>轨迹</label>
 <select id="sel"></select>
 <button onclick="refresh()">刷新列表</button>
 <input id="path" placeholder="或直接输入 .pkl 路径" size="26">
 <button onclick="loadPath()">加载路径</button>
 <button id="pp" onclick="toggle()">⏯ 播放/暂停</button>
 <label>fps</label><input id="fps" type="number" value="30" min="1" max="120" style="width:60px" onchange="setFps()">
 <input id="frame" type="range" min="0" max="0" value="0" oninput="seek(this.value)">
 <span id="status">…</span>
</div>
<iframe id="mc" src="%%MC%%"></iframe>
<script>
 const sel=document.getElementById('sel'), st=document.getElementById('status'),
       fr=document.getElementById('frame'), fps=document.getElementById('fps');
 async function j(u){const r=await fetch(u);return r.json();}
 async function refresh(){const d=await j('/list');
   sel.innerHTML=''; d.pkls.forEach(n=>{const o=document.createElement('option');o.value=n;o.text=n;sel.add(o);});
   if(d.status && d.status.name){sel.value=d.status.name;}
 }
 sel && (sel.onchange=async()=>{await j('/load?name='+encodeURIComponent(sel.value));});
 async function loadPath(){const p=document.getElementById('path').value.trim();
   if(p) await j('/load?path='+encodeURIComponent(p)); }
 async function toggle(){await j('/toggle');}
 async function setFps(){await j('/fps?v='+encodeURIComponent(fps.value));}
 let seeking=false;
 fr.addEventListener('mousedown',()=>seeking=true);
 fr.addEventListener('mouseup',()=>seeking=false);
 async function seek(v){await j('/seek?f='+v);}
 async function poll(){try{const s=await j('/status');
   st.textContent=(s.message||'')+ (s.nframes? '  ['+s.frame+'/'+s.nframes+(s.playing?' ▶':' ⏸')+']':'');
   if(s.nframes){fr.max=s.nframes-1; if(!seeking) fr.value=s.frame;}
 }catch(e){} }
 refresh(); setInterval(poll,400);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        p = u.path
        if p == "/" or p == "/index.html":
            body = PAGE.replace("%%MC%%", MESHCAT_URL).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if p == "/list":
            return self._json({"pkls": CONTROLLER.list_pkls(),
                               "data_dir": str(DATA_DIR),
                               "status": CONTROLLER.status()})
        if p == "/load":
            name = q.get("name", [None])[0]
            path = q.get("path", [None])[0]
            ok, msg = CONTROLLER.load(name=name, path=path)
            return self._json({"ok": ok, "message": msg, "status": CONTROLLER.status()})
        if p == "/play":
            CONTROLLER.set_playing(True); return self._json(CONTROLLER.status())
        if p == "/pause":
            CONTROLLER.set_playing(False); return self._json(CONTROLLER.status())
        if p == "/toggle":
            CONTROLLER.toggle(); return self._json(CONTROLLER.status())
        if p == "/fps":
            CONTROLLER.set_fps(q.get("v", ["30"])[0]); return self._json(CONTROLLER.status())
        if p == "/seek":
            CONTROLLER.seek(q.get("f", ["0"])[0]); return self._json(CONTROLLER.status())
        if p == "/status":
            return self._json(CONTROLLER.status())
        self.send_response(404); self.end_headers()


def main():
    global CONTROLLER, MESHCAT_URL
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7001, help="control panel port")
    ap.add_argument("--load", default=None, help="optional .pkl name/path to load on startup")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    mc = meshcat.Visualizer()
    MESHCAT_URL = mc.url()
    CONTROLLER = Controller(mc)

    if args.selftest:
        pkls = CONTROLLER.list_pkls()
        print("data_dir:", DATA_DIR, "| pkls:", pkls)
        assert pkls, "no .pkl in data/ to selftest"
        ok, msg = CONTROLLER.load(name=pkls[0]); print("load1:", ok, msg)
        for _ in range(3):
            with CONTROLLER.lock:
                q = pin.neutral(CONTROLLER.model)
                set_q(q, CONTROLLER.qmap, np.asarray(CONTROLLER.traj[CONTROLLER.frame]))
                CONTROLLER.viz.display(q)
                CONTROLLER.frame += 1
        ok2, msg2 = CONTROLLER.load(name=pkls[0]); print("reload(switch path):", ok2, msg2)
        print("SELFTEST_OK")
        return

    if args.load:
        CONTROLLER.load(name=args.load, path=args.load)

    threading.Thread(target=CONTROLLER.run_player, daemon=True).start()
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print("=" * 60)
    print(f"Control panel : http://localhost:{args.port}/")
    print(f"MeshCat view  : {MESHCAT_URL}")
    print("Open the control panel in your browser. Ctrl-C to stop.")
    print("=" * 60)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
