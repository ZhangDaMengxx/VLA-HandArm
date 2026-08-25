// urdf_view.js — 浏览器端 URDF 查看器基类。hand3d.js / arm3d.js 共用。
//
// 从 hand3d.js 抽出来的:臂和手的差别只有"哪些关节可驱动 / 有没有 mimic",
// URDF 解析、mesh 加载、相机、光照、resize 全一样。抽出来免得两份各改一遍。
//
// 为什么自己解析 URDF 而不用 urdf-loader:这两个 URDF 都很小(手 13 link/12 joint,
// 臂 10 link/7 joint),自己解析 ~90 行够用,省一个依赖和它的 bundler 适配。

// vendor 保留 upstream 的 examples/jsm 目录结构 —— GLTFLoader 内部有
// `import '../utils/BufferGeometryUtils.js'`,拍平目录会让它解析到 /static/utils/ 而 404。
import * as THREE from "three";
import { GLTFLoader } from "./vendor/jsm/loaders/GLTFLoader.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

export { THREE };

function parseOrigin(el) {
  const o = el?.querySelector(":scope > origin");
  const xyz = (o?.getAttribute("xyz") || "0 0 0").trim().split(/\s+/).map(Number);
  const rpy = (o?.getAttribute("rpy") || "0 0 0").trim().split(/\s+/).map(Number);
  return { xyz, rpy };
}

// ⚠ URDF 的 rpy 是**固定轴** RPY:R = Rz(yaw)·Ry(pitch)·Rx(roll)。
// three.js 的 Euler 默认 order 是 'XYZ',算的是 Rx·Ry·Rz —— 只有单轴非零时才碰巧相等。
// 手的 URDF 有 3 个关节两轴同时非零(base 180° 偏差 = 整只手上下翻);
// 臂的 URDF **7 个里有 5 个**(joint2..joint6),用默认 order 整条臂都是歪的。
// order 用 'ZYX' 与 URDF 等价(实测残差 2e-6°)。所有 origin 一律走这个函数。
export function applyOrigin(obj, xyz, rpy) {
  obj.position.set(xyz[0], xyz[1], xyz[2]);
  obj.rotation.set(rpy[0], rpy[1], rpy[2], "ZYX");
}

// `<mesh scale="sx sy sz">` —— URDF 允许 mesh 自带缩放,**必须读**。
// 不读的后果实测过一次:装配 URDF 里 rh56df_adapter_flange 的 stl 是**毫米**单位
// (包围盒 39.5×38.65×25.0),靠 scale="0.001 0.001 0.001" 折回米。漏掉这个属性
// 就按 39.5 **米** 渲染,而 _frameCamera() 按整体包围盒取景 —— 相机被撑到 32m 外,
// 0.75m 的臂和 0.14m 的手缩成几个像素,画面上**只剩那个法兰**。
// 症状是"模型只有一个零件",看着像 mesh 没加载上,其实全加载了,是取景被一个
// 放大一千倍的件带跑了。缺省 1 1 1(URDF 规范的默认值)。
export function parseScale(el) {
  const s = (el?.getAttribute("scale") || "1 1 1").trim().split(/\s+/).map(Number);
  return s.length === 3 && s.every(v => Number.isFinite(v) && v !== 0) ? s : [1, 1, 1];
}

export class UrdfViewer {
  /** opts: {zUp=true, bg=0x0e0f12, gridSize} */
  constructor(container, opts = {}) {
    this.container = container;
    this.opts = opts;
    this.joints = {};              // name -> {obj, axis, baseQuat, lower, upper}
    this.ready = false;
    this._initScene();
  }

  _initScene() {
    const o = this.opts;
    const w = this.container.clientWidth || 640;
    const h = this.container.clientHeight || 480;

    this.scene = new THREE.Scene();
    // 底色和 CSS 的 `--screen` **必须一致**:canvas 有 opacity 淡入(.loaded 类),
    // 淡入期间露的是底下 .screen 的背景色。两个数不一样的话加载时会先闪一下另一个色。
    // 灰度不是随便挑的 —— mesh 是浅灰 #b8bcc4,底色越亮机器人越不显眼。
    // #3a3d42 对 mesh 的对比度是 5.73:1(原来的近黑 #0e0f12 是 10.06:1),
    // 再亮就得连带把 mesh 调深了:#5a5e66 只剩 3.42:1、#8e939c 只剩 1.62:1。
    this.scene.background = new THREE.Color(o.bg ?? 0x3a3d42);

    this.camera = new THREE.PerspectiveCamera(42, w / h, 0.01, 50);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(w, h);
    // ⚠ 页面 CSS 有 `.screen canvas { opacity: 0 }`,靠 .loaded 类才显形(原本是给
    // Rerun iframe 做淡入的)。不加这个类,画面渲染正常但整块是黑的 —— 看着像没出图。
    this.renderer.domElement.classList.add("loaded");
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x444455, 1.7));
    // 三点布光只补模型自身的背光面，不启用 shadowMap，避免实时控制时增加阴影开销。
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(0.8, 1.2, 1.0);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xdce7ff, 0.9);
    fill.position.set(-0.9, 0.5, 0.6);
    this.scene.add(fill);
    const rim = new THREE.DirectionalLight(0xfff1df, 0.55);
    rim.position.set(0.0, 1.1, -1.0);
    this.scene.add(rim);
    const g = o.gridSize ?? 0.4;
    // 网格线跟着底色走。原来那对(0x2a2d33 / 0x1c1f24)是**比近黑底还暗**的,
    // 靠"比背景亮一点"来显形;换灰底后它们变成比底色暗,直接消失。
    // 现在这对比 #3a3d42 亮一档,对比度约 1.55:1 —— 看得见但不抢戏。
    const grid = new THREE.GridHelper(g, 16, 0x55585d, 0x4a4d53);
    grid.position.y = -0.001;
    this.scene.add(grid);

    // URDF 是 Z-up,three.js 默认 Y-up。转根节点,不然模型是躺着的。
    this.root = new THREE.Group();
    if (o.zUp !== false) this.root.rotation.x = -Math.PI / 2;
    this.scene.add(this.root);

    new ResizeObserver(() => this._resize()).observe(this.container);
    this._animate();
  }

  _resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  _animate = () => {
    this._raf = requestAnimationFrame(this._animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  /** 加载 URDF + mesh。mesh 路径按 URDF 所在目录解析。返回可动关节数。 */
  async load(urdfUrl) {
    const txt = await (await fetch(urdfUrl)).text();
    const xml = new DOMParser().parseFromString(txt, "text/xml");
    const base = urdfUrl.replace(/\/[^/]*$/, "/");

    const linkGroups = {};
    for (const link of xml.querySelectorAll("robot > link")) {
      const g = new THREE.Group();
      g.name = link.getAttribute("name");
      linkGroups[g.name] = g;
    }

    // 按 joint 建父子关系。fixed 也要建,不然树断开。
    const childOf = new Set();
    for (const j of xml.querySelectorAll("robot > joint")) {
      const name = j.getAttribute("name");
      const type = j.getAttribute("type");
      const parent = j.querySelector(":scope > parent")?.getAttribute("link");
      const child = j.querySelector(":scope > child")?.getAttribute("link");
      if (!linkGroups[parent] || !linkGroups[child]) continue;

      const { xyz, rpy } = parseOrigin(j);
      // 关节自己一层:origin 固定在这层,旋转加在子 link 上。
      // 不分层的话旋转会把 origin 一起转掉。
      const jg = new THREE.Group();
      applyOrigin(jg, xyz, rpy);
      jg.add(linkGroups[child]);
      linkGroups[parent].add(jg);
      childOf.add(child);

      if (type === "revolute" || type === "continuous") {
        const a = (j.querySelector(":scope > axis")?.getAttribute("xyz") || "1 0 0")
          .trim().split(/\s+/).map(Number);
        const lim = j.querySelector(":scope > limit");
        this.joints[name] = {
          obj: linkGroups[child],
          axis: new THREE.Vector3(a[0], a[1], a[2]).normalize(),
          baseQuat: linkGroups[child].quaternion.clone(),
          lower: parseFloat(lim?.getAttribute("lower") ?? "-3.14159"),
          upper: parseFloat(lim?.getAttribute("upper") ?? "3.14159"),
        };
      }
    }

    // ⚠ 根 link = 不是任何 joint 的 child 的那个。**必须只有一个** ——
    // 有多个的话 find() 只取第一个,其余整棵子树静默不进场景:画面上少一大块,
    // 而控制台一声不响(mesh 全加载成功了,只是没挂上)。
    const rootNames = Object.keys(linkGroups).filter(n => !childOf.has(n));
    if (rootNames.length !== 1) {
      console.warn("[urdf_view] URDF 有多个根 link,除第一个外都不会显示:", rootNames);
    }
    this.rootLink = linkGroups[rootNames[0]];
    this.root.add(this.rootLink);
    this._linkGroups = linkGroups;          // linkBoxes() 诊断用
    // 没挂进根的 link:说明 joint 的 parent/child 引用不上,那棵子树整块看不见
    const orphan = Object.keys(linkGroups).filter(n => {
      let o = linkGroups[n];
      while (o.parent) { if (o === this.rootLink) return false; o = o.parent; }
      return o !== this.rootLink && linkGroups[n] !== this.rootLink;
    });
    if (orphan.length) console.warn("[urdf_view] 这些 link 没接上根,不会显示:", orphan);

    const loader = new GLTFLoader();
    const jobs = [];
    for (const link of xml.querySelectorAll("robot > link")) {
      const lname = link.getAttribute("name");
      for (const vis of link.querySelectorAll(":scope > visual")) {
        const mesh = vis.querySelector(":scope > geometry > mesh");
        if (!mesh) continue;
        const { xyz, rpy } = parseOrigin(vis);
        jobs.push(this._loadMesh(loader, base + mesh.getAttribute("filename"),
                                 linkGroups[lname], xyz, rpy, parseScale(mesh)));
      }
    }
    const res = await Promise.allSettled(jobs);
    this.meshOk = res.filter(r => r.value === true).length;
    this.meshTotal = jobs.length;
    this.ready = true;
    this._frameCamera();
    return Object.keys(this.joints).length;
  }

  _loadMesh(loader, url, parent, xyz, rpy, scale) {
    return new Promise((resolve) => {
      loader.load(url, (gltf) => {
        const o = gltf.scene;
        applyOrigin(o, xyz, rpy);          // visual 的 origin 同样是固定轴 RPY
        if (scale && (scale[0] !== 1 || scale[1] !== 1 || scale[2] !== 1)) {
          o.scale.set(scale[0], scale[1], scale[2]);
        }
        o.traverse(c => {
          // 没材质或材质是纯黑的一律换掉。臂的 mesh 由 STL 转来,本来无材质;
          // 原始 dae 里 diffuse 是 `0 0 0 1`,照用会渲染成全黑。
          if (!c.isMesh) return;
          const col = c.material?.color;
          if (!col || (col.r + col.g + col.b) < 0.02) {
            c.material = new THREE.MeshStandardMaterial(
              { color: this.opts.meshColor ?? 0xb8bcc4, roughness: .55 });
          }
          if (c.geometry && !c.geometry.attributes.normal) {
            c.geometry.computeVertexNormals();   // STL 转的 glb 没法线
          }
        });
        parent.add(o);
        resolve(true);
      }, undefined, () => {
        console.warn("[urdf_view] mesh 加载失败,跳过:", url);
        resolve(false);
      });
    });
  }

  /** 每个 link 的世界包围盒。诊断用 —— 单位错/取景被带跑时,一眼看出是哪个件。 */
  linkBoxes() {
    this.scene.updateMatrixWorld(true);
    const out = [];
    for (const [name, g] of Object.entries(this._linkGroups || {})) {
      const b = new THREE.Box3().setFromObject(g);
      if (b.isEmpty()) { out.push({ link: name, empty: true }); continue; }
      const s = b.getSize(new THREE.Vector3());
      out.push({ link: name, size: [+s.x.toFixed(4), +s.y.toFixed(4), +s.z.toFixed(4)],
                 maxDim: +Math.max(s.x, s.y, s.z).toFixed(4) });
    }
    return out.sort((a, b) => (b.maxDim || 0) - (a.maxDim || 0));
  }

  _frameCamera() {
    // setFromObject 走世界矩阵,先刷一遍再算 —— 刚 add 完矩阵还是脏的。
    // 算出来已是世界坐标,不要再 localToWorld(那会把 root 的 -90° 又叠一次)。
    this.scene.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(this.rootLink);
    if (box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3());
    const r = box.getSize(new THREE.Vector3()).length() * 0.5 || 0.1;
    // ⚠ 把取景输入打出来。"模型只看到一个零件"这种现象,光看画面分不清是
    // mesh 没加载、还是某个件单位错了把相机撑到几十米外(踩过后者:毫米单位的
    // 法兰漏读 scale,按 39.5m 渲染,相机退到 77m,0.75m 的臂只剩 1.2% 画面宽)。
    // 有了这几个数就能直接判:r 远大于模型真实尺寸 = 有件被放大了。
    const sz = box.getSize(new THREE.Vector3());
    console.log(`[urdf_view] 取景 bbox=${sz.x.toFixed(3)}x${sz.y.toFixed(3)}`
      + `x${sz.z.toFixed(3)}m  r=${r.toFixed(3)}m  相机距=${(r * 2.2).toFixed(2)}m`
      + `  mesh=${this.meshOk}/${this.meshTotal}`);
    if (r > 3.0) {
      console.warn("[urdf_view] 取景半径 >3m,某个 mesh 可能单位错(mm 当成 m)。"
        + "最大的几个 link:", this.linkBoxes().slice(0, 3));
    }
    this.controls.target.copy(c);
    const minAspect = this.opts.minFrameAspect ?? 0;
    const frameScale = minAspect > 0 && this.camera.aspect < minAspect
      ? minAspect / this.camera.aspect : 1;
    this.camera.position.copy(c).add(
      new THREE.Vector3(r * 1.6, r * 1.2, r * 1.6).multiplyScalar(frameScale));
    this.camera.near = Math.max(0.001, r * 0.02);
    this.camera.far = Math.max(1.0, r * 60);
    this.camera.updateProjectionMatrix();
  }

  /** 设单个关节角(rad),按 URDF 限位夹取。 */
  setJoint(name, val) {
    const j = this.joints[name];
    if (!j) return;
    const v = Math.max(j.lower, Math.min(j.upper, val));
    j.obj.quaternion.copy(j.baseQuat).multiply(
      new THREE.Quaternion().setFromAxisAngle(j.axis, v));
  }

  /** 一次设一组:{jointName: rad}。 */
  setJoints(map) {
    if (!this.ready || !map) return;
    for (const [n, v] of Object.entries(map)) {
      if (v != null) this.setJoint(n, v);
    }
  }

  dispose() {
    if (this._raf) cancelAnimationFrame(this._raf);
    this.renderer?.dispose();
    const el = this.renderer?.domElement;
    if (el?.parentNode) el.parentNode.removeChild(el);
  }
}
