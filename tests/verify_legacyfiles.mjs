/* 旧示例文件迁移加载验证：example/*.json（旧 X-Y 方案）经 openDocument 迁移后可解，数值合理。 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
function makeCtx() {
  const store = {};
  return new Proxy(store, {
    get(t, k) {
      if (k === "measureText") { return () => ({ width: 10 }); }
      if (k === "getImageData") { return () => ({ data: [] }); }
      if (k in t) { return t[k]; }
      return () => undefined;
    },
    set(t, k, v) { t[k] = v; return true; },
  });
}
function makeEl(id) {
  const el = {
    _id: id, value: "", checked: false, textContent: "", innerHTML: "", title: "",
    width: 800, height: 600, clientWidth: 800, clientHeight: 600,
    style: {}, dataset: {}, children: [], childNodes: [],
    parentNode: null, nextSibling: null,
    classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    _ls: {},
    addEventListener(ev, fn) { (this._ls[ev] = this._ls[ev] || []).push(fn); },
    removeEventListener() {},
    appendChild(c) { this.children.push(c); return c; },
    insertBefore(c) { this.children.push(c); return c; },
    remove() {},
    setAttribute() {}, getAttribute() { return null; },
    getElementsByTagName() { return []; },
    getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 600 }; },
    getContext() { return makeCtx(); },
    click() { (this._ls.click || []).forEach((f) => f({ target: this, preventDefault() {} })); },
    focus() {}, blur() {},
  };
  return el;
}
const elCache = {};
globalThis.window = globalThis;
globalThis.document = {
  readyState: "complete", title: "",
  body: makeEl("body"),
  getElementById(id) { return elCache[id] || (elCache[id] = makeEl(id)); },
  createElement(tag) { return makeEl(tag); },
  addEventListener() {}, removeEventListener() {},
};
globalThis.ResizeObserver = class { observe() {} unobserve() {} disconnect() {} };
globalThis.requestAnimationFrame = () => 1;
globalThis.cancelAnimationFrame = () => {};
globalThis.devicePixelRatio = 1;
globalThis.localStorage = { getItem: () => null, setItem() {}, removeItem() {} };
globalThis.alert = (m) => { throw new Error("unexpected alert: " + m); };
globalThis.addEventListener = () => {};
globalThis.removeEventListener = () => {};
function loadLib(rel) { (0, eval)(readFileSync(path.join(ROOT, rel), "utf8")); }
loadLib("design.js");
loadLib("solver.js");
{
  const html = readFileSync(path.join(ROOT, "index.html"), "utf8");
  const m = html.match(/<script>([\s\S]*?)<\/script>/);
  (0, eval)(m[1]);
}
const { FrameLab } = globalThis;
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}
for (const f of ["framelab-2026-09-15T07-26-30.json", "framelab-2026-09-15T07-28-49.json"]) {
  console.log("## " + f);
  const snap = JSON.parse(readFileSync(path.join(ROOT, "example", f), "utf8"));
  try {
    FrameLab.openDoc(f, JSON.parse(JSON.stringify(snap)));
    check("打开", true);
  } catch (e) { check("打开", false, String(e.message || e).slice(0, 200)); continue; }
  const st = FrameLab.state();
  const badN = st.model.nodes.filter((n) => !(n.z !== undefined && isFinite(n.z)));
  check("节点均有 z", badN.length === 0, "nodes=" + st.model.nodes.length);
  const ana = st.ana;
  check("求解 ok", ana && ana.ok === true, ana && ("maxDisp=" + ana.res.maxDisp.toFixed(3) + "mm maxM=" + ana.res.maxM.toFixed(2)));
  // 与旧方案直接求解对照（迁移前后数值一致）
  const ref = globalThis.FrameSolver.solve({
    model: snap.model, NSUB: snap.D ? snap.D.NSUB : 5, AMESH: snap.D ? snap.D.AMESH : 2,
    memStiff: snap.memStiff, memMat: snap.memMat, memSec: snap.memSec, memLoads: snap.memLoads,
    nodeLoads: snap.nodeLoads, areaMat: snap.areaMat, areaSec: snap.areaSec, areaLoads: snap.areaLoads,
    memDiv: snap.memDiv, areaDiv: snap.areaDiv,
  });
  if (ana && ana.ok && ref.ok) {
    check("迁移前后 maxM 一致", Math.abs(ana.res.maxM - ref.res.maxM) < 1e-9, ana.res.maxM.toFixed(4) + " vs " + ref.res.maxM.toFixed(4));
    check("迁移前后 maxDisp 一致", Math.abs(ana.res.maxDisp - ref.res.maxDisp) < 1e-9, ana.res.maxDisp.toFixed(4) + " vs " + ref.res.maxDisp.toFixed(4));
  }
}
if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
