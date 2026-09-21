/* FrameLab OpenSees 文件读取验证（Node 运行）。
 *
 * 运行（仓库根目录）：
 *   node tests/verify_osimport.mjs
 *
 * 内容：
 *   example/openseespy_portal_frame.py -> FrameOpenSees.parse -> 快照检查
 *   example/tcl_simple_beam.tcl        -> FrameOpenSees.parse -> 快照检查
 *   两个快照分别用 FrameSolver.solve 求解；简支梁另与解析解核对
 *   （跨中弯矩 qL^2/8 + PL/4 = 58.5 kN*m，容差 2%）。
 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");

function loadLib(rel) {
  const code = readFileSync(path.join(ROOT, rel), "utf8");
  (0, eval)(code); // eslint-disable-line no-eval
}
loadLib("solver.js");
loadLib("opensees_import.js");
const { FrameSolver, FrameOpenSees } = globalThis;
if (!FrameSolver || !FrameOpenSees) {
  console.error("FAIL: solver.js / opensees_import.js 未能加载");
  process.exit(2);
}

let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}
function bcOf(snap, x, z) {
  const n = snap.model.nodes.find((nn) => (Math.abs(nn.x - x) < 1e-9 && Math.abs(((nn.z!=null)?nn.z:nn.y) - z) < 1e-9));
  return n ? n.bc : null;
}

// ---------- Case 1：OpenSeesPy 门式刚架 ----------
console.log("## Case 1 openseespy_portal_frame.py");
const pyText = readFileSync(path.join(ROOT, "example", "openseespy_portal_frame.py"), "utf8");
let py;
try {
  py = FrameOpenSees.parse(pyText, "openseespy_portal_frame.py");
  check("解析成功", true, "format=" + py.format);
} catch (e) { check("解析成功", false, String(e.message || e)); }
if (py) {
  check("格式判定为 py", py.format === "py", py.format);
  const s = py.snapshot;
  check("节点数 4", s.model.nodes.length === 4, s.model.nodes.length);
  check("杆件数 3", s.model.members.length === 3, s.model.members.length);
  const b1 = bcOf(s, 0, 0), b3 = bcOf(s, 6, 0);
  check("柱底固定", !!(b1 && b1.ux && (b1.uz||b1.uy) && (b1.ry||b1.rz) && b3 && b3.ux && (b3.uz||b3.uy) && (b3.ry||b3.rz)));
  const udls = Object.values(s.memLoads).flat().filter((l) => l.type === "udl");
  check("梁均布 -12", udls.length === 1 && Math.abs(udls[0].q + 12) < 1e-9, JSON.stringify(udls));
  check("节点荷载 2 处", Object.keys(s.nodeLoads).length === 2);
  const eVals = Object.values(s.memMat).map((m) => m.E);
  check("材料 E=3.0e7", eVals.length === 3 && eVals.every((e) => e === 3.0e7));
  let ana = null;
  try { ana = FrameSolver.solve(s); check("内置求解 ok", ana && ana.ok === true); }
  catch (e) { check("内置求解 ok", false, String(e.message || e).slice(0, 200)); }
  if (ana && ana.ok) {
    check("最大弯矩>0", ana.res.maxM > 1, "maxM=" + ana.res.maxM.toFixed(2));
    check("顶点侧移>0", Math.abs(ana.res.roof) > 1e-6, "roof=" + ana.res.roof.toFixed(3) + "mm");
  }
  if (py.warnings.length) { console.log("  WARN py: " + py.warnings.join(" | ").slice(0, 300)); }
}

// ---------- Case 2：Tcl 简支梁 ----------
console.log("## Case 2 tcl_simple_beam.tcl");
const tclText = readFileSync(path.join(ROOT, "example", "tcl_simple_beam.tcl"), "utf8");
let tc;
try {
  tc = FrameOpenSees.parse(tclText, "tcl_simple_beam.tcl");
  check("解析成功", true, "format=" + tc.format);
} catch (e) { check("解析成功", false, String(e.message || e)); }
if (tc) {
  check("格式判定为 tcl", tc.format === "tcl", tc.format);
  const s = tc.snapshot;
  check("节点数 3", s.model.nodes.length === 3, s.model.nodes.length);
  check("杆件数 2", s.model.members.length === 2, s.model.members.length);
  const bl = bcOf(s, 0, 0), br = bcOf(s, 6, 0);
  check("左铰支", !!(bl && bl.ux && (bl.uz||bl.uy) && !(bl.ry||bl.rz)), JSON.stringify(bl));
  check("右辊轴", !!(br && !br.ux && (br.uz||br.uy) && !(br.ry||br.rz)), JSON.stringify(br));
  const mid = s.model.nodes.find((n) => Math.abs(n.x - 3) < 1e-9);
  check("[expr]/$变量求值", !!mid, "x=3 节点" + (mid ? "存在" : "缺失"));
  const udls = Object.values(s.memLoads).flat().filter((l) => l.type === "udl");
  const pts = Object.values(s.memLoads).flat().filter((l) => l.type === "point");
  check("均布 2 段", udls.length === 2 && udls.every((l) => Math.abs(l.q + 8) < 1e-9));
  check("跨中集中力", pts.length === 1 && Math.abs(pts[0].P + 15) < 1e-9 && Math.abs(pts[0].a - 1.5) < 1e-9,
    JSON.stringify(pts));
  let ana = null;
  try { ana = FrameSolver.solve(s); check("内置求解 ok", ana && ana.ok === true); }
  catch (e) { check("内置求解 ok", false, String(e.message || e).slice(0, 200)); }
  if (ana && ana.ok) {
    // 手算：R左=37.75，x=3 处 M=37.75*3-8*3*1.5=77.25（-beamPoint 在第 2 段中点，
    // 即整体 x=4.5；节点集中力 -20 作用于 x=3）
    let mMax = 0;
    Object.values(ana.byMember || {}).flat().forEach((el) => {
      (el.forces || []).forEach((p) => { mMax = Math.max(mMax, Math.abs(p.M)); });
    });
    const expect = 77.25;
    check("最大弯矩解析值", Math.abs(mMax - expect) / expect < 0.02,
      "got=" + mMax.toFixed(2) + " expect=" + expect);
  }
  if (tc.warnings.length) { console.log("  WARN tcl: " + tc.warnings.join(" | ").slice(0, 300)); }
}

// ---------- Case 3：xara 导出脚本 roundtrip（导出->读取） ----------
console.log("## Case 3 xara 导出 roundtrip");
loadLib("xara.js");
if (!globalThis.FrameXara) { check("xara.js 加载", false); }
else {
  const st = {
    D: { NSUB: 5, AMESH: 2 },
    model: {
      nodes: [
        { id: 1, x: 0, z: 0, bc: { ux: 1, uz: 1, ry: 1 } },
        { id: 2, x: 0, z: 4, bc: {} },
        { id: 3, x: 6, z: 0, bc: { ux: 1, uz: 1, ry: 1 } },
        { id: 4, x: 6, z: 4, bc: {} },
      ],
      members: [
        { id: 11, a: 1, b: 2, type: "col" },
        { id: 12, a: 3, b: 4, type: "col" },
        { id: 13, a: 2, b: 4, type: "beam" },
      ],
      areas: [], cuts: [],
    },
    memStiff: {}, memMat: {}, memSec: {}, memLoads: { 13: [{ type: "udl", q: -12 }] },
    nodeLoads: { 2: { fx: 10, fz: 0, my: 0 } },
    areaMat: {}, areaSec: {}, areaLoads: {}, memDiv: {}, areaDiv: {},
  };
  const built = globalThis.FrameXara.buildScript(st, {});
  let rt = null;
  try { rt = FrameOpenSees.parse(built.script, "framelab_xara.py"); check("xara 脚本读回", true); }
  catch (e) { check("xara 脚本读回", false, String(e.message || e).slice(0, 200)); }
  if (rt) {
    // 该 state 无 memDiv、D.NSUB=5：3 根杆各 5 段 = 15 段，节点 4 + 柱 8 + 梁 4 = 16
    check("读回杆段/节点（含细分）", rt.snapshot.model.nodes.length === 16 && rt.snapshot.model.members.length === 15,
      rt.snapshot.model.nodes.length + "n/" + rt.snapshot.model.members.length + "m");
    const ana = FrameSolver.solve(rt.snapshot);
    const ref = FrameSolver.solve(st);
    // 导入侧 GA 取 1e12（Euler 梁，与 OpenSees 一致），原模型走 DEF_STIFF
    // GA=1e9，剪切变形理论差异约 1e-5 量级（同 verify_bridge Case B 逻辑）
    check("读回后求解一致", ana.ok && Math.abs(ana.res.maxM - ref.res.maxM) / ref.res.maxM < 1e-3,
      "maxM=" + ana.res.maxM.toFixed(3) + " vs " + ref.res.maxM.toFixed(3));
  }
}

if (nFail > 0) { console.log("\nRESULT: FAIL (" + nFail + ")"); process.exit(1); }
console.log("\nRESULT: ALL PASS");
