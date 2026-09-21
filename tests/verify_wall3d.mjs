/* 验证：3D 内置求解竖向共面墙（XZ/YZ）以膜刚度参与、与杆件协调 */
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
(0, eval)(readFileSync(path.join(ROOT, "solver.js"), "utf8"));
const { FrameSolver } = globalThis;
let nFail = 0;
function check(name, cond, detail) {
  if (cond) { console.log("  PASS " + name + (detail ? "  [" + detail + "]" : "")); }
  else { nFail++; console.log("  FAIL " + name + (detail ? "  [" + detail + "]" : "")); }
}
// 单层单跨：X 向 6m，Y 向 5m，Z 高 4m；Z=4 顶层板(XY) + X=0 竖向墙(XZ) + Y=0 竖向墙(YZ) + 真斜面1块
function baseState(withWalls) {
  const C30 = { label: "C30", E: 3.0e7, nu: 0.2 };
  const colS = { type: "rect", b: 0.4, h: 0.4 };
  const beamS = { type: "rect", b: 0.3, h: 0.6 };
  const FALL = { ux: 1, uy: 1, uz: 1, rx: 1, ry: 1, rz: 1 };
  const nodes = [
    { id: 1, x: 0, y: 0, z: 0, bc: FALL }, { id: 2, x: 6, y: 0, z: 0, bc: FALL },
    { id: 3, x: 6, y: 5, z: 0, bc: FALL }, { id: 4, x: 0, y: 5, z: 0, bc: FALL },
    { id: 5, x: 0, y: 0, z: 4, bc: {} }, { id: 6, x: 6, y: 0, z: 4, bc: {} },
    { id: 7, x: 6, y: 5, z: 4, bc: {} }, { id: 8, x: 0, y: 5, z: 4, bc: {} },
    { id: 9, x: 3, y: 2.5, z: 6, bc: {} },
  ];
  const members = [
    { id: 11, a: 1, b: 5, type: "col" }, { id: 12, a: 2, b: 6, type: "col" },
    { id: 13, a: 3, b: 7, type: "col" }, { id: 14, a: 4, b: 8, type: "col" },
    { id: 15, a: 5, b: 6, type: "beam" }, { id: 16, a: 6, b: 7, type: "beam" },
    { id: 17, a: 7, b: 8, type: "beam" }, { id: 18, a: 8, b: 5, type: "beam" },
  ];
  const areas = [
    { id: 21, etype: "shell", nodes: [5, 6, 7, 8] },           // 顶板 XY
    { id: 23, etype: "shell", nodes: [5, 6, 9, 8] },           // 真斜面（应跳过）
  ];
  if (withWalls) {
    areas.push({ id: 22, etype: "shell", nodes: [1, 2, 6, 5] }); // X=0? 不：y=0 竖向墙 XZ
    areas.push({ id: 24, etype: "shell", nodes: [1, 4, 8, 5] }); // x=0 竖向墙 YZ
  }
  return {
    NSUB: 4, AMESH: 3,
    D: { NSUB: 4, AMESH: 3 },
    model: { nodes, members, areas, cuts: [] },
    memStiff: {}, memMat: {}, memSec: {}, memLoads: { 15: [{ type: "udl", q: -10 }] },
    nodeLoads: { 6: { fx: 20 } },
    areaMat: {}, areaSec: {}, areaLoads: {},
    memDiv: {}, areaDiv: {},
  };
}
const noWall = FrameSolver.solve(baseState(false));
const withWall = FrameSolver.solve(baseState(true));
check("无墙 ok", noWall.ok === true);
check("有墙 ok", withWall.ok === true);
check("有墙走 3D", withWall.is3D === true);
check("仅真斜面被跳过", withWall.skippedArea3D === 1, "skipped=" + withWall.skippedArea3D);
check("无墙时顶板参与、仅斜面跳过", noWall.skippedArea3D === 1, "skipped=" + noWall.skippedArea3D);
// 参与单元的平面标记
const planes = {};
withWall.areaElems.forEach((ae) => { planes[ae.aid] = planes[ae.aid] || new Set(); planes[ae.aid].add(ae.plane + ":" + (ae.dofs || []).join(",")); });
console.log("  planes:", JSON.stringify(Object.fromEntries(Object.entries(planes).map(([k, v]) => [k, [...v]]))));
check("顶板 plane=xy", [...(planes[21] || [])].some((s) => s.startsWith("xy:")));
check("XZ 墙 plane=xz/dofs 0,2", [...(planes[22] || [])].some((s) => s === "xz:0,2"));
check("YZ 墙 plane=yz/dofs 1,2", [...(planes[24] || [])].some((s) => s === "yz:1,2"));
check("斜面无单元", !planes[23]);
// 墙体提供刚度：有墙与无墙位移应不同（墙约束侧移）
const d0 = Math.abs(noWall.res.maxDisp), d1 = Math.abs(withWall.res.maxDisp);
check("竖向墙提供侧向刚度", d1 < d0, "noWall=" + d0.toFixed(3) + " withWall=" + d1.toFixed(3));
// 共边协调：墙角分析节点位移 = 杆端位移（同一自由度），抽查顶板角 5（梁15端/柱11顶/墙22角）
function dispAt(ana, mid, dof) { const gi = ana.idx[mid]; return ana.u[6 * gi + dof]; }
const ux5 = dispAt(withWall, 5, 0), uz5 = dispAt(withWall, 5, 2);
check("角点位移有限", isFinite(ux5) && isFinite(uz5), "ux5=" + ux5.toExponential(2));
// 墙单元应力回填存在且有限
const wallSE = withWall.areaElems.filter((ae) => (ae.aid === 22 || ae.aid === 24) && ae.s);
check("竖向墙应力回填", wallSE.length > 0 && wallSE.every((ae) => isFinite(ae.s.vm)), wallSE.length + " elems");
// 墙边中点：梁中间分析节点与墙边网格节点应为同一分析节点（共享）
// 梁15 X:0->6（4 细分，x=1.5/3/4.5，应落在墙22底边 y=0,z=4 上）
{
  const u = withWall.u, all = withWall.all3;
  function findNode(x, y, z) {
    for (let i = 0; i < all.length; i++) {
      if (Math.abs(all[i][0] - x) < 1e-9 && Math.abs(all[i][1] - y) < 1e-9 && Math.abs(all[i][2] - z) < 1e-9) return i;
    }
    return -1;
  }
  const gi = findNode(3, 0, 4);
  check("梁/墙共边中点共享同一分析节点", gi >= 0, "gi=" + gi);
  // 该点应同时被梁单元与墙单元引用
  const inBeam = (withWall.byMember[15] || []).some((el) => el.a === gi || el.b === gi);
  const inWall = withWall.areaElems.some((ae) => ae.aid === 22 && ae.g.includes(gi));
  check("中点同时属梁15与墙22", inBeam && inWall);
}
if (nFail) { console.log("\nRESULT: " + nFail + " FAIL"); process.exit(1); }
else { console.log("\nRESULT: ALL PASS"); }
