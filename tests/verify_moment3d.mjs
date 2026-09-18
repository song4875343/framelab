/* 验证：3D红弯矩画在受拉侧（Mz>0 -> 局部-Y；平面借用Z保持立面同屏侧） */
function axes3(ax, ay, az, bx, by, bz) {
  let ex = bx - ax, ey = by - ay, ez = bz - az;
  const L = Math.hypot(ex, ey, ez) || 1; ex /= L; ey /= L; ez /= L;
  let yx, yy, yz;
  if (Math.abs(ez) <= 0.99) { const s = ez; yx = -s * ex; yy = -s * ey; yz = 1 - s * ez; }
  else { let ux = 0, uy = 1, uz = 0; if (Math.abs(ey) > 0.99) { ux = 1; uy = 0; } yx = uy * ez - uz * ey; yy = uz * ex - ux * ez; yz = ux * ey - uy * ex; }
  const yl = Math.hypot(yx, yy, yz) || 1; yx /= yl; yy /= yl; yz /= yl;
  return { y: [yx, yy, yz], z: [ey * yz - ez * yy, ez * yx - ex * yz, ex * yy - ey * yx] };
}
function proj(kind, x, y, z) {
  if (kind === "plan") return [x, y];
  if (kind === "secX") return [x, z];
  return [y, z];
}
// 与 index.html 新逻辑同构：返回 Mz>0 时红图偏移的【模型向量】
function redDir(kind, A, B) {
  const a = axes3(A[0], A[1], A[2], B[0], B[1], B[2]);
  const r0 = proj(kind, A[0], A[1], A[2]);
  const rY = proj(kind, A[0] + a.y[0], A[1] + a.y[1], A[2] + a.y[2]);
  const rZ = proj(kind, A[0] + a.z[0], A[1] + a.z[1], A[2] + a.z[2]);
  const hasY = Math.hypot(rY[0] - r0[0], rY[1] - r0[1]) > 1e-6;
  const hasZ = Math.hypot(rZ[0] - r0[0], rZ[1] - r0[1]) > 1e-6;
  if (hasY) return { dir: [-a.y[0], -a.y[1], -a.y[2]], how: "受拉-Y" }; // 受拉侧
  if (hasZ) return { dir: a.z, how: "借用+Z量级" };
  return { dir: null, how: "不画" };
}
let fail = 0;
function check(name, kind, A, B, tensionSide, wantBorrow) {
  const r = redDir(kind, A, B);
  // Mz>0 受拉侧 = 局部-Y；期望红图方向与受拉侧点积>0（借用情况除外）
  const a = axes3(A[0], A[1], A[2], B[0], B[1], B[2]);
  const ty = [-a.y[0], -a.y[1], -a.y[2]];
  const dot = r.dir ? r.dir[0] * ty[0] + r.dir[1] * ty[1] + r.dir[2] * ty[2] : 0;
  const ok = wantBorrow ? r.how === "借用+Z量级" : (r.how === "受拉-Y" && dot > 0.99);
  if (!ok) fail++;
  console.log((ok ? "  PASS " : "  FAIL ") + name + " -> " + r.how + (r.dir ? " [" + r.dir.map(v => v.toFixed(1)).join(",") + "]" : ""));
}
// 剖面/透视：必须受拉侧；平面梁：借用（与立面同屏侧：下方）
check("secX-X梁", "secX", [0, 0, 8], [6, 0, 8]);           // 受拉-Y=(0,0,-1)梁下 ✓
check("secX-柱", "secX", [0, 0, 4], [0, 0, 8]);            // 受拉-Y=(-1,0,0)
check("secY-Y梁", "secY", [0, 0, 8], [0, 6, 8]);           // 受拉-Y=(0,0,-1)梁下 ✓
check("secY-柱", "secY", [0, 0, 4], [0, 0, 8], null, true); // Y(X向)垂直剖面->借Z量级
check("plan-X梁", "plan", [0, 0, 8], [6, 0, 8], null, true); // 借用
check("plan-柱", "plan", [0, 0, 4], [0, 0, 8]);            // 受拉-Y水平面内 ✓
console.log(fail === 0 ? "RESULT: ALL PASS" : "RESULT: FAIL");
process.exit(fail ? 1 : 0);
