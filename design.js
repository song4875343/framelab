/* FrameLab 设计分析模块（独立文件）
 * 依据中国规范进行杆件承载力 / 配筋 / 挠度 / 裂缝 / 位移角验算，
 * 显示习惯对标 PKPM / 盈建科（YJK）：配筋简图、实配钢筋、挠度裂缝、层间位移、顶点位移。
 *
 * 梁（YJK 习惯，三段数字，面积单位 cm²）：
 *   上部 Asu1-Asu2-Asu3 ＝ 左端-跨中-右端上部纵筋；下部 Asd1-Asd2-Asd3 同理；
 *   箍筋 Genc-Gnon ＝ 加密区-非加密区（cm²/m），如 G1.3-1.2。
 * 柱：纵筋对称配筋，同样按 下端-中部-上端 三段显示（上下即 a→b 向）。
 * 钢构件：显示三段应力比 η1-η2-η3（左-中-右）。
 *
 * 规范选项：
 *   混凝土：GB50010-2010 / GB50010-2020（配套 GB55002，构造要求略严）
 *   钢结构：GB50017-2003 / GB50017-2017
 * 本模块为简化版二维框架后处理：内力取弹性分析左/中/右三截面同期值，
 * 按单筋矩形截面 / 对称配筋柱 / 钢梁压弯应力比验算。详细公式见 mdCalcBook()。
 * 单位约定（与主程序一致）：长度 m、力 kN、弯矩 kN·m、应力 kPa。
 * 配筋面积内部为 mm²，YJK 显示换算为 cm²（/100）。
 */
(function (global) {
  "use strict";

  /* ---------------- 规范参数表 ---------------- */

  // 混凝土：fck/ftk 标准值, fc/ft 设计值 (MPa), Ec (MPa)
  var CONC = {
    C25: { fck: 16.7, fc: 11.9, ftk: 1.78, ft: 1.27, Ec: 28000 },
    C30: { fck: 20.1, fc: 14.3, ftk: 2.01, ft: 1.43, Ec: 30000 },
    C35: { fck: 23.4, fc: 16.7, ftk: 2.20, ft: 1.57, Ec: 31500 },
    C40: { fck: 26.8, fc: 19.1, ftk: 2.39, ft: 1.71, Ec: 32500 },
    C45: { fck: 29.6, fc: 21.1, ftk: 2.51, ft: 1.80, Ec: 33500 },
    C50: { fck: 32.4, fc: 23.1, ftk: 2.64, ft: 1.89, Ec: 34500 },
    C55: { fck: 35.5, fc: 25.3, ftk: 2.74, ft: 1.96, Ec: 35500 },
    C60: { fck: 38.5, fc: 27.5, ftk: 2.85, ft: 2.04, Ec: 36000 }
  };

  // 钢筋：fyk 标准值 / fy 设计值 (MPa)。GB50010-2010 表4.2.3
  var REBAR = {
    HPB300: { fyk: 300, fy: 270, Es: 210000, tag: "A" },
    HRB400: { fyk: 400, fy: 360, Es: 200000, tag: "C" },
    HRB500: { fyk: 500, fy: 435, Es: 200000, tag: "B" }
  };

  // 钢材强度设计值 f (MPa)，厚度<16mm 组别
  var STEEL_F = {
    "GB50017-2003": { Q235: 215, Q345: 315, Q390: 350 },
    "GB50017-2017": { Q235: 215, Q345: 310, Q390: 350 }
  };

  function isConcrete(matLabel) {
    return !!matLabel && /^C\d/i.test(String(matLabel).trim());
  }
  function isSteel(matLabel) {
    return !!matLabel && /^Q\d/i.test(String(matLabel).trim());
  }
  function concOf(label) {
    var k = String(label || "C30").toUpperCase();
    return CONC[k] || CONC.C30;
  }
  function rebarOf(grade) {
    return REBAR[grade] || REBAR.HRB400;
  }
  function steelF(matLabel, codeSteel) {
    var tab = STEEL_F[codeSteel] || STEEL_F["GB50017-2017"];
    var k = String(matLabel || "Q235").toUpperCase();
    if (k === "Q345" || k === "Q345B") { return tab.Q345; }
    if (k.indexOf("390") >= 0) { return tab.Q390; }
    return tab.Q235;
  }

  /* ---------------- 截面几何 ---------------- */

  function rectWH(sec) {
    if (!sec) { return null; }
    if (sec.type === "rect") { return { b: sec.b, h: sec.h }; }
    if (sec.type === "circle") { return { b: sec.d, h: sec.d, circle: true }; }
    if (sec.type === "I") {
      var h = sec.h, b = sec.b, tw = sec.tw, tf = sec.tf;
      var I = (b * Math.pow(h, 3) - (b - tw) * Math.pow(h - 2 * tf, 3)) / 12;
      var A = 2 * b * tf + (h - 2 * tf) * tw;
      return { b: b, h: h, Ishape: true, A: A, I: I, W: I / (h / 2) };
    }
    return null;
  }
  function sectionA(sec) {
    if (!sec) { return 0; }
    if (sec.type === "rect") { return sec.b * sec.h; }
    if (sec.type === "circle") { return Math.PI * sec.d * sec.d / 4; }
    if (sec.type === "I") { return 2 * sec.b * sec.tf + (sec.h - 2 * sec.tf) * sec.tw; }
    return 0;
  }
  function steelW(sec) {
    var wh = rectWH(sec) || { b: 0.2, h: 0.4 };
    if (wh.W) { return wh.W; }
    if (sec.type === "circle") { return Math.PI * Math.pow(sec.d, 3) / 32; }
    return (wh.b || 0.2) * Math.pow(wh.h || 0.4, 2) / 6;
  }

  // 实配选筋：从常用直径中选最小满足面积的组合（2~6根）
  var DIA = [12, 14, 16, 18, 20, 22, 25, 28];
  function pickBars(asNeedMm2, tag) {
    var best = null, d, n;
    for (var di = 0; di < DIA.length; di++) {
      d = DIA[di];
      var a1 = Math.PI * d * d / 4;
      for (n = 2; n <= 6; n++) {
        var a = a1 * n;
        if (a + 1e-9 >= asNeedMm2) {
          if (!best || a < best.area) { best = { n: n, d: d, area: a }; }
          break;
        }
      }
    }
    if (!best) { d = 28; best = { n: 6, d: 28, area: Math.PI * 28 * 28 / 4 * 6 }; }
    return { text: best.n + tag + best.d, area: best.area, n: best.n, d: best.d };
  }
  // 箍筋选配：双肢箍，间距 100/150/200，返回首个满足加密区需要的组合
  function pickStirrup(asvNeed, tag) {
    var need = Math.max(asvNeed, 0.15); // mm2/mm 下限构造
    var dias = [8, 10, 12], gaps = [100, 150, 200], best = null;
    dias.forEach(function (d) {
      var a1 = Math.PI * d * d / 4 * 2; // 双肢
      gaps.forEach(function (g) {
        var v = a1 / g;
        if (v + 1e-9 >= need && !best) { best = { d: d, g: g, v: v }; }
      });
    });
    if (!best) { best = { d: 12, g: 100, v: Math.PI * 12 * 12 / 4 * 2 / 100 }; }
    return { text: "A" + best.d + "@" + best.g, d: best.d, g: best.g, v: best.v, tag: tag };
  }

  /* YJK 显示换算：mm² → cm²（1 位小数），箍筋 mm²/mm → cm²/m（×10） */
  // 柱平法集中标注选筋：4 角筋 + 四面中间筋（4 或 8 根），角筋直径 ≥ 边筋
  function pickColBars(totalMm2, tag) {
    var di, si, ni, best = null;
    var sideCounts = [4, 8];
    for (di = DIA.length - 1; di >= 0; di--) {
      var dc = DIA[di], ac = Math.PI * dc * dc / 4 * 4;
      if (ac > totalMm2 && dc > 12) { continue; } // C12 角筋保底，允许超配
      var need = Math.max(0, totalMm2 - ac);
      for (ni = 0; ni < sideCounts.length; ni++) {
        var n = sideCounts[ni];
        for (si = 0; si < DIA.length; si++) {
          var ds = DIA[si];
          if (ds > dc) { break; }
          var as = Math.PI * ds * ds / 4 * n;
          if (as + 1e-9 >= need) {
            var tot = ac + as;
            if (!best || tot < best.total) { best = { dc: dc, ds: ds, n: n, cornerA: ac, sideA: as, total: tot }; }
            break;
          }
        }
      }
    }
    if (!best) {
      var a12 = Math.PI * 12 * 12 / 4;
      best = { dc: 12, ds: 12, n: 4, cornerA: a12 * 4, sideA: a12 * 4, total: a12 * 8 };
    }
    return {
      corner: { n: 4, d: best.dc, area: best.cornerA, text: "4" + tag + best.dc },
      side: { n: best.n, d: best.ds, area: best.sideA, text: best.n + tag + best.ds }
    };
  }
  function cm2(mm2) { return mm2 / 100; }
  function f1(v) { return (Math.round(v * 10) / 10).toFixed(1); }
  function yjk3(arr) { return arr.map(function (v) { return f1(cm2(v)); }).join("-"); }
  function yjkG(enc, non) { return "G" + f1(enc * 10) + "-" + f1(non * 10); }
  function yjkEta(arr) { return arr.map(function (v) { return v.toFixed(2); }).join("-"); }

  // 单截面受弯配筋：返回 {x, over, asBend}
  function bendAs(Mabs_Nmm, fc, b, h0, fy) {
    var disc = h0 * h0 - 2 * Mabs_Nmm / (1.0 * fc * b);
    var x = disc <= 0 ? h0 * 0.55 : h0 - Math.sqrt(Math.max(0, disc));
    var xb = 0.55 * h0;
    var over = x > xb;
    if (over) { x = xb; }
    return { x: x, over: over, asBend: 1.0 * fc * b * x / fy };
  }

  /* ---------------- 单根杆件设计 ----------------
   * f: {N(kN, 拉+), V(kN), M(kN·m)} 控制内力（取 |M|max 截面处的同期 N、V）
   * secs: 可选三截面 [{N,V,M}×3]（左-中-右），用于 YJK 式三段显示；缺省用 f 复制
   * sec: {type:'rect', b,h}（m）；matLabel 'C30'；rebar {grade, fyk, fd}
   * L: 杆长 m；kind: 'beam'|'col'|'user'
   */
  function designMember(mid, kind, L, sec, matLabel, rebar, f, codeConc, codeSteel, secs) {
    var N = f.N || 0, V = Math.abs(f.V != null ? f.V : (f.Vy || 0)), M = Math.abs(f.M || 0);
    // 3D 双向弯曲：取 max(|M|,|My|,|Mz|) 包络验算（简化后处理，正式设计以全模型复核为准）
    function effM(s) { return Math.max(Math.abs(s.M || 0), Math.abs(s.My || 0), Math.abs(s.Mz || 0)); }
    var bi3d = false;
    if (secs && secs.length === 3) {
      secs.forEach(function (s) {
        if (Math.abs(s.My || 0) > 1e-9 && Math.abs((s.M || 0)) > 1e-9 && Math.abs(Math.abs(s.My) - Math.abs(s.M)) > 1e-6) { bi3d = true; }
        if (Math.abs(s.Mz || 0) > 1e-9 && Math.abs(Math.abs(s.Mz) - Math.abs(s.M)) > 1e-6) { bi3d = true; }
      });
    }
    if (bi3d) { M = Math.max(M, effM(f)); }
    var isC = isConcrete(matLabel), isS = isSteel(matLabel);
    var grade = (rebar && rebar.grade) || "HRB400";
    var rb = rebarOf(grade);
    var fyk = (rebar && rebar.fyk > 0) ? rebar.fyk : rb.fyk;
    var fy = (rebar && rebar.fd > 0) ? rebar.fd : rb.fy;
    var tag = rb.tag;
    var S3 = (secs && secs.length === 3) ? secs.map(function (s) { return { N: s.N || 0, V: s.V != null ? s.V : (s.Vy || 0), M: effM(s), My: s.My || 0, Mz: s.Mz != null ? s.Mz : (s.M || 0) }; }) : [{ N: f.N || 0, V: f.V || 0, M: f.M || 0 }, { N: f.N || 0, V: f.V || 0, M: f.M || 0 }, { N: f.N || 0, V: f.V || 0, M: f.M || 0 }];

    if (isS || (!isC && sec && sec.type === "I")) {
      // ---- 钢构件：三截面压弯应力比（3D 取双向弯矩包络） ----
      var A = sectionA(sec), W = steelW(sec);
      var fdes = steelF(matLabel, codeSteel);
      var ratios3 = S3.map(function (s) {
        var sg = Math.abs(s.N || 0) * 1000 / Math.max(A, 1e-9) / 1e6 + effM(s) * 1000 / Math.max(W, 1e-9) / 1e6;
        return { sigma: sg, ratio: sg / fdes };
      });
      var worst = ratios3.reduce(function (a, b) { return b.ratio > a.ratio ? b : a; }, ratios3[0]);
      var ok = worst.ratio <= 1;
      return {
        mid: mid, type: "steel", mat: matLabel, kind: kind, L: L,
        ctrl: { N: N, V: V, M: M },
        secs: S3.map(function (s, i) { return { x: ["左", "中", "右"][i], N: s.N || 0, V: s.V || 0, M: s.M || 0, sigma: ratios3[i].sigma, ratio: ratios3[i].ratio }; }),
        ratios3: ratios3.map(function (r) { return r.ratio; }),
        yjkEta: yjkEta(ratios3.map(function (r) { return r.ratio; })),
        stress: worst.sigma, allow: fdes, ratio: worst.ratio,
        asCalc: 0, asMin: 0, asTheory: 0, asReal: null,
        defl: 0, deflAllow: L / 250 * 1000, crack: 0, crackAllow: 0.3,
        ok: ok, margin: 1 - worst.ratio,
        note: "钢构件三截面压弯应力比 η=" + yjkEta(ratios3.map(function (r) { return r.ratio; })),
        codeSteel: codeSteel
      };
    }

    // ---- 混凝土构件 ----
    var cc = concOf(matLabel);
    var fc = cc.fc, ft = cc.ft; // MPa
    var wh2 = rectWH(sec) || { b: 0.3, h: 0.5 };
    var b = (wh2.b || 0.3) * 1000;      // mm
    var h = (wh2.h || 0.5) * 1000;      // mm
    if (wh2.circle) { b = (sec.d || 0.4) * 1000 * 0.8; h = (sec.d || 0.4) * 1000; }
    var h0 = Math.max(50, h - 40);
    var strict = (codeConc === "GB50010-2020");
    var rhoMin = strict ? Math.max(0.0025, 0.50 * ft / fy) : Math.max(0.0020, 0.45 * ft / fy);
    var asMinTotal = rhoMin * b * h;
    if (kind === "col") { asMinTotal = Math.max(asMinTotal, 0.006 * b * h * (strict ? 1.1 : 1.0)); }
    var asMinSide = asMinTotal / 2;

    // 逐截面计算（梁：上部抵抗负弯矩，下部抵抗正弯矩；柱：对称配筋）
    var secRes = S3.map(function (s) {
      var Mm = s.M || 0, Bs = bendAs(Math.abs(Mm) * 1e6, fc, b, h0, fy);
      var N_N = Math.abs(s.N || 0) * 1000, M_Nmm = Math.abs(Mm) * 1e6;
      var asAxial = 0;
      if (kind === "col" || Math.abs(s.N || 0) > 1e-9) {
        var e0 = M_Nmm / Math.max(N_N, 1e-9);
        var kAx = kind === "col" ? 0.55 : 0.25;
        asAxial = N_N / (0.9 * fy) * kAx * Math.min(1.5, 1 + h / Math.max(e0, h * 0.3));
      }
      var top, bot;
      if (kind === "col") {
        var tot = Math.max(Bs.asBend, asAxial, asMinTotal);
        if (Bs.over) { tot = Math.max(tot, Bs.asBend * 1.35); }
        top = tot / 2; bot = tot / 2;
      } else {
        var bend = Math.max(Bs.asBend, asAxial * 0.5);
        if (Bs.over) { bend = Math.max(bend, Bs.asBend * 1.35); }
        if (Mm >= 0) { bot = Math.max(bend, asMinSide); top = asMinSide; } // 下缘受拉为正
        else { top = Math.max(bend, asMinSide); bot = asMinSide; }
      }
      var Vc = 0.7 * ft * b * h0;
      var asv = Math.max(0, (Math.abs(s.V || 0) * 1000 - Vc) / (fy * h0));
      return { N: s.N || 0, V: s.V || 0, M: Mm, top: top, bot: bot, asv_s: asv, x: Bs.x, over: Bs.over };
    });
    var top3 = secRes.map(function (s) { return s.top; });
    var bot3 = secRes.map(function (s) { return s.bot; });
    var asv3 = secRes.map(function (s) { return s.asv_s; });
    var overAny = secRes.some(function (s) { return s.over; });
    var topMax = Math.max.apply(null, top3.concat([0])), botMax = Math.max.apply(null, bot3.concat([0]));
    var asTheory = topMax + botMax;
    var asCalc = Math.max.apply(null, secRes.map(function (s) { return s.top + s.bot; }).concat([0]));
    var realTop = pickBars(topMax, tag), realBot = pickBars(botMax, tag);
    var real = pickBars(asTheory, tag);
    // 梁：三截面上/下部分段实配；柱：平法集中标注（角筋 + 四面中间筋）
    var topBars3 = top3.map(function (a) { return pickBars(a, tag); });
    var botBars3 = bot3.map(function (a) { return pickBars(a, tag); });
    var colBars = (kind === "col") ? pickColBars(asTheory, tag) : null;
    var stir = pickStirrup(Math.max.apply(null, asv3.concat([0])), tag);
    var enc = Math.max(asv3[0] || 0, asv3[2] || 0), non = asv3[1] || 0;
    // 挠度裂缝仍按控制内力（|M|max）估算
    var M_Nmm = M * 1e6;
    var Ec = cc.Ec;
    var I = b * Math.pow(h, 3) / 12;
    var B = 0.6 * Ec * I;
    var defl = 0.125 * M_Nmm * Math.pow(L * 1000, 2) / Math.max(B, 1e-9);
    var deflAllow = L * 1000 / 250;
    var sigmaS = M_Nmm / Math.max(0.87 * h0 * real.area, 1e-9);
    var rhoTe = Math.min(0.1, Math.max(0.01, real.area / (b * h * 0.5)));
    var wmax = 2.1 * Math.min(sigmaS, fy) / rb.Es * (19 + 0.08 * real.d / Math.max(rhoTe, 1e-9)) * 0.1;
    if (!(M > 1e-9)) { wmax = 0; }
    var crackAllow = 0.3;
    var V_N = V * 1000;
    var okShear = V_N <= 0.25 * cc.fc * b * h0;
    var okBend = !overAny;
    var okDefl = defl <= deflAllow, okCrack = wmax <= crackAllow;
    var ok = okBend && okShear && okDefl && okCrack;
    var margin = Math.min((realTop.area + realBot.area) / Math.max(asTheory, 1e-9) - 1, okDefl ? (deflAllow / Math.max(defl, 1e-9) - 1) : -1);
    var A_m2 = sectionA(sec) || (b * h / 1e6);
    return {
      mid: mid, type: "conc", mat: matLabel, kind: kind, L: L,
      ctrl: { N: N, V: V, M: M },
      secs: secRes.map(function (s, i) { return { x: ["左/下", "中", "右/上"][i], N: s.N, V: s.V, M: s.M, top: s.top, bot: s.bot, asv_s: s.asv_s }; }),
      top3: top3, bot3: bot3, asv3: asv3,
      yjkTop: yjk3(top3), yjkBot: yjk3(bot3), yjkG: yjkG(enc, non),
      stirrup: stir, stirEnc: enc, stirNon: non,
      fc: fc, ft: ft, fy: fy, fyk: fyk, grade: grade,
      h0: h0, x: secRes[1] ? secRes[1].x : 0, over: overAny,
      asCalc: asCalc, asMin: asMinTotal, asTheory: asTheory,
      asReal: real, asRealTop: realTop, asRealBot: realBot, asv_s: Math.max.apply(null, asv3.concat([0])),
      topBars3: topBars3, botBars3: botBars3, colBars: colBars,
      defl: defl, deflAllow: deflAllow, crack: wmax, crackAllow: crackAllow,
      shearOk: okShear, bendOk: okBend,
      ok: ok, margin: margin,
      concVol: A_m2 * L,
      codeConc: codeConc
    };
  }

  /* 从分析结果提取每根杆件控制内力：取 max(|M|,|My|,|Mz|) 最大截面处的同期 N、V（3D 兼容） */
  function ctrlForces(ana) {
    var out = {};
    if (!ana || !ana.byMember) { return out; }
    Object.keys(ana.byMember).forEach(function (key) {
      var els = ana.byMember[key], best = { N: 0, V: 0, M: 0 }, bm = -1;
      els.forEach(function (el) {
        (el.forces || []).forEach(function (p) {
          var mAbs = Math.max(Math.abs(p.M || 0), Math.abs(p.My || 0), Math.abs(p.Mz || 0));
          if (mAbs > bm) { bm = mAbs; best = { N: el.axial || 0, V: p.S != null ? p.S : (p.Vy || 0), M: (p.Mz != null ? p.Mz : p.M) || 0 }; }
        });
      });
      out[key] = best;
    });
    return out;
  }

  function storyDrift(ana, model, H) {
    if (!ana || !model) { return { stories: [], maxRatio: 0 }; }
    var st = (ana.all && ana.all.length && ana.u && ana.u.length >= ana.all.length * 6 - 1e-9) ? 6 : 3;
    var hasZ = (model.nodes || []).some(function (n) { return Math.abs(n.z || 0) > 1e-9; });
    var ys = {};
    model.nodes.forEach(function (n) {
      var lvl = hasZ ? (n.z || 0) : n.y;
      var k = Math.round(lvl * 1e6) / 1e6;
      (ys[k] = ys[k] || []).push(n.id);
    });
    var levels = Object.keys(ys).map(Number).sort(function (a, b) { return a - b; });
    var stories = [], maxR = 0;
    for (var i = 1; i < levels.length; i++) {
      var up = avgUx(ys[levels[i]]), lo = avgUx(ys[levels[i - 1]]);
      var dh = levels[i] - levels[i - 1] || H || 4;
      var drift = Math.abs(up - lo);
      var ratio = drift / dh;
      if (ratio > maxR) { maxR = ratio; }
      stories.push({ y0: levels[i - 1], y1: levels[i], h: dh, drift: drift * 1000, ratio: ratio });
    }
    function avgUx(ids) {
      var s = 0, n = 0;
      ids.forEach(function (id) {
        var gi = ana.idx[id];
        if (gi !== undefined) { s += ana.u[st * gi] || 0; n++; }
      });
      return n ? s / n : 0;
    }
    return { stories: stories, maxRatio: maxR };
  }

  function runDesign(args) {
    // args: {model, memMat, memSec, memRebar, ana, codeConc, codeSteel, secForces?}
    var model = args.model, ana = args.ana;
    var codeConc = args.codeConc || "GB50010-2010";
    var codeSteel = args.codeSteel || "GB50017-2017";
    var ctrls = ctrlForces(ana);
    var members = {}, unsafe = 0;
    var concVol = 0, rebarWt = 0, steelWt = 0;
    (model.members || []).forEach(function (m) {
      var sec = (args.memSec || {})[m.id] || { type: "rect", b: 0.3, h: 0.5 };
      var mat = ((args.memMat || {})[m.id] || {}).label || "C30";
      var rb = (args.memRebar || {})[m.id] || { grade: "HRB400" };
      var f = ctrls[m.id] || { N: 0, V: 0, M: 0 };
      var sf = args.secForces ? args.secForces[m.id] : null;
      var r = designMember(m.id, m.type || "user", m.L || 4, sec, mat, rb, f, codeConc, codeSteel, sf);
      members[m.id] = r;
      if (!r.ok) { unsafe++; }
      if (r.type === "conc") {
        concVol += r.concVol || 0;
        var longA = (r.asRealTop && r.asRealBot) ? (r.asRealTop.area + r.asRealBot.area) : ((r.asReal || {}).area || 0);
        rebarWt += longA * 1e-6 * r.L * 7850;
        rebarWt += (r.asv_s || 0) * r.L * 1000 * 1e-6 * 7850 * 0.5;
      } else {
        steelWt += sectionA(sec) * r.L * 7850;
      }
    });
    var sd = storyDrift(ana, model);
    var topDisp = ana && ana.res ? ana.res.roof : 0;
    var H = 0;
    (model.nodes || []).forEach(function (n) { var vv = (n && isFinite(n.z)) ? n.z : (n.y || 0); H = Math.max(H, vv); });
    var topRatio = H > 0 ? Math.abs(topDisp) / 1000 / H : 0;
    return {
      members: members, unsafe: unsafe, total: (model.members || []).length,
      stories: sd.stories, maxDrift: sd.maxRatio,
      topDisp: topDisp, topRatio: topRatio, height: H,
      concVol: concVol, rebarWt: rebarWt, steelWt: steelWt,
      codeConc: codeConc, codeSteel: codeSteel
    };
  }

  /* ---------------- 详细计算书（Markdown） ---------------- */

  function mdCalcBook(m, sec, matLabel, rebar, r) {
    var L = [];
    function secTxt() {
      if (!sec) { return "—"; }
      if (sec.type === "rect") { return "矩形 " + sec.b + "×" + sec.h + " m"; }
      if (sec.type === "circle") { return "圆形 φ" + sec.d + " m"; }
      return "工字形 h=" + sec.h + " b=" + sec.b + " tw=" + sec.tw + " tf=" + sec.tf + " m";
    }
    L.push("## 构件 M" + m.id + " 详细计算书");
    L.push("");
    L.push("- 规范：混凝土 **" + (r.codeConc || "GB50010-2010") + "**，钢结构 **" + (r.codeSteel || "GB50017-2017") + "**");
    L.push("- 截面：" + secTxt() + "；材料：" + matLabel + "；杆长 L=" + Number(r.L).toFixed(3) + " m");
    if (r.type === "steel") {
      L.push("- 三截面应力比 η（左-中-右）：**" + r.yjkEta + "**");
      (r.secs || []).forEach(function (s) {
        L.push("  - " + s.x + "：N=" + s.N.toFixed(2) + "kN，M=" + s.M.toFixed(2) + "kN·m，σ=" + s.sigma.toFixed(2) + "MPa，η=" + s.ratio.toFixed(3));
      });
      L.push("");
      L.push("### 1. 压弯应力验算（GB50017 第 5.2 节，σ=|N|/A+|M|/W ≤ f=" + r.allow.toFixed(0) + "MPa）");
      L.push("");
      L.push("```");
      L.push("ηmax = " + r.ratio.toFixed(3) + (r.ok ? "  ✔ 通过" : "  ✘ 不满足"));
      L.push("```");
      L.push("");
      L.push("### 2. 结论");
      L.push("");
      L.push((r.ok ? "**安全**" : "**不安全**") + "，余度 " + (r.margin * 100).toFixed(1) + "%。");
      return L.join("\n");
    }
    var cc = concOf(matLabel);
    var bmm = 0, hmm = 0;
    if (sec && sec.type === "rect") { bmm = sec.b * 1000; hmm = sec.h * 1000; }
    else if (sec && sec.type === "circle") { bmm = sec.d * 800; hmm = sec.d * 1000; }
    else { bmm = 300; hmm = 500; }
    var names7 = ["左端", "1点", "2点", "3点", "4点", "5点", "右端"];
    L.push("- 钢筋：" + r.grade + "（fy=" + r.fy + " MPa，fyk=" + r.fyk + " MPa）");
    L.push("");
    L.push("### 1. 七点内力（左端 + 5 均分点 + 右端）");
    L.push("");
    L.push("| 位置 | x(m) | N(kN) | V(kN) | M(kN·m) |");
    L.push("|---|---|---|---|---|");
    var F7 = (r.forces7 && r.forces7.length === 7) ? r.forces7 : null;
    for (var i7 = 0; i7 < 7; i7++) {
      var pf = F7 ? F7[i7] : { N: r.ctrl.N, V: r.ctrl.V, M: r.ctrl.M };
      L.push("| " + names7[i7] + " | " + (r.L * i7 / 6).toFixed(2) + " | " + pf.N.toFixed(2) + " | " + pf.V.toFixed(2) + " | " + pf.M.toFixed(2) + " |");
    }
    L.push("");
    L.push("### 2. YJK 三段配筋（cm²）及对应内力");
    L.push("");
    L.push("- 上部 Asu（左-中-右）：**" + r.yjkTop + "**；下部 Asd（左-中-右）：**" + r.yjkBot + "**；箍筋 **" + r.yjkG + "** cm²/m");
    (r.secs || []).forEach(function (s) {
      L.push("  - " + s.x + "截面：M=" + s.M.toFixed(2) + "kN·m，V=" + s.V.toFixed(2) + "kN；上 " + f1(cm2(s.top)) + "cm²，下 " + f1(cm2(s.bot)) + "cm²");
    });
    L.push("- 实配：上部 " + r.asRealTop.text + "（" + r.asRealTop.area.toFixed(0) + "mm²），下部 " + r.asRealBot.text + "（" + r.asRealBot.area.toFixed(0) + "mm²），箍筋 " + r.stirrup.text);
    if (r.colBars) {
      L.push("- 柱集中标注（平法）：角筋 " + r.colBars.corner.text + "（" + r.colBars.corner.area.toFixed(0) + "mm²），四面中筋 " + r.colBars.side.text + "（" + r.colBars.side.area.toFixed(0) + "mm²）");
    }
    L.push("");
    L.push("### 3. 控制内力（|M|max 截面同期值）");
    L.push("");
    L.push("```");
    L.push("N=" + r.ctrl.N.toFixed(2) + "kN  V=" + r.ctrl.V.toFixed(2) + "kN  M=" + r.ctrl.M.toFixed(2) + "kN·m");
    L.push("以下正截面/斜截面/挠裂缝均依据该组控制内力计算");
    L.push("```");
    L.push("");
    var rho = r.asTheory / (bmm * hmm) * 100;
    var rhoReal = (r.asRealTop.area + r.asRealBot.area) / (bmm * hmm) * 100;
    L.push("### 4. 正截面受弯（GB50010 第 6.2 节，单筋矩形，α1=1.0）");
    L.push("");
    L.push("```");
    L.push("已知：fc=" + cc.fc + "MPa ft=" + cc.ft + "MPa fy=" + r.fy + "MPa b=" + bmm.toFixed(0) + "mm h0=" + r.h0.toFixed(0) + "mm M=" + r.ctrl.M.toFixed(2) + "kN·m");
    L.push("x = h0-√(h0²-2M/(α1·fc·b)) = " + r.x.toFixed(1) + "mm" + (r.over ? " ＞ ξb·h0=0.55h0，超筋！" : " ≤ ξb·h0，适筋"));
    L.push("As,calc = α1·fc·b·x/fy = " + r.asCalc.toFixed(0) + "mm²（" + f1(cm2(r.asCalc)) + "cm²）");
    L.push("构造要求：ρmin=" + (r.asMin / (bmm * hmm) * 100).toFixed(3) + "% → As,min=" + r.asMin.toFixed(0) + "mm²（" + f1(cm2(r.asMin)) + "cm²）");
    L.push("最终：As,theory = max = " + r.asTheory.toFixed(0) + "mm²，上max " + Math.max.apply(null, r.top3).toFixed(0) + "mm²，下max " + Math.max.apply(null, r.bot3).toFixed(0) + "mm²");
    L.push("配筋率 ρ=" + rho.toFixed(2) + "%；实配 ρ=" + rhoReal.toFixed(2) + "%（上" + r.asRealTop.text + "+下" + r.asRealBot.text + "）");
    L.push("富裕度 " + (((r.asRealTop.area + r.asRealBot.area) / Math.max(r.asTheory, 1e-9) - 1) * 100).toFixed(1) + "%" + (r.bendOk ? "  ✔" : "  ✘"));
    L.push("```");
    L.push("");
    L.push("### 5. 斜截面受剪（第 6.3 节）");
    L.push("");
    L.push("```");
    L.push("已知：V=" + r.ctrl.V.toFixed(2) + "kN  0.25·fc·b·h0=" + (0.25 * cc.fc * bmm * r.h0 / 1000).toFixed(1) + "kN" + (r.shearOk ? "  截面满足" : "  截面不足！"));
    L.push("0.7·ft·b·h0=" + (0.7 * cc.ft * bmm * r.h0 / 1000).toFixed(1) + "kN；Asv/s=max(0,(V-0.7ft·b·h0)/(fyv·h0))=" + (r.asv_s * 1000).toFixed(0) + "mm²/m");
    L.push("构造箍筋 + 计算结果：" + r.yjkG + " cm²/m，实配 " + r.stirrup.text + (r.shearOk ? "  ✔" : "  ✘"));
    L.push("```");
    L.push("");
    L.push("### 6. 柱构件偏压复核（对称配筋简化）" + (r.kind === "col" ? "" : "（本构件为梁，此节仅供参考）"));
    L.push("");
    L.push("```");
    var e0c = Math.abs(r.ctrl.M) * 1e6 / Math.max(Math.abs(r.ctrl.N) * 1000, 1e-9);
    L.push("N=" + r.ctrl.N.toFixed(1) + "kN M=" + r.ctrl.M.toFixed(1) + "kN·m → e0=M/N=" + e0c.toFixed(0) + "mm" + (r.kind === "col" ? "" : "（e0 大，以受弯为主）"));
    L.push("附加筋 ≈ N/(0.9·fy)×0.55×偏压放大；已计入三段配筋上/下值中");
    L.push("柱全截面最小 0.6%（2020版×1.1）；本构件 As,min=" + r.asMin.toFixed(0) + "mm² " + (r.kind === "col" ? (r.asTheory >= r.asMin - 1e-9 ? "✔ 满足" : "✘ 不满足") : ""));
    L.push("```");
    L.push("");
    L.push("### 7. 挠度与裂缝（第 7.1～7.2 节，开裂刚度 B=0.6EcI 估算）");
    L.push("");
    L.push("```");
    L.push("f = " + r.defl.toFixed(2) + "mm ≤ [f] = L/250 = " + r.deflAllow.toFixed(2) + "mm" + (r.defl <= r.deflAllow ? "  ✔" : "  ✘"));
    L.push("wmax = " + r.crack.toFixed(3) + "mm ≤ [w] = " + r.crackAllow.toFixed(2) + "mm" + (r.crack <= r.crackAllow ? "  ✔" : "  ✘"));
    L.push("```");
    L.push("");
    var concV = r.concVol || 0;
    var longA = (r.asRealTop.area + r.asRealBot.area); // mm2
    var longW = longA * 1e-6 * r.L * 7850;
    var stirW = (r.asv_s || 0) * r.L * 1000 * 1e-6 * 7850 * 0.5;
    L.push("### 8. 单根构件材料用量");
    L.push("");
    L.push("```");
    L.push("混凝土：A×L = " + (concV / Math.max(r.L, 1e-9)).toFixed(4) + "m² × " + r.L.toFixed(3) + "m = " + concV.toFixed(4) + "m³");
    L.push("纵筋：(" + r.asRealTop.area.toFixed(0) + "+" + r.asRealBot.area.toFixed(0) + ")mm²×" + r.L.toFixed(2) + "m×7850 = " + longW.toFixed(2) + "kg");
    L.push("箍筋(估)：Asv/s=" + (r.asv_s * 1000).toFixed(0) + "mm²/m×" + r.L.toFixed(2) + "m×0.5×7850 = " + stirW.toFixed(2) + "kg");
    L.push("合计钢筋约 " + (longW + stirW).toFixed(2) + "kg");
    L.push("```");
    L.push("");
    L.push("### 9. 结论");
    L.push("");
    L.push((r.ok ? "**安全**" : "**不安全**") + "，配筋余度 " + (r.margin * 100).toFixed(1) + "%。");
    L.push("");
    L.push("> 注：二维简化后处理；正式出图应以 PKPM/YJK 全模型复核为准。");
    return L.join("\n");
  }

  function mdSummary(design) {
    var L = [];
    L.push("## 全楼设计汇总");
    L.push("");
    L.push("- 规范：混凝土 " + design.codeConc + "；钢结构 " + design.codeSteel);
    L.push("- 杆件总数 " + design.total + "，不满足 " + design.unsafe + " 根");
    L.push("- 最大层间位移角 1/" + (design.maxDrift > 0 ? Math.round(1 / design.maxDrift) : "∞") + "（限值 1/550）");
    L.push("- 顶点位移 " + Number(design.topDisp).toFixed(2) + " mm，顶点位移角 " + design.topRatio.toExponential(2));
    L.push("- 混凝土用量 " + design.concVol.toFixed(2) + " m³");
    L.push("- 钢筋用量约 " + (design.rebarWt / 1000).toFixed(2) + " t");
    L.push("- 钢材用量约 " + (design.steelWt / 1000).toFixed(2) + " t");
    return L.join("\n");
  }

  global.FrameDesign = {
    CONC: CONC, REBAR: REBAR,
    isConcrete: isConcrete, isSteel: isSteel,
    designMember: designMember, runDesign: runDesign,
    mdCalcBook: mdCalcBook, mdSummary: mdSummary,
    ctrlForces: ctrlForces, storyDrift: storyDrift,
    CODES_CONC: ["GB50010-2010", "GB50010-2020"],
    CODES_STEEL: ["GB50017-2003", "GB50017-2017"]
  };
})(typeof window !== "undefined" ? window : globalThis);
