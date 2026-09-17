#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线运行 FrameLab 导出的 xara 脚本并提取结果。

用法：
    python run_xara.py framelab_xara.py [-o results.json] [--timeout 300]

脚本内 FRAMELAB_JSON_BEGIN/END 标记之间的 JSON 即求解结果
（位移/反力/杆端力/子单元应力），可被 xara.js 直接回填，
也可作为与其他工具链对接的稳定接口。
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

BEGIN = "FRAMELAB_JSON_BEGIN"
END = "FRAMELAB_JSON_END"


def run_file(path, timeout=300, exe=None):
    with open(path, "r", encoding="utf-8") as f:
        script = f.read()
    with tempfile.TemporaryDirectory(prefix="framelab_xara_") as td:
        fp = os.path.join(td, "model.py")
        with open(fp, "w", encoding="utf-8") as f:
            f.write(script)
        p = subprocess.run([exe or sys.executable, fp], cwd=td,
                           capture_output=True, text=True, timeout=timeout)
    out, err = p.stdout or "", p.stderr or ""
    i, j = out.find(BEGIN), out.find(END)
    if i < 0 or j < 0 or j < i:
        raise RuntimeError("脚本未输出结果块（returncode=%s）。stderr 尾部：\n%s"
                           % (p.returncode, err[-2000:]))
    return json.loads(out[i + len(BEGIN):j].strip()), out, err


def summarize(res):
    print("analyze     :", res.get("analyze"), " ok:", res.get("ok"))
    if res.get("error"):
        print("error       :", res.get("error"))
    disps = res.get("disps") or {}
    if disps:
        m = max((abs(v[0]) + abs(v[1])) for v in disps.values())
        print("nodes       : %d  max|disp| = %.6g m" % (len(disps), m))
    beams = res.get("beams") or {}
    if beams:
        mm = max(max(abs(x) for x in v[1::2] + v[2::3]) for v in beams.values())
        print("beam elems  : %d  max|V,M| = %.6g" % (len(beams), mm))
    conts = res.get("conts") or {}
    if conts:
        vm = max(abs(v[0]) for v in conts.values())
        print("cont subcells: %d  max|sx| = %.6g kPa" % (len(conts), vm))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run FrameLab-exported xara script")
    ap.add_argument("script", help="xara .py 脚本（页面“下载 xara 脚本”得到）")
    ap.add_argument("-o", "--output", default=None, help="结果 JSON 输出路径")
    ap.add_argument("--timeout", type=int, default=300)
    ap.add_argument("--python", default=None, help="用于执行的 python 解释器")
    args = ap.parse_args(argv)
    res, out, err = run_file(args.script, timeout=args.timeout, exe=args.python)
    summarize(res)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=1)
        print("results ->", args.output)
    if not res.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())