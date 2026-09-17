# -*- coding: utf-8 -*-
# FrameLab OpenSeesPy 示例：单跨门式刚架
# 单位：m / kN / kN*m（与 FrameLab 内部单位一致，读取时不做换算）
# 内容：6m 跨、4m 高门式刚架，柱底固定；梁均布荷载 + 柱顶侧向节点荷载。
# 用法：在 FrameLab 左侧“几何”面板 → OpenSees 分析 →“读取 OpenSees 文件”
# 选择本文件；或点“载入 Py 示例”。

import openseespy.opensees as ops

ops.wipe()
ops.model('basic', '-ndm', 2, '-ndf', 3)

# --- 材料（C30，E = 3.0e7 kN/m^2） ---
E = 3.0e7
A_col = 0.16        # 柱 0.4 x 0.4
I_col = 0.0021333   # 0.4*0.4^3/12
A_beam = 0.12       # 梁 0.2 x 0.6
I_beam = 0.0036     # 0.2*0.6^3/12

# --- 节点 ---
ops.node(1, 0.0, 0.0)
ops.node(2, 0.0, 4.0)
ops.node(3, 6.0, 0.0)
ops.node(4, 6.0, 4.0)

# --- 支座（固定端） ---
ops.fix(1, 1, 1, 1)
ops.fix(3, 1, 1, 1)

# --- 几何变换 ---
ops.geomTransf('Linear', 1)

# --- 单元：左柱 / 右柱 / 顶梁 ---
ops.element('elasticBeamColumn', 1, 1, 2, A_col, E, I_col, 1)
ops.element('elasticBeamColumn', 2, 3, 4, A_col, E, I_col, 1)
ops.element('elasticBeamColumn', 3, 2, 4, A_beam, E, I_beam, 1)

# --- 荷载 ---
ops.timeSeries('Linear', 1)
ops.pattern('Plain', 1, 1)
ops.load(2, 10.0, 0.0, 0.0)    # 左柱顶侧向力
ops.load(4, 0.0, -5.0, 0.0)    # 右梁端竖向力
ops.eleLoad('-ele', 3, '-type', '-beamUniform', -12.0)  # 梁均布荷载（向下）

# --- 求解（FrameLab 读取时忽略以下命令，仅供 OpenSees 直接运行） ---
ops.system('BandSPD')
ops.numberer('RCM')
ops.constraints('Plain')
ops.integrator('LoadControl', 1.0)
ops.algorithm('Linear')
ops.analysis('Static')
ops.analyze(1)
