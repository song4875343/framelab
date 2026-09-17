# FrameLab Tcl 示例：简支梁（均布 + 跨中集中力）
# 单位：m / kN / kN*m（与 FrameLab 内部单位一致，读取时不做换算）
# 内容：6m 简支梁（左铰支、右辊轴），全跨均布荷载 + 跨中集中力。
# 用法：在 FrameLab 左侧“几何”面板 → OpenSees 分析 →“读取 OpenSees 文件”
# 选择本文件；或点“载入 Tcl 示例”。

wipe
model BasicBuilder -ndm 2 -ndf 3

# --- 参数（FrameLab 读取时支持 set / $变量 / [expr]） ---
set E  3.0e7
set A  0.15
set Iz 0.0045
set L  6.0

# --- 节点 ---
node 1 0.0 0.0
node 2 [expr $L/2] 0.0
node 3 $L 0.0

# --- 支座：左铰支（Ux、Uy 约束），右辊轴（Uy 约束） ---
fix 1 1 1 0
fix 3 0 1 0

# --- 几何变换 ---
geomTransf Linear 1

# --- 单元：两段梁单元 ---
element elasticBeamColumn 1 1 2 $A $E $Iz 1
element elasticBeamColumn 2 2 3 $A $E $Iz 1

# --- 荷载 ---
pattern Plain 1 Linear {
    load 2 0.0 -20.0 0.0
    eleLoad -ele 1 -type -beamUniform -8.0
    eleLoad -ele 2 -type -beamUniform -8.0
    eleLoad -ele 2 -type -beamPoint -15.0 0.5
}
