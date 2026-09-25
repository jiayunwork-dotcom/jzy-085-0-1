# 平面刚架静力核算服务

做且只做一件事：上游把平面刚架的几何、截面与荷载以 JSON 丢进来，服务用**直接刚度法**
解出节点位移，再回代出每根杆的杆端内力与各支座反力，经一个 HTTP 入口返回。
不涉及画图、台账、账户等任何周边功能。

## 模块划分

```
app/
├── transform.py     # 方向余弦与局部/整体坐标变换（6 阶正交变换矩阵）
├── element.py       # 单元局部刚度：轴向 + 弯曲的 6 阶矩阵；均布荷载等效节点荷载
├── assembly.py      # 总刚组装、荷载向量组装、约束施加（行列缩减法）
├── solver.py        # Cholesky 求解；奇异/病态立即报错，绝不返回 NaN
├── postprocess.py   # 杆端内力与支座反力回代
├── validation.py    # 输入语义校验（全部非法情形在此拒绝）
├── service.py       # 流程编排：校验 → 组装 → 求解 → 回代
├── schemas.py       # 请求/响应数据模型（Pydantic）
├── errors.py        # 机器可读错误码
└── main.py          # FastAPI 接入层：POST /solve
tests/               # 自动化测试（非法输入、算例校核、平衡恒等式）
examples/portal_frame.json   # 可手算校核的门式刚架算例
```

## 计算假定与约定

- 平面刚架，每节点 3 个自由度：`ux`（水平）、`uy`（竖向）、`rz`（转角，逆时针为正）。
- 整体坐标系 x 向右、y 向上；杆件局部 x 轴由 `n1` 指向 `n2`，局部 y 轴为局部 x 轴逆时针转 90°。
- 支座按自由度逐个给定（`ux/uy/rz` 为 `true` 即锁死）：铰支 = 锁 `ux,uy`；
  固定端 = 三者全锁；滑动支座 = 只锁被限制的方向。
- 荷载两类：节点集中力/集中力矩（整体坐标）；杆上垂直于杆轴的均布荷载 `q`
  （**按杆件局部坐标**，以局部 +y 为正；水平杆上 `q<0` 即竖直向下）。
- 杆端内力输出为 `[N, V, M]`：节点作用于杆端的力，方向同局部自由度正方向。
  `N` 以 `n2` 端为正表示受拉（受拉时恒有 `N1 = -N2`）。
- 支座反力为支座作用于结构的力，整体坐标，力矩逆时针为正。

### 约束施加方法

**行列缩减法（划去受约束自由度对应的行列），全程仅此一种。**
受约束自由度位移恒为零，组装完总刚后只取自由自由度的方程求解；
解出后用完整总刚回代 `R = K·u − F` 得到受约束自由度上的支座反力。
不使用大数罚系数。缩减后的总刚对稳定结构对称正定，用 Cholesky 分解求解；
分解失败（机构、约束不足等）即返回 `SINGULAR_STIFFNESS` 错误，绝不返回 NaN。

## 输入校验与错误码

非法模型返回 HTTP 422，响应体为
`{"success": false, "error": {"code": "...", "message": "..."}}`。

| code | 含义 |
|---|---|
| `DUPLICATE_NODE_ID` | 节点编号重复 |
| `DUPLICATE_ELEMENT_ID` | 杆件编号重复 |
| `DUPLICATE_SUPPORT` | 同一节点重复定义支座 |
| `UNKNOWN_NODE` | 杆件端点/支座/荷载指向不存在的节点 |
| `UNKNOWN_ELEMENT` | 均布荷载指向不存在的杆件 |
| `ZERO_LENGTH_ELEMENT` | 杆长为零（端点重合） |
| `INVALID_SECTION` | 弹性模量 E、面积 A、惯性矩 I 非正或非有限 |
| `INSUFFICIENT_CONSTRAINTS` | 约束不足以消除整体刚体运动（按刚体运动基做秩检查） |
| `DISCONNECTED_STRUCTURE` | 结构被分成互不相连的几块（并查集检查） |
| `EMPTY_MODEL` | 缺少节点或杆件 |
| `INVALID_SCHEMA` | 请求体不符合数据格式 |
| `SINGULAR_STIFFNESS` | 缩减总刚奇异/病态（机构、约束不足），求解被拒绝 |

## 手算校核算例：门式刚架

`examples/portal_frame.json`：柱高 h = 4 m，横梁跨度 L = 4 m，两柱脚固结；
左柱与横梁 EI = 2.0×10⁴ kN·m²，右柱 EI 加倍；截面面积取得很大（A = 100 m²），
轴向变形可忽略，可与教材中"忽略轴向变形"的手算结果对照。
左柱顶 B 处施加水平集中力 P = 10 kN。

**手算（位移法）**：取侧移 λ = Δ/h 与节点转角 θ_B、θ_C，k₁ = EI_左柱/h，k₂ = 2k₁，k_b = EI_梁/L = k₁：

```
节点 B 弯矩平衡：  6λ + 8θ_B +  2θ_C = 0
节点 C 弯矩平衡： 12λ + 2θ_B + 12θ_C = 0
层间剪力平衡：    36λ + 6θ_B + 12θ_C = P·h/k₁
→ θ_B = -12λ/23,  θ_C = -21λ/23,  λ = 23·P·h/(504·k₁)
```

回代得（P = 10 kN，h = L = 4 m）：

| 量 | 手算值 | 数值 |
|---|---|---|
| 左柱脚水平反力 H_A | −17P/42 | −4.0476 kN |
| 右柱脚水平反力 H_D | −25P/42 | −5.9524 kN |
| 左柱脚竖向反力 V_A | −11P/28 | −3.9286 kN |
| 右柱脚竖向反力 V_D | +11P/28 | +3.9286 kN |
| 左柱脚反力矩 M_A | 19Ph/84 | 9.0476 kN·m |
| 右柱脚反力矩 M_D | 32Ph/84 | 15.2381 kN·m |
| 横梁左端弯矩 | −5Ph/28 | −7.1429 kN·m |
| 横梁右端弯矩 | −3Ph/14 | −8.5714 kN·m |
| **横梁跨中弯矩** | **−Ph/56** | **−0.7143 kN·m** |
| 柱顶侧移 Δ | 23Ph³/(504·EI_左柱) | 1.4603×10⁻³ m |

（横梁无横向荷载，弯矩沿跨长线性变化，跨中弯矩 = (M_右 − M_左)/2。）
以上数值由 `tests/test_portal_frame.py` 逐项自动核对，容差 1e-4。

## 运行

### Docker（一条命令）

```bash
docker compose up --build
```

或不用 compose：

```bash
docker build -t frame-solver .
docker run --rm -p 8000:8000 frame-solver
```

服务监听 `http://localhost:8000`，接口文档见 `http://localhost:8000/docs`。

### 本地开发

```bash
pip install -r requirements-dev.txt   # 运行时锁 Python 3.12（镜像内）
uvicorn app.main:app --reload --port 8000
```

### 调用示例

```bash
curl -s -X POST http://localhost:8000/solve \
  -H 'Content-Type: application/json' \
  -d @examples/portal_frame.json | python3 -m json.tool
```

响应：

```json
{
  "success": true,
  "displacements": [{"node": 2, "ux": 0.0014603, "uy": 0.0, "rz": -0.0001905}, ...],
  "element_forces": [{"element": 2, "n1": {"N": ..., "V": ..., "M": -7.1429},
                                   "n2": {"N": ..., "V": ..., "M": -8.5714}}, ...],
  "reactions": [{"node": 1, "fx": -4.0476, "fy": -3.9286, "mz": 9.0476}, ...]
}
```

## 测试

```bash
pip install -r requirements-dev.txt
python -m pytest
```

覆盖三类内容：

1. **非法输入**（`tests/test_validation.py`）：节点编号重复、端点指向不存在的节点、
   杆长为零、E/A/I 非正、约束不足、结构不连通等，逐条断言错误码。
2. **门式刚架算例**（`tests/test_portal_frame.py`）：支座反力、横梁端弯矩、
   跨中弯矩、柱顶侧移与上表手算值对照，容差 1e-4。
3. **平衡恒等式**（`tests/test_equilibrium.py`，容差一律 1e-6）：
   - 全部支座反力 + 全部外荷载：水平、竖向合力为零；
   - 对任一固定点（取 3 个点分别验证）的合力矩为零；
   - 同一根杆两端轴力等值反向（N1 + N2 = 0）；
   - 每个节点处交汇各杆的杆端弯矩 + 外加力矩（+ 支座反力矩）平衡。

另有 `tests/test_udl.py`（固端力 wL²/12、简支梁跨中挠度 5wL⁴/384EI、
斜杆局部坐标荷载方向）、`tests/test_solver.py`（奇异矩阵必须报错而非 NaN）、
`tests/test_api.py`（HTTP 契约）。
