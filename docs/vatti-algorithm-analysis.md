# Clipper1 仓库中 Vatti 裁剪算法实现分析

> 基于 Clipper 6.4.2 版本源码（Angus Johnson, 2010-2017）  
> 分析涵盖 C++（`cpp/clipper.hpp`, `cpp/clipper.cpp`）、Delphi（`Delphi/clipper.pas`）和 C#（`C#/clipper_library/clipper.cs`）三个实现

---

## 目录

1. [核心数据结构](#1-核心数据结构)
2. [算法主要流程：布尔运算处理](#2-算法主要流程布尔运算处理)
3. [扫描线过程中的边操作与交点计算](#3-扫描线过程中的边操作与交点计算)
4. [特殊情况处理](#4-特殊情况处理)
5. [输出多边形构建过程](#5-输出多边形构建过程)
6. [与原始 Angus Johnson Clipper 库的关系](#6-与原始-angus-johnson-clipper-库的关系)

---

## 1. 核心数据结构

### 1.1 基本类型与枚举

在 `cpp/clipper.hpp:64-71` 中定义了算法使用的基本枚举类型：

| 枚举类型 | 取值 | 用途 |
|----------|------|------|
| `ClipType` | `ctIntersection, ctUnion, ctDifference, ctXor` | 布尔运算类型 |
| `PolyType` | `ptSubject, ptClip` | 多边形角色（主体/裁剪） |
| `PolyFillType` | `pftEvenOdd, pftNonZero, pftPositive, pftNegative` | 填充规则 |

坐标类型 `IntPoint` 使用整数坐标（默认 64 位），避免浮点精度问题：

```cpp
// cpp/clipper.hpp:85-103
struct IntPoint {
  cInt X;
  cInt Y;
  // 可选 Z 坐标（use_xyz 宏）
};
```

### 1.2 边结构 TEdge

`TEdge` 是扫描线算法的核心数据结构，定义于 `cpp/clipper.cpp:66-84`：

```cpp
struct TEdge {
  IntPoint Bot;       // 边的底部点
  IntPoint Curr;      // 当前点（每个新扫描段更新）
  IntPoint Top;       // 边的顶部点
  double Dx;          // 斜率的倒数（dx/dy）
  PolyType PolyTyp;   // 所属多边形类型（subject/clip）
  EdgeSide Side;      // 当前在解多边形的哪一侧
  int WindDelta;      // 绕数增量（+1 或 -1）
  int WindCnt;        // 绕数计数
  int WindCnt2;       // 对立多边形类型的绕数
  int OutIdx;         // 输出记录索引
  TEdge *Next;        // 环形链表中的下一条边
  TEdge *Prev;        // 环形链表中的上一条边
  TEdge *NextInLML;   // 局部极小值表中的下一条
  TEdge *NextInAEL;   // 活动边表中的下一条
  TEdge *PrevInAEL;   // 活动边表中的上一条
  TEdge *NextInSEL;   // 排序边表中的下一条
  TEdge *PrevInSEL;   // 排序边表中的上一条
};
```

每条边拥有**三套链表指针**：
- **LML 链**（`NextInLML`）：局部极小值列表，用于按扫描顺序处理边的进入
- **AEL 链**（`NextInAEL/PrevInAEL`）：活动边表（Active Edge List），维护当前扫描线处的活跃边
- **SEL 链**（`NextInSEL/PrevInSEL`）：排序边表（Sorted Edge List），用于交点检测和水平边处理

Delphi 版本定义完全同构（`Delphi/clipper.pas:178-196`）。

### 1.3 局部极小值 LocalMinimum

`LocalMinimum` 定义于 `cpp/clipper.cpp:92-96`：

```cpp
struct LocalMinimum {
  cInt   Y;           // 极小值的 Y 坐标
  TEdge *LeftBound;   // 左边界边
  TEdge *RightBound;  // 右边界边
};
```

每个局部极小值代表多边形轮廓上的一个"V"形底部，是两条向上延伸的边界的起点。

### 1.4 交点节点 IntersectNode

`IntersectNode` 定义于 `cpp/clipper.cpp:86-90`：

```cpp
struct IntersectNode {
  TEdge    *Edge1;
  TEdge    *Edge2;
  IntPoint  Pt;       // 交点坐标
};
```

### 1.5 输出结构 OutRec / OutPt

输出多边形由 `OutRec`（输出记录）和 `OutPt`（输出点）组成，定义于 `cpp/clipper.cpp:100-117`：

```cpp
// 输出记录：代表裁剪解中的一条路径
struct OutRec {
  int       Idx;
  bool      IsHole;      // 是否为洞
  bool      IsOpen;      // 是否为开放路径
  OutRec   *FirstLeft;   // 父多边形（用于洞关系层级）
  PolyNode *PolyNd;
  OutPt    *Pts;          // 点链表头
  OutPt    *BottomPt;     // 最底部点
};

// 输出点：双向循环链表
struct OutPt {
  int       Idx;
  IntPoint  Pt;
  OutPt    *Next;
  OutPt    *Prev;
};
```

### 1.6 连接结构 Join

`Join` 结构（`cpp/clipper.cpp:119-123`）用于处理共享边的多边形片段连接：

```cpp
struct Join {
  OutPt    *OutPt1;
  OutPt    *OutPt2;
  IntPoint  OffPt;    // 偏移点（提供公共边的斜率信息）
};
```

### 1.7 类层次结构

| 类 | 定义位置 | 职责 |
|----|----------|------|
| `ClipperBase` | `cpp/clipper.hpp:220-260` | 将多边形坐标转换为边对象存入 LocalMinima 列表 |
| `Clipper` | `cpp/clipper.hpp:263-357` | 执行 Vatti 裁剪算法主流程 |
| `ClipperOffset` | `cpp/clipper.hpp:360-388` | 多边形偏移 |
| `PolyTree/PolyNode` | `cpp/clipper.hpp:136-171` | 输出结果的树形层级表示 |

**ClipperBase 核心成员** (`cpp/clipper.hpp:247-259`)：
- `m_MinimaList`：局部极小值列表（`std::vector<LocalMinimum>`）
- `m_ActiveEdges`：活动边表头指针（`TEdge*`）
- `m_Scanbeam`：扫描线优先队列（`std::priority_queue<cInt>`）
- `m_PolyOuts`：输出多边形列表（`std::vector<OutRec*>`）
- `m_edges`：所有边数组的容器（`std::vector<TEdge*>`）

---

## 2. 算法主要流程：布尔运算处理

### 2.1 总体流程

算法主流程在 `Clipper::ExecuteInternal()`（`cpp/clipper.cpp:1560-1621`）：

```
ExecuteInternal():
  1. Reset()                              // 重置状态，排序 LocalMinima
  2. PopScanbeam(botY)                    // 取第一条扫描线
  3. InsertLocalMinimaIntoAEL(botY)       // 将首个扫描段的边插入 AEL
  4. while (PopScanbeam(topY) || LocalMinimaPending()):
     a. ProcessHorizontals()              // 处理水平边
     b. ClearGhostJoins()
     c. ProcessIntersections(topY)        // 处理当前扫描段内的交点
     d. ProcessEdgesAtTopOfScanbeam(topY) // 处理到达扫描段顶部的边
     e. InsertLocalMinimaIntoAEL(topY)    // 插入新的局部极小值
  5. 修复输出方向
  6. JoinCommonEdges()                    // 连接共享边
  7. FixupOutPolygon()                    // 清理冗余点
  8. DoSimplePolygons() [可选]            // 简化自交多边形
```

### 2.2 输入处理：AddPath

`ClipperBase::AddPath()`（`cpp/clipper.cpp:1045-1221`）负责将路径转换为边结构：

1. **创建边数组**：为路径中的每个顶点创建一条 `TEdge`（`cpp/clipper.cpp:1061-1076`）
2. **去重与简化**：移除重复顶点和共线边（`cpp/clipper.cpp:1085-1117`）
3. **设置 Bot/Top/Dx**：确定每条边的上下端点和斜率（`cpp/clipper.cpp:1131-1139`，调用 `InitEdge2`）
4. **找到局部极小值**：调用 `FindNextLocMin` 找到轮廓上的每个"V"底部（`cpp/clipper.cpp:1181`）
5. **处理每个极小值**：确定左右边界，设置 WindDelta，调用 `ProcessBound` 建立 LML 链（`cpp/clipper.cpp:1186-1218`）

### 2.3 布尔运算判定

布尔运算的核心判定逻辑在两个函数中：

#### IsContributing (`cpp/clipper.cpp:1741-1837`)

此函数根据填充规则和裁剪类型决定一条边是否"贡献"输出多边形：

**第一步** — 按同类填充规则检查 `WindCnt`：
- `pftEvenOdd`：WindCnt == 1 时贡献
- `pftNonZero`：|WindCnt| == 1 时贡献
- `pftPositive`：WindCnt == 1 时贡献
- `pftNegative`：WindCnt == -1 时贡献

**第二步** — 按 `ClipType` 检查 `WindCnt2`（对立多边形的绕数）：
- **交集**(`ctIntersection`)：要求 WindCnt2 ≠ 0（在对方多边形内部）
- **并集**(`ctUnion`)：要求 WindCnt2 == 0（不在对方多边形内部）
- **差集**(`ctDifference`)：Subject 边要求 WindCnt2 == 0；Clip 边要求 WindCnt2 ≠ 0
- **异或**(`ctXor`)：所有闭合边都贡献

#### SetWindingCount (`cpp/clipper.cpp:1624-1722`)

通过遍历 AEL 中同类型的前驱边来设置当前边的绕数：
- 对于 EvenOdd 规则：简单翻转
- 对于 NonZero/Positive/Negative 规则：累加 WindDelta

同时通过遍历 AEL 中异类型边来设置 `WindCnt2`。

### 2.4 交点处的布尔运算 (`cpp/clipper.cpp:2106-2298`)

`IntersectEdges` 函数在两条边的交点处执行布尔运算逻辑：

1. **更新绕数**（`cpp/clipper.cpp:2164-2186`）：交换或累加 WindCnt
2. **按贡献状态分四种情况**：
   - 两边都贡献 → `AddLocalMaxPoly`（闭合输出）或交换侧面
   - 仅 e1 贡献 → 追加点并交换
   - 仅 e2 贡献 → 追加点并交换
   - 都不贡献 → 可能 `AddLocalMinPoly`（开始新输出）

---

## 3. 扫描线过程中的边操作与交点计算

### 3.1 边的插入

#### InsertLocalMinimaIntoAEL (`cpp/clipper.cpp:1978-2077`)

当扫描线到达一个局部极小值的 Y 坐标时，将该极小值的左右边界插入 AEL：

1. 调用 `InsertEdgeIntoAEL` 将边按 X 坐标有序插入 AEL（`cpp/clipper.cpp:3319-3345`）
2. 调用 `SetWindingCount` 设置绕数
3. 如果贡献输出则调用 `AddLocalMinPoly` 或 `AddOutPt`
4. 水平右边界加入 SEL
5. 处理与已有活动边之间可能存在的交叉

#### InsertEdgeIntoAEL (`cpp/clipper.cpp:3319-3345`)

按 `E2InsertsBeforeE1` 规则（主要比较 `Curr.X`）在 AEL 链表中找到正确位置插入新边。

### 3.2 边的删除

#### DeleteFromAEL (`cpp/clipper.cpp:1367-1377`)

从 AEL 双向链表中断开指定边，维护 `m_ActiveEdges` 头指针。

#### UpdateEdgeIntoAEL (`cpp/clipper.cpp:1442-1462`)

当边到达其 `Top` 点时，将其"提升"为 `NextInLML`（链条中的下一段），保持 AEL 位置不变但更新边属性。同时将新段的 `Top.Y` 加入扫描线队列。

### 3.3 交点计算

#### IntersectPoint (`cpp/clipper.cpp:622-689`)

精确计算两条边的交点，处理特殊情况：
- 相同斜率：使用当前 Y 值
- 垂直边（Dx==0）+ 水平边
- 一般情况：使用线性方程 `y = x/Dx + b` 求解
- 结果限制在当前扫描段范围内

#### BuildIntersectList (`cpp/clipper.cpp:2856-2902`)

在每个扫描段内检测交点：

1. 将 AEL 拷贝到 SEL
2. 更新每条边在 `topY` 处的 X 坐标
3. **冒泡排序** SEL，每次交换记录一个 `IntersectNode`
4. 对于 X 坐标反序的相邻边对，调用 `IntersectPoint` 计算交点

#### FixupIntersectionOrder (`cpp/clipper.cpp:2934-2954`)

确保交点按从底到顶排序，且每个交点的两条边在 SEL 中相邻。通过调整顺序使得算法可以正确地逐个处理交点。

#### ProcessIntersectList (`cpp/clipper.cpp:2906-2918`)

逐个处理排好序的交点：
1. 调用 `IntersectEdges`（执行布尔运算逻辑）
2. 调用 `SwapPositionsInAEL`（维护 AEL 有序性）

### 3.4 扫描线顶部处理 (`cpp/clipper.cpp:3009-3113`)

`ProcessEdgesAtTopOfScanbeam` 执行四步操作：

1. **处理极大值**：到达顶点的边对调用 `DoMaxima`，可能闭合输出多边形
2. **提升水平边**：中间边如果下一段是水平的，提升后加入 SEL
3. **处理水平边**：排序 Maxima 列表后调用 `ProcessHorizontals`
4. **提升中间顶点**：更新中间边为下一段，检查是否需要连接

---

## 4. 特殊情况处理

### 4.1 水平边处理

水平边（`Dx == HORIZONTAL`，即 `-1.0E+40`）是 Vatti 算法中最复杂的特殊情况。

#### ProcessHorizontal (`cpp/clipper.cpp:2636-2824`)

详细注释在 `cpp/clipper.cpp:2626-2634`：

> 水平边在扫描线交点处（即在扫描段的顶部或底部）被当作分层处理。
> HE 的处理顺序无关紧要。HE 只与其他 HE 的 Bot.X 相交，
> 也与非水平边相交。

处理流程：
1. 确定方向（`GetHorzDirection`，`cpp/clipper.cpp:2610-2623`）
2. 找到连续水平边的最后一条
3. 循环处理每条连续水平边：
   - 在 Maxima 触点处插入额外坐标（简化用）
   - 与沿途的非水平活动边相交（调用 `IntersectEdges`）
   - 到达尽头后提升（`UpdateEdgeIntoAEL`）或删除
4. 使用 **Ghost Join** 机制处理不确定的水平重合

#### ReverseHorizontal (`cpp/clipper.cpp:756-765`)

交换水平边的 Top.X 和 Bot.X，使其与相邻下方边的方向一致。

### 4.2 重合边（共线边）处理

#### 输入阶段 (`cpp/clipper.cpp:1085-1113`)

在 `AddPath` 中移除共线顶点：
- 默认合并相邻共线边为单边
- 若 `PreserveCollinear == true`，仅移除重叠共线边（即尖刺/spike）

#### 连接阶段

`JoinPoints`（`cpp/clipper.cpp:3458-3613`）处理三种连接类型：

1. **水平重合连接**（`cpp/clipper.cpp:3464-3465`）：两个输出点沿水平共线边的任意位置
2. **非水平重合连接**（`cpp/clipper.cpp:3466-3467`）：两个输出点在重叠段底部相同位置
3. **StrictSimple 接触连接**（`cpp/clipper.cpp:3468-3469`）：边接触但不共线

水平重合的具体处理由 `JoinHorz`（`cpp/clipper.cpp:3371-3455`）完成。

### 4.3 自相交多边形

#### DoSimplePolygons (`cpp/clipper.cpp:4221-4280`)

当 `StrictlySimple` 选项启用时，在后处理阶段拆分自交多边形：

1. 遍历每个输出多边形的点链表
2. 查找重复点（即自交点）
3. 在重复点处将多边形**拆分为两个**
4. 使用 `Poly2ContainsPoly1` 确定新多边形之间的包含/洞关系
5. 通过 `FixupFirstLefts1/2` 修复层级引用

#### SimplifyPolygon (`cpp/clipper.cpp:4296-4301`)

高层接口，通过 `ctUnion + StrictlySimple` 将自交多边形简化为简单多边形。

### 4.4 开放路径（线段裁剪）

使用 `use_lines` 宏启用（`cpp/clipper.hpp:47`），开放路径的 `WindDelta == 0`。

在 `IntersectEdges` 中有专门的分支处理开放路径（`cpp/clipper.cpp:2115-2161`）：
- 忽略两个开放路径之间的相交
- 处理开放路径与闭合多边形的相交
- 根据裁剪类型切换开放路径的输出状态

---

## 5. 输出多边形构建过程

### 5.1 事件驱动的增量构建

输出多边形的构建是在扫描线处理过程中增量完成的：

#### AddOutPt (`cpp/clipper.cpp:2463-2499`)

为边添加一个输出点：
- 如果边还没有关联的 `OutRec`，创建新的输出记录和首个点
- 如果已有 `OutRec`，根据边在输出多边形的哪一侧（`esLeft/esRight`）将新点追加到双向环链表的前端或后端

#### AddLocalMinPoly (`cpp/clipper.cpp:1841-1881`)

在局部极小值处启动新的输出轮廓：
- 根据 Dx 大小决定哪条边在左侧
- 创建首个输出点
- 两条边共享同一个 `OutIdx`
- 检查与前驱边的共线情况并记录 Join

#### AddLocalMaxPoly (`cpp/clipper.cpp:1884-1897`)

在局部极大值处闭合输出轮廓：
- 为两条边添加极大值点
- 如果两条边属于同一 OutRec，断开关联
- 否则合并两个 OutRec（`AppendPolygon`）

### 5.2 多边形合并 AppendPolygon (`cpp/clipper.cpp:2367-2460`)

当两条边在极大值点相遇且属于不同 OutRec 时，合并它们：
1. 确定洞状态（使用 `GetLowermostRec` 和 `OutRec1RightOfOutRec2`）
2. 根据双方的左右侧（4 种组合）重新连接点链表
3. 更新 AEL 中所有引用旧 OutRec 的边

### 5.3 后处理

#### JoinCommonEdges (`cpp/clipper.cpp:3679-3763`)

处理在扫描过程中记录的所有 Join：
- 如果两个 OutPt 属于同一 OutRec → 拆分为两个多边形
- 如果属于不同 OutRec → 合并为一个多边形
- 使用 `Poly2ContainsPoly1` 确定包含关系并设置洞状态

#### FixupOutPolygon (`cpp/clipper.cpp:3143-3181`)

清理输出多边形：
- 移除重复点
- 移除共线中间顶点（除非 `PreserveCollinear` 或 `StrictSimple`）

### 5.4 最终结果输出

#### BuildResult (`cpp/clipper.cpp:3199-3217`)

将 `OutRec` 的点链表转换为 `Paths`（点数组的数组）。

#### BuildResult2 (`cpp/clipper.cpp:3220-3262`)

将 `OutRec` 转换为 `PolyTree`，建立父子层级关系：
- 修复洞链接（`FixHoleLinkage`）
- 为每个有效 OutRec 创建 `PolyNode`
- 根据 `FirstLeft` 关系建立树形结构
- 开放路径直接挂载到根节点

---

## 6. 与原始 Angus Johnson Clipper 库的关系

### 6.1 作者与版本

三个实现文件一致标注：
- **作者**：Angus Johnson
- **版本**：6.4.2
- **日期**：27 February 2017
- **许可**：Boost Software License Ver 1

参考：`cpp/clipper.hpp:3-7`, `Delphi/clipper.pas:5-9`, `C#/clipper_library/clipper.cs:3-7`

### 6.2 算法来源声明

三个实现都明确声明（例如 `cpp/clipper.hpp:14-17`）：

> The code in this library is an extension of Bala Vatti's clipping algorithm:  
> "A generic solution to polygon clipping"  
> Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.

### 6.3 语言移植关系

- **Delphi 版本**（`Delphi/clipper.pas`）是**原始实现**
- **C++ 版本**（`cpp/clipper.cpp:34-38`）明确说明是 Delphi 版本的翻译：  
  > *This is a translation of the Delphi Clipper library and the naming style used has retained a Delphi flavour.*
- **C# 版本**（`C#/clipper_library/clipper.cs:34-38`）同样标注为翻译

三个实现的函数签名、逻辑结构、甚至注释风格都高度一致。

### 6.4 仓库身份

README.md 指向 `https://sourceforge.net/projects/polyclipping`，即 Angus Johnson 的 Clipper 项目官方地址。本仓库是该库在 GitHub 上的镜像/fork。

### 6.5 对 Vatti 原始算法的扩展

Clipper 库在 Vatti 原始算法基础上增加了以下功能：
1. **多种填充规则**：EvenOdd、NonZero、Positive、Negative（原始 Vatti 仅支持 EvenOdd）
2. **多种布尔运算**：交、并、差、异或
3. **开放路径裁剪**：线段与多边形的裁剪（`use_lines`）
4. **自交多边形处理**：`StrictlySimple` 模式
5. **多边形偏移**：`ClipperOffset` 类
6. **树形输出**：`PolyTree` 保留完整的洞/外轮廓层级关系
7. **Z 坐标支持**：可选的第三维坐标传播（`use_xyz`）
8. **128 位整数运算**：`Int128` 类用于大坐标范围下的精确交叉乘积计算

---

## 附录：关键函数索引（C++ 实现）

| 函数 | 文件位置 | 用途 |
|------|----------|------|
| `ClipperBase::AddPath` | `cpp/clipper.cpp:1045` | 输入路径转换为边结构 |
| `ClipperBase::Reset` | `cpp/clipper.cpp:1247` | 重置状态，排序 LocalMinima |
| `ClipperBase::InsertScanbeam` | `cpp/clipper.cpp:1335` | 将 Y 值加入扫描线队列 |
| `ClipperBase::PopScanbeam` | `cpp/clipper.cpp:1341` | 取出下一条扫描线 |
| `ClipperBase::PopLocalMinima` | `cpp/clipper.cpp:1286` | 取出指定 Y 的局部极小值 |
| `ClipperBase::DeleteFromAEL` | `cpp/clipper.cpp:1367` | 从 AEL 删除边 |
| `ClipperBase::UpdateEdgeIntoAEL` | `cpp/clipper.cpp:1442` | 提升边到下一段 |
| `ClipperBase::SwapPositionsInAEL` | `cpp/clipper.cpp:1395` | 交换 AEL 中两条边的位置 |
| `Clipper::ExecuteInternal` | `cpp/clipper.cpp:1560` | 算法主循环 |
| `Clipper::SetWindingCount` | `cpp/clipper.cpp:1624` | 设置边的绕数 |
| `Clipper::IsContributing` | `cpp/clipper.cpp:1741` | 判定边是否贡献输出 |
| `Clipper::AddLocalMinPoly` | `cpp/clipper.cpp:1841` | 启动新输出轮廓 |
| `Clipper::AddLocalMaxPoly` | `cpp/clipper.cpp:1884` | 闭合输出轮廓 |
| `Clipper::InsertLocalMinimaIntoAEL` | `cpp/clipper.cpp:1978` | 插入局部极小值到 AEL |
| `Clipper::IntersectEdges` | `cpp/clipper.cpp:2106` | 交点处的布尔运算 |
| `Clipper::AppendPolygon` | `cpp/clipper.cpp:2367` | 合并两个输出多边形 |
| `Clipper::AddOutPt` | `cpp/clipper.cpp:2463` | 添加输出点 |
| `Clipper::ProcessHorizontals` | `cpp/clipper.cpp:2512` | 处理所有水平边 |
| `Clipper::ProcessHorizontal` | `cpp/clipper.cpp:2636` | 处理单条水平边 |
| `Clipper::ProcessIntersections` | `cpp/clipper.cpp:2827` | 交点处理总入口 |
| `Clipper::BuildIntersectList` | `cpp/clipper.cpp:2856` | 构建交点列表 |
| `Clipper::FixupIntersectionOrder` | `cpp/clipper.cpp:2934` | 修正交点处理顺序 |
| `Clipper::ProcessIntersectList` | `cpp/clipper.cpp:2906` | 逐个处理交点 |
| `Clipper::DoMaxima` | `cpp/clipper.cpp:2957` | 处理极大值 |
| `Clipper::ProcessEdgesAtTopOfScanbeam` | `cpp/clipper.cpp:3009` | 扫描段顶部处理 |
| `Clipper::InsertEdgeIntoAEL` | `cpp/clipper.cpp:3319` | 有序插入 AEL |
| `Clipper::BuildResult` | `cpp/clipper.cpp:3199` | 生成 Paths 结果 |
| `Clipper::BuildResult2` | `cpp/clipper.cpp:3220` | 生成 PolyTree 结果 |
| `Clipper::JoinPoints` | `cpp/clipper.cpp:3458` | 执行点连接 |
| `Clipper::JoinCommonEdges` | `cpp/clipper.cpp:3679` | 连接共享边 |
| `Clipper::DoSimplePolygons` | `cpp/clipper.cpp:4221` | 拆分自交多边形 |
| `Clipper::FixupOutPolygon` | `cpp/clipper.cpp:3143` | 清理输出多边形 |
| `IntersectPoint` | `cpp/clipper.cpp:622` | 计算两边交点 |
| `SlopesEqual` | `cpp/clipper.cpp:541-575` | 判断斜率相等（共线检测） |
| `IsHorizontal` | `cpp/clipper.cpp:578` | 判断是否水平边 |
