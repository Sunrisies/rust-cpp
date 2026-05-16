# CoverMapObsPlan Rust 迁移版

这是覆盖地图观察规划算法的 Rust crate。当前主规划入口 `plan()` 已优先走 Rust 实现；父目录中的 MATLAB Coder 生成 C++ 代码仍会通过一个很薄的 C ABI shim 编译进来，主要用于 C++/Rust 结果对比测试，保证迁移过程可验证。

## 项目结构

```
rust/
├── Cargo.toml          # Rust项目配置
├── build.rs            # 构建脚本，用于编译C++代码
├── README.md           # 本文件
└── src/
    ├── main.rs         # 主程序入口
    ├── lib.rs          # 库入口
    ├── ffi.rs          # FFI绑定到C++函数，主要用于对比测试
    ├── types.rs        # 与 CoverMapObsPlan_types.h 对齐的数据结构
    ├── geometry.rs     # Rust侧几何、扫描、边界和障碍物处理
    └── planner.rs      # Rust侧主规划入口
```

## 构建说明

### 前提条件

1. 安装Rust工具链
2. 安装C++编译器（GCC或MSVC）
3. 确保原始C++代码在父目录中

### 构建步骤

```bash
cd rust
cargo build --release
```

如果当前环境的全局 `CARGO_TARGET_DIR` 不可写，可以把构建输出放回本项目：

```powershell
$env:CARGO_TARGET_DIR='target'
cargo build --release
```

### 运行

```bash
cargo run --release --bin cover_map_obs_plan_cli
```

## 使用方法

### 在 Rust 中调用迁移后的规划函数

```rust
use cover_map_obs_plan_rs::{plan, MapData, ObstacleData, PlanningParams, PolygonData};

fn main() {
    // 填充 map_data、polygon_data、obstacle_data、params 后调用规划函数。
    let result = plan(
        &map_data,
        &polygon_data,
        &mut obstacle_data,
        &params,
    );

    match result {
        Ok(result) => {
            println!("Planning succeeded!");
            println!("Waypoints: {}", result.waypoint_count);
            println!("Path length: {}", result.path_length);
        }
        Err(e) => {
            println!("Planning failed: {}", e);
        }
    }
}
```

### 如何确认 Rust 结果和 C++ 一样

本项目保留了 C++ FFI 基准，测试会同时运行 C++ 和 Rust，再逐点比较航点、路径长度、覆盖面积、时间和 yaw：

```powershell
$env:CARGO_TARGET_DIR='target'
cargo test
```

也可以只运行完整规划对比测试：

```powershell
$env:CARGO_TARGET_DIR='target'
cargo test --test planning_smoke -- --nocapture
```

## 渐进式迁移计划

### 阶段1：FFI集成
- [x] 创建基本的项目结构
- [x] 实现FFI绑定
- [x] 集成C++代码
- [x] 测试基本构建和初始化/释放流程

### 阶段2：几何、扫描和障碍物模块
- [x] 实现基础几何计算
- [x] 迁移第一批叶子函数：距离、航向、eps、mod、norm、det、sum、VectSin
- [x] 迁移第二批二维几何函数：点线判断、线段相交、顺逆时针、凹凸判断
- [x] 迁移边界辅助函数：shrOut、zoomInOut、circleSegment、minDisVerticeToVerticeAndLine
- [x] 迁移 mainShrOut 地块边界安全距离偏移，并加入 C++ 对照测试
- [x] 迁移 coverMapbyYaw 普通地块扫描线生成，并加入多方向 C++ 对照测试
- [x] 迁移 segmentIntersection2 和 edgeCollision 绕边连接逻辑，并加入 C++ 对照测试
- [ ] 迁移 coverMapbyYaw 的复杂凹地块分段逻辑
- [x] 在主规划流程中接入无障碍普通地块 Rust 分支，并加入 C++ 主算法对照测试
- [x] 迁移 circleSegment 与 circleAvoidRTL 的 Rust 圆障碍基础绕行逻辑
- [x] 在主规划流程中接入圆形障碍输入；非相交圆障碍已加入 C++ 对照测试
- [x] 对齐 circleAvoidRTL 单圆相交绕行点的 C++ 切线策略（水平、竖直、对角场景）
- [x] 在主规划流程中验证相交圆障碍物，并加入 C++ 对照测试
- [x] 对齐 circleAvoidRTL 多圆同航段和航段起点贴圆边界场景
- [x] 迁移 pygObsCollision 单多边形绕行逻辑，并加入 C++ 对照测试
- [x] 迁移 pygObsAvoid 单矩形障碍聚合逻辑，并加入 C++ 对照测试
- [x] 在主规划流程中接入单多边形障碍 Rust 分支；非相交和相交矩形障碍已加入 C++ 对照测试
- [x] 在主规划流程中接入多多边形 Rust 分支，并验证双矩形障碍物完整规划对齐
- [x] 验证圆形 + 多边形混合障碍物完整规划对齐
- [x] 对齐 pygObsAvoid/pygObsCollision 的 `State=0` 特殊场景和凹障碍物
- [ ] 针对更多真实输入验证结果一致性

说明：新增 Rust 迁移代码中的复杂逻辑已补充中文注释，重点说明扫描线求交、边界绕行、C++ 兼容输出规则等行为。

### 阶段3：完全迁移
- [ ] 完成所有模块迁移
- [ ] 在测试覆盖足够后清理或隔离 C++ 对比代码
- [ ] 优化整体性能

## 当前迁移进度

按核心功能和验证覆盖估算，当前完成度约 **92%**：

- 已迁移并接入：主规划流程、地块偏移、扫描线覆盖、边界连接、圆形障碍物、多边形障碍物、基础混合障碍物。
- 已验证：无障碍矩形地块、L 形凹地块、非相交圆、相交圆、双圆、圆边界端点、多圆同航段、非相交多边形、`State=0` 多边形、单多边形、双多边形、凹多边形、圆形+多边形混合障碍物、正/负 yaw、正/反 dir、safe_dist_map、safe_dist_obs。
- 已通过：`cargo test`、`cargo clippy -- -D warnings`、`cargo build --release`。
- 待继续：更多真实输入样例、性能和内存分配检查。

## 注意事项

1. **内存管理**
   - Rust分配的内存由Rust释放
   - C++分配的内存由C++释放
   - 使用RAII确保资源释放

2. **线程安全**
   - 注意Rust和C++的线程模型
   - 必要时使用互斥锁保护共享数据

3. **性能考虑**
   - 减少跨语言调用次数
   - 批量处理数据
   - 避免频繁的内存分配

## 许可证

待添加
