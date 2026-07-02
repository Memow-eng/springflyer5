# VINS-Fusion 后端稳定版：保守观测压缩

本文档记录当前分支上的一个稳定后端版本：**Backend Observation Compression Stable**。

这个版本的目标不是继续堆初始化 gate，也不是改前端特征或 IMU 逻辑，而是在 VINS-Fusion 已经进入 `NON_LINEAR` 后，对后端视觉残差做保守压缩，减少冗余长 track 的中间观测，降低 Ceres 后端负载，同时尽量不破坏弱纹理段和边缘化结构。

## 当前结论

当前推荐冻结配置为：

```yaml
backend_selector_mode: 2
backend_selector_warmup_frames: 10
backend_max_obs_per_feature: 4
backend_selector_min_total_obs: 500
backend_selector_min_track_len: 8
```

这组配置对应：

- `mode=2`：开启真实后端观测压缩。
- `min_total_obs=500`：一帧后端候选观测数低于 500 时不压缩，保护弱纹理/低约束段。
- `min_track_len=8`：只压缩观测数不少于 8 的长 track，短 track 全部保留。
- `max_obs_per_feature=4`：每条被压缩的长 track 最多保留 4 个关键观测。

`backend_landmark_budget` 目前仅保留为历史参数，当前保守压缩逻辑**不删除整条 landmark**。

## 做了什么

本版本在 `vins_estimator/src/estimator/estimator.cpp` 中增加了后端 selector：

- 只在 `solver_flag == NON_LINEAR` 后生效。
- 初始化阶段完全放行，避免把初始化问题和后端筛点问题混在一起。
- 不删除 landmark，不改 `feature_per_frame` 容器。
- 不改变 `feature_index` 枚举顺序，避免 `para_Feature` 索引错位。
- 只通过外部 mask 决定某个 visual factor 是否加入 Ceres。
- `MARGIN_OLD` 中 `start_frame == 0` 的边缘化相关观测强制保留。
- 每条 track 的 anchor、anchor 下一帧、newest 观测强制保留。
- 对可压缩的中间观测按归一化平面视差排序，保留视差信息量较大的观测。

也在 `parameters.cpp/.h` 和 `fast_drone_250.yaml` 中加入了可配置参数：

- `backend_selector_mode`
- `backend_selector_warmup_frames`
- `backend_max_obs_per_feature`
- `backend_selector_min_total_obs`
- `backend_selector_min_track_len`

同时回归脚本 `tools/vins_regression/run_regression.sh` 会把每个 bag 的 `backend_selector_stats.csv` 保存到对应 run 目录，方便按包分析 selector 行为。

## 为什么不是乱堆

这版和之前那些初始化/low-flow 叠加机制不同，它满足几个工程约束：

- **作用域窄**：只影响后端 visual residual 是否加入优化。
- **初始化隔离**：初始化阶段不启用 selector。
- **结构安全**：不删特征容器、不改特征深度生命周期、不动 IMU 因子。
- **边缘化安全**：边缘化 pin 住的观测全部放行。
- **弱段保护**：低 `total_obs` 帧直接不压缩。
- **可回退**：改 `backend_selector_mode: 1` 可回到 log-only，改 `0` 可完全关闭。

## 回归结果

### 选型过程

我们比较过三组配置：

| 配置 | 结论 |
| --- | --- |
| `K_high=250, max_obs=4` | 淘汰，`17-16-27` 出现一次约 `20m` 离群 |
| `K_high=500, max_obs=5` | 稳定但收益偏弱，`18-36-17` 退化到约 `1.30m` |
| `K_high=500, max_obs=4` | 当前冻结候选，收益和稳定性最好 |

### 全量 RUNS=3：`K_high=500, max_obs=4`

结果目录：

```text
/tmp/vins_sf3_selector_k500_gate_runs3
```

关键结果：

| Bag | end_norm 三次结果 | 判断 |
| --- | --- | --- |
| `17-10-48` | `1.354 / 0.991 / 1.122m` | 可接受 |
| `17-13-59` | `1.257 / 2.759 / 1.199m` | 出现一次尾部风险，后续单包复测 |
| `17-16-27` | `1.957 / 1.971 / 1.985m` | 稳定，无 20m 离群 |
| `16-19-34` | `0.185 / 0.137 / 0.237m` | 很稳 |
| `16-23-14` | `0.446 / 0.499 / 0.444m` | 很稳 |
| `18-36-17` | `0.935 / 1.156 / 1.165m` | 稳定，优于更温和 obs5 |

### 尾部风险复测：`17-13-59` RUNS=5

结果目录：

```text
/tmp/vins_sf3_selector_k500_obs4_17_13_59_runs5
```

结果：

```text
1.8550 / 1.1911 / 1.2048 / 1.1258 / 1.0487m
```

5 次里 **0 次 > 2.5m**，所以此前 `2.759m` 按偶发处理，不视为稳定系统风险。

## 已知问题

`2026-07-01-16-43-33.bag` 仍会初始化，导致负样本 `expect_init=0` 不通过。这个问题属于初始化判据/低激励初始化问题，和当前后端 observation compression 不同线，不在本版本内解决。

## 使用建议

当前版本可以作为后端稳定候选版继续飞行前离线验证：

- 保留 `backend_selector_mode: 2`。
- 不再横向调 `max_obs=3/5/6` 或 `K_high=250/350/700`，避免过拟合当前 8 个 bag。
- 如果真机或新 bag 出现尾部风险，优先临时切回：

```yaml
backend_selector_mode: 1
```

这样只保留日志账本，不真实裁剪 visual factor。

如果需要完全关闭：

```yaml
backend_selector_mode: 0
```

## 当前版本定位

这是一个**稳定后端版本**，不是最终初始化修复版。

它解决的是后端冗余视觉观测压缩和 Ceres 减负问题；初始化负样本误通过、低激励启动、现场红绿轨迹分离等问题，需要单独开线分析。
