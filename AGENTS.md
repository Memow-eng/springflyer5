# Springflyer3 项目维护规范

这份文件是后续维护 `/home/jun/springflyer3` 的统一施工标准。目标很明确：在不乱堆机制、不污染原有结构的前提下，把当前 VINS-Fusion 做到比原版更适合现场飞行，重点解决弱纹理、白墙、低视差、高动态和短时视觉退化时容易飘飞的问题，至少在退化发生后能稳定坚持几秒并给控制端清晰的可信度语义。

任何后续修改都必须先读本文件，再读相关目录下的 README/策略文档。没有完成需求澄清、代码边界确认、回归设计之前，不允许直接动核心算法。

## 总原则

- 先摸清需求，再实现。每次改动前必须明确：解决哪个现象、影响哪个模块、验证哪个指标、失败后如何回退。
- 一次只改一条线。初始化、前端、后端、控制接口、回归脚本不能在同一次实验里混改，否则结果不可归因。
- 不为了 RViz 好看牺牲控制安全。连续发布、平滑轨迹、降噪显示都不能掩盖 VIO 不可信状态。
- 不新增“看起来保险”的硬 gate。新增门控前必须说明它保护什么、误杀什么、在哪些 bag 上验证。
- 不主动删除仍在成功追踪的长 track。前端筛点只能做日志、排序偏好或新点补充偏好，不能随意 remove 可靠老点。
- 不写状态。低光流、弱纹理、白墙、静止检测可以用于日志和质量判断，不能直接硬写 `P/V/Ba/Bg`。
- 不污染数据结构。不要为了一个实验改 `feature_per_frame` 容器语义、`feature_index` 枚举、边缘化参数块或深度生命周期。
- 记账与行为必须物理隔离。任何质量、统计、fate 信息一律存入与 `feature_id` 关联的独立旁路容器，例如 `unordered_map<feature_id, QualityRecord>`；禁止修改 `FeatureObservation`、`feature_per_frame` 等前后端共享观测结构的字段布局。
- “纯记账”的唯一判据是零行为泄漏：删掉所有日志代码后，系统输出必须 bit 级不变。若带回任何记录代码后出现两条里程计分离、轨迹变化或回归指标变化，该改动即不是纯记账，必须退回并按完整行为改动流程重新评估。
- LOG_ONLY 不是免责牌。任何旁路日志若在 VIO 热路径中引入高频同步 IO、锁竞争、内存暴涨或调度抖动，导致绿色 `imu_propagate` 与 `/vins_fusion/odometry` 跳动、分离或轨迹变化，都属于行为泄漏，必须立即关闭、回退或改成限频批量输出。
- 严禁在 `optimization()`、`fastPredictIMU()`、前端 LK 跟踪、残差装配、边缘化等实时热路径里逐 feature/逐 residual 打开文件写日志。sidecar 必须默认关闭；打开时也只能做一次打开批量写入、限频采样、或异步低优先级输出，不能阻塞 estimator 主流程。
- `AGENTS.md` 只描述规则、边界和验收流程，不复制当前 YAML 基线或冻结候选参数。任何具体参数值的唯一权威来源是 `Fast-Drone-250/VINS-Fusion/config/fast_drone_250.yaml` 和对应专题 README。
- 判断一个前端改动是否被禁止，不看它叫什么名字，只看它是否同时满足三条豁免条件：删掉它后输出轨迹 bit 级不变；它不写入任何进入优化的共享结构；它不新增或删除进入 `n_pts` 的特征。任一条件不满足，即属于行为改动，必须走完整 LOG_ONLY、相关性验证、最小行为改动流程。
- 所有实验必须可关闭、可回滚、可回归。新增参数默认应支持 `off/log-only/on` 三档，正式开启前先跑 log-only。

## 标准开发流程

1. 需求澄清
   - 先写清楚用户现象：例如“初始化后 z 飘高”“白墙弱纹理红绿分离”“高动态几秒后跳飞”。
   - 区分问题类型：初始化失败、初始化误通过、前端跟踪质量差、后端约束病态、IMU 时间戳异常、控制端误用不可信 odom。
   - 明确成功标准：不是只看 `end_norm`，还要看最后位置、最大步长、最大速度、是否初始化、是否有 failure log、质量标记是否正确。

2. 现状检查
   - 先查当前源码、YAML、二进制是否一致。
   - 必须记录 `git status --short`、`fast_drone_250.yaml` 参数、`devel/lib/vins/vins_node` 编译时间或 hash。
   - 如果 `HEAD/main` 已污染，不允许用整文件 `git restore` 当“干净回退”。应从真实原版或已确认稳定版做局部恢复。

3. 方案评估
   - 优先参考 VINS-Fusion 原架构、ORB-SLAM3、OKVIS、Kimera、MSCKF/OC-EKF、IMU preintegration 等成熟方案的核心思想。
   - 参考论文只能转成工程约束，不能直接堆概念。每个机制必须落到具体文件、数据结构、开关和回归指标。
   - 先判断是否会破坏索引一致性、边缘化、深度重锚、时间单调、控制端语义。

4. 实现顺序
   - 第一版只做最小可观测改动，优先 log-only。
   - log-only 必须不改变轨迹；如果轨迹变了，说明代码有副作用，不能继续。
   - 质量记录、统计记录、feature fate 记录只能走旁路容器，不能改变主观测结构维度、字段顺序或共享数据结构类型。
   - 旁路日志实现必须先做实时安全设计：默认关闭；不在循环内反复 `open/write/close`；不在优化、IMU propagation、前端跟踪、边缘化热路径做高频同步 IO；需要逐 feature 诊断时按帧批量写入并记录日志行数、写入耗时和开关状态。
   - 真开启前只改一处行为开关，不同时调多个参数。
   - 每次改完必须编译，关键改动必须跑回归。
   - 每条线对应独立实验分支或独立提交。严禁一个提交里同时出现前端和后端的行为改动，严禁把初始化、前端、后端 selector 的行为变化揉进一个提交。

5. 验证与记录
   - 每次带回改动，必须先验证 bit 级零影响：同一个 bag、同一个参数、同一个二进制构建环境下，新版本与原版 baseline 的输出轨迹每一帧位姿数值完全相同。不是只比较 `end_norm`，也不是“看起来差不多”；任何浮点级差异都视为存在行为泄漏。
   - 如果要带回质量记录，必须保持 `FeatureObservation` 本体维度不变，质量信息挂在独立的 `feature_id -> QualityRecord` 旁路表上，确保记录信息永远不会进入优化观测本体。
   - 回归结果必须给出 `summary.csv`、运行目录、配置和版本。
   - 报告必须包含最后位置 `last_x/last_y/last_z`，不能只给 `end_norm`。
- 任何 on/off、baseline/candidate 对比表必须同时列出两边的 `last_x/last_y/last_z`，并给出 `delta_last_x/delta_last_y/delta_last_z`；只列 `end_norm` 或只说改善/变差视为报告不完整。
- 对弱纹理/白墙/高动态问题，要额外看 `max_step`、`max_speed`、视觉观测数、selector 账本、状态日志。
   - 如果打开任何 LOG_ONLY/sidecar 后出现 `imu_propagate` 跳动、odom 绿线跳变、两条轨迹分离、`max_step/max_speed` 异常增大，第一怀疑对象是日志代码造成的实时链路阻塞或 IMU 时间链污染；先关闭该日志重跑确认，不能继续叠加算法补丁。
   - 失败时先回退最近一条实验，不继续叠第二层补丁。

## 目录维护规则

### `/home/jun/springflyer3`

- 项目工作根目录。只放全局维护文档、顶层构建入口和子项目。
- `AGENTS.md` 是最高层维护规范；后续代理或开发者必须先遵循这里的流程。
- 不在根目录堆临时脚本、bag 输出、日志 CSV。临时结果放 `/tmp`，重要结果写入对应 `tools/` 或 README。

### `Fast-Drone-250/VINS-Fusion`

- VINS-Fusion 主算法目录。只接受可解释、可关闭、可回归的算法改动。
- 新增稳定机制必须配套 README，说明目标、参数、作用域、回归结果和回退方式。
- 当前后端稳定版说明见 `README_backend_observation_compression_stable.md`，不要把它和初始化修复混在一起。
- 不保留 `.orig`、`.rej`、临时备份和下载残留文件。

### `Fast-Drone-250/VINS-Fusion/config/fast_drone_250.yaml`

- 这是现场 Fast-Drone-250 的主运行参数。
- 只放真实运行需要的参数。实验参数必须有明确注释和默认安全值。
- 参数命名要能看出模块边界，例如 `backend_selector_*`、`z_drift_*`，不要用含糊的 `enable_fix`。
- 具体参数值以本 YAML 和对应专题 README 为准，`AGENTS.md` 不复制当前数值。
- 改动任何稳定参数时，必须在同一次提交里同步更新对应 README，写清楚为什么改、跑了哪些回归、如何回退；不同步则提交无效。
- 不允许重新加入低光流 hard lock、ZUPT 硬约束、bias anchor freeze 这类会写 estimator 状态的开关，除非单独立项并有明确理论和回归证据。

### `Fast-Drone-250/VINS-Fusion/vins_estimator/src/featureTracker`

- 前端只负责特征跟踪、补点、空间分布和基础 outlier 剔除。
- 不允许主动删除“仍被成功追踪”的长 track；长 track 是最有价值的几何约束。
- `setMask()` 主排序必须保持 `track_cnt` 优先。质量分最多作为同 `track_cnt` 的 tie-breaker，不能取代主键。
- `flow_back: 1` 已经做前后向 LK 检查，不要重复做第二次反向光流。
- 如果做高质量点筛选，正确顺序是：
  - 第 0 档：LOG_ONLY，记录 `feature_id / track_cnt / fb_error / ransac fate / corner response`，不改点集，不改 `FeatureObservation` 或 `feature_per_frame` 字段布局。
  - 第 1 档：只在 `track_cnt` 相同时用质量分 tie-break。
  - 第 2 档：只影响新点候选补充，不碰老点。
- 前端质量账本必须使用独立旁路容器与 `feature_id` 关联；主观测结构一个字节都不动，质量信息不能被优化器误读成观测数据。
- 下列历史机制是违反“旁路记账、主观测结构不动、bit 级零影响”原则的具体案例，仅作警示，不构成穷举：
  - `FeatureObservation` 8 维质量字段强塞进后端；
  - `qualityRight`、`track_quality`、`n_pts_quality`；
  - `addAdaptiveCorners()`、`addGradientFeatures()`；
  - `FRONTEND_ADAPTIVE_*`、`FRONTEND_GRADIENT_*`；
  - 前端直接驱动后端 gate 或状态写入。

### `Fast-Drone-250/VINS-Fusion/vins_estimator/src/estimator`

- 这是最敏感目录。任何改动都必须先确认 `feature_index`、`para_Feature`、marginalization、`slideWindow`、`double2vector/vector2double` 是否一致。
- 后端 selector 只能门控 `AddResidualBlock`，不能改变 `++feature_index` 的枚举集合。
- 验证 `feature_index` 未被污染的机械判据：selector 全关时，`para_Feature` 的打包数量必须与原版 baseline 逐帧相同；任何差异都说明枚举集合被污染，必须回退。
- 不允许 erase 或重排 `feature_per_frame` 来实现筛点。
- `MARGIN_OLD` 中 `start_frame == 0` 的边缘化相关观测强制保留。
- 每条 track 的 anchor、anchor 下一帧、newest 观测强制保留，防止破坏 `removeBackShiftDepth()` 深度重锚。
- 初始化阶段 `solver_flag != NON_LINEAR` 时，后端 selector 必须完全放行，避免把初始化问题和后端筛点混在一起。
- `fastPredictIMU()` 必须保持 IMU 时间单调：`dt < 0` 只能 drop，不允许把 `latest_time` 回拨到更早时间。
- `fastPredictIMU()` 对 `dt <= 0` 必须直接丢弃 propagation 更新；最多更新缓存的原始 IMU 样本，禁止用非正 `dt` 更新 `latest_P/latest_V/latest_Q`。绿色 `imu_propagate` 跳动时必须优先检查 IMU stamp 单调性、`td`、rosbag 播放节奏和日志阻塞。
- `imu_propagate` 可以在初始化前发布 preview，但必须用 covariance 或其他明确语义告诉下游“不可信”。
- 不允许低光流、静止、白墙检测直接写 `Ps/Vs/Bas/Bgs`。如果要做地面/高度/重力相关约束，必须先单独写设计文档，明确可观测性、触发条件、退出条件和误触发后果。

### `Fast-Drone-250/VINS-Fusion/vins_estimator/src/factor`

- 因子目录只放数学残差和 Jacobian 相关代码。
- 新增因子前必须说明观测模型、残差单位、协方差/白化方式、鲁棒核、退化场景。
- 不允许在 factor 内读全局状态或做策略判断；策略应在 estimator 装配 residual block 的地方完成。

### `Fast-Drone-250/VINS-Fusion/vins_estimator/src/initial`

- 初始化模块目标是“少 gate、可解释、可恢复”。
- 静止/低视差窗口应进入 WAIT_MOTION，不计失败、不刷 WARN、不反复清空历史。
- Candidate slide retry 只用于真正候选求解失败，不用于纯静止等待。
- 初始化增强的优先路线是 active candidate scoring：多候选求解并选择最安全结果，而不是继续堆硬拒绝条件。
- 若研究 ORB-SLAM3 风格 inertial-only MAP initializer，必须作为新模块设计，不能把大型架构半截塞进现有初始化路径。

### `Fast-Drone-250/VINS-Fusion/vins_estimator/src/utility` 与 `visualization.cpp`

- 只做发布、可视化和日志语义，不做算法决策。
- `/vins_fusion/odometry` 与 `/vins_fusion/imu_propagate` 的 covariance 语义必须一致、可追溯。
- 下游控制安全优先于 RViz 连续性。blind/degraded 时不能把漂移位置伪装成健康真值。

### `Fast-Drone-250/tools/vins_regression`

- 回归目录是所有 VINS 改动的验收入口。
- `bags.csv` 是固定回归集定义。新增 bag 要标清 `expect_init=1/0/any` 和原因。
- `run_regression.sh` 负责复现运行、保存 metadata、复制 selector/state 日志，不应包含算法逻辑。
- 账本 CSV 的 schema 一旦定版即冻结，新增列只能追加到末尾，不得改动已有列名、顺序、单位或语义。前端质量账本与后端 fate log 必须共享同一个 `feature_id` 定义，保证可以稳定 join。
- 禁止以修改共享观测结构的方式实现 fate 记录；fate 记录本身是前后端关联分析的必要组件，应使用旁路容器重新实现。
- `summarize_run.py` 必须保留：
  - init/pass/failure；
  - odom 与 imu 计数；
  - first/last stamp；
  - first/last position；
  - `end_norm`；
  - `max_rel/max_step/max_speed`；
  - covariance 质量标记。
- `end_norm` 不是 ATE，不能用一个比例把它换算成 ATE。有真值时单独算 ATE/RPE；无真值时只当回归和安全指标。
- `2026-07-01-16-43-33.bag` 的误初始化目前作为已知 fail/哨兵处理：它不用于证明新机制更好，但用于发现新机制是否意外影响初始化路径。
- 每次对比都必须先看 `metadata.txt`，确认源码、YAML、二进制一致。

### `Fast-Drone-250/record` 或 `/home/jun/record`

- bag 是回归资产，不在源码提交里复制大文件。
- 每个新 bag 要记录现场条件：纹理、白墙、地面、光照、高动态、是否静止、是否应初始化。
- 静止负样本不初始化是正确行为，不应被 WARN retry 污染。
- `2026-07-01-16-43-33.bag` 的误初始化属于当前已知原版遗留问题，回归中作为哨兵记录，不应被无关改动改变其行为。若某个新改动使它的初始化行为变化，无论变好还是变坏，都说明该改动影响了初始化路径，必须单独归因。

### `Fast-Drone-250/1111`

- 视为历史/备份/对照目录，不作为当前开发主线。
- 不允许从这里盲目拷贝文件覆盖当前 `VINS-Fusion`。
- 如果必须引用，只能用 diff 提取明确的小块，并在提交说明中写清楚来源和原因。

### 黄金基线与回退锚点

- “真实原版”必须有不可变锚点：上游仓库的明确 commit/tag，或经确认的只读干净备份路径。
- 在黄金基线锚点写入本文件或对应 README 之前，禁止把任何来历不明的备份目录当作干净原版。
- 局部恢复只能以黄金基线锚点为参照，不得以污染的 `HEAD/main`、`1111` 或临时下载目录作为权威。

### `vision_opencv`

- ROS/OpenCV 依赖包，不属于 VINS 算法实验区域。
- 除非明确是依赖编译问题，不要修改。

## 弱纹理、白墙、高动态的工程路线

目标不是让 VINS 在无信息环境里“假装正常”，而是在退化时延迟发散、暴露不可信、保护控制端，并在信息恢复时平稳回到可用状态。

### 弱纹理/白墙

- 首先记录前端事实：特征数、track length 分布、FB error、RANSAC 保留率、空间覆盖率、观测数谷底。
- 前端优先做 LOG_ONLY 质量账本，验证 cheap signal 是否能预测后端命运。
- 不主动删老点；只允许在补新点、同寿命冲突排序、后端冗余观测压缩中做保守选择。
- 后端弱段保护靠 `total_obs` 门限和 marg-safe 白名单，不在弱段裁剪视觉约束。

### 高动态

- 优先检查 IMU 时间戳单调、`dt > 0.05` gap、相机曝光/运动模糊、LK 跟踪失败率。
- 不用严格 violent-motion veto 作为正常飞行硬拒绝；极端 IMU 只能作为异常 prefilter。
- 高动态下应关注 `max_step/max_speed`、相邻 odom 跳变、`imu_propagate` covariance，而不是只看最终漂移。

### 初始化后飘飞

- 先查初始化后的坐标、速度、Ba/Bg norm、重力方向、首尾位置和第一秒轨迹。
- 初始化误通过比初始化失败更危险；负样本误通过必须单独处理。
- 不能用运行期 low-flow lock 或地面约束去掩盖坏初始化。

### z 轴漂移与地面/高度约束

- z 漂移先诊断，不先上约束。必须区分重力对齐错误、尺度错误、Ba 污染、视觉弱约束、外参/时间偏差、控制端反馈问题。
- 地面约束只能在传感器和场景语义明确时使用，例如明确知道起飞前地面高度、飞行器未离地、或有可靠 range/高度观测。
- 不允许无条件把 z 拉回 0。飞行中地面不等于视觉世界原点，错误地面约束会比漂移更危险。
- 若未来引入高度/range/地面因子，必须是独立因子、带 covariance、带触发和退出条件，并先 log-only。

## 回归最低标准

每次算法改动至少执行全量三次回归；每个 bag 默认必须跑 `RUNS=3`，禁止只用单次 `RUNS=1` 作为验收结论：

```bash
cd /home/jun/springflyer3/Fast-Drone-250
source /opt/ros/noetic/setup.bash
catkin_make --source VINS-Fusion --pkg vins -DCMAKE_BUILD_TYPE=Release
RUNS=3 tools/vins_regression/run_regression.sh
```

`RUNS=1` 只能用于烟测、脚本调试或确认程序能跑通，不得写成稳定性结论，不得用于决定上线/回退。

如出现单包尾部风险，必须对该包单独 `RUNS=5` 复测。若 5 次中任意一次越过安全红线，应优先选择更保守配置。

## 提交与文档标准

- 每个稳定机制必须有中文 README，写清楚：目标、作用域、参数、为什么不是乱堆、回归结果、已知问题、回退方式。
- 提交说明必须包含：
  - 改了哪条线；
  - 没碰哪些模块；
  - 跑了哪些测试；
  - 如果失败怎么关掉。
- 不提交 bag、临时 CSV、大日志、`.orig`、`.rej`、编译产物。
- 没有回归证据的实验只能留在工作区或实验分支，不标成稳定版。

## 明确禁止

- 禁止把低光流、白墙、静止检测直接变成 estimator 状态写入。
- 禁止在前端主动删除可靠长 track。
- 禁止改变 `feature_index` 枚举集合来实现后端筛点。
- 禁止在边缘化相关观测上做 selector 裁剪。
- 禁止一边改初始化一边改后端 selector，再用一个回归结果解释全部变化。
- 禁止只凭 RViz 判断跳变是否消失，必须看数值差分。
- 禁止只报 `end_norm`，必须同时报最后位置和安全指标。
- 禁止为了“看起来稳”把不可信 odom 当健康输出给控制器。
- 禁止把质量、统计、fate 字段塞进 `FeatureObservation`、`feature_per_frame` 或任何会进入优化的主观测结构。
- 禁止把未通过 bit 级零影响验证的日志代码标成 LOG_ONLY。
- 禁止在 `AGENTS.md` 中复制当前 YAML 基线或冻结参数数值；数值必须查 YAML 和专题 README。

## 当前稳定基线

- 后端稳定候选：`Backend Observation Compression Stable`，见 `Fast-Drone-250/VINS-Fusion/README_backend_observation_compression_stable.md`。
- 初始化策略说明：见 `Fast-Drone-250/tools/vins_regression/INIT_STRATEGY.md`。
- 回归规范：见 `Fast-Drone-250/tools/vins_regression/README.md`。

后续所有开发都应从这三个文档出发。若实际源码、YAML、二进制和文档不一致，先修一致性，再谈算法结论。
