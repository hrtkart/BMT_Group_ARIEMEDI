# tracker_node.cpp 逐行详解（面向 ROS2/C++ 初学者）

本文档详细讲解 `src/tracker_node.cpp` 的结构、参数、函数、数据流与外部 API 调用，目标读者是对 ROS2 和 C++ 都不熟悉的同学。

## 1. 这个节点在做什么

`TrackerNode` 是一个 ROS2 节点，核心职责：

1. 连接 Ariemedi/ARMD 追踪设备。
2. 加载工具模型（`.arom` 文件）。
3. 启动追踪（可选启用成像）。
4. 以 120Hz 的定时器循环拉取最新追踪数据。
5. 发布自定义 ROS2 话题 `/ARMDpos`。
6. 同时广播 TF 坐标变换（`tracker_base -> tool`）。

## 2. 文件总体结构

- 头文件与依赖（1-23 行）
- 类定义与构造/析构（25-49 行）
- 设备初始化 `initDevice()`（51-154 行）
- 停止逻辑 `stopTracking()`（156-179 行）
- 周期循环 `step()`（181-199 行）
- 数据处理 `processToolData()`（201-357 行）
- 成员变量（359-374 行）
- 程序入口 `main()`（377-384 行）

## 3. 运行流程（先看这个最容易理解）

1. `main()` 初始化 ROS2。
2. 创建 `TrackerNode` 对象。
3. 构造函数里声明参数、创建发布器和 TF 广播器，并调用 `initDevice()`。
4. `initDevice()` 扫描/连接设备、加载工具、开启追踪、启动 120Hz 定时器。
5. 定时器每次触发 `step()`：
   - 检查连接是否中断
   - 调用 SDK 更新缓存
   - 调用 `processToolData()` 处理并发布
6. 程序退出时析构函数调用 `stopTracking()`，停止追踪并断开连接。

## 4. ROS2 参数、话题、坐标系

### 4.1 参数（Parameter）

代码中声明了两个参数：

1. `hostname`（字符串，默认 `""`）
   - 作用：设备主机名或 IP。
   - 如果留空，会自动扫描网络设备。

2. `enable_imaging`（布尔，默认 `false`）
   - 作用：是否开启成像功能。
   - `false` 时通常追踪吞吐更高。

### 4.2 发布话题

- 话题名：`/ARMDpos`
- 消息类型：`ariemedi_tracker::msg::ToolTrackingData`
- 队列深度：`10`

### 4.3 TF 广播

- 父坐标系：`tracker_base`
- 子坐标系：工具名（若为空用 `toolcali0`）
- 注意单位：SDK 平移是毫米，发布 TF 时转换成米（除以 1000）。

## 5. 逐行细致讲解（按代码顺序）

说明：空行用于可读性，一般不承载逻辑；以下重点解释有效代码行。

### 5.1 头文件与命名空间（1-23 行）

- 1 行：`rclcpp/rclcpp.hpp`
  - ROS2 C++ 客户端库核心头文件，提供节点、日志、参数、定时器等。
- 2 行：`tf2_ros/transform_broadcaster.h`
  - 提供 TF 广播器，用于发布坐标变换。
- 3 行：`geometry_msgs/msg/transform_stamped.hpp`
  - TF 消息类型，包含时间戳、父子坐标系、平移和旋转。
- 4 行：`ament_index_cpp/get_package_share_directory.hpp`
  - 根据包名查找安装后的 `share` 目录。
- 7-10 行：自定义消息头文件
  - 来自当前包 `ariemedi_tracker/msg/*`。
- 12 行：`ARMDCombinedAPI.h`
  - 设备厂商 SDK 的核心 API 封装类。
- 13 行：`DeviceScan.h`
  - SDK 设备扫描工具类。
- 15-21 行：C++ 标准库（字符串、容器、线程、时间、智能指针、数学）。
- 23 行：`using namespace std::chrono_literals;`
  - 允许直接写 `500ms` 这种时间字面量。

### 5.2 类声明与构造函数（25-42 行）

- 25 行：定义 `TrackerNode`，继承 `rclcpp::Node`。
- 28-30 行：构造函数初始化列表
  - `Node("ariemedi_tracker")`：节点名。
  - `tracker_(new ARMDCombinedAPI())`：创建 SDK 对象。
  - 若干状态变量初始化为 0/false，避免未定义行为。
- 32 行：声明参数 `hostname`。
- 33 行：声明参数 `enable_imaging`。
- 34 行：读取 `enable_imaging` 参数到成员变量。
- 36 行：创建 TF 广播器。
- 39 行：创建话题发布器 `/ARMDpos`。
- 41 行：调用 `initDevice()` 进入设备初始化流程。

### 5.3 析构函数（44-48 行）

- 46 行：先调用 `stopTracking()`，优雅停止设备侧流程。
- 47 行：释放 `tracker_` 指针。

### 5.4 设备初始化 `initDevice()`（51-154 行）

#### A) 获取 hostname 或扫描设备（53-78 行）

- 53 行：读取参数 `hostname`。
- 55 行：如果为空字符串，进入自动扫描分支。
- 56 行：日志提示开始扫描。
- 57 行：创建 `DeviceScan` 对象。
- 59 行：最多扫描 10 次。
- 60-65 行：循环扫描
  - `updateDeviceInfo()` 更新设备列表。
  - `sleep_for(500ms)` 两次扫描间隔 0.5 秒。
  - `scan_attempts--` 防止无限循环。
- 67-71 行：仍为空则报错并关闭 ROS2。
- 73-77 行：取第一个设备作为连接目标，并打印主机名和 IP。

#### B) 连接设备与错误诊断（80-123 行）

- 80-81 行：打印连接目标和调试信息。
- 84 行：`tracker_->connect(hostname, true, true, true)`
  - 第一个参数是目标主机。
  - 后三个布尔参数由 SDK 定义（此处主要用于打开更详细输出）。
- 86 行：若返回非 0，表示连接失败。
- 87-120 行：按错误码输出更友好的诊断建议：
  - `-1`：拒绝连接。
  - `-2`：连接超时。
  - `-3/-4`：协议/通信异常。
  - `>0`：设备特定错误码。
- 121-122 行：失败后关闭 ROS2 并返回。

#### C) 连接成功后的追踪启动（125-154 行）

- 125 行：输出连接成功日志。
- 128 行：通过包名定位 `share` 目录。
- 129 行：拼接工具文件路径 `tool/toolcali0.arom`。
- 131-133 行：将路径放入 `vector<string>` 并调用
  `loadPassiveToolAROM()`。
- 135 行：设置追踪数据传输类型为被动模式 `Passive`。
- 136 行：启动追踪 `startTracking()`。
- 137-142 行：按参数决定是否 `startImaging()`。
- 144 行：`tracking_started_ = true`，表示循环可运行。
- 145-146 行：记录起始时间和上一帧时间。
- 149-151 行：创建 120Hz 定时器，回调 `step()`。
- 153 行：输出定时器启动日志。

### 5.5 停止函数 `stopTracking()`（156-179 行）

- 158 行：先把状态改成未启动。
- 159-161 行：取消定时器，避免继续回调。
- 163 行：确保 `tracker_` 非空。
- 165-168 行：如果连接已中断，直接 `disconnect()`。
- 171 行：正常情况下先 `stopTracking()`。
- 172-174 行：若开过成像则执行 `stopImaging()`。
- 175 行：最后断开连接。
- 178 行：打印停止完成日志。

### 5.6 定时回调 `step()`（181-199 行）

- 183-185 行：如果没处于追踪状态，直接返回。
- 187-195 行：若连接中断，报错、停标志、停定时器并返回。
- 197 行：调用 SDK `trackingUpdate()` 刷新内部缓存。
- 198 行：调用 `processToolData()` 读取并发布。

### 5.7 数据处理 `processToolData()`（201-357 行）

#### A) 拉取原始数据（203-207 行）

- 203 行：`getAllMarkers()` 获取所有标记点。
- 204 行：`getTrackingData(allMarkerData)` 解析为工具数据。
- 205-206 行：无标记时每 2 秒节流打印一次日志。

#### B) 计算时间与 FPS（210-214 行）

- `current_time`：当前时刻。
- `frame_time_diff`：距上一次有效帧的时间差。
- `total_time`：运行总时长。
- `fps`：平均 FPS。
- `instant_fps`：瞬时 FPS（当前帧间隔倒数）。

#### C) 遍历每个工具并做去重（218-251 行）

- 220 行：遍历每个工具数据 `data`。
- 222 行：只处理 `matchStatus == true` 的有效匹配。
- 227-230 行：首选去重策略
  - 若 `timespec` 与上一帧相同，认为是重复帧，`continue`。
- 233-240 行：备选去重策略
  - 当没有 `timespec` 时，用位置差阈值 + 时间阈值判重。
  - 三轴变化小于 `0.01` 且间隔小于 `5ms`，视为重复。
- 243-251 行：确认有效后计数并刷新“上一帧缓存”。

#### D) 打日志、组织 ROS 消息（253-331 行）

- 253-263 行：打印每帧关键信息（位置、时差、FPS、时间戳）。
- 265 行：创建 `ToolTrackingData` 消息。
- 267-268 行：填充 ROS 标准头（时间戳 + 坐标系）。
- 270-276 行：填充工具基础状态字段。
- 278-280 行：填充方向向量 `dir`。
- 282-292 行：填充位姿与欧拉角。
- 294-300 行：把 4x4 矩阵展平成长度 16 的一维数组。
- 302-305 行：填充平面参数。
- 307-317 行：把 `plane.markers` 转换为 ROS 消息数组。
- 319-329 行：把 `points` 转换为 ROS 消息数组。
- 331 行：发布消息到 `/ARMDpos`。

#### E) 发送 TF（333-348 行）

- 334 行：创建 `TransformStamped`。
- 335 行：复用同一时间戳与父坐标系。
- 336 行：子坐标系优先使用工具名，否则 `toolcali0`。
- 339-341 行：毫米转米后写入平移。
- 343-346 行：写入四元数旋转。
- 348 行：调用 `sendTransform()` 广播。

#### F) 更新时间基准（353-356 行）

- 353 行：只有本轮出现有效追踪数据才更新时间。
- 355 行：刷新 `last_frame_time_`。

### 5.8 成员变量（359-374 行）

- `tracker_`：SDK 核心对象指针。
- `tracking_started_`：追踪状态开关。
- `enable_imaging_`：是否启用成像。
- `timer_`：120Hz 定时器句柄。
- `tf_broadcaster_`：TF 广播器。
- `armd_pub_`：`/ARMDpos` 发布器。
- `frame_count_`：累计有效帧数。
- `last_frame_time_`：最近有效帧时间。
- `start_time_`：节点开始追踪的时间。
- `last_tx_/last_ty_/last_tz_`：去重用上一帧平移缓存。
- `last_timespec_`：去重用上一帧时间戳缓存。
- `has_last_data_`：是否已有上一帧缓存。

### 5.9 程序入口 `main()`（377-384 行）

- 379 行：`rclcpp::init` 初始化 ROS2 运行时。
- 380 行：创建节点对象。
- 381 行：`rclcpp::spin(node)` 进入事件循环。
- 382 行：退出前关闭 ROS2。
- 383 行：返回 0 表示正常结束。

## 6. 外部 API 对照讲解

> 这里把你在代码里看到的“调用点”翻译成“它到底做什么”。

### 6.1 ROS2 (`rclcpp`) 常用 API

1. `declare_parameter<T>(name, default)`
   - 声明一个可在启动时覆盖的参数。

2. `get_parameter(name).as_xxx()`
   - 读取参数并转成目标类型。

3. `create_publisher<Msg>(topic, queue)`
   - 创建发布器。

4. `create_wall_timer(period, callback)`
   - 创建按“系统墙钟”触发的周期任务。

5. `get_clock()->now().seconds()`
   - 获取当前时间（秒）。

6. `RCLCPP_INFO / RCLCPP_ERROR`
   - 日志输出。

7. `RCLCPP_INFO_THROTTLE(logger, clock, ms, msg)`
   - 节流日志，避免刷屏。

8. `rclcpp::init / spin / shutdown`
   - 生命周期入口、主循环、清理退出。

### 6.2 TF2 (`tf2_ros`)

1. `tf2_ros::TransformBroadcaster`
   - 负责发布坐标变换。
2. `sendTransform(TransformStamped)`
   - 把一次位姿更新发送给 TF 系统。

### 6.3 ament 索引 API

1. `ament_index_cpp::get_package_share_directory("ariemedi_tracker")`
   - 定位包的安装共享目录，便于读取资源文件（如 `.arom`）。

### 6.4 ARMD SDK（来自 `ARMDCombinedAPI` / `DeviceScan`）

1. `DeviceScan::updateDeviceInfo()`
   - 扫描网络中的设备并更新缓存。
2. `DeviceScan::getDeviceInfo()`
   - 读取扫描结果（主机名/IP 等）。
3. `ARMDCombinedAPI::connect(host, ...)`
   - 连接设备，返回错误码。
4. `loadPassiveToolAROM(paths)`
   - 加载工具模型文件。
5. `setTrackingDataTransmissionType(TransmissionType::Passive)`
   - 设置追踪数据传输模式。
6. `startTracking() / stopTracking()`
   - 启停追踪。
7. `startImaging() / stopImaging()`
   - 启停成像。
8. `trackingUpdate()`
   - 刷新当前追踪数据缓存。
9. `getAllMarkers()`
   - 获取当前所有 marker 点。
10. `getTrackingData(markers)`
    - 将 marker 点解析为工具追踪数据。
11. `getConnectionStatus()` / `disconnect()`
    - 查询连接状态、断开连接。

## 7. 关键设计点（初学者必须知道）

1. 定时器 120Hz 不等于设备一定 120Hz 输出。
   - 所以代码做了“去重”，防止重复发布同一帧。

2. SDK 单位与 ROS 单位不同。
   - SDK 平移是 mm，TF 里必须用 m。

3. 程序稳定性来自“失败即退出 + 明确日志”。
   - 连接失败直接 shutdown，避免节点以无效状态运行。

4. 数据发布和 TF 广播同时进行。
   - 下游既可订阅数值消息，也可在 TF 树看位姿关系。

## 8. 建议的上手步骤

1. 先只关注 `main()` + 构造函数 + `initDevice()`，理解“如何跑起来”。
2. 再看 `step()`，理解“循环在哪里”。
3. 最后细看 `processToolData()`，理解“数据如何从 SDK 变成 ROS 消息与 TF”。
4. 用 `ros2 topic echo /ARMDpos` 与 `rviz2` 对照观察消息和 TF。

## 9. 可改进方向（后续优化）

1. 将 `tracker_` 改为 `std::unique_ptr<ARMDCombinedAPI>`，避免手动 `new/delete`。
2. 将错误码解释提取成独立函数，减少 `initDevice()` 复杂度。
3. 将日志级别调成可配置（比如参数控制是否打印每帧日志）。
4. 把去重阈值（0.01、5ms）参数化，便于现场调优。
