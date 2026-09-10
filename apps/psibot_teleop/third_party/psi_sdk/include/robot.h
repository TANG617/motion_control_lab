#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifndef PSIBOT_ROBOT_SDK_API
#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(PSIBOT_ROBOT_SDK_BUILD_SHARED)
#define PSIBOT_ROBOT_SDK_API __declspec(dllexport)
#elif defined(PSIBOT_ROBOT_SDK_USE_SHARED)
#define PSIBOT_ROBOT_SDK_API __declspec(dllimport)
#else
#define PSIBOT_ROBOT_SDK_API
#endif
#else
#if defined(PSIBOT_ROBOT_SDK_BUILD_SHARED)
#define PSIBOT_ROBOT_SDK_API __attribute__((visibility("default")))
#else
#define PSIBOT_ROBOT_SDK_API
#endif
#endif
#endif

#include "robot_type.h"

namespace psibot_robot_sdk {

class RobotImpl;

/// PSI 人形机器人控制 SDK 主接口。
///
/// 返回 `int32_t` 的接口均使用 `RobotErrorCode`：`0` 表示成功，负值表示错误。
/// 位姿统一为 `PoseArray`：`[x, y, z, qx, qy, qz, qw]`（米 + 四元数 xyzw）。
/// 典型流程：构造并等待连接 → `setProfile` → `setEnable(true)` → 运动/查询。
class PSIBOT_ROBOT_SDK_API Robot {
 public:
  // ----- 生命周期 -----
  /// 构造机器人客户端并开始连接。
  /// @param ip       运控地址，格式 `"host:port"`，默认 `"192.168.1.100:6060"`
  /// @param username 认证信息，格式 `"user:pass"`，默认 `"admin:admin"`
  /// @param reserve  预留参数，当前未使用
  explicit Robot(const std::string &ip = "192.168.1.100:6060",
                 const std::string &username = "admin:admin",
                 const std::string &reserve = "");
  ~Robot();

  Robot(const Robot &) = delete;
  Robot &operator=(const Robot &) = delete;

  /// 查询是否已与当前控制后端建立连接。
  /// @return `true` 已连接；断线或重连过程中为 `false`
  bool isConnected() const;

  /// 主动断开控制后端连接并释放会话资源。
  /// @note Inex 底层依赖 Zenoh/Tokio：进程退出阶段（含 Python 解释器卸载）再调
  ///       Zenoh 会 abort。请在业务结束、进程仍存活时调用本接口；Python 示例建议
  ///       `robot.disconnect()` 后 `os._exit(code)`（对齐 Inex SDK 官方示例）。
  void disconnect();

  /// 注册运行事件回调（连接成功 / 断开等）。传空则取消。
  /// @note 在 SDK 内部线程调用；当前事件的 `data` 为 `nullptr`。
  void setEventCallback(RobotEventCallback cb);

  /// 切换运动控制后端。
  /// @param backend `0`=Simulation（预留），`1`=Inex（默认），`2`=Psi
  /// @return `SUCCESS`；非法取值返回 `INVALID_ARGUMENT`
  /// @note 构造时已创建各后端实例，本接口仅切换后续 API 的转发目标。
  int32_t setControlBackend(int32_t backend);

  /// 查询当前控制后端。
  /// @return `0`=Simulation，`1`=Inex，`2`=Psi
  int32_t getControlBackend() const;

  // ----- 机型元信息 -----
  /// 查询机器人基本信息（型号、SDK 版本、序列号等）。
  /// @param[out] info 基本信息
  /// @return `SUCCESS` 或错误码
  int32_t getRobotInfo(RobotInfo &info) const;

  /// 查询关节限位与末端笛卡尔运动限制。
  /// @param[out] limits 限制参数；关节顺序与 `getJointStates` 对齐
  /// @return `SUCCESS` 或错误码
  int32_t getRobotLimits(RobotLimits &limits) const;

  // ----- 订阅 -----
  /// 开始订阅指定类型数据。
  /// @param data_types         数据类型列表，见 `RobotDataType`
  /// @param update_periods_sec 与 `data_types` 一一对应的上报周期（s）；
  ///        空或元素 ≤0 时默认 `0.002`。彩色图像通道忽略本参数。
  /// @param cache_sizes        与 `data_types` 一一对应的缓存深度；空则默认 1
  /// @note 当前仅 `*_COLOR_IMAGE` 生效；其余类型会被忽略（状态读不依赖订阅）。
  void startSubscribe(const std::vector<RobotDataType> &data_types,
                      const std::vector<double> &update_periods_sec = {},
                      const std::vector<int32_t> &cache_sizes = {});

  /// 停止订阅指定类型数据。
  /// @param data_types 待停止的数据类型列表
  /// @note 当前仅对已订阅的 `*_COLOR_IMAGE` 生效。
  void stopSubscribe(const std::vector<RobotDataType> &data_types);

  // ----- 上电与运动档位 -----
  /// 设置机器人会话分区模式（单臂 / 双臂 / 全身等）。
  /// @param mode 目标模式，见 `RobotMode`
  /// @return `SUCCESS` 或错误码
  /// @note Inex 连接成功后默认自动设为 `WholeBody`（重连后会再次设置）。
  int32_t setRobotMode(RobotMode mode);

  /// 使能或下电指定部件。
  /// @param component 目标部件：`All` / `LeftArm` / `RightArm` / `Waist` /
  ///                  `Head`；`All` 表示整机
  /// @param enable    `true` 上电，`false` 下电
  /// @return `SUCCESS`；超时返回 `TIMEOUT`
  /// @note 成功时阻塞直至状态与目标一致。臂/头运动前通常需先 `setProfile`。
  int32_t setEnable(RobotComponent component, bool enable);

  /// 清除伺服/会话故障（热复位，转发 `robot/hot_reset`）。
  /// @param component 目标部件：`All` / `LeftArm` / `RightArm` / `Waist` /
  ///                  `Head`；`All` 表示整机
  /// @return `SUCCESS` 或错误码（如 `REJECTED` / `NOT_SUPPORTED`）
  /// @note 用于驱动器故障锁存等；不替代 `setEnable(false)`。急停未解除时可能失败。
  /// @note Inex：`state==ERROR` 且单纯下电无法恢复时，可先 `clearError` 再上下电。
  int32_t clearError(RobotComponent component = RobotComponent::All);

  /// 查询指定部件是否已使能。
  /// @param component     目标部件；`All` 表示各可上电部件均已上电
  /// @param[out] enabled  使能状态
  /// @return `SUCCESS` 或错误码
  int32_t getEnabled(RobotComponent component, bool &enabled) const;

  /// 设置运动控制档位（位置 / 阻抗 / 拖动等）。
  /// @param component 目标部件：`All` / `LeftArm` / `RightArm` / `Waist` /
  ///                  `Head`
  /// @param profile   目标档位，见 `ArmProfile`
  /// @return `SUCCESS` 或错误码
  /// @pre 通常须在下电空闲时调用；已上电改档可能失败。
  /// @note 推荐顺序：`setProfile` → `setEnable(true)` → 运动接口。
  /// @note 若当前档已是目标档，直接返回 `SUCCESS`（幂等）。
  int32_t setProfile(RobotComponent component, ArmProfile profile);

  /// 查询当前运动控制档位。
  /// @param component     目标部件；`All` 表示各部件档位一致时的公共值
  /// @param[out] profile  当前档位
  /// @return `SUCCESS`；`All` 且各部件不一致时返回 `INVALID_ARGUMENT`
  int32_t getProfile(RobotComponent component, ArmProfile &profile) const;

  // ----- 状态查询 -----
  /// 查询整机关节状态。
  /// @param[out] names          关节名（与其余向量同序）
  /// @param[out] positions      关节位置 (rad)
  /// @param[out] velocities     关节速度 (rad/s)
  /// @param[out] accelerations  关节加速度 (rad/s²)；暂无数据时可能为空或零
  /// @param[out] torques        关节力矩 (N·m)；暂无数据时可能为空或零
  /// @return `SUCCESS`；无数据时返回 `NO_DATA`；未连接返回 `NOT_CONNECTED`
  /// @note 不依赖 `startSubscribe`（当前订阅仅支持图像）。
  int32_t getJointStates(std::vector<std::string> &names,
                         std::vector<double> &positions,
                         std::vector<double> &velocities,
                         std::vector<double> &accelerations,
                         std::vector<double> &torques) const;

  /// 一次获取关节 / 双臂末端(Base+Arm) / 双臂六维力 / 双手角。
  /// @param[out] obs 观测快照；无效通道对应 `has_*=false`，字段保持默认/空
  /// @return `SUCCESS`：至少 joints 或任一侧 Base 位姿有效；
  ///         `NOT_CONNECTED`；全无有效通道时 `NO_DATA`
  /// @note 策略 B（尽力）：力/手/Arm 缺失不导致失败。无 pose_frame 参数；
  ///       Arm 系位姿当前预留可空。
  int32_t getObservation(RobotObservation &obs) const;

  /// 查询单臂末端位姿。
  /// @param component   目标臂，仅 `LeftArm` / `RightArm`
  /// @param[out] pose   末端位姿 `[x,y,z,qx,qy,qz,qw]`
  /// @param pose_frame  参考坐标系，默认 `PoseFrame::Base`
  /// @return `SUCCESS` 或错误码
  int32_t getEndPose(RobotComponent component, PoseArray &pose,
                     PoseFrame pose_frame = PoseFrame::Base) const;

  /// 查询左右臂末端位姿。
  /// @param[out] left_pose   左臂末端位姿
  /// @param[out] right_pose  右臂末端位姿
  /// @param pose_frame       参考坐标系，默认 `PoseFrame::Base`
  /// @return `SUCCESS` 或错误码
  int32_t getEndPose(PoseArray &left_pose, PoseArray &right_pose,
                     PoseFrame pose_frame = PoseFrame::Base) const;

  // ----- 运动控制 -----
  /// 笛卡尔空间流式伺服（单臂，高频下发目标位姿）。
  /// @param component 目标臂，仅 `LeftArm` / `RightArm`
  /// @param pose      目标末端位姿 `[x,y,z,qx,qy,qz,qw]`
  /// @param options   运动参数；`period_s` 为流式周期（s），≤0 时使用默认
  /// @return `SUCCESS` 或错误码
  /// @note 非阻塞单帧接口；调用方按控制周期持续调用。
  ///       Inex 路径须先 `startServoMode(..., Cartesian)`。
  int32_t moveEndPose(RobotComponent component, const PoseArray &pose,
                      const MoveCartesianOptions &options = {});

  /// 笛卡尔空间流式伺服（双臂各一帧）。
  /// @param left_pose   左臂目标位姿
  /// @param right_pose  右臂目标位姿
  /// @param options     运动参数；`period_s` 为流式周期（s）
  /// @return `SUCCESS` 或错误码
  /// @note Inex 路径两侧均须先 `startServoMode(..., Cartesian)`。
  int32_t moveEndPose(const PoseArray &left_pose, const PoseArray &right_pose,
                      const MoveCartesianOptions &options = {});

  /// 笛卡尔空间直线运动（单臂 PTP）。
  /// @param component 目标臂，仅 `LeftArm` / `RightArm`
  /// @param pose      目标末端位姿 `[x,y,z,qx,qy,qz,qw]`
  /// @param sync      `true` 等待到位（平移/姿态 ≤ options 容差）；
  ///                  `false` 下发后立即返回
  /// @param options   速度、坐标系与 sync 容差等参数
  /// @return `SUCCESS` 或错误码
  int32_t moveL(RobotComponent component, const PoseArray &pose,
                bool sync = true, const MoveCartesianOptions &options = {});

  /// 笛卡尔空间直线运动（双臂可选）。
  /// @param left_pose   左臂目标；空表示该侧不运动
  /// @param right_pose  右臂目标；空表示该侧不运动
  /// @param sync        `true` 等待到位；`false` 下发后立即返回
  /// @param options     速度、坐标系与 sync 容差等参数
  /// @return `SUCCESS` 或错误码
  int32_t moveL(const std::optional<PoseArray> &left_pose,
                const std::optional<PoseArray> &right_pose, bool sync = true,
                const MoveCartesianOptions &options = {});

  /// 切入流式伺服模式（SERVO_J / SERVO_L）。
  /// @param component  `MotionSpace::Joint`：`All` / `LeftArm` / `RightArm` /
  ///                   `Waist` / `Head`；`Cartesian`：仅 `LeftArm` / `RightArm`
  /// @param space      `MotionSpace::Joint` → SERVO_J；`Cartesian` → SERVO_L
  /// @return `SUCCESS` 或错误码
  /// @note 流式前调用一次；之后按控制周期持续调用 `moveServoJ`（关节）或
  ///       `moveEndPose`（笛卡尔）。`All` 仅用于 Joint 切档（Inex
  ///       `PLAN_PART_ALL`），流式命令仍须按 part 下发。
  int32_t startServoMode(RobotComponent component, MotionSpace space);

  /// 结束流式伺服并平滑制动（退出 SERVO_J / SERVO_L）。
  /// @param component    制动范围：`All` / `LeftArm` / `RightArm` / `Waist` /
  ///                     `Head`（与 `startServoMode` 的 part 对齐）
  /// @param timeout_sec  等待静止超时 (s)；`>0` 等待可再改档/发 PTP，超时
  ///                     `TIMEOUT`；`<=0` 仅下发 halt 即返回
  /// @return `SUCCESS` / `TIMEOUT` 或其它错误码
  /// @note Inex：`robot/halt`（`brake_level=0`）。典型顺序：
  ///       `startServoMode` → 流式 → **`stopServoMode`** → `setProfile` /
  ///       `moveJ` / `moveL`。不替代 `stopImpedance`。
  int32_t stopServoMode(RobotComponent component = RobotComponent::All,
                        double timeout_sec = kDefaultStopServoTimeoutSec);

  /// 关节空间流式伺服（按部件高频下发目标关节角）。
  /// @param component  `LeftArm` / `RightArm`（7）/ `Waist`（4）/ `Head`（2）
  /// @param positions  目标关节角 (rad)，长度须与部件自由度一致；腰为公开序
  ///                   torso_yaw→torso_pitch→knee→ankle（下发前转 wire）
  /// @param options    运动参数；`period_s` 为流式周期（s），≤0 时使用默认
  /// @return `SUCCESS` 或错误码
  /// @note 非阻塞单帧接口；调用方按控制周期持续调用。
  ///       Inex 路径须先 `startServoMode(..., Joint)`。不支持 `All`。
  int32_t moveServoJ(RobotComponent component,
                     const std::vector<double> &positions,
                     const MoveJointOptions &options = {});

  /// 关节空间点到点运动。
  /// @param component  目标部件：`All`（全身 20
  /// 轴）、`LeftArm`/`RightArm`（7）、
  ///                   `Waist`（4）、`Head`（2）
  /// @param positions  目标关节角 (rad)，长度须与部件自由度一致；
  ///                   **始终按公开序填写**（与 `getJointStates` 一致）：
  ///                   All = head(2)+waist(4 public)+left(7)+right(7)；
  ///                   Head = head_yaw→head_pitch（与 Inex 一致，无需重排）；
  ///                   Waist = torso_yaw→torso_pitch→knee→ankle
  ///                   （SDK 仅在发往 Inex 前将腰部重排为线序）。
  /// @param sync       `true` 等待到位（关节误差 ≤
  /// options.position_tolerance_rad）；
  ///                   `false` 下发后立即返回
  /// @param options    速度与 sync 容差等参数
  /// @return `SUCCESS` 或错误码；目标越软限位返回 `OUT_OF_LIMIT`（不下发）
  /// @note connect 成功后缓存软限位；`moveServoJ` /
  ///       `moveJointWaypointPath` 同样预检。全身建议 `RobotMode::WholeBody`。
  int32_t moveJ(RobotComponent component, const std::vector<double> &positions,
                bool sync = true, const MoveJointOptions &options = {});

  /// 切入多段路点 plan 模式（WAYPOINT_JOINT / WAYPOINT_CART）。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param space      `Joint` 或 `Cartesian`
  /// @return `SUCCESS` 或错误码
  /// @note 仅 `robot/set_mode`；不 load、不 start。完整路点运动请用
  ///       `moveJointWaypointPath` / `moveCartesianWaypointPath`。
  int32_t startWaypointPath(RobotComponent component, MotionSpace space);

  /// 关节空间多段路点运动。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param waypoints  路点序列；每个路点为长度 7 的关节角 (rad)
  /// @param sync       `true` 等待路径执行结束；`false` 启动后立即返回
  /// @param options    速度等参数
  /// @param loop       `true` 循环执行路点序列
  /// @return `SUCCESS` 或错误码
  int32_t moveJointWaypointPath(
      RobotComponent component,
      const std::vector<std::vector<double>> &waypoints, bool sync = true,
      const MoveJointOptions &options = {}, bool loop = false);

  /// 笛卡尔空间多段路点运动。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param waypoints  路点序列；每个路点为 `PoseArray`
  /// @param sync       `true` 等待路径执行结束；`false` 启动后立即返回
  /// @param options    速度与坐标系等参数
  /// @param loop       `true` 循环执行路点序列
  /// @return `SUCCESS` 或错误码
  int32_t moveCartesianWaypointPath(RobotComponent component,
                                    const std::vector<PoseArray> &waypoints,
                                    bool sync = true,
                                    const MoveCartesianOptions &options = {},
                                    bool loop = false);

  /// 运动到命名预置姿态。
  /// @param pose_name 姿态名，默认 `"initial"`
  /// @param sync      `true` 等待到位；`false` 下发后立即返回
  /// @return `SUCCESS` 或错误码
  int32_t resetToPose(const std::string &pose_name = "initial",
                      bool sync = true);

  // ----- 运动学（仅 LeftArm / RightArm）-----
  /// 正运动学（FK）：由臂关节角计算末端位姿（离线，不驱动运动）。
  /// @param component 目标臂，仅 `LeftArm` / `RightArm`
  /// @param joints    臂关节角 (rad)，长度 7
  /// @param[out] pose 末端位姿 `[x,y,z,qx,qy,qz,qw]`（m + 单位四元数）
  /// @return `SUCCESS`；非臂 / 参数非法为 `INVALID_ARGUMENT`
  int32_t calcFK(RobotComponent component, const std::vector<double> &joints,
                 PoseArray &pose);

  /// 逆运动学（IK）：由目标末端位姿求解臂关节角（离线，不驱动运动）。
  /// @param component   目标臂，仅 `LeftArm` / `RightArm`
  /// @param pose        目标末端位姿 `[x,y,z,qx,qy,qz,qw]`（m + 单位四元数）
  /// @param ref_joints  参考关节角 / IK seed (rad)，长度 7
  /// @param[out] joints 求解得到的关节角 (rad)，长度 7
  /// @return `SUCCESS`；非臂为 `INVALID_ARGUMENT`；无解可能返回
  ///         `IK_FAILED` / `IK_NO_SOLUTION`
  int32_t calcIK(RobotComponent component, const PoseArray &pose,
                 const std::vector<double> &ref_joints,
                 std::vector<double> &joints);

  // ----- 力控 / 阻抗 / 拖动 -----
  /// 标定臂力矩传感器零点（`robot/calibrate_force`）。
  /// @param component 目标臂，仅 `LeftArm` / `RightArm`
  /// @return `SUCCESS` 或错误码
  /// @pre inex v0.2+：与上下电无关；须 `state∈{IDLE,READY}` 且无运动/
  ///      阻抗 HOLD/拖动（SDK 侧会短等门禁）。
  /// @note 标定期间请保持末端无外力；成功后 settle 约 2s。建议在阻抗/
  ///       拖动前调用。
  int32_t calibrateForce(RobotComponent component);

  /// 编码器零点标定（`robot/calibrate_zero`）。
  /// @param component 目标部件：`LeftArm` / `RightArm` / `Waist` / `Head`
  /// @param axis      组内轴索引；`<0` 表示整组（`CALIB_AXIS_ALL`）
  /// @return `SUCCESS`；上电中通常返回 `POWER_OFF_REQUIRED`
  /// @pre 须 RM `state==IDLE`（已下电）；`RUNNING` / 已上电会拒绝
  /// @note 不自动下电；请先 `setEnable(..., false)`。
  int32_t calibrateZero(RobotComponent component, int32_t axis = -1);

  /// 读取当前阻抗刚度参数。
  /// @param component       目标臂，仅 `LeftArm` / `RightArm`
  /// @param[out] stiffness  填充 `joint` / `cartesian` / `nullspace`
  /// @return `SUCCESS`；失败时 `stiffness` 各字段被清空
  int32_t getImpedanceStiffness(RobotComponent component,
                                ImpedanceStiffness &stiffness);

  /// 设置关节阻抗刚度（只使用 `stiffness.joint`）。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param stiffness  `joint` 长度须为 7；`cartesian`/`nullspace` 忽略
  /// @return `SUCCESS` 或错误码
  int32_t setJointImpedanceStiffness(RobotComponent component,
                                     const ImpedanceStiffness &stiffness);

  /// 设置笛卡尔阻抗刚度（只使用 `cartesian` / `nullspace`）。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param stiffness  `cartesian` 非空时长度须为 6；`nullspace` 非空时
  ///                   长度 1~8；两者皆空返回 `INVALID_ARGUMENT`；
  ///                   `joint` 忽略
  /// @return `SUCCESS` 或错误码
  int32_t setCartesianImpedanceStiffness(RobotComponent component,
                                         const ImpedanceStiffness &stiffness);

  /// 启动阻抗会话（`robot/start_impedance`）。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param space      `Joint` / `Cartesian`：决定使用 `stiffness` 哪组字段
  /// @param stiffness  可选；空字段表示沿用当前/默认刚度
  /// @return `SUCCESS` 或错误码
  /// @pre 须先 `setProfile(JointImpedance)` 或 `CartesianImpedance`（本接口
  ///      不改档）。建议事先 `calibrateForce`。
  /// @note `Joint` 只看 `stiffness.joint`；`Cartesian` 只看
  ///       `cartesian`/`nullspace`。
  /// @note 若已在阻抗 HOLD，直接返回 `SUCCESS`（幂等；HOLD 中不可再设刚度）。
  int32_t startImpedance(RobotComponent component, MotionSpace space,
                         const ImpedanceStiffness &stiffness = {});

  /// 停止阻抗会话（`robot/stop_impedance_session`）。
  /// @param component  目标臂，仅 `LeftArm` / `RightArm`
  /// @param space      `Joint` / `Cartesian`（预留；当前实现不区分）
  /// @return `SUCCESS` 或错误码
  int32_t stopImpedance(RobotComponent component, MotionSpace space);

  // ----- 六维力 -----
  /// 查询单臂末端六维力/力矩。
  /// @param component      目标臂，仅 `LeftArm` / `RightArm`
  /// @param[out] wrench    `[Fx,Fy,Fz,Mx,My,Mz]`（N, N·m）
  /// @return `SUCCESS`；尚无数据时返回 `NO_DATA`
  int32_t getWrench(RobotComponent component, WrenchArray &wrench) const;

  /// 查询左右臂末端六维力/力矩。
  /// @param[out] left_wrench   左臂力/力矩
  /// @param[out] right_wrench  右臂力/力矩
  /// @return `SUCCESS`；尚无数据时返回 `NO_DATA`
  int32_t getWrench(WrenchArray &left_wrench, WrenchArray &right_wrench) const;

  // ----- 末端负载 -----
  /// 设置臂末端负载参数（质量、质心、惯量）。
  /// @param component 目标臂，仅 `LeftArm` / `RightArm`
  /// @param payload   负载参数；`configured` 字段被忽略
  /// @return `SUCCESS` 或错误码
  int32_t setPayload(RobotComponent component, const TcpPayload &payload);

  /// 查询臂末端负载参数。
  /// @param component       目标臂，仅 `LeftArm` / `RightArm`
  /// @param[out] payload    当前负载；`configured` 表示是否已配置非基线负载
  /// @return `SUCCESS` 或错误码
  int32_t getPayload(RobotComponent component, TcpPayload &payload);

  // ----- 急停 -----
  /// 查询急停状态。
  /// @param[out] emergency_stop_state 位掩码：bit0=硬件急停，bit1=软件急停；
  ///                                  `0` 表示未急停
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t getEmergencyStopState(int32_t &emergency_stop_state) const;

  /// 设置或清除软件急停。
  /// @param enable `true` 触发软件急停；`false` 清除
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t setSoftEmergencyStop(bool enable);

  // ----- TF -----
  /// 查询坐标系 `parent` → `child` 的相对位姿。
  /// @param parent_frame_id 父坐标系名
  /// @param child_frame_id  子坐标系名
  /// @param[out] transform  相对位姿 `[x,y,z,qx,qy,qz,qw]`
  /// @return `SUCCESS` 或错误码
  /// @note Inex：`robot/get_tf`（`use_measured=1`）；Psi：cortex TF 服务。
  int32_t getTfTransform(const std::string &parent_frame_id,
                         const std::string &child_frame_id,
                         PoseArray &transform) const;

  /// 查询机器人各连杆相对指定父坐标系的位姿。
  /// @param parent_frame_id     父坐标系名
  /// @param[out] frame_ids      坐标系名列表（与 `transforms` 一一对应）
  /// @param[out] transforms     各位姿
  /// @return `SUCCESS` 或错误码
  /// @note Inex：`robot/list_frames` + 逐帧 `robot/get_tf`（失败帧跳过）。
  int32_t getRobotTfTransforms(const std::string &parent_frame_id,
                               std::vector<std::string> &frame_ids,
                               std::vector<PoseArray> &transforms) const;

  /// 添加或更新自定义坐标系（相对父坐标系）。
  /// @param parent_frame_id 父坐标系名
  /// @param frame_id        新坐标系名
  /// @param transform       相对位姿 `[x,y,z,qx,qy,qz,qw]`
  /// @return `SUCCESS` 或错误码
  /// @note Inex：会话级 `robot/add_tf`（不可覆盖 URDF 名）。
  int32_t addTfTransform(const std::string &parent_frame_id,
                         const std::string &frame_id,
                         const PoseArray &transform);

  /// 移除自定义坐标系。
  /// @param frame_id 待移除的坐标系名
  /// @return `SUCCESS` 或错误码
  /// @note Inex：`robot/remove_tf`（有子系时不可删父）。
  int32_t removeTfTransform(const std::string &frame_id);

  // ----- 运控会话（WBC）-----
  /// 启动全身 WBC 控制会话。
  /// @param frequency_hz 控制频率（Hz）。`0` 表示使用 cortex 配置默认值
  /// @return `SUCCESS` 或错误码
  /// @note Psi 路径走 cortex `start_wbc`；Inex 暂 `UNAVAILABLE`。
  int32_t startWbc(uint32_t frequency_hz = 0);

  /// 停止 WBC，回到默认分组控制会话。
  /// @return `SUCCESS` 或错误码
  /// @note Psi 路径走 cortex `stop_wbc`；Inex 暂 `UNAVAILABLE`。
  int32_t stopWbc();

  /// WBC 笛卡尔目标（双臂，可只填一侧）。须先 `startWbc`。
  /// @param left_pose   左臂目标；空表示该侧不更新（hold）
  /// @param right_pose  右臂目标；空表示该侧不更新（hold）
  /// @param options     笛卡尔速度等（对齐 `moveEndPose` 的 `MoveCartesianOptions`）；
  ///                    主要使用 `options.speed`；`period_s` / sync 容差当前忽略
  /// @return `SUCCESS` 或错误码
  /// @note 非阻塞下发，对齐 `moveEndPose`。两侧都空返回 `INVALID_ARGUMENT`。
  ///       Inex 暂 `UNAVAILABLE`。torso / head 暂未对外开放。
  int32_t moveWbcPose(
      const std::optional<PoseArray> &left_pose,
      const std::optional<PoseArray> &right_pose,
      const MoveCartesianOptions &options = MoveCartesianOptions{});

  /// 设置 WBC 关节软权重。须先 `startWbc`。
  /// @param weights 全身 20 轴公开序（head2+waist4+left7+right7）；
  ///                >0 生效，≤0 表示该轴不设权重任务
  /// @return `SUCCESS`；非 WBC 会话返回 `INCOMPATIBLE_STATE`；
  ///         长度不符返回 `INVALID_ARGUMENT`；Inex 暂 `UNAVAILABLE`
  int32_t setWbcJointWeights(const std::vector<double> &weights);

  /// 设置 WBC 关节屏蔽（不参与求解，保持当前角）。须先 `startWbc`。
  /// @param masked 全身 20 轴；非 0=屏蔽，0=参与
  /// @return `SUCCESS`；非 WBC 会话返回 `INCOMPATIBLE_STATE`；
  ///         长度不符返回 `INVALID_ARGUMENT`；Inex 暂 `UNAVAILABLE`
  int32_t setWbcJointMask(const std::vector<uint8_t> &masked);

  /// 启动负载辨识会话。
  /// @param component 目标部件
  /// @param sync      `true` 等待辨识完成；`false` 异步启动
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t startPayloadIdent(RobotComponent component, bool sync = true);

  /// 停止负载辨识，回到默认分组控制会话。
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t stopPayloadIdent();

  /// 启动摩擦辨识会话。
  /// @param component 目标部件
  /// @param sync      `true` 等待辨识完成；`false` 异步启动
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t startFricIdent(RobotComponent component, bool sync = true);

  /// 停止摩擦辨识，回到默认分组控制会话。
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t stopFricIdent();

  /// 查询当前运控会话类型。
  /// @param[out] session 当前会话，见 `RobotSession`
  /// @return `SUCCESS` 或错误码
  /// @note 当前版本尚未支持，固定返回 `UNAVAILABLE`。
  int32_t getSession(RobotSession &session) const;

  // ----- 手部 -----
  /// 查询手部关节角（含关节名）。
  /// @param[out] names   关节名（`left_*` / `right_*` 前缀）
  /// @param[out] angles  行程归一化 [0,1]（与 synrobot 睿尔曼 EC 手控一致，非
  /// rad）
  /// @return `SUCCESS` 或错误码
  /// @note Inex：订阅 `hardware/ryhand_*/get_pdo` 后可读；Psi 路径暂
  /// `UNAVAILABLE`。
  int32_t getHandAngles(std::vector<std::string> &names,
                        std::vector<double> &angles) const;

  /// 查询单侧手部关节角。
  /// @param component      目标侧，仅 `LeftArm` / `RightArm`
  /// @param[out] angles    行程归一化 [0,1]（6 维：拇指旋转/弯曲 + 四指）
  /// @return `SUCCESS` 或错误码
  /// @pre Inex 须已收到对应侧 `get_pdo`
  int32_t getHandAngles(RobotComponent component,
                        std::vector<double> &angles) const;

  /// 查询双侧手部关节角。
  /// @param[out] left_angles   左手 [0,1]×6
  /// @param[out] right_angles  右手 [0,1]×6
  /// @return `SUCCESS` 或错误码
  int32_t getHandAngles(std::vector<double> &left_angles,
                        std::vector<double> &right_angles) const;

  /// 设置手部关节角。
  /// @param component 目标侧，仅 `LeftArm` / `RightArm`
  /// @param angles    目标行程归一化 [0,1]×6
  /// @param sync      `true` 等待反馈接近目标；`false` 下发后立即返回
  /// @return `SUCCESS` 或错误码
  /// @note Inex：经 `hardware/ryhand_*/set_pdo` 下发（0xAA 电机帧）；Psi 暂
  /// `UNAVAILABLE`。
  int32_t setHandAngles(RobotComponent component,
                        const std::vector<double> &angles, bool sync = false);

  // ----- 图像 -----
  /// 按数据类型查询彩色压缩图像。
  /// @param data_type    彩色图像数据类型（`*_COLOR_IMAGE`）
  /// @param[out] info    图像元信息
  /// @param[out] data    压缩图像字节流
  /// @return `SUCCESS` 或错误码
  /// @pre 须先订阅对应的 `COLOR_IMAGE` 类型
  int32_t getColorImage(RobotDataType data_type, ImageInfo &info,
                        std::vector<uint8_t> &data) const;

 private:
  std::unique_ptr<RobotImpl> impl_;
};

}  // namespace psibot_robot_sdk
