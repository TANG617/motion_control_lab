#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace psibot_robot_sdk {

/// 末端位姿维度：[x, y, z, qx, qy, qz, qw]（m + 四元数 xyzw）。
inline constexpr std::size_t kPoseSize = 7;
/// 六维力/力矩维度：[fx, fy, fz, tx, ty, tz]（N, N·m）。
inline constexpr std::size_t kWrenchSize = 6;
/// 惯量张量维度：Ixx, Ixy, Ixz, Iyy, Iyz, Izz（kg·m²）。
inline constexpr std::size_t kInertiaSize = 6;

/// sync 默认容差 / 超时（Options 默认值与 wait 回退共用）。
inline constexpr double kDefaultMoveJPositionToleranceRad =
    0.5 * 3.14159265358979323846 / 180.0;  // 0.5°
inline constexpr double kDefaultMoveLPositionToleranceM = 0.001;  // 1 mm
inline constexpr double kDefaultMoveLOrientationToleranceRad =
    1.0 * 3.14159265358979323846 / 180.0;  // 1°
inline constexpr double kDefaultMoveSyncTimeoutSec = 30.0;
/// stopServoMode 等待 READY/idle 的默认超时（预检/清残留用，宜短于 PTP sync）。
inline constexpr double kDefaultStopServoTimeoutSec = 5.0;

using PoseArray = std::array<double, kPoseSize>;
using WrenchArray = std::array<double, kWrenchSize>;
using InertiaArray = std::array<double, kInertiaSize>;

/// API 错误码；`SUCCESS`(0) 表示成功，错误为负值并按百位分段。
/// -1xx 本地/参数/前置条件；-2xx 连接/通信；-3xx 控制器/执行；-4xx IK；-9xx 兜底。
enum class RobotErrorCode : int32_t {
  SUCCESS = 0,  ///< 成功

  // -1xx 本地/参数/前置条件
  INVALID_ARGUMENT = -100,    ///< 参数非法（维度、取值、坐标系、group/axis 等）
  NOT_SUBSCRIBED = -101,      ///< 未订阅对应数据类型
  NO_DATA = -102,             ///< 尚无缓存数据
  NOT_READY = -103,           ///< 机器人或控制器未就绪（未上电、状态非预期等）
  NOT_SUPPORTED = -104,       ///< 功能或部件不支持（如无对应传感器通道）
  POWER_OFF_REQUIRED = -105,  ///< 须先下电（如改 motion profile；力矩标定无需下电）

  // -2xx 连接/通信
  NOT_CONNECTED = -200,   ///< 未与运控建立 TCP 连接（或已断线）
  TIMEOUT = -201,         ///< 等待数据或运动完成超时
  UNAVAILABLE = -202,     ///< 服务或 Action 不可用
  INTERNAL_ERROR = -203,  ///< SDK 或通信层内部错误

  // -3xx 控制器/执行
  CONTROLLER_BUSY = -300,     ///< 控制器/规划器忙，无法接受新指令
  REJECTED = -301,            ///< 通用拒绝（兼容保留；优先用更具体错误码）
  EXECUTION_FAILED = -302,    ///< 控制器执行失败（含标定 controller failed）
  OUT_OF_LIMIT = -303,        ///< 目标或当前关节/笛卡尔位姿越软限位
  INCOMPATIBLE_STATE = -304,  ///< 当前活动/模式与指令不兼容
  IMPEDANCE_ACTIVE = -305,    ///< 阻抗 HOLD 中，禁止 PTP/jog 等运动指令

  // -4xx 逆运动学
  IK_FAILED = -400,            ///< 逆运动学求解失败
  IK_NO_SOLUTION = -401,       ///< 逆运动学无可行解
  IK_OUT_OF_WORKSPACE = -402,  ///< 逆运动学目标超出工作空间

  // -9xx 兜底
  UNKNOWN = -900,  ///< 未知或未分类错误（含未识别的后端 reject_reason）
};

enum class RobotType {
  PSI_R1 = 1,
};

/// 臂末端负载（映射 inex robot/set|get_payload）。
struct TcpPayload {
  double mass = 0.0;                      ///< 质量（kg）
  std::array<double, 3> com{{0, 0, 0}};    ///< 质心（m），相对法兰/工具系
  InertiaArray inertia{{0, 0, 0, 0, 0, 0}};  ///< Ixx Ixy Ixz Iyy Iyz Izz
  bool configured = false;  ///< get 时有效；set 时忽略
};

/// 压缩图像元信息（由 getColorImage 填充；像素数据见 data
/// 出参）。
struct ImageInfo {
  std::string format;    ///< 压缩格式，如 "jpeg"
  int32_t width = 0;     ///< 图像宽度（像素）；未知时为 0
  int32_t height = 0;    ///< 图像高度（像素）；未知时为 0
  double fps = 0.0;      ///< 估算帧率（Hz）；首帧或未知时为 0
  int64_t stamp_us = 0;  ///< 时间戳（μs）
};

/// 机器人静态信息（由 getRobotInfo 填充）。
struct RobotInfo {
  RobotType type = RobotType::PSI_R1;  ///< 机器人类型
  std::string sdk_version = "";        ///< SDK 版本号
  std::string serial_number = "";      ///< 机器人序列号（预留，当前为空）
  std::string firmware_version = "";   ///< 固件版本（预留，当前为空）
  std::string details_json = "";       ///< 扩展详情 JSON（预留，当前为空）
};

/// 单关节静态限制（与 getJointStates 的关节名/顺序对齐）。
struct JointLimits {
  std::string name;             ///< 关节名
  double position_lower = 0.0;  ///< 位置下限（rad），URDF `<limit lower>`
  double position_upper = 0.0;  ///< 位置上限（rad）
  double model_velocity_limit = 0.0;      ///< 模型速度上限（rad/s）
  double model_acceleration_limit = 0.0;  ///< 模型加速度上限（rad/s²，预留）
  double max_velocity = 0.0;              ///< 控制器轨迹速度上限（rad/s）
  double max_acceleration = 0.0;          ///< 控制器轨迹加速度上限（rad/s²）
};

/// 机器人部件标识。
/// 数值与 inex power group 对齐：0=整机，1=左臂，2=右臂，3=腰/body，4=头。
enum class RobotComponent {
  All = 0,      ///< 整机（setEnable / getEnabled / moveJ 全身 20 轴）
  LeftArm = 1,  ///< 左臂（含左手相机/手部控制）
  RightArm,     ///< 右臂（含右手相机/手部控制）
  Waist,        ///< 腰/躯干（对外 4 轴：yaw→pitch→knee→ankle；发 Inex 前转 wire）
  Head,         ///< 头部（对外/Inex 同序 2 轴：yaw→pitch；走 HeadTargetDeg，不经 body 槽）
};

/// 会话分区激活（inex robot/set_robot_mode；对齐 controller::RobotMode）。
enum class RobotMode : uint32_t {
  LeftOnly = 0,
  RightOnly = 1,
  DualArmOnly = 2,
  WholeBody = 3,
  BodyOnly = 4,
  HeadOnly = 5,
};

/// 单臂末端位姿笛卡尔运动限制（与 getEndPose 对应）。
struct EndPoseLimits {
  double max_linear_velocity = 0.0;      ///< 路径切向线速度上限（m/s）
  double max_linear_acceleration = 0.0;  ///< 路径切向线加速度上限（m/s²）
};

/// 机器人运动限制（关节 + 末端笛卡尔）。
struct RobotLimits {
  std::vector<JointLimits> joints;  ///< 关节限制（顺序与 getJointStates 对齐）
  std::vector<EndPoseLimits>
      end_poses;  ///< 末端限制；约定 [0]=LeftArm，[1]=RightArm
};

/// 一次读取的机器人观测快照（尽力填充；用 has_* 判断通道是否有效）。
///
/// 成功策略（getObservation）：未连接 → NOT_CONNECTED；
/// 至少 joints 或任一侧 Base 位姿有效 → SUCCESS；否则 → NO_DATA。
/// 力 / 手 / Arm 系位姿缺失不导致失败。
struct RobotObservation {
  // ----- 整机关节（与 getJointStates 同序：head+waist+left+right）-----
  std::vector<std::string> joint_names;
  std::vector<double> positions;      ///< rad
  std::vector<double> velocities;     ///< rad/s
  std::vector<double> accelerations;  ///< rad/s²；暂无时可能为空或零
  std::vector<double> torques;        ///< N·m；暂无时可能为空或零

  // ----- 双臂末端：Base 系（当前 Inex CartesianState 有效）-----
  PoseArray left_pose_base{};   ///< [x,y,z,qx,qy,qz,qw]
  PoseArray right_pose_base{};

  // ----- 双臂末端：Arm 系（预留；当前可空）-----
  PoseArray left_pose_arm{};
  PoseArray right_pose_arm{};

  // ----- 双臂六维力 [Fx,Fy,Fz,Mx,My,Mz] -----
  WrenchArray left_wrench{};
  WrenchArray right_wrench{};

  // ----- 双手：名 + 行程归一化 [0,1]×6（与 getHandAngles 一致）-----
  std::vector<std::string> left_hand_names;
  std::vector<std::string> right_hand_names;
  std::vector<double> left_hand_angles;
  std::vector<double> right_hand_angles;

  // ----- 通道有效标志 -----
  bool has_joints = false;
  bool has_left_pose_base = false;
  bool has_right_pose_base = false;
  bool has_left_pose_arm = false;
  bool has_right_pose_arm = false;
  bool has_left_wrench = false;
  bool has_right_wrench = false;
  bool has_left_hand = false;
  bool has_right_hand = false;
};

/// SDK 可接收的机器人数据类型（与底层传输无关）。
enum class RobotDataType {
  SYSTEM_STATUS,
  JOINT_STATES,
  CARTESIAN_STATES,
  WRENCH_STATES,
  HAND_JOINT_STATES,
  GRIPPER_STATES,

  // Color compressed — {BODY}_{STREAM}_COLOR_IMAGE
  // ↔ /hal/camera/{body}/{stream}/compressed
  HAND_LEFT_COLOR_IMAGE,
  HAND_RIGHT_COLOR_IMAGE,
  HAND_FISHEYE_LEFT_COLOR_IMAGE,
  HAND_FISHEYE_RIGHT_COLOR_IMAGE,
  CHEST_COLOR_IMAGE,
  HEAD_COLOR_IMAGE,
  HEAD_FISHEYE_LEFT_COLOR_IMAGE,
  HEAD_FISHEYE_REAR_COLOR_IMAGE,
  HEAD_FISHEYE_RIGHT_COLOR_IMAGE,
  HEAD_STEREO_LEFT_COLOR_IMAGE,
  HEAD_STEREO_RIGHT_COLOR_IMAGE,
};

/// 笛卡尔位姿的参考坐标系。
enum class PoseFrame {
  Base = 1,  ///< 基坐标系（当前默认、唯一生效）
  Arm,       ///< 臂坐标系（预留）
};

/// 运动空间（关节 / 笛卡尔）；servo、路点、阻抗 start/stop 等接口共用。
enum class MotionSpace {
  Joint = 0,      ///< 关节空间（SERVO_J / WAYPOINT_JOINT / 关节阻抗）
  Cartesian = 1,  ///< 笛卡尔空间（SERVO_L / WAYPOINT_CART / 笛卡尔阻抗）
};

/// 阻抗刚度参数（关节 / 笛卡尔 / 零空间共用结构体）。
/// 空 vector 表示该字段未指定（set/start 时跳过；get 时由后端填充）。
struct ImpedanceStiffness {
  std::vector<double> joint;       ///< 关节刚度，用时长度 7
  std::vector<double> cartesian;   ///< 笛卡尔刚度，用时长度 6 (Fx..Mz)
  std::vector<double> nullspace;   ///< 零空间刚度，用时长度 1~8
};

/// 速度/加速度/jerk 的表示方式（`MotionSpeed` 共用）。
enum class SpeedMode {
  Absolute,  ///< 物理量：笛卡尔 m/s、m/s²、m/s³；关节 rad/s、rad/s²、rad/s³
  Relative,  ///< 相对上限的比例，取值 (0, 1]
};

/// 四类运动共用的速度/加速度/jerk 参数。
struct MotionSpeed {
  SpeedMode mode = SpeedMode::Relative;
  double velocity = 0.1;
  double acceleration = 0.5;
  double jerk = 1.0;
};

/// 笛卡尔空间运动参数（moveL / moveEndPose / 笛卡尔路点共用）。
struct MoveCartesianOptions {
  MotionSpeed speed{SpeedMode::Relative, 0.1, 0.5, 1.0};
  PoseFrame pose_frame{PoseFrame::Base};
  /// 流式命令周期 (s)，仅 moveEndPose（ServoL）使用（≈ 1/控制频率）。
  /// 映射 inex ServoCmd.segment_time_s；<=0 时 SDK 默认 0.005（200Hz）。
  /// moveL / 笛卡尔路点忽略此字段。
  double period_s{0.0};
  /// sync 平移容差 (m)；仅 sync=true 的 moveL 使用。默认 1 mm。
  /// moveEndPose / 笛卡尔路点当前忽略。
  double position_tolerance_m{kDefaultMoveLPositionToleranceM};
  /// sync 姿态容差 (rad)；四元数夹角。默认 1°。
  /// moveEndPose / 笛卡尔路点当前忽略。
  double orientation_tolerance_rad{kDefaultMoveLOrientationToleranceRad};
};

/// 关节空间运动参数（moveJ / moveServoJ / 关节路点共用）。
struct MoveJointOptions {
  MotionSpeed speed{SpeedMode::Relative, 0.1, 0.5, 1.0};
  /// 流式命令周期 (s)，仅 moveServoJ 使用（≈ 1/控制频率）。
  /// 映射 inex ServoCmd.segment_time_s；<=0 时 SDK 默认 0.005（200Hz）。
  /// moveJ / 关节路点忽略此字段。
  double period_s{0.0};
  /// sync 到位容差 (rad)；仅 sync=true 的 moveJ 使用。默认 0.5°。
  /// moveServoJ / 关节路点当前忽略。
  double position_tolerance_rad{kDefaultMoveJPositionToleranceRad};
};

/// 运控会话（同一时刻仅一个生效；默认 GroupControl）。
enum class RobotSession {
  GroupControl = 1,  ///< 分组控制（默认）
  Wbc,               ///< 全身 WBC
  PayloadIdent,      ///< 负载辨识
  FrictionIdent,     ///< 摩擦辨识
};

/// SDK 运行事件（连接等）。回调第二参数 void* 供后续事件扩展，当前传 nullptr。
enum class RobotEvent : int32_t {
  Connected = 1,
  Disconnected = 2,
};

using RobotEventCallback = std::function<void(RobotEvent event, void *data)>;

/// 臂控制 profile（映射 inex MotionProfile；Admittance 无对应后端）。
enum class ArmProfile {
  Idle = 0,                ///< 空闲
  JointPosition = 1,       ///< 关节空间位置控制
  JointImpedance = 2,      ///< 关节空间阻抗
  CartesianPosition = 3,   ///< 笛卡尔空间位置控制
  CartesianImpedance = 4,  ///< 笛卡尔空间阻抗
  Admittance = 5,          ///< 导纳控制
  JointDrag = 6,           ///< 关节空间零力拖动
  CartesianDrag = 7,       ///< 笛卡尔空间零力拖动
  Bypass = 8,              ///< 旁路
};

}  // namespace psibot_robot_sdk
