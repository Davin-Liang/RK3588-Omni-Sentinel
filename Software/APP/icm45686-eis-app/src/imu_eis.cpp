/*
 * imu_eis.cpp - ICM45686防抖应用层接口实现
 *
 * 基于当前已调通的 /dev/icm45686 字符设备方案实现IMU读取、环形缓冲区和EIS偏移计算接口
 *
 * 日期: 2026-06-08
 */

#include "imu_eis.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NS_PER_MS 1000000ULL
#define NS_PER_SEC 1000000000ULL

/* ============================================================================
 * 数学工具函数 —— 线性代数 + 四元数 + 相机模型
 *
 * 防抖算法涉及大量 3D 旋转和投影运算，以下工具函数构成了整个数学基础库。
 * 统一约定：
 *   - 3×3 矩阵用 float[9] 表示，row-major 存储（下标 = row*3 + col）
 *   - 四元数用 float[4] 表示，顺序为 (w, x, y, z)，其中 w 是实部
 *   - 所有旋转约定为"被动旋转"——旋转坐标系而非向量本身
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * vec3_norm — 计算 3D 向量的欧几里得长度
 *
 * 用于：归一化、角度幅值计算、震动等级判定
 * --------------------------------------------------------------------------- */
static inline float vec3_norm(float x, float y, float z)
{
    return std::sqrt(x*x + y*y + z*z);
}

/* ---------------------------------------------------------------------------
 * mat3_mul_vec3 — 3×3 矩阵 × 3D 列向量
 *   out = M * v
 *
 * 用于：坐标系变换（如陀螺仪从 IMU 坐标系旋转到机体坐标系）
 * --------------------------------------------------------------------------- */
static inline void mat3_mul_vec3(const float M[9], const float v[3], float out[3])
{
    out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
    out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
    out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

/* ---------------------------------------------------------------------------
 * mat3_mul — 3×3 矩阵乘法
 *   C = A * B
 *
 * 用于：组合多次旋转变换（如 R_C_B * RcompB * R_C_B^T）
 * --------------------------------------------------------------------------- */
static inline void mat3_mul(const float A[9], const float B[9], float C[9])
{
    float T[9];
    for (int r=0; r<3; ++r) {
        for (int c=0; c<3; ++c) {
            T[r*3+c] = A[r*3+0]*B[0*3+c] + A[r*3+1]*B[1*3+c] + A[r*3+2]*B[2*3+c];
        }
    }
    memcpy(C, T, sizeof(T));
}

/* ---------------------------------------------------------------------------
 * mat3_transpose — 3×3 矩阵转置
 *
 * 旋转矩阵的转置 = 逆矩阵（因为旋转矩阵是正交矩阵），
 * 所以 R^T 等价于"反向旋转"
 * --------------------------------------------------------------------------- */
static inline void mat3_transpose(const float A[9], float At[9])
{
    At[0]=A[0]; At[1]=A[3]; At[2]=A[6];
    At[3]=A[1]; At[4]=A[4]; At[5]=A[7];
    At[6]=A[2]; At[7]=A[5]; At[8]=A[8];
}

/* ---------------------------------------------------------------------------
 * make_K / make_K_inv — 相机内参矩阵及其逆矩阵
 *
 * 小孔相机模型：
 *       [ fx   0   cx ]
 *   K = [  0  fy   cy ]    将相机坐标系 3D 点投影到像素坐标系
 *       [  0   0    1 ]
 *
 *   fx, fy: 焦距（像素单位），通常 fx ≈ fy
 *   cx, cy: 主点（光心在图像中的像素坐标），通常 cx = width/2, cy = height/2
 *
 * 逆矩阵 K^-1 的作用是"反投影"——将像素坐标变回相机坐标系下的方向向量。
 * 在防抖算法中，H = K * R * K^-1 的含义就是：
 *   先把像素坐标反投影 → 旋转相机 → 再投影回像素坐标
 * --------------------------------------------------------------------------- */
static inline void make_K(float fx, float fy, float cx, float cy, float K[9])
{
    K[0]=fx;   K[1]=0.0f; K[2]=cx;
    K[3]=0.0f; K[4]=fy;   K[5]=cy;
    K[6]=0.0f; K[7]=0.0f; K[8]=1.0f;
}

static inline void make_K_inv(float fx, float fy, float cx, float cy, float Kinv[9])
{
    Kinv[0]=1.0f/fx; Kinv[1]=0.0f;    Kinv[2]=-cx/fx;
    Kinv[3]=0.0f;    Kinv[4]=1.0f/fy; Kinv[5]=-cy/fy;
    Kinv[6]=0.0f;    Kinv[7]=0.0f;    Kinv[8]=1.0f;
}

/* ============================================================================
 * 四元数运算 —— 旋转的数学语言
 *
 * 为什么用四元数而不是欧拉角？
 *   1. 无万向节死锁 —— 任何姿态都能唯一表示
 *   2. 球面插值自然 —— SLERP 天然保证最短路径旋转
 *   3. 数值稳定 —— 归一化就能修正累积误差
 *
 * 四元数 q = (w, x, y, z) 表示绕单位轴 (x,y,z)/|(x,y,z)| 旋转 2*acos(w) 弧度
 *   或等价地：q = (cos(θ/2), sin(θ/2)*axisX, sin(θ/2)*axisY, sin(θ/2)*axisZ)
 *
 * 旋转向量 (rotation vector) 与四元数的关系：
 *   旋转向量 w = (wx, wy, wz)，它的方向是旋转轴，长度 |w| = 旋转角（弧度）
 *   四元数 q = (cos(|w|/2), w/|w| * sin(|w|/2))
 * ============================================================================ */

/* 单位四元数 —— 表示"不旋转" */
static inline void quat_identity(float q[4])
{
    q[0]=1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f;
}

/* 归一化 —— 保证四元数在单位球面上，否则累积误差会导致旋转变形 */
static inline void quat_normalize(float q[4])
{
    float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) { quat_identity(q); return; }
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

/* ---------------------------------------------------------------------------
 * quat_mul — 四元数乘法
 *   out = a * b
 *
 * 物理含义：先做旋转 b，再做旋转 a（或反过来，取决于约定）。
 * 本项目采用"被动旋转"约定：
 *   q_new = dq * q_old  表示在当前姿态 q_old 的基础上再旋转 dq
 *
 * 四元数乘法公式（Hamilton 约定，i*j=k, j*k=i, k*i=j）：
 *   (w1 + x1i + y1j + z1k) * (w2 + x2i + y2j + z2k) = ...
 *
 * 记忆技巧：w1*w2 - dot(v1,v2)  为实部
 *           w1*v2 + w2*v1 + cross(v1,v2)  为虚部
 * --------------------------------------------------------------------------- */
static inline void quat_mul(const float a[4], const float b[4], float out[4])
{
    float t[4];
    t[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];   /* w1*w2 - v1·v2 */
    t[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];   /* w1*x2 + x1*w2 + y1*z2 - z1*y2 */
    t[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];   /* w1*y2 - x1*z2 + y1*w2 + z1*x2 */
    t[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];   /* w1*z2 + x1*y2 - y1*x2 + z1*w2 */
    memcpy(out, t, sizeof(t));
}

/* ---------------------------------------------------------------------------
 * quat_inverse — 四元数的逆
 *
 * 对单位四元数，逆 = 共轭（虚部取反）
 * 物理含义：q 的逆就是"反向旋转"——撤销 q 所表示的旋转
 * --------------------------------------------------------------------------- */
static inline void quat_inverse(const float q[4], float qi[4])
{
    qi[0]=q[0]; qi[1]=-q[1]; qi[2]=-q[2]; qi[3]=-q[3];
}

/* ---------------------------------------------------------------------------
 * quat_from_rotvec — 旋转向量 → 四元数
 *
 * 输入旋转向量 w = (wx, wy, wz)：
 *   |w| = 旋转角度（弧度）
 *   w/|w| = 旋转轴
 *
 * 这在防抖中非常关键：陀螺仪测得的是角速度 (rad/s)，
 * gyro * dt 就是这段微小时段内的旋转向量，
 * 把它转成四元数就能和当前姿态相乘，实现逐帧积分。
 * --------------------------------------------------------------------------- */
static inline void quat_from_rotvec(const float w[3], float q[4])
{
    float angle = vec3_norm(w[0], w[1], w[2]);
    if (angle < 1e-9f) { quat_identity(q); return; }
    float half = 0.5f * angle;
    float s = std::sin(half) / angle;
    q[0] = std::cos(half);
    q[1] = w[0] * s;
    q[2] = w[1] * s;
    q[3] = w[2] * s;
    quat_normalize(q);
}

/* ---------------------------------------------------------------------------
 * quat_to_mat3 — 四元数 → 3×3 旋转矩阵
 *
 * 输出 row-major 旋转矩阵 R，满足 v_rotated = R * v_original
 *
 * 这是连接"四元数世界"和"相机投影世界"的桥梁：
 * 姿态用四元数积分更稳定，但最终生成单应矩阵需要旋转矩阵形式。
 * --------------------------------------------------------------------------- */
static inline void quat_to_mat3(const float qIn[4], float R[9])
{
    float q[4] = {qIn[0], qIn[1], qIn[2], qIn[3]};
    quat_normalize(q);
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0]=1.0f-2.0f*(y*y+z*z); R[1]=2.0f*(x*y-w*z);     R[2]=2.0f*(x*z+w*y);
    R[3]=2.0f*(x*y+w*z);     R[4]=1.0f-2.0f*(x*x+z*z); R[5]=2.0f*(y*z-w*x);
    R[6]=2.0f*(x*z-w*y);     R[7]=2.0f*(y*z+w*x);     R[8]=1.0f-2.0f*(x*x+y*y);
}

/* ---------------------------------------------------------------------------
 * quat_slerp — 球面线性插值 (Spherical Linear intERPolation)
 *
 * 在两四元数 q0 和 q1 之间，沿四维单位球面上的最短弧插值。
 * 参数 alpha ∈ [0,1]：
 *   alpha=0 → 返回 q0
 *   alpha=1 → 返回 q1
 *   alpha=0.5 → 返回中间姿态
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * 为什么 SLERP 在防抖中至关重要？
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * 防抖的本质是"频率分离"：
 *   - 低频信号 = 手持平移/转向 = 应该保留（跟到 qSmooth 里）
 *   - 高频信号 = 抖动/震动 = 应该消除（留在 qRaw 里不跟过去）
 *
 * SLERP(qSmooth, qRaw, α) 实现了这个分离：
 *   α 小 → qSmooth 变化慢 → 只跟低频 → 更多高频被"甩出去"成为补偿量
 *   α 大 → qSmooth 变化快 → 跟随更紧 → 较少补偿
 *
 * α 由指数平滑公式计算：α = 1 - exp(-dt/τ)
 *   τ 是时间常数（smoothTauSec），τ 越大 → α 越小 → 防抖越强但不跟手
 *
 * 为什么不能用普通的线性插值 (LERP)？
 *   LERP(q0, q1, α) 的中间结果不在单位球面上，会导致旋转"加速/减速"不均匀。
 *   SLERP 保证恒定角速度——物理上更接近真实旋转的合成方式。
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * 算法细节：
 *   1. 计算四元数夹角 θ = acos(dot(q0, q1))
 *   2. 若夹角很小（dot > 0.9995），退化为线性插值避免数值不稳定
 *   3. 用 sin(θ(1-α)) / sin(θ) 和 sin(θα) / sin(θ) 作为权重
 *   4. 如果 dot < 0，取 q1 的相反数——因为 q 和 -q 表示同一旋转，
 *      但选正 dot 的那个保证走短弧而非长弧
 * --------------------------------------------------------------------------- */
static inline void quat_slerp(const float q0[4], const float q1[4], float alpha, float out[4])
{
    float b[4] = {q1[0], q1[1], q1[2], q1[3]};

    /* 计算两四元数的点积（夹角余弦） */
    float dot = q0[0]*b[0] + q0[1]*b[1] + q0[2]*b[2] + q0[3]*b[3];

    /* q 和 -q 表示同一旋转，选 dot>0 的一面保证走最短弧 */
    if (dot < 0.0f) { dot=-dot; b[0]=-b[0]; b[1]=-b[1]; b[2]=-b[2]; b[3]=-b[3]; }

    /* 夹角太小时退化为 LERP，避免 sin(θ)/sin(θ) 除零 */
    if (dot > 0.9995f) {
        out[0] = q0[0] + alpha*(b[0]-q0[0]);
        out[1] = q0[1] + alpha*(b[1]-q0[1]);
        out[2] = q0[2] + alpha*(b[2]-q0[2]);
        out[3] = q0[3] + alpha*(b[3]-q0[3]);
        quat_normalize(out);
        return;
    }

    /* SLERP 标准公式 */
    float theta0 = std::acos(std::max(-1.0f, std::min(1.0f, dot)));
    float theta = theta0 * alpha;
    float sinTheta = std::sin(theta);
    float sinTheta0 = std::sin(theta0);
    float s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    float s1 = sinTheta / sinTheta0;
    out[0] = s0*q0[0] + s1*b[0];
    out[1] = s0*q0[1] + s1*b[1];
    out[2] = s0*q0[2] + s1*b[2];
    out[3] = s0*q0[3] + s1*b[3];
    quat_normalize(out);
}

/* ---------------------------------------------------------------------------
 * quat_angle — 计算四元数表示的旋转角（弧度）
 *
 * 公式：angle = 2 * acos(w)，因为 q = (cos(θ/2), sin(θ/2)*axis)
 * --------------------------------------------------------------------------- */
static inline float quat_angle(const float q[4])
{
    float w = std::max(-1.0f, std::min(1.0f, q[0]));
    return 2.0f * std::acos(w);
}

/* ---------------------------------------------------------------------------
 * quat_clamp_angle — 限制四元数的旋转角不超过 maxAngle
 *
 * 为什么需要？
 *   陀螺仪偶尔会产生异常尖峰（如撞击、EMI 干扰），
 *   导致一帧之间的积分角异常大——如果不限制，画面会瞬间飞出去。
 *
 * 做法：若当前四元数的旋转角超过 maxAngle，则等比例缩小旋转向量，
 *       保持旋转轴不变但减小旋转幅度。
 * --------------------------------------------------------------------------- */
static inline void quat_clamp_angle(float q[4], float maxAngle)
{
    if (maxAngle <= 0.0f) return;
    quat_normalize(q);
    float angle = quat_angle(q);
    if (angle <= maxAngle || angle < 1e-9f) return;
    float scale = maxAngle / angle;
    float rotvec[3];
    float sinHalf = std::sqrt(std::max(0.0f, 1.0f - q[0]*q[0]));
    if (sinHalf < 1e-6f) return;
    rotvec[0] = q[1] / sinHalf * angle * scale;
    rotvec[1] = q[2] / sinHalf * angle * scale;
    rotvec[2] = q[3] / sinHalf * angle * scale;
    quat_from_rotvec(rotvec, q);
}

/* ---------------------------------------------------------------------------
 * apply_homography_center — 将单应矩阵 H 作用于点 (x, y)
 *
 * 单应变换（齐次坐标形式）：
 *   [x']     [h11 h12 h13] [x]
 *   [y']  =  [h21 h22 h23] [y]
 *   [w ]     [h31 h32 h33] [1]
 *
 *   输出：(x'/w, y'/w)
 *
 * 在防抖中的用途：计算图像中心 (cx, cy) 经旋转补偿后移到了哪个像素位置，
 * 两者之差就是需要裁剪的像素偏移量。
 * --------------------------------------------------------------------------- */
static inline void apply_homography_center(const float H[9], float x, float y, float& ox, float& oy)
{
    float w = H[6]*x + H[7]*y + H[8];
    if (std::fabs(w) < 1e-6f) { ox=x; oy=y; return; }
    ox = (H[0]*x + H[1]*y + H[2]) / w;
    oy = (H[3]*x + H[4]*y + H[5]) / w;
}

/* ---------------------------------------------------------------------------
 * quat_to_euler_xyz — 四元数 → XYZ 欧拉角 (roll, pitch, yaw)
 *
 * 仅用于调试日志输出，不参与算法计算。
 * 输出顺序为 (roll, pitch, yaw)，单位弧度。
 * --------------------------------------------------------------------------- */
static inline void quat_to_euler_xyz(const float qIn[4], float e[3])
{
    float R[9]; quat_to_mat3(qIn, R);
    e[0] = std::atan2(R[7], R[8]);                                   /* roll  */
    e[1] = std::asin(std::max(-1.0f, std::min(1.0f, -R[6])));        /* pitch */
    e[2] = std::atan2(R[3], R[0]);                                   /* yaw   */
}

/**************************实现函数********************************************
 *函数原型:     uint64_t imu_get_time_ns(void)
 *功    能:     获取单调递增时间戳，单位ns，基于 CLOCK_MONOTONIC
 *
 * CLOCK_MONOTONIC 保证不受系统时间跳变影响，是传感器时间戳的标准选择。
 *输入参数:     无
 *输出参数:     时间戳，单位ns
 ******************************************************************************/
uint64_t imu_get_time_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * NS_PER_SEC + (uint64_t)ts.tv_nsec;
}

/* ============================================================================
 * ImuRingBuffer —— IMU 样本环形缓冲区
 *
 * 存储最近 N 个 IMU 样本（默认 512 个），提供按时间戳范围查询的能力。
 *
 * 为什么用环形缓冲区？
 *   IMU 以 100Hz 持续采样，防抖算法需要按帧曝光时间戳查附近样本。
 *   环形缓冲区保证了：
 *     1. 内存有上限，不会无限增长
 *     2. 旧数据自动淘汰，新数据始终可用
 *     3. 线程安全（mutex 保护读写）
 *
 * 数据结构：std::deque（双端队列），支持高效的头删尾插
 * ============================================================================ */

/**************************实现函数********************************************
 *函数原型:     ImuRingBuffer::ImuRingBuffer(size_t maxSamples)
 *功    能:     构造IMU环形缓冲区
 *输入参数:     maxSamples - 最大缓存样本数量
 *输出参数:     无
 ******************************************************************************/
ImuRingBuffer::ImuRingBuffer(size_t maxSamples)
    : maxSamples_(maxSamples)
{
}

/**************************实现函数********************************************
 *函数原型:     void ImuRingBuffer::push(const ImuSample& sample)
 *功    能:     写入一个IMU样本
 *
 * 当缓冲区满时，自动丢弃最旧的样本（FIFO 淘汰策略）。
 * 加锁保护，读线程和写线程可以安全并发。
 *输入参数:     sample - IMU样本
 *输出参数:     无
 ******************************************************************************/
void ImuRingBuffer::push(const ImuSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);

    samples_.push_back(sample);
    while (samples_.size() > maxSamples_) {
        samples_.pop_front();
    }
}

/**************************实现函数********************************************
 *函数原型:     void ImuRingBuffer::clear(void)
 *功    能:     清空环形缓冲区
 *
 * 通常在 IMU 重启或需要重置状态时调用。
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void ImuRingBuffer::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

/**************************实现函数********************************************
 *函数原型:     size_t ImuRingBuffer::size(void) const
 *功    能:     获取当前缓存样本数量
 *输入参数:     无
 *输出参数:     样本数量
 ******************************************************************************/
size_t ImuRingBuffer::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

/**************************实现函数********************************************
 *函数原型:     bool ImuRingBuffer::latest(ImuSample& sample) const
 *功    能:     获取最新IMU样本
 *
 * 用于：计算震动等级、获取 IMU 当前状态
 *输入参数:     sample - 输出样本
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool ImuRingBuffer::latest(ImuSample& sample) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) {
        return false;
    }

    sample = samples_.back();
    return true;
}

/**************************实现函数********************************************
 *函数原型:     bool ImuRingBuffer::getSamplesBetween(uint64_t startTimeNs,
 *                                                    uint64_t endTimeNs,
 *                                                    std::vector<ImuSample>& samples) const
 *功    能:     按时间范围获取IMU样本
 *
 * 这是防抖算法最核心的查询接口。
 *
 * 给定帧曝光时间戳，查 [startTimeNs, endTimeNs] 范围内的所有 IMU 样本，
 * 用于后续的陀螺仪积分。
 *
 * 时间复杂度 O(n)，n 为缓冲区样本数（通常 ≤ 512）。
 * 对于 100Hz × 512 样本 ≈ 5.12s 的窗口来说，线性扫描完全可接受。
 *
 *输入参数:     startTimeNs - 起始时间戳，endTimeNs - 结束时间戳
 *输出参数:     samples - 输出样本列表
 *返 回 值:     true获取到样本，false没有样本
 ******************************************************************************/
bool ImuRingBuffer::getSamplesBetween(uint64_t startTimeNs,
                                      uint64_t endTimeNs,
                                      std::vector<ImuSample>& samples) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    samples.clear();
    if (startTimeNs > endTimeNs) {
        return false;
    }

    for (const auto& sample : samples_) {
        if (sample.timestampNs >= startTimeNs && sample.timestampNs <= endTimeNs) {
            samples.push_back(sample);
        }
    }

    return !samples.empty();
}

/* ============================================================================
 * Icm45686Reader —— ICM45686 IMU 读取器
 *
 * 封装 /dev/icm45686 字符设备的访问：
 *   1. 通过 ioctl 读取原始 IMU 数据（gyro、accel、temp）
 *   2. 后台线程以固定频率（默认 100Hz）持续读取
 *   3. 每次读取打 CLOCK_MONOTONIC 时间戳
 *   4. 推入 ImuRingBuffer 供防抖算法消费
 *
 * 线程模型：
 *   - write 线程：reader 后台线程（readLoop）只写 ringBuffer
 *   - read 线程：调用方（一般是 EisStabilizer）只读 ringBuffer
 *   - 并行安全由 ImuRingBuffer 内部的 mutex 保证
 * ============================================================================ */

/**************************实现函数********************************************
 *函数原型:     Icm45686Reader::Icm45686Reader(size_t ringBufferSamples)
 *功    能:     构造ICM45686读取器
 *输入参数:     ringBufferSamples - 环形缓冲区最大样本数量
 *输出参数:     无
 ******************************************************************************/
Icm45686Reader::Icm45686Reader(size_t ringBufferSamples)
    : fd_(-1),
      sampleHz_(100.0f),
      running_(false),
      ringBuffer_(ringBufferSamples),
      totalSamples_(0),
      failedReads_(0)
{
}

/**************************实现函数********************************************
 *函数原型:     Icm45686Reader::~Icm45686Reader(void)
 *功    能:     析构读取器，停止线程并关闭设备
 *
 * 析构顺序：先停线程，再关设备。确保线程不再访问 fd_ 后再关闭。
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
Icm45686Reader::~Icm45686Reader()
{
    stop();
    closeDevice();
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::openDevice(const std::string& devPath)
 *功    能:     打开ICM45686字符设备
 *输入参数:     devPath - 设备节点路径，默认 "/dev/icm45686"
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::openDevice(const std::string& devPath)
{
    closeDevice();

    fd_ = icm45686_open(devPath.c_str());
    if (fd_ < 0) {
        return false;
    }

    return true;
}

/**************************实现函数********************************************
 *函数原型:     void Icm45686Reader::closeDevice(void)
 *功    能:     关闭ICM45686字符设备
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void Icm45686Reader::closeDevice()
{
    if (fd_ >= 0) {
        icm45686_close(fd_);
        fd_ = -1;
    }
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::setAccelRange(uint8_t range)
 *功    能:     设置加速度计量程
 *输入参数:     range - 0/1/2/3对应±2G/±4G/±8G/±16G
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::setAccelRange(uint8_t range)
{
    if (fd_ < 0) {
        return false;
    }

    return icm45686_set_accel_fs(fd_, range) == 0;
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::setGyroRange(uint8_t range)
 *功    能:     设置陀螺仪量程
 *
 * 防抖场景推荐 ±250 DPS（range=0），此时分辨率最高（~0.008 DPS/LSB）。
 * 因为抖动幅度通常很小（<10°/s），不需要大量程。
 *输入参数:     range - 0/1/2/3对应±250/±500/±1000/±2000DPS
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::setGyroRange(uint8_t range)
{
    if (fd_ < 0) {
        return false;
    }

    return icm45686_set_gyro_fs(fd_, range) == 0;
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::readSample(ImuSample& sample)
 *功    能:     主动读取一个IMU样本并打时间戳
 *
 * 调用内核驱动 ioctl 读取原始数据 → 转为物理单位 → 打上 monotonic 时间戳。
 *
 * 注意：时间戳打在 read 完成时刻，而非 IMU 内部采样时刻。
 * 对于 100Hz 的采样率来说，这个延迟是固定的（约 10ms），
 * 在 30fps 视频防抖中可以忽略。
 *
 *输入参数:     sample - 输出样本
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::readSample(ImuSample& sample)
{
    icm45686_data_t data;

    if (fd_ < 0) {
        return false;
    }

    memset(&data, 0, sizeof(data));
    if (icm45686_read_data(fd_, &data) < 0) {
        return false;
    }

    sample.timestampNs = imu_get_time_ns();
    sample.accelX = data.accel_x;
    sample.accelY = data.accel_y;
    sample.accelZ = data.accel_z;
    sample.gyroX = data.gyro_x;
    sample.gyroY = data.gyro_y;
    sample.gyroZ = data.gyro_z;
    sample.temperature = data.temp;

    return true;
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::start(float sampleHz)
 *功    能:     启动后台读取线程
 *
 * 启动 readLoop 线程，以指定频率持续读取 IMU 数据到环形缓冲区。
 * 如果已经在运行则直接返回 true（幂等操作）。
 *
 *输入参数:     sampleHz - 应用层读取频率，默认 100.0
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::start(float sampleHz)
{
    if (fd_ < 0 || sampleHz <= 0.0f) {
        return false;
    }

    if (running_.load()) {
        return true;
    }

    sampleHz_ = sampleHz;
    running_.store(true);
    worker_ = std::thread(&Icm45686Reader::readLoop, this);
    return true;
}

/**************************实现函数********************************************
 *函数原型:     void Icm45686Reader::stop(void)
 *功    能:     停止后台读取线程
 *
 * 设置 running_=false → 等待 readLoop 线程退出 → join
 * 即使线程当前在 nanosleep 中，下一次醒来后会检查 running_ 并退出。
 * 最坏情况等待一个 period（1/sampleHz 秒）。
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void Icm45686Reader::stop()
{
    if (!running_.load()) {
        return;
    }

    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::isRunning(void) const
 *功    能:     查询读取线程是否正在运行
 *输入参数:     无
 *输出参数:     true运行中，false未运行
 ******************************************************************************/
bool Icm45686Reader::isRunning() const
{
    return running_.load();
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::getLatestSample(ImuSample& sample) const
 *功    能:     获取最新IMU样本
 *输入参数:     sample - 输出样本
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::getLatestSample(ImuSample& sample) const
{
    return ringBuffer_.latest(sample);
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::getSamplesBetween(uint64_t startTimeNs,
 *                                                     uint64_t endTimeNs,
 *                                                     std::vector<ImuSample>& samples) const
 *功    能:     从环形缓冲区中按时间范围获取样本
 *
 * 透明代理到 ringBuffer_.getSamplesBetween()。
 *输入参数:     startTimeNs - 起始时间戳，endTimeNs - 结束时间戳
 *输出参数:     samples - 样本列表
 *返 回 值:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::getSamplesBetween(uint64_t startTimeNs,
                                       uint64_t endTimeNs,
                                       std::vector<ImuSample>& samples) const
{
    return ringBuffer_.getSamplesBetween(startTimeNs, endTimeNs, samples);
}

/* ---------------------------------------------------------------------------
 * getAssistState — 获取 IMU 辅助状态（震动等级评估）
 *
 * 用途：上位机（如 SentinelVisioner）可以用这个接口判断当前是否处于高震动环境，
 *       从而决定是否动态降低曝光时间、调整防抖参数等策略。
 *
 * 震动等级判定（基于陀螺仪 RMS）：
 *   0 (低震动): gyroRms < 0.03 rad/s  → 约 < 2°/s，几乎静止
 *   1 (中震动): 0.03 ~ 0.15 rad/s      → 轻微振动
 *   2 (高震动): > 0.15 rad/s           → 剧烈震动，可能需要降低曝光
 * --------------------------------------------------------------------------- */
bool Icm45686Reader::getAssistState(ImuAssistState& state, uint32_t windowMs) const
{
    ImuSample latestSample;
    if (!getLatestSample(latestSample)) {
        return false;
    }

    uint64_t endNs = latestSample.timestampNs;
    uint64_t windowNs = (uint64_t)windowMs * NS_PER_MS;
    uint64_t startNs = endNs > windowNs ? endNs - windowNs : 0;

    /* 取最近 windowMs 毫秒内的样本计算 RMS */
    std::vector<ImuSample> samples;
    if (!getSamplesBetween(startNs, endNs, samples)) {
        samples.push_back(latestSample);
    }

    /* 计算陀螺仪 RMS = sqrt(mean(gyroX² + gyroY² + gyroZ²)) */
    double gyro2Sum = 0.0;
    for (const auto& s : samples) {
        gyro2Sum += (double)s.gyroX * s.gyroX + (double)s.gyroY * s.gyroY + (double)s.gyroZ * s.gyroZ;
    }

    state.timestampNs = latestSample.timestampNs;
    state.accelX = latestSample.accelX;
    state.accelY = latestSample.accelY;
    state.accelZ = latestSample.accelZ;
    state.gyroX = latestSample.gyroX;
    state.gyroY = latestSample.gyroY;
    state.gyroZ = latestSample.gyroZ;
    state.accelNorm = vec3_norm(state.accelX, state.accelY, state.accelZ);
    state.gyroNorm = vec3_norm(state.gyroX, state.gyroY, state.gyroZ);
    state.gyroRms = samples.empty() ? state.gyroNorm : (float)std::sqrt(gyro2Sum / (double)samples.size());

    /* 三级震动分类 */
    if (state.gyroRms < 0.03f) {
        state.vibrationLevel = 0;
    } else if (state.gyroRms < 0.15f) {
        state.vibrationLevel = 1;
    } else {
        state.vibrationLevel = 2;
    }

    return true;
}

size_t Icm45686Reader::bufferedSamples() const
{
    return ringBuffer_.size();
}

uint64_t Icm45686Reader::totalSamples() const
{
    return totalSamples_.load();
}

uint64_t Icm45686Reader::failedReads() const
{
    return failedReads_.load();
}

/**************************实现函数********************************************
 *函数原型:     void Icm45686Reader::readLoop(void)
 *功    能:     后台线程读取函数
 *
 * 定时循环：
 *   1. 调用 readSample() 从内核驱动读一帧 IMU 数据
 *   2. 推入环形缓冲区
 *   3. 精确睡眠到下一个采样时刻（基于绝对时间而非相对时间，避免累积漂移）
 *
 * 如果某次读取耗时过长导致错过下次采样时刻，会重置时间基准避免追赶。
 * 这种"精确睡眠"设计保证了 IMU 采样间隔的均匀性。
 *
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void Icm45686Reader::readLoop()
{
    const uint64_t periodNs = (uint64_t)(NS_PER_SEC / sampleHz_);
    uint64_t nextTime = imu_get_time_ns();

    while (running_.load()) {
        ImuSample sample;

        if (readSample(sample)) {
            ringBuffer_.push(sample);
            totalSamples_.fetch_add(1);
        } else {
            failedReads_.fetch_add(1);
        }

        /* 精确到下一采样时刻，基于绝对时间防止累积漂移 */
        nextTime += periodNs;
        uint64_t now = imu_get_time_ns();
        if (nextTime > now) {
            uint64_t sleepNs = nextTime - now;
            struct timespec ts;
            ts.tv_sec = sleepNs / NS_PER_SEC;
            ts.tv_nsec = sleepNs % NS_PER_SEC;
            nanosleep(&ts, NULL);
        } else {
            /* 本次耗时已经超过周期，重置基准避免追赶 */
            nextTime = now;
        }
    }
}

/* ============================================================================
 * EisStabilizer —— EIS 防抖计算器
 *
 * 这是整个防抖系统的算法核心。提供两套算法：
 *
 *   A. calculate_eis_offset() —— 简单小角度近似法
 *      - 陀螺仪梯形积分 → 小角度近似 → 焦距比例缩放 → clamp
 *      - 适合：快速验证、低算力场景
 *      - 局限：不能正确处理大角度旋转、不考虑相机内外参
 *
 *   B. calculate_imu_only_eis_offset() —— 全 3D 单应变换法（生产使用）
 *      - 四元数连续积分 → SLERP 频率分离 → 补偿四元数 →
 *        外参变换到相机坐标系 → 内参单应映射 → 像素偏移
 *      - 核心思想：qRaw（实时姿态）和 qSmooth（平滑姿态）的差异 =
 *        需要补偿的高频抖动 → 转为 2D 像素偏移
 *
 * 坐标系约定：
 *   + 机体坐标系 B (Body)：
 *       +X_B = 指向 CAM1 方向（设备右侧）
 *       +Y_B = 垂直于屏幕向外（设备前方）
 *       +Z_B = 朝上
 *
 *   + 相机坐标系 C (Camera)：
 *       +X_C = 图像向右
 *       +Y_C = 图像向下
 *       +Z_C = 相机视线方向向外
 *
 *   + IMU 原始坐标系 (raw)：
 *       芯片的 XYZ 轴，由芯片物理封装决定
 *
 * 坐标变换链：
 *   IMU raw → (R_B_imu_raw) → Body B → (R_C_B) → Camera C → (K) → Pixel
 * ============================================================================ */

/**************************实现函数********************************************
 *函数原型:     EisStabilizer::EisStabilizer(void)
 *功    能:     构造EIS防抖计算器
 *
 * 默认参数：
 *   signX_ = -1, signY_ = 1  适应典型的 IMU-相机安装方向
 *   maxOffsetPixel_ = 200    较大的默认限幅，兼容 1080p/4K
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
EisStabilizer::EisStabilizer()
    : reader_(NULL),
      lastCostMs_(0.0),
      lastUsedSamples_(0),
      signX_(-1.0f),
      signY_(1.0f),
      maxOffsetPixel_(200)
{
}

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::bindReader(Icm45686Reader* reader)
 *功    能:     绑定IMU读取器
 *
 * Stabilizer 自己不拥有 reader，只持有一个指针。
 * 调用方负责保证 reader 在 Stabilizer 使用期间存活。
 *输入参数:     reader - 读取器指针
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool EisStabilizer::bindReader(Icm45686Reader* reader)
{
    reader_ = reader;
    return reader_ != NULL;
}

/**************************实现函数********************************************
 *函数原型:     void EisStabilizer::setAxisSign(float signX, float signY)
 *功    能:     设置像素偏移方向符号，用于适配IMU安装方向和图像坐标系
 *
 * 为什么需要符号反转？
 *   IMU 测到"相机向右转" → 画面内容向左移动 → 补偿需要向右裁剪
 *   如果硬件安装导致 IMU 的正方向定义相反，就需要翻转符号。
 *
 *   实际调试中通常的做法：
 *     1. 开启防抖，让设备向右转
 *     2. 观察画面是更稳了还是更抖了
 *     3. 如果更抖了，把对应的 sign 取反
 *
 *输入参数:     signX - X方向符号（+1或-1），signY - Y方向符号（+1或-1）
 *输出参数:     无
 ******************************************************************************/
void EisStabilizer::setAxisSign(float signX, float signY)
{
    signX_ = signX >= 0.0f ? 1.0f : -1.0f;
    signY_ = signY >= 0.0f ? 1.0f : -1.0f;
}

/**************************实现函数********************************************
 *函数原型:     void EisStabilizer::setMaxOffset(int32_t maxOffsetPixel)
 *功    能:     设置最大补偿像素偏移，防止异常陀螺数据导致输出过大
 *
 * 例如设为 200 表示补偿偏移不会超过 ±200 像素。
 * 对于 1080p 画面，200px ≈ 18.5% 的画面宽度。
 * 实际裁剪量由 RGA 的 margin 参数配合控制。
 *
 *输入参数:     maxOffsetPixel - 最大偏移像素，≤0 时重置为默认 200
 *输出参数:     无
 ******************************************************************************/
void EisStabilizer::setMaxOffset(int32_t maxOffsetPixel)
{
    maxOffsetPixel_ = maxOffsetPixel > 0 ? maxOffsetPixel : 200;
}

/* ============================================================================
 * 算法 A：integrateGyro + calculate_eis_offset（简单小角度近似法）
 *
 * 工作原理：
 *   Step 1 — 梯形积分
 *     对时间窗口内相邻样本的角速度取平均，乘以时间间隔，累加得到角度增量。
 *     梯形积分比矩形积分精度高一阶（O(dt²) vs O(dt)）。
 *
 *   Step 2 — 小角度近似
 *     offsetX ≈ signX_ * focalX * thetaY   （绕 Y 轴旋转 → 水平偏移）
 *     offsetY ≈ signY_ * focalY * thetaX   （绕 X 轴旋转 → 垂直偏移）
 *
 *     核心假设：tan(θ) ≈ θ 当 θ 很小时（如 < 5° 时误差 < 0.25%）
 *
 *   Step 3 — Clamp
 *     截断到 [-maxOffsetPixel, maxOffsetPixel]，防止异常值导致画面飞走
 *
 * 适用场景：
 *   - 小幅抖动（< 5°）
 *   - 不需要外参标定
 *   - 计算量极小（几微秒）
 *
 * 局限：
 *   - 不考虑相机旋转轴心位置
 *   - 不考虑三轴耦合（绕 Z 轴的 roll 旋转被忽略）
 *   - 不考虑相机内外参
 * ============================================================================ */

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::integrateGyro(const std::vector<ImuSample>& samples,
 *                                                float& thetaX, float& thetaY, float& thetaZ) const
 *功    能:     对时间窗口内陀螺仪角速度进行梯形积分
 *
 * 梯形积分公式：
 *   对于每对相邻样本 (i-1, i)：
 *     dt = (timestamp[i] - timestamp[i-1]) / 1e9    （ns → s）
 *     theta += 0.5 * (gyro[i-1] + gyro[i]) * dt
 *
 * 为什么是梯形而不是矩形？
 *   矩形积分 = gyro[i-1] * dt（用前一点近似整个区间）
 *   梯形积分 = 0.5*(gyro[i-1]+gyro[i])*dt（用区间两端平均值）
 *   梯形法的误差是 O(h²)，矩形法是 O(h)，精度高一阶。
 *
 *输入参数:     samples - IMU样本列表（按时间升序）
 *输出参数:     thetaX/thetaY/thetaZ - 三轴角度增量，单位rad
 *返 回 值:     true成功，false失败
 ******************************************************************************/
bool EisStabilizer::integrateGyro(const std::vector<ImuSample>& samples,
                                  float& thetaX,
                                  float& thetaY,
                                  float& thetaZ) const
{
    thetaX = 0.0f;
    thetaY = 0.0f;
    thetaZ = 0.0f;

    /* 至少需要 2 个样本才能计算区间积分 */
    if (samples.size() < 2) {
        return false;
    }

    for (size_t i = 1; i < samples.size(); ++i) {
        /* 时间间隔（ns → s） */
        uint64_t dtNs = samples[i].timestampNs - samples[i - 1].timestampNs;
        float dt = (float)((double)dtNs / (double)NS_PER_SEC);

        /* 梯形积分：用区间两端角速度的平均值估算区间平均角速度 */
        thetaX += 0.5f * (samples[i - 1].gyroX + samples[i].gyroX) * dt;
        thetaY += 0.5f * (samples[i - 1].gyroY + samples[i].gyroY) * dt;
        thetaZ += 0.5f * (samples[i - 1].gyroZ + samples[i].gyroZ) * dt;
    }

    return true;
}

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::calculate_eis_offset(float focalX, float focalY,
 *                                                       uint64_t targetTimestampNs,
 *                                                       uint32_t halfWindowMs,
 *                                                       int32_t& offsetX, int32_t& offsetY)
 *功    能:     计算防抖像素补偿量（简单小角度近似法）
 *
 * 完整步骤：
 *   1. 确定 IMU 查询窗口：[targetTs - halfWindow, targetTs + halfWindow]
 *      halfWindow 的意义：曝光不是瞬间完成的，取曝光中点前后各半窗的 IMU 数据
 *   2. 从环形缓冲区查询该窗口内的样本
 *   3. 梯形积分得到三轴角度增量
 *   4. 小角度近似：offset = focal * theta
 *      gyroY → offsetX（绕 Y 轴旋转 → 水平像素偏移）
 *      gyroX → offsetY（绕 X 轴旋转 → 垂直像素偏移）
 *   5. 截断到 [-maxOffsetPixel, maxOffsetPixel]
 *
 *输入参数:     focalX/focalY - 相机焦距，单位pixel；targetTimestampNs - 目标曝光时间戳；halfWindowMs - 时间窗口半径
 *输出参数:     offsetX/offsetY - 像素补偿量
 *返 回 值:     true成功，false失败
 ******************************************************************************/
bool EisStabilizer::calculate_eis_offset(float focalX,
                                         float focalY,
                                         uint64_t targetTimestampNs,
                                         uint32_t halfWindowMs,
                                         int32_t& offsetX,
                                         int32_t& offsetY)
{
    uint64_t costStartNs = imu_get_time_ns();
    uint64_t windowNs = (uint64_t)halfWindowMs * NS_PER_MS;
    uint64_t startNs;
    uint64_t endNs;
    std::vector<ImuSample> samples;
    float thetaX, thetaY, thetaZ;

    offsetX = 0;
    offsetY = 0;
    lastUsedSamples_ = 0;
    lastCostMs_ = 0.0;

    /* 参数检查 */
    if (!reader_ || focalX <= 0.0f || focalY <= 0.0f || halfWindowMs == 0) {
        return false;
    }

    /* 以帧曝光时间戳为中心，前后各取 halfWindowMs 的 IMU 数据 */
    startNs = targetTimestampNs > windowNs ? targetTimestampNs - windowNs : 0;
    endNs = targetTimestampNs + windowNs;

    if (!reader_->getSamplesBetween(startNs, endNs, samples)) {
        return false;
    }

    lastUsedSamples_ = samples.size();

    /* Step 1: 梯形积分 → 三轴角度增量 */
    if (!integrateGyro(samples, thetaX, thetaY, thetaZ)) {
        return false;
    }

    /*
     * Step 2: 小角度近似 → 像素偏移
     *
     * 原理（以 offsetX 为例）：
     *   相机绕 Y 轴旋转 thetaY 弧度 → 图像上一点水平移动了
     *     dx = focalX * tan(thetaY) 像素
     *   当 thetaY 很小时 tan(thetaY) ≈ thetaY，所以
     *     dx ≈ focalX * thetaY
     *
     *   signX_/signY_ 修正 IMU 安装方向与图像坐标系的不一致。
     *   默认 signX_=-1, signY_=1 适应常见的前置 IMU 安装。
     */
    offsetX = (int32_t)lroundf(signX_ * focalX * thetaY);
    offsetY = (int32_t)lroundf(signY_ * focalY * thetaX);

    /* Step 3: 安全限幅 */
    offsetX = std::max(-maxOffsetPixel_, std::min(maxOffsetPixel_, offsetX));
    offsetY = std::max(-maxOffsetPixel_, std::min(maxOffsetPixel_, offsetY));

    lastCostMs_ = (double)(imu_get_time_ns() - costStartNs) / 1000000.0;
    return true;
}


/* ============================================================================
 * 算法 B：calculate_imu_only_eis_offset（全 3D 单应变换法）
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * 这是生产环境使用的完整防抖算法。核心直觉：
 *
 *   想象一个人手持相机拍摄——手在缓慢平移（有意运动），同时手指在
 *   高频抖动（不需要的抖动）。如果 IMU 能区分"手的慢运动"和
 *   "手指的快抖动"，就能只补偿后者。
 *
 * 算法将问题分解为六个步骤：
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ Step 1: 陀螺仪 → 四元数连续积分                                        │
 * │   对 [上次积分时刻, 当前帧时刻] 之间的每个 IMU 样本：                    │
 * │     gyro_B  = R_B_imu_raw * gyro_raw    (IMU芯片 → 机体坐标系)         │
 * │     dθ      = gyro_B * dt                (本步的角度增量向量)           │
 * │     dq      = quat_from_rotvec(dθ)       (增量 → 四元数)               │
 * │     qRaw_B  = qRaw_B * dq                (累积积分)                     │
 * │                                                                         │
 * │   qRaw_B 表示设备在机体坐标系下的实时姿态，从启动开始持续积分。          │
 * │   ⚠ 持续积分会导致漂移，但防抖只关心"相对抖动"而非绝对姿态，            │
 * │   所以漂移不影响效果。                                                  │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ Step 2: SLERP 平滑（频率分离）                                          │
 * │   α      = 1 - exp(-dtFrame / τ)        (指数平滑系数)                  │
 * │   qSmooth_B = SLERP(qSmooth_B, qRaw_B, α)                              │
 * │                                                                         │
 * │   这是防抖最巧妙的部分：                                                │
 * │     qRaw_B     = 即时姿态（含所有频率）                                  │
 * │     qSmooth_B  = 平滑姿态（只保留低频 = 有意运动）                       │
 * │     α          = 平滑强度，由 τ（时间常数）和帧间隔共同决定               │
 * │                                                                         │
 * │   τ 越大 → α 越小 → qSmooth 变化越慢 → 补偿越多 → 画面越稳              │
 * │   τ 越小 → α 越大 → qSmooth 紧跟 qRaw → 补偿越少 → 画面跟手            │
 * │                                                                         │
 * │   典型值：τ = 0.25s 是稳和跟手之间的平衡点                              │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ Step 3: 补偿四元数                                                      │
 * │   qComp_B = qSmooth_B * qRaw_B⁻¹                                      │
 * │   clampAngle(qComp_B, maxCompAngleRad)   (限制最大补偿角度)             │
 * │                                                                         │
 * │   物理含义：要"撤销抖动"，就是把实时姿态"拉回"到平滑姿态。               │
 * │   qRaw_B⁻¹ 撤销实时的旋转，qSmooth_B 再转到平滑姿态，                   │
 * │   两者合成就是需要的补偿旋转。                                          │
 * │                                                                         │
 * │   maxCompAngleRad（默认 5°）防止异常值导致补偿过大。                     │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ Step 4: 补偿旋转 → 相机坐标系                                           │
 * │   RCompB    = quat_to_mat3(qComp_B)      (四元数 → 旋转矩阵)            │
 * │   RCompC    = R_C_B * RCompB * R_C_Bᵀ   (相似变换到相机坐标系)         │
 * │                                                                         │
 * │   R_C_B 是"机体 → 相机"的外参旋转矩阵。                                 │
 * │   因为 IMU 的安装位置和方向因设备设计而异，                              │
 * │   R_C_B 把在机体坐标系下计算的补偿量旋转到相机坐标系下。                  │
 * │                                                                         │
 * │   两路相机（CAM0 左视、CAM1 右视）有不同的 R_C_B，                       │
 * │   因为它们相对机体的安装朝向不同。                                       │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ Step 5: 旋转 → 像素单应矩阵                                              │
 * │   H = K * RCompC * K⁻¹                                                │
 * │                                                                         │
 * │   这是计算机视觉中的经典公式：                                           │
 * │     相机纯旋转（不平移）引起的图像变换是一个单应矩阵 H。                  │
 * │     H 将"静止相机拍的图像"映射到"旋转后相机拍的图像"。                   │
 * │                                                                         │
 * │   推导：                                                                 │
 * │     - 一个 3D 点 X 在静止相机下投影为 x₁ = K * I * X = KX              │
 * │     - 同一个点在旋转后相机下投影为 x₂ = K * RCompC * X                  │
 * │     - 所以 x₂ = K * RCompC * K⁻¹ * x₁ = H * x₁                         │
 * │                                                                         │
 * │   注意：这里假设场景点都在同一深度（平面场景假设）。                      │
 * │   对于远距离场景（如安防监控）这个假设近似成立；                          │
 * │   对于近距离场景，平移补偿（lever arm compensation）才需要深度信息。     │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ Step 6: 图像中心偏移量                                                   │
 * │   (centerAfterX, centerAfterY) = H * (cx, cy)                          │
 * │   offsetX = cx - centerAfterX                                          │
 * │   offsetY = cy - centerAfterY                                          │
 * │                                                                         │
 * │   双重限幅：                                                             │
 * │     - maxOffsetPixel      绝对上限，防止画面飞出去                       │
 * │     - maxOffsetStepPixel  帧间变化量上限，防止画面突变                   │
 * │                                                                         │
 * │   最后输出 offsetX / offsetY 给 RGA 硬件做裁剪。                         │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * setImuOnlyConfig — 设置单路相机的完整防抖配置
 *
 * 每路相机独立的配置包括：
 *   - intr:  内参（fx, fy, cx, cy）—— 相机焦距和主点
 *   - extr:  外参（R_C_B, t_B）—— 相机在机体上的安装位置和朝向
 *   - R_B_imu_raw: IMU 芯片坐标系 → 机体坐标系的旋转
 *   - timeOffsetNs: 时间偏移（补偿相机曝光延迟）
 *   - smoothTauSec: SLERP 平滑时间常数（控制防抖强度）
 *   - maxCompAngleRad: 最大补偿角度上限
 *   - maxOffsetPixel / maxOffsetStepPixel: 像素偏移限幅
 * --------------------------------------------------------------------------- */
void EisStabilizer::setImuOnlyConfig(int camId, const ImuOnlyEisConfig& config)
{
    if (camId < 0 || camId >= 2) {
        return;
    }
    imuOnlyCfg_[camId] = config;
    resetImuOnlyState(camId);   /* 换配置后状态必须清零，否则 qRaw 和 qSmooth 不匹配 */
}

/* ---------------------------------------------------------------------------
 * resetImuOnlyState — 重置单路或全部状态
 *
 * 在以下场景需要重置：
 *   1. 切换配置参数（如调整 smoothTauSec）
 *   2. IMU 断连重连后
 *   3. 相机重新标定后
 *
 * 不重置会导致 qRaw 保持旧坐标系下的积分值，新配置的 R_B_imu_raw 不兼容。
 * --------------------------------------------------------------------------- */
void EisStabilizer::resetImuOnlyState(int camId)
{
    if (camId >= 0 && camId < 2) {
        imuOnlyState_[camId] = ImuOnlyCameraState();
        return;
    }
    /* camId < 0: 重置全部相机 */
    imuOnlyState_[0] = ImuOnlyCameraState();
    imuOnlyState_[1] = ImuOnlyCameraState();
}

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::calculate_imu_only_eis_offset(
 *                  int camId, uint64_t frameTimestampNs,
 *                  int32_t& offsetX, int32_t& offsetY,
 *                  ImuOnlyEisOutput* out)
 *功    能:     计算防抖像素补偿量（全3D单应变换法，生产版本）
 *
 * 这是整个防抖系统的核心函数，每帧调用一次。
 *
 *输入参数:
 *   camId            - 相机编号（0 或 1），每路独立状态和配置
 *   frameTimestampNs - 当前帧的曝光时间戳（CLOCK_MONOTONIC，ns）
 *   out              - 可选输出，填入调试信息（单应矩阵、各阶段角度等）
 *
 *输出参数:
 *   offsetX/offsetY  - 需要在图像上裁剪的像素偏移量
 *
 *返回值:
 *   true  - 计算成功，offsetX/offsetY 有效
 *   false - 计算失败（IMU 未就绪、缓冲区无数据等），offset=0
 *
 *详细算法步骤见本节开头的大框图。
 ******************************************************************************/
bool EisStabilizer::calculate_imu_only_eis_offset(int camId,
                                                  uint64_t frameTimestampNs,
                                                  int32_t& offsetX,
                                                  int32_t& offsetY,
                                                  ImuOnlyEisOutput* out)
{
    uint64_t costStartNs = imu_get_time_ns();
    offsetX = 0;
    offsetY = 0;
    lastUsedSamples_ = 0;
    lastCostMs_ = 0.0;

    if (out) {
        *out = ImuOnlyEisOutput();
    }

    /* 前置条件检查：reader 就绪 + camId 合法 */
    if (!reader_ || camId < 0 || camId >= 2) {
        return false;
    }

    ImuOnlyEisConfig& cfg = imuOnlyCfg_[camId];
    ImuOnlyCameraState& st = imuOnlyState_[camId];

    /* 加上时间偏移：补偿相机曝光到 IMU 读取之间的固定延迟 */
    const uint64_t targetNs = frameTimestampNs + (uint64_t)cfg.timeOffsetNs;

    /* 获取最新 IMU 样本用于兜底和初始化 */
    ImuSample latest;
    if (!reader_->getLatestSample(latest)) {
        return false;
    }

    /* ── 首次调用：初始化四元数状态 ── */
    if (!st.initialized) {
        quat_identity(st.qRaw);
        quat_identity(st.qSmooth);
        st.lastIntegratedNs = latest.timestampNs;
        st.lastFrameNs = targetNs;
        st.initialized = true;
        if (out) {
            out->valid = true;
        }
        return true;   /* 返回 offset=0；第一帧不做补偿，从第二帧开始 */
    }

    /* ── Step 1: 确定需要积分的时间范围 ── */
    /* 从上次积分结束的时刻到当前帧时刻（含 timeOffset）之间的 IMU 数据 */
    uint64_t startNs = st.lastIntegratedNs;
    if (startNs == 0 || startNs > targetNs) {
        /* 异常保护：如果上次积分时间异常，回退到最近 50ms */
        startNs = targetNs > 50 * NS_PER_MS ? targetNs - 50 * NS_PER_MS : 0;
    }

    std::vector<ImuSample> samples;
    if (!reader_->getSamplesBetween(startNs, targetNs, samples)) {
        /* 查询失败时用最近 50ms 数据兜底 */
        uint64_t fallbackStart = latest.timestampNs > 50 * NS_PER_MS ? latest.timestampNs - 50 * NS_PER_MS : 0;
        reader_->getSamplesBetween(fallbackStart, latest.timestampNs, samples);
    }

    if (samples.size() < 2) {
        return false;   /* 至少需要 2 个样本做梯形积分 */
    }

    lastUsedSamples_ = samples.size();

    /* ── Step 2: 逐对样本积分更新 qRaw_B ── */
    /* 遍历每一对连续样本，逐个做旋转向量 → 四元数转换并累积 */
    float lastGyroRaw[3] = {0.0f, 0.0f, 0.0f};
    float lastGyroB[3] = {0.0f, 0.0f, 0.0f};

    for (size_t i = 1; i < samples.size(); ++i) {
        /* 跳过时间戳倒退的异常样本 */
        if (samples[i].timestampNs <= samples[i - 1].timestampNs) {
            continue;
        }

        /* dt：相邻两样本的时间间隔（秒） */
        float dt = (float)((double)(samples[i].timestampNs - samples[i - 1].timestampNs) / (double)NS_PER_SEC);

        /* 过滤异常大的 dt（IMU 读取卡顿导致）：超过 50ms 跳过，
           因为会引入过大的积分误差 */
        if (dt <= 0.0f || dt > 0.05f) {
            continue;
        }

        /* 2.1 取两样本的平均角速度（梯形积分的中点近似） */
        float gyroRawAvg[3] = {
            0.5f * (samples[i - 1].gyroX + samples[i].gyroX),
            0.5f * (samples[i - 1].gyroY + samples[i].gyroY),
            0.5f * (samples[i - 1].gyroZ + samples[i].gyroZ)
        };

        /* 2.2 IMU 芯片坐标系 → 机体坐标系
         *
         * R_B_imu_raw 是 3×3 旋转矩阵，将 IMU 芯片测量值转到机体坐标系。
         *
         * 实测映射关系（以本项目的 IMU 安装方向为例）：
         *   gyro_B.x = -gyro_raw.y     (芯片 Y 轴 → 机体 X 轴，取反)
         *   gyro_B.y = -gyro_raw.x     (芯片 X 轴 → 机体 Y 轴，取反)
         *   gyro_B.z =  gyro_raw.z     (芯片 Z 轴 → 机体 Z 轴，同向)
         *
         * 注释：此处的 R_B_imu_raw 默认值 =
         *   [ 0  -1   0 ]
         *   [-1   0   0 ]
         *   [ 0   0   1 ]
         */
        float gyroB[3];
        mat3_mul_vec3(cfg.R_B_imu_raw, gyroRawAvg, gyroB);

        /* 2.3 旋转向量 = 角速度 × 时间 = 这段时间内的微小旋转 */
        float dtheta[3] = {gyroB[0] * dt, gyroB[1] * dt, gyroB[2] * dt};

        /* 2.4 旋转向量 → 增量四元数 */
        float dq[4];
        quat_from_rotvec(dtheta, dq);

        /* 2.5 qRaw_B = qRaw_B * dq（被动旋转约定） */
        float qNew[4];
        quat_mul(st.qRaw, dq, qNew);
        memcpy(st.qRaw, qNew, sizeof(qNew));
        quat_normalize(st.qRaw);   /* 每次乘后归一化，防止累积误差发散 */

        /* 保存最后一步的角速度，供调试输出 */
        memcpy(lastGyroRaw, gyroRawAvg, sizeof(lastGyroRaw));
        memcpy(lastGyroB, gyroB, sizeof(lastGyroB));
    }

    /* 更新"上次积分到"的时间戳，下次从这继续 */
    st.lastIntegratedNs = samples.back().timestampNs;

    /* ── Step 3: SLERP 平滑（频率分离） ── */

    /* 3.1 计算帧间隔 dtFrame */
    float dtFrame = 1.0f / 30.0f;   /* 默认假设 30fps */
    if (st.lastFrameNs > 0 && targetNs > st.lastFrameNs) {
        dtFrame = (float)((double)(targetNs - st.lastFrameNs) / (double)NS_PER_SEC);
        if (dtFrame <= 0.0f || dtFrame > 0.2f) {
            dtFrame = 1.0f / 30.0f;  /* 异常保护：间隔 > 200ms 回退到 30fps 默认 */
        }
    }
    st.lastFrameNs = targetNs;

    /* 3.2 计算指数平滑系数 α
     *
     * α = 1 - exp(-dtFrame / τ)
     *
     * 这个公式来自一阶低通滤波器的离散化：
     *   y[n] = α·x[n] + (1-α)·y[n-1]
     *   连续时间常数 τ 的对应关系：α = 1 - exp(-Ts/τ)
     *
     * τ (smoothTauSec) 的含义：
     *   τ = 0.10s → 很跟手，快速响应（适合 FPV 穿越机）
     *   τ = 0.25s → 平衡点（适合手持拍摄）
     *   τ = 0.50s → 很稳，但跟手慢（适合监控/固定机位）
     *
     * 为什么用指数平滑而不是 LERP？
     *   指数平滑的 α 随帧率自适应（dtFrame 变化时 α 自动调整），
     *   而固定 LERP 因子在帧率波动时会表现出不同的平滑强度。
     */
    float tau = cfg.smoothTauSec > 0.001f ? cfg.smoothTauSec : 0.25f;
    float alpha = 1.0f - std::exp(-dtFrame / tau);
    if (alpha < 0.001f) alpha = 0.001f;   /* 下限：至少有一点跟随，否则永远不跟 */
    if (alpha > 1.0f) alpha = 1.0f;       /* 上限：不超过 1 */

    /* 3.3 SLERP 平滑：qSmooth_B 向 qRaw_B 移动 alpha 比例 */
    float qSmoothNew[4];
    quat_slerp(st.qSmooth, st.qRaw, alpha, qSmoothNew);
    memcpy(st.qSmooth, qSmoothNew, sizeof(qSmoothNew));
    quat_normalize(st.qSmooth);

    /* ── Step 4: 补偿四元数 ── */
    /* qComp_B = qSmooth_B * qRaw_B⁻¹
     *
     * 直观理解：
     *   假设设备姿态是 A，我们想要它是 B。
     *   "把 A 变成 B" 的旋转 = B * A⁻¹
     *   这里 A = qRaw（实时的），B = qSmooth（想要的），
     *   所以补偿 = qSmooth * qRaw⁻¹
     */
    float qRawInv[4];
    quat_inverse(st.qRaw, qRawInv);

    float qCompB[4];
    quat_mul(st.qSmooth, qRawInv, qCompB);
    quat_normalize(qCompB);

    /* 限制最大补偿角度，防止陀螺尖峰导致画面突然大幅偏移 */
    quat_clamp_angle(qCompB, cfg.maxCompAngleRad);

    /* 补偿四元数 → 旋转矩阵 */
    float RCompB[9];
    quat_to_mat3(qCompB, RCompB);

    /* ── Step 5: 机体 → 相机坐标系变换 ── */
    /*
     * RCompC = R_C_B * RCompB * R_C_Bᵀ
     *
     * 这是一个相似变换（similarity transform），
     * 把在机体坐标系 B 下表达的旋转 RCompB 变换到相机坐标系 C 下。
     *
     * 类比：如果 R_C_B 是"B 语 → C 语"的翻译器，
     *   那么 B 语说 "绕某个轴转 θ"，
     *   翻译器把它变成 C 语说 "绕对应的轴转同样的 θ"。
     *
     * RCompC 就是 C 语版本的补偿旋转。
     */
    float RCB[9];
    memcpy(RCB, cfg.extr.R_C_B, sizeof(RCB));

    float RBC[9];
    mat3_transpose(RCB, RBC);          /* RBC = R_C_Bᵀ = R_B_C */

    float tmp[9];
    float RCompC[9];
    mat3_mul(RCB, RCompB, tmp);         /* tmp = R_C_B * RCompB        */
    mat3_mul(tmp, RBC, RCompC);         /* RCompC = tmp * R_B_C        */

    /* ── Step 6: 单应矩阵 + 像素偏移 ── */

    /* 6.1 构建相机内参矩阵 K 及其逆 */
    float K[9], Kinv[9];
    make_K(cfg.intr.fx, cfg.intr.fy, cfg.intr.cx, cfg.intr.cy, K);
    make_K_inv(cfg.intr.fx, cfg.intr.fy, cfg.intr.cx, cfg.intr.cy, Kinv);

    /* 6.2 H = K * RCompC * K⁻¹
     *
     * 这是一个 3×3 单应矩阵，含义：
     *   给定"无旋转"相机上的像素点 p，
     *   在"旋转了 RCompC"相机上，同一个 3D 点会出现在 p' = H * p
     *
     *   所以要把"旋转后的画面"校正回"无旋转的画面"，
     *   只需要把每个像素从 p' 位置搬回 p 位置。
     */
    float Htmp[9];
    float H[9];
    mat3_mul(K, RCompC, Htmp);          /* Htmp = K * RCompC            */
    mat3_mul(Htmp, Kinv, H);            /* H = Htmp * K⁻¹               */

    /* 归一化：齐次矩阵最后一项应为 1 */
    if (std::fabs(H[8]) > 1e-6f) {
        float inv = 1.0f / H[8];
        for (int i=0; i<9; ++i) H[i] *= inv;
    }

    /* 6.3 将 H 作用到图像中心 (cx, cy)，得到旋转后的中心位置 */
    float cx = cfg.intr.cx;
    float cy = cfg.intr.cy;
    float centerAfterX = cx;
    float centerAfterY = cy;
    apply_homography_center(H, cx, cy, centerAfterX, centerAfterY);

    /* 6.4 像素偏移 = 原始中心 - 旋转后中心
     *
     * 例如：相机右转 → 画面内容左移 → 中心点"跑"到了 (cx+10, cy)
     *       → centerAfterX = cx+10 → offsetX = cx - (cx+10) = -10
     *       → 画面向左裁剪 10px（把画面内容"拉回来"）
     */
    int32_t ox = (int32_t)lroundf(cx - centerAfterX);
    int32_t oy = (int32_t)lroundf(cy - centerAfterY);

    /* 6.5 绝对限幅：偏移量不能超过 maxOffsetPixel */
    if (cfg.maxOffsetPixel >= 0) {
        ox = std::max(-cfg.maxOffsetPixel, std::min(cfg.maxOffsetPixel, ox));
        oy = std::max(-cfg.maxOffsetPixel, std::min(cfg.maxOffsetPixel, oy));
    }

    /* 6.6 帧间限幅：相邻帧偏移量变化不能超过 maxOffsetStepPixel
     *
     * 这个限幅很关键——防止防抖从无到有的瞬间大幅跳变。
     * 例如：防抖刚开启时 offset 从 0 跳到 50 → 画面体验不好。
     * 加了帧间限幅后，每帧最多变化 step 像素，渐变到目标值。
     *
     * 典型值：
     *   cam0（左前方，稳定安装）：maxOffsetStepPixel = 6
     *   cam1（右前方，稳定安装）：maxOffsetStepPixel = 10
     */
    if (cfg.maxOffsetStepPixel > 0) {
        ox = std::max(st.lastOffsetX - cfg.maxOffsetStepPixel,
                      std::min(st.lastOffsetX + cfg.maxOffsetStepPixel, ox));
        oy = std::max(st.lastOffsetY - cfg.maxOffsetStepPixel,
                      std::min(st.lastOffsetY + cfg.maxOffsetStepPixel, oy));
    }

    /* 保存状态供下一帧使用 */
    st.lastOffsetX = ox;
    st.lastOffsetY = oy;
    offsetX = ox;
    offsetY = oy;

    /* ── 调试输出 ── */
    if (out) {
        out->valid = true;
        memcpy(out->H, H, sizeof(out->H));
        out->offsetX = ox;
        out->offsetY = oy;

        /* roll 角：用于评估防抖对旋转的补偿效果 */
        out->rollRad = std::atan2(RCompC[3], RCompC[0]);

        /* 各级角速度，用于调试算法各个环节 */
        memcpy(out->gyroRaw, lastGyroRaw, sizeof(out->gyroRaw));
        memcpy(out->gyroB, lastGyroB, sizeof(out->gyroB));

        /* 相机坐标系下的角速度 */
        mat3_mul_vec3(RCB, lastGyroB, out->gyroCam);

        /* 原始和平滑后的欧拉角 */
        quat_to_euler_xyz(st.qRaw, out->rawAngleB);
        quat_to_euler_xyz(st.qSmooth, out->smoothAngleB);
    }

    lastCostMs_ = (double)(imu_get_time_ns() - costStartNs) / 1000000.0;

    /* 调试日志：cfg.debugLog=true 时打印完整诊断信息 */
    if (cfg.debugLog) {
        float gyroCam[3];
        mat3_mul_vec3(RCB, lastGyroB, gyroCam);
        fprintf(stderr,
                "[IMU-only EIS Cam %d] offset=(%d,%d) gyroRaw=(%.4f,%.4f,%.4f) gyroB=(%.4f,%.4f,%.4f) gyroCam=(%.4f,%.4f,%.4f) roll=%.4f cost=%.3fms samples=%zu\n",
                camId, (int)offsetX, (int)offsetY,
                lastGyroRaw[0], lastGyroRaw[1], lastGyroRaw[2],
                lastGyroB[0], lastGyroB[1], lastGyroB[2],
                gyroCam[0], gyroCam[1], gyroCam[2],
                out ? out->rollRad : 0.0f,
                lastCostMs_, lastUsedSamples_);
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * 性能诊断接口
 *   lastCostMs()      - 上次 calculate 调用的耗时（毫秒），用于性能监控
 *   lastUsedSamples() - 上次使用了多少个 IMU 样本，用于判断 IMU 采样是否充足
 * --------------------------------------------------------------------------- */
double EisStabilizer::lastCostMs() const
{
    return lastCostMs_;
}

size_t EisStabilizer::lastUsedSamples() const
{
    return lastUsedSamples_;
}
