#ifndef AI_REPORT_WORKER_H
#define AI_REPORT_WORKER_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <atomic>

#include "deepseek_inference.h"

/**
 * @brief AI 系统状态分析工作线程 — 在独立 QThread 中运行 DeepSeek 推理
 *
 * 设计目的：
 *   将耗时的 NPU 推理（~60~130 秒）从 Qt 主线程剥离，避免阻塞 UI 和相机预览。
 *   主线程每秒更新状态快照，用户触发（按钮/Web/定时器）时本线程生成报告。
 *
 * 线程模型：
 * @code
 *   主线程                      AIReportWorker 子线程
 *   ────────                    ─────────────────
 *   updateStatus() [每秒]         start()
 *     ├─ 加锁写快照              ├─ inference_.initialize() → 加载模型到 NPU
 *     └─ 解锁                    └─ while(running_):
 *                                    每 200ms 检测 pending_ 标志
 *   requestReport()                       │ pending_ == true
 *     └─ pending_ = true ──────────────→  │   ├─ buildPrompt_() → 读快照生成中文问题
 *                                          │   ├─ inference_.inferSync() → NPU 推理（阻塞 ~2 分钟）
 *                                          │   ├─ emit reportReady(result) → 主线程显示
 *                                          │   └─ pending_ = false
 *   on_ai_report_ready_()  ←───── Qt::QueuedConnection ──┘
 *     └─ 显示报告到屏幕和 Web
 * @endcode
 *
 * 线程安全：
 *   - updateStatus() 和 buildPrompt_() 通过 QMutex 保护共享状态快照
 *   - pending_ 使用 std::atomic + compare_exchange_strong 防重复触发
 *   - running_ 使用 std::atomic，主线程 stop() 写入，子线程轮询读取
 *   - reportReady / error 信号通过 Qt::QueuedConnection 跨线程投递（默认行为）
 */
class AIReportWorker : public QObject
{
    Q_OBJECT

public:
    explicit AIReportWorker(QObject* parent = nullptr);
    ~AIReportWorker();

    /**
     * @brief 设置推理参数（模型路径、温度等）
     * @param cfg 透传给 DeepSeekInference::initialize() 的配置
     * @note 必须在 start() 之前调用，线程启动后修改无效
     */
    void setConfig(const DeepSeekInference::Config& cfg);

    /**
     * @brief 更新系统状态快照（每秒由主线程调用）
     *
     * 所有参数都会被加锁拷贝到成员变量中，供 buildPrompt_() 读取。
     * 参数说明见 .cpp 文件中的详细注释。
     */
    void updateStatus(int cpuTemp, int cpuUsage,
                      const QString& cam0Status, const QString& cam1Status,
                      const QString& lidarStatus, const QString& imuStatus,
                      const QString& fusionStatus,
                      double cam0Fps, double cam1Fps);

public slots:
    /**
     * @brief Worker 线程主入口（由 QThread::started 信号触发）
     *
     * 工作流程：
     *   1. 调用 inference_.initialize() 加载模型到 NPU
     *   2. 进入 while(running_) 轮询，每 200ms 检查 pending_ 标志
     *   3. 检测到推理请求后：buildPrompt_() → inferSync() → emit reportReady/error
     *   4. 循环退出时 inference_.destroy() 释放 NPU 资源
     */
    void start();

    /**
     * @brief 通知 Worker 线程退出（设置 running_ = false）
     * @note 析构函数自动调用；调用后需要 QThread::wait() 等待线程退出
     */
    void stop();

    /**
     * @brief 触发一次 AI 分析（异步，立即返回）
     *
     * 并发保护：使用 compare_exchange_strong 原子操作，
     * 只有当前 pending_ == false 时才设置为 true。
     * 如果上一次推理还在进行，本次请求被静默忽略。
     *
     * 调用来源：
     *   - QT 屏幕按钮: on_btn_ai_analysis_()
     *   - Web 远程 API: web_ai_report_()
     *   - 自动定时器:   on_ai_auto_tick_() 倒计时归零
     */
    void requestReport();

signals:
    /**
     * @brief 推理完成信号（跨线程 QueuedConnection 投递到主线程）
     * @param report 模型生成的完整中文分析报告
     */
    void reportReady(const QString& report);

    /**
     * @brief 推理出错信号（模型初始化失败、推理超时、返回空等）
     * @param msg 错误描述，供 UI 显示
     */
    void error(const QString& msg);

private:
    // ---- 推理引擎 ----
    DeepSeekInference       inference_;  ///< 底层 RKLLM 封装实例（唯一，不可拷贝）
    DeepSeekInference::Config config_;   ///< 模型路径、温度等配置（start 前设置）

    // ---- 线程控制 ----
    std::atomic<bool>       running_{false};  ///< 控制轮询循环是否继续
    std::atomic<bool>       pending_{false};  ///< 是否有待处理的推理请求

    // ---- 系统状态快照（mutex 保护，主线程写 / Worker 线程读） ----
    QMutex  statusMutex_;        ///< 保护以下所有状态成员的互斥锁
    int     cpuTemp_   = 0;      ///< CPU 温度（°C）
    int     cpuUsage_  = 0;      ///< CPU 占用率（%）
    QString cam0Status_;         ///< 相机 0 状态描述（预览/推流/录像等）
    QString cam1Status_;         ///< 相机 1 状态描述
    QString lidarStatus_;        ///< 激光雷达状态（运行中/已停止，频率）
    QString imuStatus_;          ///< IMU 状态（运行中/未启用）
    QString fusionStatus_;       ///< 融合跟踪状态（目标数/确认数/告警/引擎状态）
    double  cam0Fps_   = 0.0;    ///< 相机 0 实时帧率
    double  cam1Fps_   = 0.0;    ///< 相机 1 实时帧率

    /**
     * @brief 将系统状态快照组装成 DeepSeek 可理解的中文问题
     * @return 约 400 字符的中文 prompt（不含 chat template 前缀/后缀）
     * @note 加锁读取状态快照，保证读到一致的副本
     */
    QString buildPrompt_();
};

#endif
