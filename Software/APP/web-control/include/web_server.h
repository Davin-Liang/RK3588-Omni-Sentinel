#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <QImage>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

/**
 * @brief 嵌入式 Web 服务器，提供 HTTP REST + WebSocket 远程控制接口
 *
 * 运行在独立 std::thread 中，通过 cpp-httplib 提供 HTTP 1.1 服务。
 * 与 Qt 主线程通过 CommandHandler (BlockingQueuedConnection) 和消息队列通信。
 */
class WebServer
{
public:
    explicit WebServer(uint16_t port);
    ~WebServer();

    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    /** @brief 启动 Web 服务器（阻塞在独立线程） */
    bool start();

    /** @brief 停止服务器，等待所有 in-flight 请求完成后 join 线程 */
    void stop();

    bool is_running() const;

    // ---- Qt 主线程 → WebServer 线程（非阻塞推送） ----

    /** @brief 推送系统状态快照到所有 WebSocket 客户端 (1Hz) */
    void push_status(const std::string& json);

    /** @brief 推送流/录像事件到所有 WebSocket 客户端（即时） */
    void push_event(const std::string& eventType, const std::string& json);

    /** @brief 推送融合跟踪目标数据到所有 WebSocket 客户端 (5Hz) */
    void push_tracking(const std::string& json);

    /** @brief 推送告警回溯通知到所有 WebSocket 客户端
     *  @param json  告警数据 JSON: {"targetId":"3","files":["backtrack_...mp4",...]} */
    void push_alert(const std::string& json);

    // ---- 预览帧缓存（线程安全，供 MJPEG 端点读取） ----

    QImage get_cached_preview(int camNum) const;
    void set_cached_preview(int camNum, const QImage& img);

    // ---- 命令回调 ----

    /**
     * @brief HTTP 命令处理器类型
     * @param method  HTTP 方法 ("GET", "POST", "PUT", "DELETE")
     * @param path    请求路径 (如 "/api/v1/cam/0/stream/start")
     * @param body    请求体 (JSON 字符串，GET 请求为空)
     * @return JSON 响应字符串 ({"ok": true} 或 {"ok": false, "error": "..."})
     *
     * 回调在 WebServer 线程中调用，内部必须通过 BlockingQueuedConnection
     * 将实际操作转发到 Qt 主线程。
     */
    using CommandHandler = std::function<std::string(
        const std::string& method,
        const std::string& path,
        const std::string& body)>;

    void set_command_handler(CommandHandler handler);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // WEB_SERVER_H
