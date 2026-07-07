#include "web_server.h"

// 注意: httplib 使用 #ifdef 检测功能开关，定义为 0 照样视为"已定义"。
// 所以不要 #define CPPHTTPLIB_OPENSSL_SUPPORT 或 CPPHTTPLIB_ZLIB_SUPPORT。
// 只需定义线程池大小即可。
#define CPPHTTPLIB_THREAD_POOL_COUNT 2
#include "httplib.h"
#include "json.hpp"

#include <queue>
#include <mutex>
#include <set>
#include <atomic>
#include <thread>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
#include <QBuffer>

using json = nlohmann::json;

// ---- 前置声明 ----
static std::string load_html_file_();
static const char* get_embedded_html_();

// ======================================================================
//  Impl
// ======================================================================

struct WebServer::Impl
{
    uint16_t port;
    std::atomic<bool> running_{false};
    std::thread serverThread_;

    httplib::Server httpServer_;

    // WebSocket 连接集合（mutex 保护）
    std::mutex wsMutex_;
    std::set<httplib::ws::WebSocket*> wsClients_;

    // 消息发送队列（Qt 主线程 push → WebServer 线程消费广播）
    std::mutex queueMutex_;
    std::queue<std::string> sendQueue_;

    // 预览帧缓存（mutex 保护）
    mutable std::mutex previewMutex_;
    QImage cachedPreview_[2];

    // 命令回调
    WebServer::CommandHandler cmdHandler_;

    Impl(uint16_t p) : port(p) {}

    // 在锁内编码为 JPEG
    std::string encode_jpeg_locked_(int camNum) const
    {
        std::lock_guard<std::mutex> lk(previewMutex_);
        const QImage& img = cachedPreview_[camNum];
        if (img.isNull())
            return {};
        QByteArray buf;
        QBuffer buffer(&buf);
        buffer.open(QIODevice::WriteOnly);
        const_cast<QImage&>(img).save(&buffer, "JPEG", 75);
        return buf.toStdString();
    }

    void broadcast_to_ws(const std::string& msg)
    {
        std::lock_guard<std::mutex> lk(wsMutex_);
        for (auto* ws : wsClients_) {
            if (ws && ws->is_open()) {
                ws->send(msg);
            }
        }
    }

    void drain_send_queue()
    {
        std::lock_guard<std::mutex> lk(queueMutex_);
        while (!sendQueue_.empty()) {
            broadcast_to_ws(sendQueue_.front());
            sendQueue_.pop();
        }
    }
};

// ======================================================================
//  WebServer 生命周期
// ======================================================================

WebServer::WebServer(uint16_t port)
    : impl_(new Impl(port))
{
}

WebServer::~WebServer()
{
    stop();
}

bool WebServer::start()
{
    if (impl_->running_.load())
        return true;

    // ---- 根路径: 服务 SPA ----
    impl_->httpServer_.Get("/", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(get_embedded_html_(), "text/html; charset=utf-8");
    });

    // ---- REST API 路由 ----
    auto handle_api = [this](const httplib::Request& req, httplib::Response& res,
                              const std::string& method) {
        if (!impl_->cmdHandler_) {
            res.set_content(R"({"ok":false,"error":"no handler"})", "application/json");
            return;
        }
        std::string result = impl_->cmdHandler_(method, req.path, req.body);
        res.set_content(result, "application/json; charset=utf-8");
    };

    // GET
    auto wrap_get = [handle_api](const httplib::Request& req, httplib::Response& res) {
        handle_api(req, res, "GET");
    };
    impl_->httpServer_.Get(R"(/api/v1/status)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/status/hw)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/videos)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/fusion/config)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/eis/config)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/eis/visible)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/backtrack/files)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/backtrack/auto-status)", wrap_get);
    impl_->httpServer_.Get(R"(/api/v1/ai/report)", wrap_get);

    // MJPEG snapshots
    impl_->httpServer_.Get(R"(/api/v1/cam/0/snapshot.jpg)", [this](const httplib::Request&, httplib::Response& res) {
        std::string jpeg = impl_->encode_jpeg_locked_(0);
        if (jpeg.empty()) { res.status = 404; res.set_content("no frame", "text/plain"); return; }
        res.set_content(jpeg, "image/jpeg");
    });
    impl_->httpServer_.Get(R"(/api/v1/cam/1/snapshot.jpg)", [this](const httplib::Request&, httplib::Response& res) {
        std::string jpeg = impl_->encode_jpeg_locked_(1);
        if (jpeg.empty()) { res.status = 404; res.set_content("no frame", "text/plain"); return; }
        res.set_content(jpeg, "image/jpeg");
    });

    // POST
    auto wrap_post = [handle_api](const httplib::Request& req, httplib::Response& res) {
        handle_api(req, res, "POST");
    };
    impl_->httpServer_.Post(R"(/api/v1/cam/0/preview/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/preview/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/stream/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/stream/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/record/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/record/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/pause)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/resume)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/osd/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/osd/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/eis/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/eis/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/preview/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/preview/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/stream/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/stream/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/record/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/record/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/pause)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/resume)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/osd/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/osd/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/eis/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/eis/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/lidar-osd/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/0/lidar-osd/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/lidar-osd/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/cam/1/lidar-osd/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/system/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/system/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/lidar/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/lidar/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/fusion/start)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/fusion/stop)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/fusion/config)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/eis/config)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/fusion/camera/0/intrinsics)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/fusion/camera/1/intrinsics)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/backtrack/query)", wrap_post);
    impl_->httpServer_.Post(R"(/api/v1/backtrack/auto-toggle)", wrap_post);

    // PUT
    auto wrap_put = [handle_api](const httplib::Request& req, httplib::Response& res) {
        handle_api(req, res, "PUT");
    };
    impl_->httpServer_.Put(R"(/api/v1/cam/0/record-resolution)", wrap_put);
    impl_->httpServer_.Put(R"(/api/v1/cam/1/record-resolution)", wrap_put);

    // DELETE
    auto wrap_delete = [handle_api](const httplib::Request& req, httplib::Response& res) {
        handle_api(req, res, "DELETE");
    };
    impl_->httpServer_.Delete(R"(/api/v1/videos)", wrap_delete);
    impl_->httpServer_.Delete(R"(/api/v1/backtrack/files)", wrap_delete);

    // 录像文件播放（流式输出，支持 Range 请求）
    impl_->httpServer_.Get(R"(/api/v1/playback)", [](const httplib::Request& req, httplib::Response& res) {
        std::string filePath = req.get_param_value("path");
        if (filePath.empty()) { res.status = 400; res.set_content("missing path", "text/plain"); return; }

        // URL 解码
        std::string decoded;
        for (size_t i = 0; i < filePath.size(); ++i) {
            if (filePath[i] == '%' && i + 2 < filePath.size()) {
                auto hex = [](char c) -> int {
                    if (c >= '0' && c <= '9') return c - '0';
                    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                    return -1;
                };
                int h = hex(filePath[i+1]), l = hex(filePath[i+2]);
                if (h >= 0 && l >= 0) { decoded += (char)((h << 4) | l); i += 2; continue; }
            }
            decoded += filePath[i];
        }
        filePath = decoded;

        // 获取文件大小
        std::ifstream fsize(filePath, std::ios::binary | std::ios::ate);
        if (!fsize.is_open()) { res.status = 404; res.set_content("not found", "text/plain"); return; }
        size_t fileSize = fsize.tellg();
        fsize.close();

        // 用 cpp-httplib 的内容提供器流式输出
        // cpp-httplib 会据此自动处理 Range 请求
        res.set_content_provider(
            fileSize,          // Content-Length
            "video/mp4",       // Content-Type
            [filePath](size_t offset, size_t length, httplib::DataSink& sink) -> bool {
                std::ifstream f(filePath, std::ios::binary);
                if (!f) return false;
                f.seekg(offset);
                std::vector<char> buf(length);
                f.read(buf.data(), length);
                size_t n = f.gcount();
                if (n > 0) sink.write(buf.data(), n);
                return true;
            },
            [](bool success) { /* done */ }
        );
    });

    // ---- WebSocket（新版 API：server.WebSocket() 大写 W）----
    impl_->httpServer_.WebSocket("/ws", [this](const httplib::Request& /*req*/,
                                                httplib::ws::WebSocket& ws) {
        {
            std::lock_guard<std::mutex> lk(impl_->wsMutex_);
            impl_->wsClients_.insert(&ws);
        }
        fprintf(stderr, "[WebServer] WebSocket client connected\n");

        // 事件循环（阻塞直到连接关闭）
        while (ws.is_open()) {
            std::string msg;
            httplib::ws::ReadResult rr = ws.read(msg);
            if (rr == httplib::ws::ReadResult::Fail)
                break; // 连接断开

            // 客户端发来的命令
            if (rr == httplib::ws::ReadResult::Text && impl_->cmdHandler_) {
                try {
                    auto j = json::parse(msg);
                    std::string wsMethod = j.value("method", "POST");
                    std::string wsPath   = j.value("path", "");
                    std::string wsBody   = j.value("body", "");
                    std::string result = impl_->cmdHandler_(wsMethod, wsPath, wsBody);
                    ws.send(result);
                } catch (...) {
                    ws.send(R"({"ok":false,"error":"invalid json"})");
                }
            }
        }

        {
            std::lock_guard<std::mutex> lk(impl_->wsMutex_);
            impl_->wsClients_.erase(&ws);
        }
        fprintf(stderr, "[WebServer] WebSocket client disconnected\n");
    });

    // ---- 启动服务器线程 ----
    impl_->running_.store(true);
    impl_->serverThread_ = std::thread([this]() {
        fprintf(stderr, "[WebServer] Starting on port %d\n", impl_->port);

        // 定时器线程：消费发送队列并广播
        std::thread timerThread([this]() {
            while (impl_->running_.load()) {
                impl_->drain_send_queue();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

        impl_->httpServer_.listen("0.0.0.0", impl_->port);

        impl_->running_.store(false);
        if (timerThread.joinable())
            timerThread.join();
        fprintf(stderr, "[WebServer] Stopped\n");
    });

    // 等待服务器就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return impl_->running_.load();
}

void WebServer::stop()
{
    if (!impl_->running_.load())
        return;

    impl_->httpServer_.stop();
    if (impl_->serverThread_.joinable())
        impl_->serverThread_.join();
}

bool WebServer::is_running() const
{
    return impl_->running_.load();
}

// ======================================================================
//  消息推送（非阻塞，推入队列由 WebServer 线程消费）
// ======================================================================

void WebServer::push_status(const std::string& json)
{
    std::string msg = R"({"type":"status","data":)" + json + "}";
    {
        std::lock_guard<std::mutex> lk(impl_->queueMutex_);
        impl_->sendQueue_.push(std::move(msg));
    }
}

void WebServer::push_event(const std::string& eventType, const std::string& json)
{
    std::string msg = R"({"type":"event","data":{"event":")" + eventType + R"(","detail":)" + json + "}}";
    {
        std::lock_guard<std::mutex> lk(impl_->queueMutex_);
        impl_->sendQueue_.push(std::move(msg));
    }
}

void WebServer::push_tracking(const std::string& json)
{
    std::string msg = R"({"type":"tracking","data":)" + json + "}";
    {
        std::lock_guard<std::mutex> lk(impl_->queueMutex_);
        impl_->sendQueue_.push(std::move(msg));
    }
}

void WebServer::push_alert(const std::string& json)
{
    std::string msg = R"({"type":"alert","data":)" + json + "}";
    {
        std::lock_guard<std::mutex> lk(impl_->queueMutex_);
        impl_->sendQueue_.push(std::move(msg));
    }
}

// ======================================================================
//  预览帧缓存（线程安全）
// ======================================================================

QImage WebServer::get_cached_preview(int camNum) const
{
    std::lock_guard<std::mutex> lk(impl_->previewMutex_);
    return impl_->cachedPreview_[camNum];
}

void WebServer::set_cached_preview(int camNum, const QImage& img)
{
    std::lock_guard<std::mutex> lk(impl_->previewMutex_);
    impl_->cachedPreview_[camNum] = img;
}

void WebServer::set_command_handler(CommandHandler handler)
{
    impl_->cmdHandler_ = std::move(handler);
}

// ======================================================================
//  静态文件服务（SPA 从 filesystem 加载，回退到嵌入式最小页面）
// ======================================================================

static std::string g_htmlCache_;
static std::mutex g_htmlMutex_;

static std::string load_html_file_()
{
    std::lock_guard<std::mutex> lk(g_htmlMutex_);
    if (!g_htmlCache_.empty())
        return g_htmlCache_;

    const char* paths[] = {
        "web/index.html",
        "../web/index.html",
        "../../web-control/web/index.html",
        "/opt/sentinel/web/index.html",
    };

    for (const char* p : paths) {
        std::ifstream f(p);
        if (f.is_open()) {
            std::ostringstream ss;
            ss << f.rdbuf();
            g_htmlCache_ = ss.str();
            fprintf(stderr, "[WebServer] Loaded SPA from %s (%zu bytes)\n",
                    p, g_htmlCache_.size());
            return g_htmlCache_;
        }
    }

    fprintf(stderr, "[WebServer] SPA file not found, using fallback\n");
    g_htmlCache_ = R"(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RK3588 Omni Sentinel</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#549688;font-family:sans-serif;color:#e6edf3;padding:40px;text-align:center}
h1{margin-bottom:10px}p{color:#8b949e}
</style></head><body>
<h1>RK3588 Omni Sentinel</h1><p>Web server is running.</p>
<p style="font-size:12px;margin-top:20px">Place <code>index.html</code> in the web/ directory to load the full SPA.</p>
</body></html>)";
    return g_htmlCache_;
}

static const char* get_embedded_html_()
{
    static std::string html = load_html_file_();
    return html.c_str();
}
