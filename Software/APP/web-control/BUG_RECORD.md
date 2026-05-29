# BUG_RECORD — WebControl 问题记录

## 1. HTTP 请求无响应 + 偶发 segfault（Lambda use-after-free）

**现象**: 网页按钮点击后 QT 界面无反应，终端无 `[WebServer]` 或 `[WebCmd]` 日志输出。偶尔出现 `Segmentation fault`，此时进程崩溃。页面静态资源（`GET /`）正常返回，但所有 REST API（POST/PUT/DELETE）均无响应。

**原因**: `WebServer::start()` 中定义了局部 `std::function` 变量 `handle_api`，然后创建了 4 个 `wrap_*` lambda（`wrap_get`、`wrap_post`、`wrap_put`、`wrap_delete`），这些 lambda 使用 `[&]` 捕获了 `handle_api` 的**引用**。这些 lambda 被 `httpServer_.Get/Post/Put/Delete()` 存入 cpp-httplib 内部路由表。当 `start()` 函数返回后，局部变量 `handle_api` 随栈帧销毁。后续 HTTP 请求触发路由回调时，lambda 内的 `handle_api` 引用指向已释放的栈内存 → **use-after-free** → 未定义行为（表现为静默失败或 segfault）。

`GET /` 路由不受影响是因为它的 lambda 只捕获了 `[this]`，没有引用局部 `handle_api`。

**解决**: 将 4 个 `wrap_*` lambda 的捕获方式从 `[&]`（按引用捕获全部局部变量）改为 `[handle_api]`（按值捕获 `handle_api`，拷贝一份 `std::function` 副本）。修复后每个 lambda 拥有一份独立的 `handle_api` 副本，不随 `start()` 返回而失效。

```cpp
// 修复前（悬空引用）
auto wrap_post = [&](const httplib::Request& req, httplib::Response& res) {
    handle_api(req, res, "POST");  // handle_api 引用已失效
};

// 修复后（按值捕获，独立副本）
auto wrap_post = [handle_api](const httplib::Request& req, httplib::Response& res) {
    handle_api(req, res, "POST");  // handle_api 是 lambda 持有的独立副本
};
```

---

## 2. httplib 编译错误：OpenSSL / zlib 功能开关误启用

**现象**: 交叉编译时报 `#error Sorry, OpenSSL versions prior to 3.0.0 are not supported` 和链接时 `undefined reference to inflateEnd / deflateEnd` 错误。

**原因**: httplib.h 使用 `#ifdef CPPHTTPLIB_OPENSSL_SUPPORT` 和 `#ifdef CPPHTTPLIB_ZLIB_SUPPORT` 检测功能开关（注意是 `#ifdef` 而非 `#if`）。代码中写了 `#define CPPHTTPLIB_OPENSSL_SUPPORT 0` 和 `#define CPPHTTPLIB_ZLIB_SUPPORT 0`，意图通过定义为 0 来禁用功能。但 `#ifdef` 只检查宏是否**已定义**（不管值是多少），因此定义为 0 照样启用，导致编译和链接错误。

**解决**: 完全删除这两个 `#define` 语句。不定义这些宏，`#ifdef` 检查即为 false，OpenSSL 和 zlib 功能被正确禁用。添加注释说明原因，防止后续维护者重复此错误。

```cpp
// 注意: httplib 使用 #ifdef 检测功能开关，定义为 0 照样视为"已定义"。
// 所以不要 #define CPPHTTPLIB_OPENSSL_SUPPORT 或 CPPHTTPLIB_ZLIB_SUPPORT。
#define CPPHTTPLIB_THREAD_POOL_COUNT 2
#include "httplib.h"
```

---

## 3. WebSocket API 命名空间变更导致的编译错误

**现象**: 新版 cpp-httplib（master 分支）编译时报多个错误：
- `'httplib::WebSocket' is not a member of 'httplib'`，实际在 `httplib::ws::WebSocket`
- `'class httplib::Server' has no member named 'set_web_socket_handler'`
- `ws->read(msg, op, timeout)` 签名不匹配

**原因**: cpp-httplib 的 master 分支（v0.16+）重构了 WebSocket API：
- `WebSocket` 类从 `httplib` 命名空间移至 `httplib::ws` 子命名空间
- 注册 WebSocket handler 的方法名从 `set_web_socket_handler` 改为 `WebSocket`（大写 W，即 `server.WebSocket(path, handler)`）
- handler 签名从 `(const WebSocket::Connection&)` 改为 `(const Request&, ws::WebSocket&)`
- `read()` 不再接收 `Opcode` 和 `timeout` 参数，改为返回 `ReadResult` 枚举（Fail/Text/Binary）
- `send()` 不再接收完成回调参数

**解决**: 全面适配新版 API：
- 所有 `httplib::WebSocket*` 改为 `httplib::ws::WebSocket*`
- `httpServer_.set_web_socket_handler("/ws", ...)` 改为 `httpServer_.WebSocket("/ws", ...)`
- handler 改为 `[this](const httplib::Request&, httplib::ws::WebSocket& ws) { ... }`
- `ws.read(msg, op, timeout)` 改为 `ws.read(msg)`，返回值与 `httplib::ws::ReadResult::Text` 比较
- `ws.send(msg, callback)` 改为 `ws.send(msg)`
