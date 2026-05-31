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

---

## 4. 录像文件在线播放失败（Range 请求解析崩溃 + 二进制处理不当）

**现象**: 录像文件列表新增播放按钮后，点击播放弹出播放器但一直转圈，最终播放失败。浏览器 Network 面板显示请求未完成。但同样 URL 在浏览器地址栏直接访问可下载完整文件，下载后可本地播放。

**原因**: 两个独立问题：
1. 浏览器 `<video>` 元素发送的首个探测请求为 `Range: bytes=0-`（`-` 后为空）。代码用 `std::stoull(rangeVal.substr(dashPos + 1))` 解析，空字符串传入 `std::stoull` 抛出 `std::invalid_argument` 异常，handler 未捕获导致请求失败。
2. `res.set_content(binary_data, "video/mp4")` 对二进制 MP4 数据传输可能经过文本处理，导致浏览器无法正确解码。

**解决**: 
1. 检查 `dashPos + 1` 之后的子串是否为空，为空则保持默认值 `fileSize - 1`。
2. 改用 cpp-httplib 自带的 `res.set_content_provider(fileSize, "video/mp4", lambda)` 流式输出。该方法自动处理 Range 请求切片（浏览器发送 `bytes=START-END` → cpp-httplib 自动计算 offset 和 length 调用 provider），且直接写入 socket 不经文本处理。

```cpp
// 修复后：使用 set_content_provider 流式输出
res.set_content_provider(
    fileSize, "video/mp4",
    [filePath](size_t offset, size_t length, httplib::DataSink& sink) -> bool {
        std::ifstream f(filePath, std::ios::binary);
        f.seekg(offset);
        std::vector<char> buf(length);
        f.read(buf.data(), length);
        sink.write(buf.data(), f.gcount());
        return true;
    },
    [](bool) {}
);
```

---

## 5. 融合参数帮助提示不自动消失

**现象**: 网页融合参数面板的 `?` 帮助按钮点击后，提示框持续显示不消失，遮挡其他参数。

**原因**: 重设计时只恢复了帮助按钮的 HTML 结构和点击切换逻辑，遗漏了 QT 界面的 4 秒自动隐藏功能。

**解决**: `toggleHelp()` 函数增加 4 秒 `setTimeout` 自动隐藏。再次点击或切换其他帮助按钮时取消前一个定时器，确保只有一个提示框显示。
