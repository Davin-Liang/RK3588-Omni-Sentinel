### 当前瓶颈与进阶调优指南 (Optimization & Tuning)

当前的零拷贝软件流水线已在资源消耗上达到了极高的效率，但结合实际工业落地场景，本 Demo 仍有以下几个明确的进阶调优方向：

* **传感器帧率解锁 (Sensor AE Tuning)**：
  * **现状**：软件流水线耗时极短（< 5ms），但实测吞吐量仅约为 15 FPS。
  * **优化方案**：该瓶颈系物理 Sensor（如 OV13855）在暗光环境下的 Auto-Exposure（自动曝光）策略导致。后续需通过修改 `rkaiq` 算法服务的 IQ 调教文件（`.xml`），或通过 V4L2/Media-ctl 接口强行锁定曝光时间（Manual Exposure），以在暗光下强制解锁 30/60 FPS。
* **设备节点动态解析 (Dynamic Topology Parsing)**：
  * **现状**：Demo 中硬编码了 ISP 输出节点为 `/dev/video11`。在设备重启或多摄接入时，该节点序号可能会发生漂移。
  * **优化方案**：后续需引入 Media Controller API（或封装 `media-ctl` 指令），通过解析 `/dev/mediaX` 拓扑图，根据 Sensor 名字自动寻找并绑定对应的 ISP 输出 `video` 节点，提高程序的即插即用鲁棒性。
* **安全队列的优雅唤醒 (Graceful Thread Unblocking)**：
  * **现状**：在系统 Shut down 阶段，当前依赖推入空指针（Dummy Task）或超时机制来唤醒阻塞在 `wait_get_xxx` 上的消费者线程。
  * **优化方案**：升级 `ThreadSafeQueue` 组件，为其加入 `abort()` 或 `unblock_all()` 方法。在触发退出信号时，直接通过 `std::condition_variable::notify_all()` 唤醒所有等待线程，实现更加干净利落的退出与资源回收。
* **RGA 异步调度 (Asynchronous Hardware Dispatch)**：
  * **现状**：当前 RGA 使用的是 `improcess/imcopy` 同步阻塞接口，单核负载仅 5%。
  * **优化方案**：若未来扩展为 4 路/8 路多摄拼接场景，可将其替换为 `improcess_async` 异步接口，激活芯片内部闲置的多个 RGA 调度核心（如 `rga2`, `rga3_1`），实现极致的多核硬件并发。
* **挂载OV5647问题**：
  * **现状**：当前无法和OV5647建立正确的图像传输，初步判定为驱动未适配rockchip芯片。
  * **优化方案**：待后续继续重写对应驱动，正确怎么改都有问题，没招了，现在能挂载ov13855，不影响后续项目进展。
