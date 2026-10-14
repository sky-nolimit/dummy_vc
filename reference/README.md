# reference/ —— 外部参考资料

**这个目录里的东西不是我们的代码。** 它们是从别处拿到的原始资料，放在这里
便于对照，但**不要修改**，也不要依赖它们构建。

**用之前先看 [../docs/05-固件源码事实.md](../docs/05-固件源码事实.md)** ——
那份文档把固件里与我们相关的部分摘录并标注了行号，通常不需要直接翻这里。

---

## 目录内容

| 路径 | 是什么 | 体积 | 是否跟踪 |
|---|---|---|---|
| `Firmware/Dummy 源码注释/` | **7 个带注释的核心固件文件** | 0.1 MB | ✅ |
| `Firmware/dummy-ref-core-fw/` | Dummy 参考核心固件（完整工程） | 44 MB | ✅ |
| `Firmware/dummy-35motor-fw/` | 35 电机固件 | 4.3 MB | ✅ |
| `Firmware/dummy-42motor-fw/` | 42 电机固件 | 4.4 MB | ✅ |
| `Firmware/README.md` | 原项目说明（源自 peng-zhihui/Dummy-Robot） | — | ✅ |
| `Firmware/esp32-iot/` | **Mongoose OS 第三方库** | 80 MB | ❌ 见下 |
| `dummy_moveit_ws/` | 参考 MoveIt 工作空间 | — | ✅ |
| `upstream-local-changes/` | 上游本地改动记录 | — | ✅ |

## 为什么 `esp32-iot` 不进版本库

它是 **Mongoose OS 的库**（Sergey Lyubka / Cesanta），2851 个文件、80 MB，
里面绝大部分是十几个各厂商开发板的 demo 工程和 STM32 HAL/CMSIS 生成代码。

**那是 Dummy 的 ESP32 联网板用的，与机械臂本体无关**，而且上游有仓库可取。

`.gitignore` 里按路径忽略它，所以 `git add reference/` 不会把它再带进来。
**文件仍然在磁盘上**（`git rm --cached` 只改索引），需要时直接读。

## 三条使用原则

**1. 只读。** 不要在这里改代码。需要改的东西属于 `dummy_vc_ws/`。

**2. 结论要摘出来。** 从这些资料里得到的结论（例如夹爪角度范围是 0~30）
应当写进 `docs/`，并注明出处文件与行号 —— 因为这里的资料**不随项目演进**，
将来换一份资料时，结论还在。

**3. 不要把这里的代码复制进项目。** 它是参考资料，不是依赖。上游仓库和我们的
项目之间只有"读"这一种关系。
