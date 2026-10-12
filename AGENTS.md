# 协作约定

## 版本控制：AI 负责提交，人负责推送

**AI 在每次完成一组改动后直接 `git commit`，不需要每次询问。**
人只负责在方便的时候 `git push`。

这条约定的由来：曾经出现"改完了但没提交，所以推不上去"的情况。既然人只做
push，提交这一步就该由 AI 一并完成。

规则：

- 改动完成并通过检查后**直接提交**，commit message 用中文、说明**为什么**改，
  而不只是改了什么。
- **不要 `git push`。** 推送由人决定时机（可能需要先在虚拟机上验证）。
- 提交前跑到全绿：
  ```bash
  python tools/check_syntax.py
  python tools/selftest_conversion.py
  python tools/check_conversions.py
  python tools/check_launch_consistency.py
  ```
  编码与换行符在 Windows 上改过文件时也要跑：
  ```bash
  python tools/check_encoding.py <改过的文件>
  powershell -File tools/fix_line_endings.ps1 -Check
  ```
- 提交后报告：commit hash、一句话摘要、`git status` 是否干净、待推送几个提交。

## 不要替人决定删文件

新增分析工具、删除冗余文件这类事，**先说明理由再动手**；不确定时保留并说明，
不要静默处理。曾经把新建的工具卷进别人的提交里，这不合适。

## 说话方式

- 直接给结论，不要用"可能""也许"堆砌；不确定就说不确定，并说明怎么验证。
- 出了问题先认自己的错，不要找借口，也不要为了显得在干活而继续加复杂度。
  **能工作的东西不要改坏** —— 需要在能工作的方案和"更好"的方案之间选时，
  先保住能工作的那个。
- 报告现象时说清：**成功判据是什么**、失败了**说明什么问题**。
