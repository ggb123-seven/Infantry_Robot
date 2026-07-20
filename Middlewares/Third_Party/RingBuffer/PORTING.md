# RingBuffer 移植说明

- 上游仓库：https://github.com/MaJerle/lwrb
- 发布版本：`v3.2.0`
- 提交：`77b2bdbecf60cd80e449f5c61bde72ec195efac4`
- 下载日期：2026-07-19
- GitHub zipball SHA-256：`DE4904B4C044A89086BF7330EEFEF1F90AF56CC3F666F6D787A4D456FA5AC5E5`
- 许可证：MIT，完整文本见 `LICENSE`

当前工程只引入上游核心实现，并将文件改为更直观的名称：

- `lwrb.c` 改名为 `ringbuffer.c`，仅同步修改了头文件 include 路径。
- `lwrb.h` 改名为 `ringbuffer.h`，文件内容保持上游原样。
- `LICENSE` 文件名和内容均保持上游原样。

内部 `lwrb_*` API 和数据结构名称保持不变，便于继续对照上游版本。CAN BSP 使用扩展读写接口的
`LWRB_FLAG_WRITE_ALL` 和 `LWRB_FLAG_READ_ALL`，确保固定长度 CAN 消息只能整帧入队和整帧出队。
