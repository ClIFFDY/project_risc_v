// npu_defines.vh — NPU 全局参数(所有模块共用)
//
// 约定：
//   1. 除 MMIO 对外地址外，NPU 内部 Buffer 地址均为 32 位字地址；
//   2. 参数宏集中放在本文件，避免各模块重复写常量导致位宽和容量不一致；
//   3. 容量宏(NPU_TK_MAX / NPU_*BUF_WORDS)是 start_ctrl 配置校验的唯一依据，
//      软件可见的合法范围由它们决定，不要把容量约束写死在别处。
`ifndef NPU_DEFINES_VH
`define NPU_DEFINES_VH

// ---- 脉动阵列规模 P x Q ----
// pe_array 用 generate 例化 P*Q 个 PE；a_bt_skew_pipeline 按 lane 号生成
// r/c 级延迟；drain_controller 需要 P+Q-2 个 enable 拍。
`define NPU_P 4
`define NPU_Q 4
`define NPU_PE_NUM 16                  // P*Q，PE 总数

// ---- PE 累加器宽度 ----
// INT8 x INT8，K<=63 时最大 |acc| = 63*127*128 < 2^21；取 32 位留量化余量，
// 并且跨 K Tile、跨任务累加都不会溢出。
`define NPU_ACC_W 32

// ---- Buffer 容量：地址宽度 AW 对应 2^AW 个 32 位字 ----
`define NPU_ABUF_AW 6                  // A Buffer  64 字 = 256 个 INT8
`define NPU_BBUF_AW 6                  // BT Buffer 64 字
`define NPU_CBUF_AW 8                  // C Buffer  256 字，一个字存一个 C 元素
`define NPU_ABUF_WORDS 64              // 配置校验用，必须等于 2^NPU_ABUF_AW（每 bank）
// ---- 双缓冲（2026-11-12）：A/BT 各两份，物理 RAM 地址比逻辑地址多 1 位 ----
//   bit[AW] = bank。逻辑侧（tile_controller / 配置校验）仍然按 64 字算，
//   只有 npu_buffer 里的阵列和 npu_top 的译码知道 bank 的存在。
`define NPU_ABUF_REG_AW 7
`define NPU_BBUF_REG_AW 7
`define NPU_BBUF_WORDS 64
`define NPU_CBUF_WORDS 256

// ---- FIFO 深度 ----
// A/BT 预取 FIFO：预取的单位是"一个输出 Tile 在 K 方向上的整段连续数据"（长度 K），
// 但突发是"边填边被消费"的——喂数每拍消费 1 对、突发每拍补充 1 个字，两者速率相同，
// 所以 FIFO 只需要容纳"突发领先量"，即首字到达前 S_PREFETCH 等的那几拍。
// 实测（26 项回归，含 K=63 的 G11）峰值只用到 3 个字，深度取 16 留 5 倍余量。
// tb_npu.v 里有 full 看门狗：一旦出现 full 说明这个假设被破坏，回归会直接报错。
`define NPU_FIFO_AW 4                  // 深度 16
`define NPU_CWF_AW 5                   // C 写 FIFO 深度 32，一个 C Tile 最多 16 项

// ---- 参数位宽 ----
`define NPU_DIM_W 6                    // M/N/K，合法范围 1..63
`define NPU_TILE_W 3                   // TM/TN，硬件固定 4
`define NPU_TK_W 5                     // TK
`define NPU_QS_W 5                     // 量化右移位数
`define NPU_TIDX_W 6                   // tile 计数器位宽
`define NPU_CBASE_W 9                  // C 元素基地址中间位宽
`define NPU_WORD_W 10                  // 配置校验里 A/BT 字数中间量位宽

// ---- 容量约束(start_ctrl 配置校验使用) ----
`define NPU_TK_MAX 16                  // TK 合法上界
`define NPU_TM_TN 4                    // TM=TN=4 固定

// ---- AHB-Lite 从机基址 ----
// NPU 对外只有 AHB-Lite 一条总线：HADDR 减去 NPU_ADDR_BASE 之后才是下面
// NPU_ADDR_* 定义的内部字地址（所以那些宏都是"偏移"，不是绝对地址）。
// 默认 0x4000_0000：0x0000_0000 是 Cortex-M 的向量表/启动别名区，不能占用。
`define NPU_ADDR_BASE 32'h2000_0000

// ---- MMIO 地址映射(npu_top 内联译码使用，均为相对基址的偏移) ----
`define NPU_ADDR_CTRL_BASE 32'h0000_0000  // 0x0000-0x001F 控制寄存器
`define NPU_ADDR_STAT_BASE 32'h0000_0020  // 0x0020-0x002F 状态寄存器
`define NPU_ADDR_ABUF_BASE 32'h0000_1000  // 0x1000-0x10FF A Buffer
`define NPU_ADDR_BBUF_BASE 32'h0000_2000  // 0x2000-0x20FF BT Buffer
`define NPU_ADDR_CBUF_BASE 32'h0000_3000  // 0x3000-0x33FF C Buffer

`endif
