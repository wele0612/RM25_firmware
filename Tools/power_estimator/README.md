# Power Estimator

使用这个串口示波器[https://createskyblue.github.io/justfloat-web-oscilloscope/]采集数据。

从示波器导出的 JSON 数据中提取电流、转速、功率，拟合出功率模型并导出为 C 数组。

## 运行步骤

### 1. 从 JSON 提取数据
`plot_dataset.py` 读取 `dataset/` 里的所有 JSON，按文件名顺序拼接，提取三个 channel 保存为 `power_profile.csv`。

```bash
python plot_dataset.py -c <current_id> -s <speed_id> -p <power_id>
```

参数：
- `-c`：**电流**对应的 channel ID（从 0 开始）
- `-s`：**转速**对应的 channel ID
- `-p`：**功率**对应的 channel ID

输出：
- `power_profile.csv` — 三列数据：current, speed, power
- `power_profile.png` — 三个 channel 的波形图

---

### 2. 数据均衡（过滤）
`power_data_profile_filter.py` 把 current/speed 平面划分成 `n×n` 的格子，统计每格里的数据点数量。对于数据点**远多于其他格**的密集区域，随机丢弃多余点，使数据分布更均匀，防止拟合时被某几个区间主导。

```bash
python power_data_profile_filter.py -n <bins> -t <percentile>
```

参数：
- `-n`：每轴划分的段数（如 `30` 表示 30×30 的格子）
- `-t`：百分位阈值（如 `0.9` 表示 90%）。程序会计算所有非空格子的数据点数量，取第 90% 分位数作为上限；超过该上限的格子会被随机降采样到该数量。

输出：
- `power_profile_filtered.csv` — 过滤后的数据
- `current_speed_heatmap_comparison.png` — 过滤前后的热力图对比（左侧原始，右侧过滤后）

---

### 3. 拟合并导出 C 头文件
`power_polyfit.py` 用最小二乘法对 CSV 数据做二次多项式拟合：

```
power = K0
      + K1·current
      + K2·speed
      + K3·current·speed
      + K4·current²
      + K5·speed²
```

```bash
# 使用过滤后的数据（推荐）
python power_polyfit.py -d power_profile_filtered.csv

# 或直接拟合原始数据
python power_polyfit.py
```

参数：
- `-d`：输入的 CSV 文件，默认是 `power_profile.csv`

输出：
- `power_model.h` — C 语言数组 `POLYMODEL[]`，包含 K0~K5 六个系数
