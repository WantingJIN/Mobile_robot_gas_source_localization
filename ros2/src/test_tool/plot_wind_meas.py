import pandas as pd
import matplotlib.pyplot as plt

# 1. 加载数据
df = pd.read_csv('wind_log.csv')


# smooth data for better visualization (optional)
# moving average
# df['wind_u'] = df['wind_u'].rolling(window=1, center=True).mean()
# df['wind_v'] = df['wind_v'].rolling(window=1, center=True).mean()
# df['robot_speed_x'] = df['robot_speed_x'].rolling(window=1, center=True).mean()
# df['robot_speed_y'] = df['robot_speed_y'].rolling(window=1, center=True).mean()
# df['clean_wind_u'] = df['clean_wind_u'].rolling(window=10, center=True).mean()
# df['clean_wind_v'] = df['clean_wind_v'].rolling(window=10, center=True).mean()   
# moving average with delta
def moving_average_with_delta(series, window_size=5, delta=0.1):
    smoothed = series.rolling(window=window_size, center=True).mean()
    smoothed = smoothed.fillna(method='bfill').fillna(method='ffill')
    return smoothed + delta * (series - smoothed)
# df['wind_u'] = moving_average_with_delta(df['wind_u'], window_size=5, delta=0.1)
# df['wind_v'] = moving_average_with_delta(df['wind_v'], window_size=5, delta=0.1)
# df['robot_speed_x'] = moving_average_with_delta(df['robot_speed_x'], window_size=5, delta=0.1)
# df['robot_speed_y'] = moving_average_with_delta(df['robot_speed_y'], window_size=5, delta=0.1)
# df['clean_wind_u'] = moving_average_with_delta(df['clean_wind_u'], window_size=20, delta=0.1)
# df['clean_wind_v'] = moving_average_with_delta(df['clean_wind_v'], window_size=20, delta=0.1)

# 2. 创建画布，包含两个垂直堆叠的子图，并共享 x 轴
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# --- 上方子图: U 和 X 分量 (蓝色系) ---
# ax1.plot(df.index, df['raw_wind_u'], label='Raw Wind $u$', color='green', alpha=0.3, linewidth=2)
ax1.plot(df.index, df['wind_u'], label='Wind $u$', color='blue', alpha=0.3, linestyle='--')
ax1.plot(df.index, df['clean_wind_u'], label='Clean Wind $u$', color='darkblue', linewidth=2)
ax1.plot(df.index, df['robot_speed_x'], label='Robot Speed $x$', color='skyblue', alpha=0.7)

ax1.set_ylabel('$u$ / $x$ Components (m/s)')
ax1.set_title('Wind and Robot Speed Analysis - $u$ and $x$ Components')
ax1.legend(loc='upper right', ncol=3)
ax1.grid(True, linestyle=':', alpha=0.6)

# --- 下方子图: V 和 Y 分量 (红色系) ---
# ax2.plot(df.index, -df['raw_wind_v'], label='Raw Wind $v$', color='purple', alpha=0.3, linewidth=2)
ax2.plot(df.index, df['wind_v'], label='Wind $v$', color='red', alpha=0.3, linestyle='--')
ax2.plot(df.index, df['clean_wind_v'], label='Clean Wind $v$', color='darkred', linewidth=2)
ax2.plot(df.index, df['robot_speed_y'], label='Robot Speed $y$', color='orange', alpha=0.7)

ax2.set_xlabel('Sample Index')
ax2.set_ylabel('$v$ / $y$ Components (m/s)')
ax2.set_title('Wind and Robot Speed Analysis - $v$ and $y$ Components')
ax2.legend(loc='upper right', ncol=3)
ax2.grid(True, linestyle=':', alpha=0.6)

# 3. 调整间距并保存
plt.tight_layout()
plt.savefig('wind_plot_stacked.png')
plt.show()