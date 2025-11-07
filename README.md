# lidar_preprocess

3D LiDAR前処理パッケージ（ROS2 + PCL）

## 機能

- **ノイズ除去**: Statistical Outlier Removal (SOR)
- **高さフィルタ**: Z軸でのPassThrough filter
- **フットプリントフィルタ**: 車両周辺の点群除去

## ビルド

```bash
colcon build --packages-select lidar_preprocess
```

## 実行

```bash
ros2 launch lidar_preprocess lidar_preprocess.launch.py
```

## トピック

- 入力: `/lidar/points_raw` (sensor_msgs/PointCloud2)
- 出力: `/lidar/points_filtered` (sensor_msgs/PointCloud2)

## パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `height_min` | double | -2.0 | 高さフィルタ最小値 (m) |
| `height_max` | double | 5.0 | 高さフィルタ最大値 (m) |
| `footprint_x` | double | 1.5 | フットプリント半径X (m) |
| `footprint_y` | double | 1.0 | フットプリント半径Y (m) |
| `sor_mean_k` | int | 50 | SOR近傍点数 |
| `sor_stddev` | double | 1.0 | SOR標準偏差倍率 |
| `enable_noise_filter` | bool | true | ノイズフィルタ有効化 |
| `enable_height_filter` | bool | true | 高さフィルタ有効化 |
| `enable_footprint_filter` | bool | true | フットプリントフィルタ有効化 |

`config/params.yaml`で設定変更可能
