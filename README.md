# lidar_preprocess

3D LiDAR前処理パッケージ（ROS2 + PCL）

## 機能

- **距離フィルタ**: 原点から一定距離以下の点群を除去
- **高さフィルタ**: Z軸でのPassThrough filter
- **ノイズ除去**: Statistical Outlier Removal (SOR)
- **フットプリントフィルタ**: PolygonStampedトピックからポリゴンを受信し内外判定（Ray casting algorithm）

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
- 入力: `/footprint` (geometry_msgs/PolygonStamped) - フットプリントフィルタ用
- 出力: `/lidar/points_filtered` (sensor_msgs/PointCloud2)

## パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `min_distance` | double | 0.5 | 最小距離 (m) |
| `height_min` | double | -2.0 | 高さフィルタ最小値 (m) |
| `height_max` | double | 5.0 | 高さフィルタ最大値 (m) |
| `sor_mean_k` | int | 50 | SOR近傍点数 |
| `sor_stddev` | double | 1.0 | SOR標準偏差倍率 |
| `enable_distance_filter` | bool | true | 距離フィルタ有効化 |
| `enable_height_filter` | bool | true | 高さフィルタ有効化 |
| `enable_noise_filter` | bool | true | ノイズフィルタ有効化 |
| `enable_footprint_filter` | bool | true | フットプリントフィルタ有効化 |

`config/params.yaml`で設定変更可能
