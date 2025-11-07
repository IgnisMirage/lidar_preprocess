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
ros2 launch lidar_preprocess lidar_preprocess.launch.xml
```

トピック名をカスタマイズする場合:
```bash
ros2 launch lidar_preprocess lidar_preprocess.launch.xml \
  input_topic:=/your/input/topic \
  output_topic:=/your/output/topic \
  footprint_topic:=/your/footprint/topic
```

## トピック

- 入力: `input` (sensor_msgs/PointCloud2) - デフォルト: `/lidar/points_raw`
- 入力: `footprint` (geometry_msgs/PolygonStamped) - デフォルト: `/footprint`
- 出力: `output` (sensor_msgs/PointCloud2) - デフォルト: `/lidar/points_filtered`

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
