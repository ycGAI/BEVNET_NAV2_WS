# msg/SemanticCostmap.msg
# Semantic costmap with multiple layers

# Header (timestamp and frame)
std_msgs/Header header

# Map metadata
nav_msgs/MapMetaData info

# Traditional costmap data
# Values: 0-254 (cost), 255 (unknown)
uint8[] costmap

# Semantic layers
# Each layer represents a different semantic class
uint8[] road_layer
uint8[] sidewalk_layer
uint8[] obstacle_layer
uint8[] vegetation_layer

# Layer weights for fusion
float32 road_weight
float32 sidewalk_weight
float32 obstacle_weight
float32 vegetation_weight

# Fusion method
string fusion_method