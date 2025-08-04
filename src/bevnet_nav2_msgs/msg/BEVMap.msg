# msg/BEVMap.msg
# BEV semantic map message

# Header (timestamp and frame)
std_msgs/Header header

# Map metadata (similar to OccupancyGrid)
nav_msgs/MapMetaData info

# Semantic map data
# Each cell contains semantic class ID
uint8[] semantic_map

# Number of semantic classes
uint8 num_classes

# Class probabilities for each cell (optional)
# Size: width * height * num_classes
float32[] class_probabilities

# Semantic class names (optional)
string[] class_names