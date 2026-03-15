# Custom message type combining octomap with optional bounding box information
# This allows the occupancy map server to communicate both the octomap data
# and the configured bounding box limits to downstream consumers

std_msgs/Header header

# The serialized octomap data
octomap_msgs/Octomap octomap

# Whether a bounding box is configured (if false, ignore bbx_min/bbx_max)
bool has_bounding_box

# Bounding box minimum corner (only valid if has_bounding_box is true)
geometry_msgs/Point bbx_min

# Bounding box maximum corner (only valid if has_bounding_box is true)
geometry_msgs/Point bbx_max
