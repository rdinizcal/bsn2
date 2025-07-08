# Message for energy status updates
std_msgs/Header header
string source        # Source node ID
string target        # Target node ID (usually "system")
string content       # Content in format "energy:X:cost:Y"