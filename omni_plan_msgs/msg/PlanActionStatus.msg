uint8 PENDING=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 CANCELLED=4
uint8 SKIPPED=5

string action_name
string[] parameters
uint8 status

# Graph structure (set from the planning graph at execution start)
int32 node_id
int32 level
int32[] depends_on

# Wall-clock timestamps (sec + nanosec since Unix epoch, same clock as rclcpp::Node::now())
builtin_interfaces/Time wall_start
builtin_interfaces/Time wall_end
