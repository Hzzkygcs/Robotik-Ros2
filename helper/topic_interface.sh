topic_info=$(ros2 topic info $1)
interface_name=`echo "$topic_info" | head -n 1`
interface_name=`echo "$interface_name" |cut -d ' ' -f2`
echo "Interface name: $interface_name"

ros2 interface show "$interface_name"
