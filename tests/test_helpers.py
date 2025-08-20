"""Helper utilities for testing MCP tools."""

import autoware_mcp.server as server_module


def get_tool_function(tool_name):
    """Get the actual function from an MCP tool."""
    tool = getattr(server_module, tool_name)
    return tool.fn


# Export all tool functions for easy access in tests
health_check = get_tool_function("health_check")
get_system_status = get_tool_function("get_system_status")
verify_ros2_environment = get_tool_function("verify_ros2_environment")
get_configuration = get_tool_function("get_configuration")
monitor_system_heartbeat = get_tool_function("monitor_system_heartbeat")
monitor_diagnostics = get_tool_function("monitor_diagnostics")
reset_diagnostics = get_tool_function("reset_diagnostics")

list_ros2_nodes = get_tool_function("list_ros2_nodes")
list_ros2_topics = get_tool_function("list_ros2_topics")
list_ros2_services = get_tool_function("list_ros2_services")
get_node_info = get_tool_function("get_node_info")
get_topic_info = get_tool_function("get_topic_info")
get_topic_frequency = get_tool_function("get_topic_frequency")
echo_topic_messages = get_tool_function("echo_topic_messages")
call_ros2_service = get_tool_function("call_ros2_service")
publish_to_topic = get_tool_function("publish_to_topic")

check_autoware_status = get_tool_function("check_autoware_status")
get_vehicle_state = get_tool_function("get_vehicle_state")

set_operation_mode = get_tool_function("set_operation_mode")
monitor_operation_mode = get_tool_function("monitor_operation_mode")

set_route = get_tool_function("set_route")
set_route_points = get_tool_function("set_route_points")
get_current_route = get_tool_function("get_current_route")

initialize_localization = get_tool_function("initialize_localization")
monitor_localization_state = get_tool_function("monitor_localization_state")

request_mrm = get_tool_function("request_mrm")
list_mrm_behaviors = get_tool_function("list_mrm_behaviors")
monitor_mrm_state = get_tool_function("monitor_mrm_state")

send_velocity_command = get_tool_function("send_velocity_command")
send_acceleration_command = get_tool_function("send_acceleration_command")
send_steering_command = get_tool_function("send_steering_command")
send_pedals_command = get_tool_function("send_pedals_command")
monitor_motion_state = get_tool_function("monitor_motion_state")

get_cooperation_policies = get_tool_function("get_cooperation_policies")
set_cooperation_policies = get_tool_function("set_cooperation_policies")
send_cooperation_commands = get_tool_function("send_cooperation_commands")
