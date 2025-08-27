# Using Autoware MCP with Claude Desktop

## Quick Setup

### 1. Configure Claude Desktop

The configuration file has been created at:
```
~/.config/claude/claude_desktop_config.json
```

It contains:
```json
{
  "mcpServers": {
    "autoware-mcp": {
      "command": "bash",
      "args": [
        "-c",
        "source /home/aeon/repos/autoware/0.45.1-ws/install/setup.bash && cd /home/aeon/repos/autoware-mcp && rye run autoware-mcp"
      ],
      "env": {
        "ROS_DOMAIN_ID": "0"
      }
    }
  }
}
```

### 2. Restart Claude Desktop

1. **Completely quit Claude Desktop** (Cmd+Q on Mac, or right-click tray icon → Quit on Linux)
2. **Start Claude Desktop again**
3. The Autoware MCP server will now be available

### 3. Test in Claude Desktop

Start a new conversation and try these prompts:

#### Basic Status Checks
- "Can you check what Autoware components are running?"
- "Show me the health status of the system"
- "List all ROS2 nodes"
- "What ROS2 topics are available?"

#### Specific Information
- "Get information about the /tf topic"
- "What's the publishing frequency of /localization/kinematic_state?"
- "Show me messages from the /diagnostics topic"
- "List all ROS2 services"

#### System Monitoring
- "Check the system resource usage"
- "Is Autoware running properly?"
- "Verify the ROS2 environment"

## Testing Without Claude Desktop

### Option 1: Direct Tool Testing

Run the test script with sourced environment:
```bash
source ~/autoware/install/setup.bash
cd /home/aeon/repos/autoware-mcp
python test_with_mcp.py
```

### Option 2: Manual Server Testing

In one terminal, start the server:
```bash
source ~/autoware/install/setup.bash
cd /home/aeon/repos/autoware-mcp
rye run autoware-mcp
```

The server will show:
- Startup messages
- MCP protocol initialization
- Tool invocations when Claude uses them

## Troubleshooting

### "MCP server not found" in Claude Desktop
1. Check the config file exists: `~/.config/claude/claude_desktop_config.json`
2. Verify the paths are correct for your system
3. Completely restart Claude Desktop (not just close window)

### "ROS2 environment not detected"
1. The config automatically sources the setup.bash
2. Check the path in the config is correct: `/home/aeon/repos/autoware/0.45.1-ws/install/setup.bash`
3. Verify Autoware is installed at that location

### Server doesn't start
1. Test manually first:
   ```bash
   source ~/autoware/install/setup.bash
   rye run autoware-mcp
   ```
2. Check for error messages
3. Ensure `rye sync` has been run in the autoware-mcp directory

## How It Works

When you ask Claude about Autoware in Claude Desktop:

1. Claude Desktop starts the MCP server using the config
2. The server sources your ROS2/Autoware environment
3. Claude can then use MCP tools to:
   - Query running nodes and topics
   - Check system health
   - Monitor Autoware components
   - Get real-time information

## Available Tools in Claude Desktop

Once configured, Claude has access to these tools:

1. **health_check** - Complete system and Autoware health status
2. **check_autoware_status** - Detailed Autoware component analysis
3. **list_ros2_nodes** - List all active nodes
4. **list_ros2_topics** - List all topics
5. **list_ros2_services** - List all services
6. **get_topic_info** - Detailed topic information
7. **get_topic_frequency** - Measure publishing rates
8. **echo_topic_messages** - Capture topic messages
9. **get_system_status** - System resource monitoring
10. **verify_ros2_environment** - Check ROS2 setup

## Example Conversation

You: "Can you check if my Autoware planning stack is working?"

Claude will:
1. Use `check_autoware_status` to find planning nodes
2. Use `list_ros2_topics` to verify planning topics
3. Use `get_topic_frequency` to check if messages are flowing
4. Provide a comprehensive report

## Security Note

The MCP server:
- Only observes running Autoware (doesn't control it)
- Cannot launch or stop Autoware processes
- Runs with your user permissions
- Only accessible to Claude Desktop on your machine