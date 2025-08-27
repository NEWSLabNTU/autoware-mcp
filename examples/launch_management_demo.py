#!/usr/bin/env python3
"""Demo script for launch session management capabilities.

This demonstrates the new Phase 4 launch management features:
- Starting and stopping launch files with PID/PGID tracking
- Generating launch files and node configurations
- Session monitoring and cleanup
"""

import asyncio
import sys
import os
from pathlib import Path

# Add parent directory to path to import autoware_mcp
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from autoware_mcp.launch_manager import LaunchSessionManager, LaunchGenerator
from autoware_mcp.launch_manager.session import SessionState


async def demonstrate_launch_management():
    """Demonstrate launch management capabilities."""

    print("=== Autoware MCP Launch Management Demo ===\n")

    # Initialize managers
    session_manager = LaunchSessionManager()
    generator = LaunchGenerator()

    print("1. Generating a sample launch file...")
    # Generate a perception pipeline launch file
    launch_result = generator.generate_launch_file(
        name="demo_perception",
        components=["lidar_processing", "camera_detection", "fusion"],
        template="perception_pipeline",
        parameters={"model_path": "./models/demo.pt"},
    )

    if launch_result["success"]:
        print(f"   ✓ Generated: {launch_result['file_path']}")
        print(f"   Version: {launch_result['version']}")
    else:
        print(f"   ✗ Failed: {launch_result.get('error')}")
        return

    print("\n2. Generating node configuration...")
    config_result = generator.generate_node_config(
        node_name="perception_node",
        parameters={
            "update_rate": 10.0,
            "threshold": 0.5,
            "enable_visualization": True,
        },
        format="yaml",
    )

    if config_result["success"]:
        print(f"   ✓ Generated: {config_result['file_path']}")

    print("\n3. Generating a custom Python node...")
    node_result = generator.generate_custom_node(
        name="demo_processor",
        language="python",
        node_type="basic",
        interfaces=["/perception/objects", "/planning/trajectory"],
    )

    if node_result["success"]:
        print(f"   ✓ Generated: {node_result['file_path']}")

    print("\n4. Validating the launch file...")
    validation = generator.validate_launch_file(launch_result["file_path"])

    if validation["success"]:
        print(f"   ✓ {validation['message']}")
    else:
        print(f"   ✗ {validation.get('error')}")

    print("\n5. Starting a launch session...")
    print(
        "   (Note: This will fail without ROS2 environment, but demonstrates the API)"
    )

    # Try to start the generated launch file
    start_result = await session_manager.start_launch(
        launch_file=launch_result["file_path"], parameters={"use_sim_time": "true"}
    )

    if start_result["success"]:
        session_id = start_result["session_id"]
        print(f"   ✓ Session started: {session_id}")
        print(f"   Main PID: {start_result.get('main_pid')}")
        print(f"   Process Group: {start_result.get('pgid')}")

        # Get session status
        print("\n6. Checking session status...")
        status = await session_manager.get_session_status(session_id)
        if status["success"]:
            print(f"   State: {status.get('state')}")
            print(f"   Active: {status.get('is_active')}")

        # List all sessions
        print("\n7. Listing all sessions...")
        sessions = await session_manager.list_sessions()
        print(f"   Found {len(sessions)} active session(s)")
        for sess in sessions:
            print(f"   - {sess['session_id']}: {sess['state']}")

        # Stop the session
        print("\n8. Stopping the session...")
        stop_result = await session_manager.stop_launch(session_id)
        if stop_result["success"]:
            print(f"   ✓ {stop_result['message']}")
    else:
        print(f"   ✗ Failed to start: {start_result.get('error')}")
        print("   (This is expected without a ROS2 environment)")

    print("\n9. Checking for orphaned sessions...")
    cleanup_result = await session_manager.cleanup_orphans()
    if cleanup_result["success"]:
        print(f"   Cleaned up {cleanup_result['cleaned_up_count']} orphaned session(s)")

    print("\n10. Listing all generated files...")
    files = generator.list_generated_files()
    print(f"   Launch files: {len(files['launches'])}")
    print(f"   Node files: {len(files['nodes'])}")
    print(f"   Config files: {len(files['configs'])}")

    for category, file_list in files.items():
        if file_list:
            print(f"\n   {category.title()}:")
            for f in file_list:
                print(f"   - {Path(f).name}")

    print("\n=== Demo Complete ===")
    print("\nKey Features Demonstrated:")
    print("• Launch file generation with templates")
    print("• Node and configuration generation")
    print("• Session management with PID/PGID tracking")
    print("• Orphan process cleanup")
    print("• File versioning and organization")
    print("\nGenerated files are stored in: .autoware-mcp/generated/")
    print("Session data is stored in: .autoware-mcp/instance_*/sessions/")


if __name__ == "__main__":
    # Run the demo
    asyncio.run(demonstrate_launch_management())
