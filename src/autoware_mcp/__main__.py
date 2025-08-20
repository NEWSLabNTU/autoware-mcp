"""Main entry point for Autoware MCP server."""

import asyncio
import sys
import os
from .server import main as server_main


def main():
    """Main entry point function."""
    print("Starting Autoware MCP Server...")
    print(
        f"ROS_DISTRO: {os.getenv('ROS_DISTRO', 'Not set - please source setup.bash')}"
    )
    print(f"ROS_DOMAIN_ID: {os.getenv('ROS_DOMAIN_ID', '0 (default)')}")
    print("-" * 50)

    try:
        # Run the async main function
        asyncio.run(server_main())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
        sys.exit(0)
    except Exception as e:
        print(f"Server error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
