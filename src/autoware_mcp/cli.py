"""Command-line interface utilities for Autoware MCP."""

import sys
import subprocess
from pathlib import Path


def run_tests():
    """Run the test suite with coverage."""
    print("Running Autoware MCP test suite...")
    
    # Get the project root directory
    project_root = Path(__file__).parent.parent.parent
    
    # Run pytest with coverage
    cmd = [
        sys.executable, "-m", "pytest",
        str(project_root / "tests"),
        "-v",
        "--cov=autoware_mcp",
        "--cov-report=term-missing",
        "--asyncio-mode=auto"
    ]
    
    try:
        result = subprocess.run(cmd, cwd=project_root)
        sys.exit(result.returncode)
    except KeyboardInterrupt:
        print("\nTest run interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error running tests: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    run_tests()