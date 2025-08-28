#!/bin/bash
# Migration script from Rye to uv

echo "Migrating Autoware MCP from Rye to uv..."
echo ""

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi

echo "uv version: $(uv --version)"
echo ""

# Clean up old Rye artifacts
echo "Cleaning up old Rye artifacts..."
rm -f requirements.lock requirements-dev.lock
rm -rf .rye

# Sync dependencies with uv
echo "Installing dependencies with uv..."
uv sync --all-extras --dev

echo ""
echo "Migration complete!"
echo ""
echo "You can now use uv commands:"
echo "  uv run autoware-mcp        # Run the MCP server"
echo "  uv run pytest              # Run tests"
echo "  uv run pytest -v           # Run tests with verbose output"
echo "  uv sync                    # Install/update dependencies"
echo ""
echo "For more information, see: https://docs.astral.sh/uv/"