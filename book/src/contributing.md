# Contributing

Thank you for your interest in contributing to the Autoware MCP Server! This guide will help you get started with development and contribution.

## Development Setup

### Prerequisites

- Python 3.10 or higher
- ROS2 Humble
- Autoware (tested with version 0.45.1)
- Git

### Setting Up Development Environment

1. **Fork and clone the repository**:
```bash
git clone https://github.com/yourusername/autoware-mcp.git
cd autoware-mcp
```

2. **Install uv** (Python package manager):
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

3. **Install dependencies**:
```bash
uv sync --all-extras --dev
```

4. **Source ROS2 environment**:
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash  # If using Autoware
```

5. **Run tests to verify setup**:
```bash
uv run pytest tests/ -v
```

## Development Workflow

### Project Structure

```
autoware-mcp/
├── src/autoware_mcp/       # Main package
│   ├── server.py           # MCP server implementation
│   ├── ad_api_ros2.py      # ROS2 AD API interface
│   ├── launch_manager/     # Launch session management
│   └── tools/              # MCP tool implementations
├── tests/                  # Test suite
├── examples/               # Usage examples
├── book/                   # Documentation (mdBook)
└── pyproject.toml          # Project configuration
```

### Making Changes

1. **Create a feature branch**:
```bash
git checkout -b feature/your-feature-name
```

2. **Make your changes**:
   - Follow existing code style and patterns
   - Add docstrings to all functions and classes
   - Update tests for new functionality

3. **Run code formatting**:
```bash
uv run ruff format .
```

4. **Run linting**:
```bash
uv run ruff check .
```

5. **Run tests**:
```bash
# All tests
uv run pytest tests/ -v

# Specific test file
uv run pytest tests/test_ad_api.py -v

# With coverage
uv run pytest tests/ --cov=autoware_mcp --cov-report=html
```

6. **Test with MCP server**:
```bash
# Start the server
uv run autoware-mcp

# In another terminal, test with example scripts
python examples/autonomous_drive_mcp_tools.py
```

## Code Style Guidelines

### Python Code

- Use type hints for all function parameters and returns
- Follow PEP 8 style guide
- Maximum line length: 100 characters
- Use descriptive variable names
- Add docstrings in Google style:

```python
def example_function(param1: str, param2: int) -> dict:
    """Brief description of function.
    
    Args:
        param1: Description of param1
        param2: Description of param2
        
    Returns:
        Description of return value
        
    Raises:
        ValueError: When invalid input provided
    """
    pass
```

### Async Code

- Use `async`/`await` for all asynchronous operations
- Handle exceptions properly with try/except
- Use `asyncio.gather()` for concurrent operations
- Add timeouts to prevent hanging:

```python
async def example_async():
    try:
        async with asyncio.timeout(10):
            result = await some_operation()
    except asyncio.TimeoutError:
        logger.error("Operation timed out")
```

### Error Handling

- Log errors with appropriate levels
- Provide helpful error messages
- Clean up resources in finally blocks
- Return structured error responses:

```python
{
    "success": False,
    "error": "Descriptive error message",
    "details": {...}  # Optional additional context
}
```

## Testing

### Writing Tests

- Place tests in `tests/` directory
- Name test files `test_*.py`
- Use pytest fixtures for setup/teardown
- Test both success and failure cases
- Mock external dependencies

Example test:
```python
import pytest
from autoware_mcp.tools import LaunchTools

@pytest.mark.asyncio
async def test_start_launch():
    """Test starting a launch session."""
    tools = LaunchTools()
    
    # Create test launch file
    test_file = "test.launch.py"
    
    # Start session
    result = await tools.start_launch(test_file)
    
    # Verify
    assert result["success"] is True
    assert "session_id" in result
    
    # Cleanup
    await tools.stop_launch(result["session_id"])
```

### Test Categories

- **Unit tests**: Test individual functions/methods
- **Integration tests**: Test component interactions
- **End-to-end tests**: Test complete workflows

### Running Tests

```bash
# Run all tests
uv run pytest tests/

# Run with verbose output
uv run pytest tests/ -v

# Run specific test
uv run pytest tests/test_ad_api.py::test_health_check

# Run with coverage
uv run pytest tests/ --cov=autoware_mcp

# Run only marked tests
uv run pytest tests/ -m "not slow"
```

## Adding New Features

### Adding a New MCP Tool

1. **Create tool implementation** in `src/autoware_mcp/tools/`:
```python
# src/autoware_mcp/tools/my_tool.py
from fastmcp import Tool

class MyTool:
    @Tool()
    async def my_operation(self, param: str) -> dict:
        """Tool description.
        
        Args:
            param: Parameter description
            
        Returns:
            Operation result
        """
        # Implementation
        return {"success": True, "result": ...}
```

2. **Register in server** (`src/autoware_mcp/server.py`):
```python
from .tools.my_tool import MyTool

# In setup_tools()
my_tool = MyTool()
mcp.add_tool(my_tool.my_operation)
```

3. **Add tests**:
```python
# tests/test_my_tool.py
@pytest.mark.asyncio
async def test_my_operation():
    tool = MyTool()
    result = await tool.my_operation("test")
    assert result["success"] is True
```

4. **Update documentation**:
   - Add to API reference
   - Include usage examples
   - Update CLAUDE.md if needed

### Adding ROS2 Service Integration

1. **Add to AD API** (`src/autoware_mcp/ad_api_ros2.py`):
```python
async def new_service_call(self, request_data):
    """Call a new ROS2 service."""
    return await self._call_service(
        "/service/path",
        "package/srv/ServiceType",
        request_data
    )
```

2. **Expose via MCP tool**:
```python
@Tool()
async def new_operation(self, param: str) -> dict:
    """MCP tool wrapping the service."""
    result = await self.ad_api.new_service_call({"param": param})
    return {
        "success": result.success,
        "data": result.data
    }
```

## Documentation

### Documentation Structure

```
book/
├── src/
│   ├── SUMMARY.md          # Table of contents
│   ├── introduction.md     # Project overview
│   ├── getting-started/    # Installation, quickstart
│   ├── user-guide/         # Usage documentation
│   ├── architecture/       # System design
│   ├── developer/          # API reference
│   └── reference/          # Troubleshooting, glossary
```

### Building Documentation

```bash
# Install mdBook
cargo install mdbook

# Build documentation
cd book
mdbook build

# Serve locally for preview
mdbook serve --open
```

### Writing Documentation

- Use clear, concise language
- Include code examples
- Add diagrams where helpful
- Keep examples up-to-date
- Test all code snippets

## Submitting Changes

### Pull Request Process

1. **Ensure all tests pass**:
```bash
uv run pytest tests/ -v
```

2. **Update documentation** if needed

3. **Create pull request**:
   - Use descriptive title
   - Reference any related issues
   - Describe what changes do
   - Include test results

4. **PR checklist**:
   - [ ] Tests pass
   - [ ] Code formatted
   - [ ] Documentation updated
   - [ ] CHANGELOG updated
   - [ ] No merge conflicts

### Commit Messages

Follow conventional commits format:
```
type(scope): brief description

Longer explanation if needed.

Fixes #123
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Code style changes
- `refactor`: Code refactoring
- `test`: Test additions/changes
- `chore`: Build/tooling changes

## Getting Help

### Resources

- [Project Issues](https://github.com/your-org/autoware-mcp/issues)
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [MCP Specification](https://modelcontextprotocol.io/)

### Communication

- **GitHub Issues**: Bug reports and feature requests
- **Discussions**: General questions and ideas
- **Pull Requests**: Code contributions

### Debugging Tips

1. **Enable debug logging**:
```bash
export AUTOWARE_MCP_LOG_LEVEL=DEBUG
uv run autoware-mcp
```

2. **Check ROS2 communication**:
```bash
ros2 node list
ros2 topic list
ros2 service list
```

3. **Monitor MCP traffic**:
```python
# In your test code
import logging
logging.basicConfig(level=logging.DEBUG)
```

4. **Use interactive debugging**:
```python
import pdb; pdb.set_trace()  # Or use debugger in IDE
```

## Code of Conduct

- Be respectful and inclusive
- Welcome newcomers
- Provide constructive feedback
- Focus on what is best for the community
- Show empathy towards others

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (Apache 2.0).

## Recognition

Contributors will be recognized in:
- AUTHORS.md file
- Release notes
- Project documentation

Thank you for contributing to Autoware MCP Server!