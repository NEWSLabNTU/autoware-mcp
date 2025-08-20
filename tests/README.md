# Autoware MCP Test Suite

This directory contains comprehensive tests for the Autoware MCP server tools.

## Test Organization

The tests are organized by functionality:

- `test_system_tools.py` - System health, status, and configuration tools
- `test_ros2_tools.py` - ROS2 node, topic, and service interaction tools  
- `test_autoware_tools.py` - Autoware-specific status and vehicle state tools
- `test_operation_mode_tools.py` - Operation mode control (stop, autonomous, etc.)
- `test_routing_tools.py` - Route planning and management
- `test_localization_tools.py` - Localization initialization and monitoring
- `test_mrm_tools.py` - Minimum Risk Maneuver (MRM) tools
- `test_control_tools.py` - Vehicle control commands (velocity, steering, etc.)
- `test_cooperation_tools.py` - Cooperation policies and commands
- `test_integration_autonomous_drive.py` - End-to-end autonomous driving scenarios

## Running Tests with Rye

### Run all tests
```bash
rye run test
```

### Run tests with verbose output
```bash
rye run test-verbose
```

### Run tests with coverage report
```bash
rye run test-cov
```

### Run integration tests only
```bash
rye run test-integration
```

### Run all tests with summary
```bash
rye run test-all
```

### Run specific test file
```bash
rye run python -m pytest tests/test_system_tools.py
```

### Run specific test function
```bash
rye run python -m pytest tests/test_system_tools.py::test_health_check
```

## Test Coverage

The test suite aims to provide comprehensive coverage of all MCP tools:

1. **Unit Tests**: Test individual MCP tools with mocked dependencies
2. **Integration Tests**: Test complete workflows like autonomous driving

## Writing New Tests

When adding new MCP tools, follow these steps:

1. Add the tool function to `test_helpers.py`
2. Create or update the appropriate test file
3. Mock the underlying dependencies (ros2_manager, ad_api, etc.)
4. Test both success and error cases

Example test structure:
```python
@pytest.mark.asyncio
async def test_my_new_tool():
    """Test description."""
    with patch('autoware_mcp.server.dependency') as mock_dep:
        mock_dep.method = AsyncMock(return_value={...})
        
        result = await my_new_tool(param="value")
        assert result.success is True
```

## Dependencies

Test dependencies are managed by rye and specified in `pyproject.toml`:
- pytest
- pytest-asyncio  
- pytest-cov

## Continuous Integration

These tests are designed to run in CI/CD pipelines. Ensure all tests pass before merging changes.