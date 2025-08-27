# Introduction

Welcome to the **Autoware MCP Server Developer Guide**! This comprehensive guide will help you understand, use, and contribute to the Autoware MCP integration project.

## What is Autoware MCP?

The Autoware Model Context Protocol (MCP) Server is a universal bridge that enables **any AI agent** (Claude, GPT, Gemini, or custom agents) to control and interact with the [Autoware](https://autoware.org/) autonomous driving stack. It provides a standardized interface for AI-driven mission planning, real-time vehicle control, and adaptive decision-making for autonomous vehicles.

## Key Features

- 🤖 **Universal AI Support**: Works with any MCP-compatible AI agent
- 🚗 **Full Vehicle Control**: Complete control over Autoware's autonomous driving features
- 📊 **Real-Time Monitoring**: Concurrent monitoring of perception, planning, and vehicle state
- 🛡️ **Multi-Layer Safety**: Comprehensive safety validation at every level
- 🔄 **Adaptive Behavior**: Dynamic adjustment based on real-time conditions
- 🎯 **Mission Execution**: Support for complex multi-step missions with waypoints
- 🚀 **Launch Management**: Advanced ROS2 launch session management with PID/PGID tracking

## Why Use Autoware MCP?

### For AI Researchers
- Test and develop AI-driven autonomous driving algorithms
- Experiment with different decision-making strategies
- Create adaptive behaviors based on real-time sensor data

### For Developers
- Standardized API for vehicle control
- Easy integration with existing AI systems
- Comprehensive testing and simulation support

### For Robotics Engineers
- Bridge between high-level AI planning and low-level vehicle control
- Safe testing environment with multiple safety layers
- Real-world deployment capabilities

## Architecture Overview

The Autoware MCP Server acts as a middleware layer between AI agents and the Autoware stack:

```
┌─────────────┐     MCP Protocol      ┌─────────────────┐
│  AI Agent   │◄─────────────────────►│  Autoware MCP   │
│   (Claude,  │                        │     Server      │
│  GPT, etc.) │                        └────────┬────────┘
└─────────────┘                                 │
                                          ROS2 Interface
                                                │
                                    ┌───────────▼───────────┐
                                    │    Autoware Stack     │
                                    │  ┌─────────────────┐  │
                                    │  │   Perception    │  │
                                    │  ├─────────────────┤  │
                                    │  │    Planning     │  │
                                    │  ├─────────────────┤  │
                                    │  │    Control      │  │
                                    │  └─────────────────┘  │
                                    └───────────────────────┘
```

## Guide Structure

This guide is organized into several sections:

- **Getting Started**: Installation, setup, and your first autonomous driving mission
- **User Guide**: Core concepts and practical examples
- **Architecture**: Deep dive into system design and components
- **Developer Guide**: API reference, testing, and contributing
- **Advanced Topics**: Specialized topics for power users
- **Reference**: Quick reference, troubleshooting, and roadmap

## Prerequisites

Before diving in, you should have:

- Basic understanding of autonomous driving concepts
- Familiarity with ROS2 (Robot Operating System 2)
- Python programming experience
- Ubuntu 22.04 LTS or compatible system

## Quick Links

- [Installation Guide](./getting-started/installation.md) - Get up and running quickly
- [Quick Start](./getting-started/quickstart.md) - Your first autonomous mission
- [API Reference](./developer/api-reference.md) - Complete API documentation
- [Examples](./user-guide/autonomous-driving.md) - Real-world usage examples

## Getting Help

If you need assistance:

- 📖 Check the [Troubleshooting Guide](./reference/troubleshooting.md)
- 🐛 Report issues on [GitHub](https://github.com/your-org/autoware-mcp/issues)
- 💬 Join our community discussions
- 📧 Contact the maintainers

Let's begin your journey into AI-powered autonomous driving!