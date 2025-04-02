# Changelog

All notable changes to the isaac-sim-mcp project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2025-04-02

### Added
- Initial implementation of Isaac Sim MCP Extension
- Natural language control interface for Isaac Sim through MCP framework
- Core robot manipulation capabilities:
  - Dynamic placement and positioning of robots (Franka, G1, Go1, Jetbot)
  - Robot movement controls with position updates
  - Multi-robot grid creation (3x3 arrangement support)
- Advanced simulation features:
  - Quadruped robot walking simulation with waypoint navigation
  - Physics-based interactions between robots and environment
  - Custom lighting controls for better scene visualization
- Environment enrichment:
  - Various obstacle types: boxes, spheres, cylinders, cones
  - Wall creation for maze-like environments
  - Dynamic obstacle placement with customizable properties
- Development tools:
  - MCP server integration with Cursor AI
  - Debug interface accessible via local web server
  - Connection status verification with `get_scene_info`
- Documentation:
  - Installation instructions
  - Example prompts for common simulation scenarios
  - Configuration guidelines

### Technical Details
- Extension server running on localhost:8766
- Compatible with NVIDIA Isaac Sim 4.2.0
- Support for Python 3.9+
- MIT License for open development 