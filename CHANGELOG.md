# Changelog

All notable changes to the isaac-sim-mcp project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## [0.3.0] - 2024-04-22

### Added
- USD asset search integration with `search_3d_usd_by_text` tool
- Ability to search and load pre-existing 3D models from USD libraries
- Support for custom positioning and scaling of USD models
- Direct model transformation capabilities with the improved `transform` tool
- Enhanced scene management with multi-object placement

### Improved
- Scene object manipulation with precise positioning controls
- Asset loading performance and reliability
- Error handling for model search and placement
- Integration with existing physics scene management

### Technical Details
- Advanced USD model retrieval system
- Optimized asset loading pipeline
- Position and scale customization for USD models
- Better compatibility with Isaac Sim's native USD handling

## [0.2.1] - 2024-04-15

### Added
- Beaver3D integration for 3D model generation from text prompts and images
- Asynchronous model loading with asyncio support
- Task caching system to prevent duplicate model generation
- New MCP tools:
  - `generate_3d_from_text_or_image` for AI-powered 3D asset creation
  - `transform` for manipulating generated 3D models in the scene
- Texture and material binding for generated 3D models

### Improved
- Asynchronous command execution with `run_coroutine`
- Error handling and reporting for 3D generation tasks
- Performance optimizations for model loading

### Technical Details
- Integration with Beaver3D API for 3D generation
- Task monitoring with callback support
- Position and scale customization for generated models

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