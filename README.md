# Isaac Sim MCP Extension

This extension enables natural language control of NVIDIA Isaac Sim through the Mechanical, Computer, and Programmable (MCP) framework, allowing simulation manipulation through conversational AI.

## Features

- Natural language control of Isaac Sim
- Dynamic robot positioning and movement
- Custom lighting and scene creation
- Advanced robot simulations with obstacle navigation
- Interactive code preview before execution

## Requirements

- NVIDIA Isaac Sim 4.2.0 or higher
- Python 3.9+
- Cursor AI editor for MCP integration

## Pre-requisite

- Install uv/uvx: [https://github.com/astral-sh/uv](https://github.com/astral-sh/uv)
- Install mcp[cli]: [https://github.com/cursor-ai/mcp-python](https://github.com/cursor-ai/mcp-python)

## Installation

```bash
cd ~/Documents
git clone https://github.com/omni-mcp/isaac-sim-mcp
```

### Install and Enable Extension

Isaac Sim extension folder should point to your project folder:
- Extension location: `~/Documents/isaac-sim-mcp` 
- Extension ID: `isaac.sim.mcp_extension`

```bash
# Enable extension in Isaac Simulation
# cd to your Isaac Sim installation directory
# You can change assets root to local with --/persistent/isaac/asset_root/default="<your asset location>"
# By default it is an AWS bucket, e.g. --/persistent/isaac/asset_root/default="/share/Assets/Isaac/4.2"

cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./isaac-sim.sh --ext-folder /home/ubuntu/Documents/isaac-sim-mcp/ --enable isaac.sim.mcp_extension 
```

Verify the extension starts successfully. The output should look like:

[7.160s] [ext: isaac.sim.mcp_extension-0.1.0] startup
trigger  on_startup for:  isaac.sim.mcp_extension-0.1.0
settings:  {'envPath': '/home/ubuntu/.local/share/ov/data/Kit/Isaac-Sim/4.2/pip3-envs/default', 'archiveDirs': {}, 'installCheckIgnoreVersion': False, 'allowOnlineIndex': True, 'tryUpgradePipOnFirstUse': False}
Server thread startedIsaac Sim MCP server started on localhost:8766
```

The extension should be listening at **localhost:8766** by default.

### Install MCP Server

1. Start Cursor and open the folder `~/Documents/isaac-sim-mcp`
2. Go to Cursor preferences, choose MCP and add a global MCP server:

```json
{
    "mcpServers": {
        "isaac-sim": {
            "command": "uv run /home/ubuntu/Documents/isaac-sim-mcp/isaac_mcp/server.py"
        }
    }
}
```

### Development Mode

To develop the MCP Server, start the MCP inspector:

```bash
uv run mcp dev ~/Documents/isaac-sim-mcp/isaac_mcp/server.py
```

You can visit the debug page through http://localhost:5173

## Example Prompts for Simulation

### Robot Party
```
# Create robots and improve lighting
create  3x3 frankas robots in these current stage across location [3, 0, 0] and [6, 3, 0]
always check connection with get_scene_info before execute code.
add more light in the stage


# Add specific robots at positions
create a g1 robot at [3, 9, 0]
add Go1 robot at location [2, 1, 0]
move go1 robot to [1, 1, 0]
```

### Factory Setup
```
# Create multiple robots in a row
acreate  3x3 frankas robots in these current stage across location [3, 0, 0] and [6, 3, 0]
always check connection with get_scene_info before execute code.
add more light in the stage


```
### Vibe Coding from scratch
```
reference to g1.py to create an new g1 robot simulation and allow robot g1 walk straight  from [0, 0, 0] to [3, 0, 0] and [3, 3, 0]
create more obstacles in the stage

```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
