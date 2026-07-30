#!/usr/bin/env python3
"""Register the project-local MiMo vision MCP server for Claude Code."""

import json
import os
from pathlib import Path


HOME = Path(os.environ.get("HOME", "/home/user"))
CONFIG = HOME / ".claude.json"
SERVER = "/workspace/.devcontainer/mimo-vision-server.py"
KEY_FILE = str(HOME / ".config/xiaomi-mimo/api_key")


def main():
    if CONFIG.is_file():
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
    else:
        config = {}

    config.setdefault("mcpServers", {})["mimo-vision"] = {
        "type": "stdio",
        "command": "/usr/bin/python3",
        "args": [SERVER],
        "env": {"MIMO_API_KEY_FILE": KEY_FILE},
    }

    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    temporary = CONFIG.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.chmod(0o600)
    temporary.replace(CONFIG)
    print("Configured Claude Code MCP server: mimo-vision")


if __name__ == "__main__":
    main()
