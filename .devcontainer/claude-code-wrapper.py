#!/usr/bin/env python3
"""Launch Claude Code with the provider currently selected in cc-switch.

The cc-switch database is mounted read-only into the devcontainer.  Keeping
the lookup here (instead of copying a token into the repository or VS Code
settings) means a provider/model switch on the host is picked up by the next
Claude Code process automatically.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sqlite3
import sys


CC_SWITCH_DB = Path("/home/user/.cc-switch/cc-switch.db")
EXTENSIONS_DIR = Path("/home/user/.vscode-server/extensions")


def _provider_config() -> tuple[str, dict[str, str]]:
    if not CC_SWITCH_DB.is_file():
        raise RuntimeError(f"cc-switch database is not mounted: {CC_SWITCH_DB}")

    uri = f"file:{CC_SWITCH_DB}?mode=ro"
    with sqlite3.connect(uri, uri=True, timeout=3) as connection:
        row = connection.execute(
            """
            SELECT name, settings_config
            FROM providers
            WHERE app_type = 'claude' AND is_current = 1
            ORDER BY rowid DESC
            LIMIT 1
            """
        ).fetchone()

    if not row:
        raise RuntimeError("cc-switch has no current Claude provider")

    name, raw_config = row
    config = json.loads(raw_config)
    values = config.get("env", {})
    if not isinstance(values, dict):
        raise RuntimeError("cc-switch current provider has an invalid env section")

    # Only pass string environment values to Claude Code.  In particular, do
    # not print the auth token while diagnosing configuration problems.
    environment = {
        str(key): str(value)
        for key, value in values.items()
        if isinstance(key, str) and value is not None
    }
    if not environment.get("ANTHROPIC_BASE_URL"):
        raise RuntimeError("cc-switch current provider has no ANTHROPIC_BASE_URL")
    if not environment.get("ANTHROPIC_AUTH_TOKEN"):
        raise RuntimeError("cc-switch current provider has no ANTHROPIC_AUTH_TOKEN")
    return str(name), environment


def _native_claude() -> Path:
    candidates = list(
        EXTENSIONS_DIR.glob(
            "anthropic.claude-code-*/resources/native-binary/claude"
        )
    )
    candidates.extend(
        EXTENSIONS_DIR.glob(
            "anthropic.claude-code-*/resources/native-binaries/*/claude"
        )
    )
    candidates = [path for path in candidates if path.is_file() and os.access(path, os.X_OK)]
    if not candidates:
        raise RuntimeError("Claude Code native binary was not found in the VS Code server")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def main() -> int:
    try:
        provider_name, provider_env = _provider_config()
        executable = _native_claude()
    except (OSError, RuntimeError, sqlite3.Error, json.JSONDecodeError) as error:
        print(f"claude-code-wrapper: {error}", file=sys.stderr)
        return 78

    environment = os.environ.copy()
    environment.update(provider_env)
    # This is useful for DeepSeek-compatible endpoints and is harmless for
    # providers that do not use the optional tool-search path.
    environment.setdefault("CLAUDE_CODE_ENTRYPOINT", "claude-vscode")
    if os.environ.get("CLAUDE_CODE_WRAPPER_DEBUG") == "1":
        model = provider_env.get("ANTHROPIC_MODEL", "(provider default)")
        print(
            f"claude-code-wrapper: provider={provider_name!r} "
            f"base_url={provider_env['ANTHROPIC_BASE_URL']!r} model={model!r}",
            file=sys.stderr,
        )

    os.execve(str(executable), [str(executable), *sys.argv[1:]], environment)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
