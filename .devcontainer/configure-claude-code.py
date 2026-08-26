#!/usr/bin/env python3
"""Configure the Claude Code VS Code extension to use the cc-switch wrapper."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3


DB = Path("/home/user/.cc-switch/cc-switch.db")
MACHINE_SETTINGS = Path("/home/user/.vscode-server/data/Machine/settings.json")
WRAPPER = "/workspace/.devcontainer/claude-code-wrapper.py"


def current_provider() -> tuple[str, str, str]:
    uri = f"file:{DB}?mode=ro"
    with sqlite3.connect(uri, uri=True, timeout=3) as connection:
        provider = connection.execute(
            """
            SELECT name, settings_config
            FROM providers
            WHERE app_type = 'claude' AND is_current = 1
            ORDER BY rowid DESC
            LIMIT 1
            """
        ).fetchone()
        if not provider:
            raise RuntimeError("cc-switch has no current Claude provider")
        endpoint = connection.execute(
            """
            SELECT url
            FROM provider_endpoints
            WHERE app_type = 'claude'
              AND provider_id = (
                  SELECT id FROM providers
                  WHERE app_type = 'claude' AND is_current = 1
                  ORDER BY rowid DESC LIMIT 1
              )
            ORDER BY id DESC LIMIT 1
            """
        ).fetchone()

    config = json.loads(provider[1])
    env = config.get("env", {})
    base_url = str(env.get("ANTHROPIC_BASE_URL", endpoint[0] if endpoint else ""))
    model = str(env.get("ANTHROPIC_MODEL", "(provider default)"))
    return str(provider[0]), base_url, model


def update_machine_settings() -> None:
    if MACHINE_SETTINGS.is_file():
        try:
            settings = json.loads(MACHINE_SETTINGS.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            settings = {}
    else:
        settings = {}
    settings["claudeCode.claudeProcessWrapper"] = WRAPPER
    MACHINE_SETTINGS.parent.mkdir(parents=True, exist_ok=True)
    temporary = MACHINE_SETTINGS.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(settings, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.chmod(0o600)
    temporary.replace(MACHINE_SETTINGS)


def main() -> int:
    try:
        name, base_url, model = current_provider()
        update_machine_settings()
    except (OSError, RuntimeError, sqlite3.Error, json.JSONDecodeError) as error:
        print(f"Claude Code provider setup skipped: {error}")
        return 0

    print(
        "Configured Claude Code cc-switch wrapper: "
        f"provider={name!r}, base_url={base_url!r}, model={model!r}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
