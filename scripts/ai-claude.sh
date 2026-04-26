#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if ! command -v claude >/dev/null 2>&1; then
  echo "claude CLI not found in PATH" >&2
  exit 127
fi
exec claude "$@"
