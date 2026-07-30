#!/usr/bin/env python3
"""Claude Code MCP server backed by Xiaomi MiMo image understanding."""

import base64
import json
import mimetypes
import os
import sys
from pathlib import Path

import requests


API_URL = os.environ.get(
    "MIMO_API_URL", "https://api.xiaomimimo.com/v1/chat/completions"
)
MODEL = os.environ.get("MIMO_VISION_MODEL", "mimo-v2.5")
KEY_FILE = Path(
    os.environ.get("MIMO_API_KEY_FILE", "~/.config/xiaomi-mimo/api_key")
).expanduser()
MAX_IMAGE_BYTES = 50 * 1024 * 1024

TOOL = {
    "name": "analyze_image",
    "description": (
        "使用 Xiaomi MiMo-V2.5 识别或分析一张图片。"
        "支持本地图片路径和公开的 HTTP/HTTPS 图片 URL。"
    ),
    "inputSchema": {
        "type": "object",
        "properties": {
            "image": {
                "type": "string",
                "description": "容器内的本地图片路径，或公开图片 URL",
            },
            "question": {
                "type": "string",
                "description": "希望模型针对图片回答的问题",
                "default": "请详细描述并分析这张图片。",
            },
        },
        "required": ["image"],
    },
}


def get_api_key() -> str:
    key = os.environ.get("MIMO_API_KEY", "").strip()
    if not key and KEY_FILE.is_file():
        key = KEY_FILE.read_text(encoding="utf-8").strip()
    if not key:
        raise RuntimeError(f"MiMo API Key 未配置：{KEY_FILE}")
    return key


def image_part(source: str) -> dict:
    if source.startswith(("https://", "http://")):
        return {"type": "image_url", "image_url": {"url": source}}

    path = Path(source).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"图片不存在或不是文件：{path}")
    if path.stat().st_size > MAX_IMAGE_BYTES:
        raise ValueError("单张图片不能超过 50 MB")
    mime, _ = mimetypes.guess_type(path.name)
    if not mime or not mime.startswith("image/"):
        raise ValueError(f"无法识别图片格式：{path.name}")
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return {
        "type": "image_url",
        "image_url": {"url": f"data:{mime};base64,{encoded}"},
    }


def analyze(arguments: dict) -> str:
    part = image_part(arguments["image"])
    question = arguments.get("question") or "请详细描述并分析这张图片。"
    response = requests.post(
        API_URL,
        headers={
            "Authorization": f"Bearer {get_api_key()}",
            "Content-Type": "application/json",
        },
        json={
            "model": MODEL,
            "messages": [
                {
                    "role": "user",
                    "content": [part, {"type": "text", "text": question}],
                }
            ],
        },
        timeout=120,
    )
    response.raise_for_status()
    content = response.json()["choices"][0]["message"]["content"]
    if isinstance(content, list):
        content = "\n".join(
            item.get("text", "") for item in content if isinstance(item, dict)
        )
    return str(content).strip() or "（模型未返回文本）"


def result(request_id, value):
    return {"jsonrpc": "2.0", "id": request_id, "result": value}


def error(request_id, code, message):
    return {
        "jsonrpc": "2.0",
        "id": request_id,
        "error": {"code": code, "message": message},
    }


def handle(message: dict):
    request_id = message.get("id")
    method = message.get("method")
    if request_id is None:
        return None
    if method == "initialize":
        requested_version = message.get("params", {}).get(
            "protocolVersion", "2024-11-05"
        )
        return result(
            request_id,
            {
                "protocolVersion": requested_version,
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "mimo-vision", "version": "1.0.0"},
            },
        )
    if method == "ping":
        return result(request_id, {})
    if method == "tools/list":
        return result(request_id, {"tools": [TOOL]})
    if method == "tools/call":
        params = message.get("params", {})
        if params.get("name") != TOOL["name"]:
            return error(request_id, -32602, "未知工具")
        try:
            text = analyze(params.get("arguments", {}))
            return result(
                request_id, {"content": [{"type": "text", "text": text}]}
            )
        except requests.HTTPError as exc:
            body = exc.response.text[:1000] if exc.response is not None else ""
            text = f"MiMo API 请求失败：{exc}\n{body}"
        except Exception as exc:
            text = f"识图失败：{exc}"
        return result(
            request_id,
            {"content": [{"type": "text", "text": text}], "isError": True},
        )
    return error(request_id, -32601, f"不支持的方法：{method}")


def main():
    for line in sys.stdin:
        try:
            response = handle(json.loads(line))
            if response is not None:
                print(json.dumps(response, ensure_ascii=False), flush=True)
        except Exception as exc:
            print(json.dumps(error(None, -32700, str(exc))), flush=True)


if __name__ == "__main__":
    main()
