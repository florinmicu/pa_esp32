from __future__ import annotations

import json
from urllib import request


def get_json(url: str, timeout_s: float) -> dict:
    with request.urlopen(url, timeout=timeout_s) as resp:
        data = resp.read().decode("utf-8", errors="ignore")
    parsed = json.loads(data)
    return parsed if isinstance(parsed, dict) else {}


def request_ok(url: str, timeout_s: float, method: str = "GET", data: bytes | None = None) -> bool:
    req = request.Request(url, data=data, method=method)
    with request.urlopen(req, timeout=timeout_s):
        return True

