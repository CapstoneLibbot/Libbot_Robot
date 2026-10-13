from __future__ import annotations

import json
from typing import Any, Dict, Optional
from urllib import parse, request

from core.context import RobotContext
from steps.base_step import BaseStep


class ProcessDataStep(BaseStep):
    """Step Type: PROCESS_DATA"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.operation = str(self.config.get("action", self.config.get("operation", "identity"))).upper()
        self.payload = self.config.get("payload", {})
        self.backend_base_url = self.config.get("backend_base_url")
        self.timeout_sec = float(self.config.get("timeout_sec", 10))

    def execute(self, context: RobotContext) -> Any:
        payload = self.resolve(context, self.payload)
        base_url = str(self.backend_base_url or context.get("backend_base_url") or "").rstrip("/")
        try:
            if self.operation == "SEARCH_BOOK":
                self._require_base_url(base_url)
                query = parse.urlencode({"query": payload.get("query", ""), "matchMode": payload.get("match_mode", "FUZZY")})
                data = self._json_request(f"{base_url}/api/book-guidance/search?{query}")
            elif self.operation == "GET_BOOK_DETAIL":
                self._require_base_url(base_url)
                book_id = parse.quote(str(payload.get("book_id", "")), safe="")
                data = self._json_request(f"{base_url}/api/book-guidance/books/{book_id}")
            elif self.operation == "SAVE_RATING":
                self._require_base_url(base_url)
                data = self._json_request(
                    f"{base_url}/api/book-guidance/feedback",
                    method="POST",
                    body={"taskId": payload.get("task_id"), "rating": int(payload.get("rating"))},
                )
            elif self.operation in {"IDENTITY", ""}:
                data = payload
            else:
                return self._error(f"Unsupported PROCESS_DATA action: {self.operation}")
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            return self._error(str(exc))

        result = {"status": "processed", "type": "PROCESS_DATA", "action": self.operation, "data": data}
        context.set("processed_data", result)
        return result

    def _json_request(self, url: str, method: str = "GET", body: Any = None) -> Any:
        payload = None if body is None else json.dumps(body).encode("utf-8")
        http_request = request.Request(url, data=payload, method=method)
        http_request.add_header("Accept", "application/json")
        if payload is not None:
            http_request.add_header("Content-Type", "application/json")
        with request.urlopen(http_request, timeout=self.timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))

    def _require_base_url(self, base_url: str) -> None:
        if not base_url:
            raise ValueError("backend_base_url is required for this PROCESS_DATA action.")

    def _error(self, message: str) -> Dict[str, Any]:
        return {"status": "error", "type": "PROCESS_DATA", "action": self.operation, "message": message}
