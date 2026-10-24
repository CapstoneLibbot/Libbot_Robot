from __future__ import annotations

import json
import math
import os
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
        runtime_id = str(
            payload.get("runtime_id")
            or context.get("runtime_id")
            or os.getenv("LIBOT_RUNTIME_ID")
            or "real"
        ).strip().lower()
        base_url = self._resolve_backend_base_url(context)
        try:
            if self.operation == "SEARCH_BOOK":
                self._require_base_url(base_url)
                query = parse.urlencode({"query": payload.get("query", ""), "matchMode": payload.get("match_mode", "FUZZY")})
                search_route = "/api/workflow-data/simulation/books/search" if runtime_id == "sim" else "/api/book-guidance/search"
                data = self._json_request(f"{base_url}{search_route}?{query}")
            elif self.operation == "GET_BOOK_DETAIL":
                self._require_base_url(base_url)
                book_id = parse.quote(str(payload.get("book_id", "")), safe="")
                detail_route = "/api/workflow-data/simulation/books" if runtime_id == "sim" else "/api/book-guidance/books"
                data = self._json_request(f"{base_url}{detail_route}/{book_id}")
                copies = data.get("bookItems", []) if isinstance(data, dict) else []
                selected_item_id = context.get("book_item_id") or payload.get("book_item_id")
                if selected_item_id:
                    selected_item = next(
                        (item for item in copies if str(item.get("id", "")).lower() == str(selected_item_id).lower()),
                        None,
                    )
                else:
                    selected_item = next((item for item in copies
                        if str(item.get("status", "")).lower() == "available"
                        and isinstance(item.get("location"), dict)
                        and isinstance(item["location"].get("endpoint"), dict)), None)
                if selected_item is None or str(selected_item.get("status", "")).lower() != "available":
                    raise ValueError("No available copy of this book has a navigation endpoint.")
                location = selected_item.get("location")
                endpoint = location.get("endpoint") if isinstance(location, dict) else None
                if not isinstance(endpoint, dict):
                    raise ValueError("The selected book copy has no navigation endpoint.")
                try:
                    pose = [float(endpoint[key]) for key in ("x", "y", "theta")]
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError("The selected book copy has an invalid navigation endpoint.") from exc
                if not all(math.isfinite(value) for value in pose):
                    raise ValueError("The selected book copy has an invalid navigation endpoint.")
                data = dict(data)
                data["location"] = location
                data["selectedBookItem"] = selected_item
            elif self.operation == "SAVE_RATING":
                if runtime_id == "sim":
                    data = {"status": "simulated", "rating": int(payload.get("rating"))}
                else:
                    self._require_base_url(base_url)
                    data = self._json_request(
                        f"{base_url}/api/book-guidance/feedback",
                        method="POST",
                        body={"taskId": payload.get("task_id"), "rating": int(payload.get("rating"))},
                    )
            elif self.operation == "SEARCH_FACILITY":
                self._require_base_url(base_url)
                query = parse.urlencode({"query": payload.get("query", ""), "matchMode": payload.get("match_mode", "FUZZY")})
                search_route = "/api/workflow-data/simulation/facilities/search" if runtime_id == "sim" else "/api/facility-guidance/search"
                data = self._json_request(f"{base_url}{search_route}?{query}")
            elif self.operation == "GET_FACILITY_DETAIL":
                self._require_base_url(base_url)
                facility_id = parse.quote(str(payload.get("facility_id", "")), safe="")
                detail_route = "/api/workflow-data/simulation/facilities" if runtime_id == "sim" else "/api/facility-guidance/facilities"
                data = self._json_request(f"{base_url}{detail_route}/{facility_id}")
            elif self.operation == "SAVE_FACILITY_RATING":
                if runtime_id == "sim":
                    data = {"status": "simulated", "rating": int(payload.get("rating"))}
                else:
                    self._require_base_url(base_url)
                    data = self._json_request(
                        f"{base_url}/api/facility-guidance/feedback",
                        method="POST",
                        body={"taskId": payload.get("task_id"), "rating": int(payload.get("rating"))},
                    )
            elif self.operation == "RECORD_INVENTORY":
                self._require_base_url(base_url)
                data = self._json_request(
                    f"{base_url}/api/workflow-data/inventory/scan",
                    method="POST",
                    body={
                        "missionId": payload.get("mission_id") or context.get("mission_id"),
                        "runtimeId": payload.get("runtime_id") or runtime_id,
                        "barcode": payload.get("barcode"),
                        "shelfCode": payload.get("shelf_code"),
                    },
                )
            elif self.operation == "FINALIZE_INVENTORY":
                self._require_base_url(base_url)
                data = self._json_request(
                    f"{base_url}/api/workflow-data/inventory/reports",
                    method="POST",
                    body={
                        "missionId": payload.get("mission_id") or context.get("mission_id"),
                        "runtimeId": payload.get("runtime_id") or runtime_id,
                        "shelfCode": payload.get("shelf_code"),
                        "scopeType": payload.get("scope_type", "SHELF"),
                        "reportType": payload.get("report_type", "INVENTORY"),
                    },
                )
            elif self.operation == "RECORD_PATROL_OBSERVATION":
                self._require_base_url(base_url)
                data = self._json_request(
                    f"{base_url}/api/workflow-data/patrol/observations",
                    method="POST",
                    body={
                        "missionId": payload.get("mission_id") or context.get("mission_id"),
                        "runtimeId": payload.get("runtime_id") or runtime_id,
                        "observation": payload.get("observation"),
                    },
                )
            elif self.operation == "FINALIZE_PATROL":
                self._require_base_url(base_url)
                data = self._json_request(
                    f"{base_url}/api/workflow-data/patrol/reports",
                    method="POST",
                    body={
                        "missionId": payload.get("mission_id") or context.get("mission_id"),
                        "runtimeId": payload.get("runtime_id") or runtime_id,
                    },
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

    def _resolve_backend_base_url(self, context: RobotContext) -> str:
        # The API request host may be `localhost`, which refers to the robot
        # container when the simulation runs in Docker. Prefer the URL configured
        # for robot-to-backend traffic, then fall back to the workflow context.
        for candidate in (
            self.backend_base_url,
            os.getenv("LIBOT_BACKEND_URL"),
            context.get("backend_base_url"),
        ):
            if candidate is not None and str(candidate).strip():
                return str(candidate).strip().rstrip("/")
        return ""

    def _error(self, message: str) -> Dict[str, Any]:
        return {"status": "error", "type": "PROCESS_DATA", "action": self.operation, "message": message}
