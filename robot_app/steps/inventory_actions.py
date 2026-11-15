from __future__ import annotations

from typing import Any, Callable, Dict

from core.context import RobotContext


INVENTORY_ACTIONS = {
    "RESOLVE_INVENTORY_SCOPE",
    "ADVANCE_INVENTORY_SCOPE",
    "VERIFY_BOOK",
    "UPDATE_BOOK_ITEM_INVENTORY_STATUS",
    "GENERATE_REPORT",
}


class InventoryActionClient:
    """Calls the backend inventory action endpoint from PROCESS_DATA steps."""

    def __init__(
        self,
        json_request: Callable[..., Any],
        idempotency_key: Callable[[Dict[str, Any], RobotContext], str],
    ):
        self._json_request = json_request
        self._idempotency_key = idempotency_key

    def execute(
        self,
        action: str,
        payload: Dict[str, Any],
        context: RobotContext,
        runtime_id: str,
        backend_base_url: str,
    ) -> Dict[str, Any]:
        if not backend_base_url:
            raise ValueError("backend_base_url is required for this inventory PROCESS_DATA action.")
        mission_id = payload.get("mission_id") or context.get("mission_id") or context.get("task_id")
        execution_id = context.get("execution_id") or mission_id
        if not mission_id or not execution_id:
            raise ValueError("mission_id and execution_id are required for inventory actions.")

        response = self._json_request(
            f"{backend_base_url.rstrip('/')}/api/workflow-data/inventory/actions",
            method="POST",
            body={
                "missionId": mission_id,
                "runtimeId": runtime_id,
                "executionId": execution_id,
                "action": action,
                "payload": payload,
                "idempotencyKey": self._idempotency_key(payload, context),
            },
        )
        if not isinstance(response, dict):
            raise ValueError("Inventory action returned an invalid response.")
        action_status = str(response.get("action_status", response.get("actionStatus", ""))).upper()
        if action_status != "SUCCEEDED":
            raise ValueError("Inventory action did not return SUCCEEDED.")
        return {
            "status": "processed",
            "type": "PROCESS_DATA",
            "action": action,
            "data": response.get("data", {}),
            "action_status": action_status,
            "request_id": response.get("request_id", response.get("requestId")),
            "affected_records": response.get("affected_records", response.get("affectedRecords", 0)),
        }
