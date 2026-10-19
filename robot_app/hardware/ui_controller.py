from __future__ import annotations

import os
import queue
import threading
from typing import Any, Callable, Dict, Optional


class UIController:
    """Touchscreen adapter. Gazebo uses the web UI; a real robot uses a Tk fullscreen LCD."""

    _queue: queue.Queue[Any] = queue.Queue()
    _start_lock = threading.Lock()
    _started = False

    def __init__(
        self,
        settings: Optional[Dict[str, Any]] = None,
        event_callback: Optional[Callable[[str, Any], None]] = None,
    ):
        self.settings = settings or {}
        self.event_callback = event_callback

    def show(self, message: str) -> None:
        self.show_screen("MESSAGE", {"message": message})

    @staticmethod
    def probe() -> Dict[str, Any]:
        try:
            import tkinter as tk

            root = tk.Tk()
            root.withdraw()
            root.destroy()
            return {"status": "ready"}
        except Exception as exc:
            return {"status": "error", "message": f"Touchscreen display unavailable: {exc}"}

    def show_screen(self, screen: str, data: Any = None, event_type: Optional[str] = None) -> None:
        runtime_id = os.getenv("LIBOT_RUNTIME_ID", "real").lower()
        driver = str(self.settings.get("driver", "tkinter")).lower()
        if not self.settings.get("enabled", True) or runtime_id == "sim" or driver != "tkinter":
            print(f"UI[{screen}]: {data if data is not None else ''}", flush=True)
            return
        if not os.getenv("DISPLAY") and os.name != "nt":
            print(f"UI[{screen}]: {data if data is not None else ''}", flush=True)
            return
        self._ensure_started()
        inferred_event, options = self._screen_options(screen, data)
        self._queue.put({
            "screen": screen,
            "data": data,
            "event_type": event_type or inferred_event,
            "options": options,
            "callback": self.event_callback,
        })

    @classmethod
    def _ensure_started(cls) -> None:
        with cls._start_lock:
            if cls._started:
                return
            cls._started = True
            threading.Thread(target=cls._run_ui, name="libot-touchscreen", daemon=True).start()

    @classmethod
    def _run_ui(cls) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk

            root = tk.Tk()
            root.title("LiBot")
            root.attributes("-fullscreen", True)
            root.configure(background="#f4f6f8")
            frame = ttk.Frame(root, padding=24)
            frame.pack(fill="both", expand=True)

            def render() -> None:
                while True:
                    try:
                        item = cls._queue.get_nowait()
                    except queue.Empty:
                        break
                    for child in frame.winfo_children():
                        child.destroy()
                    ttk.Label(frame, text=str(item["screen"]).replace("_", " "), font=("Arial", 24, "bold")).pack(pady=(8, 20))
                    body = item.get("data")
                    prompt = body.get("prompt") if isinstance(body, dict) else None
                    if prompt:
                        ttk.Label(frame, text=str(prompt), font=("Arial", 17)).pack(pady=8)
                    if isinstance(body, dict):
                        heading = body.get("title") or body.get("name") or body.get("message")
                        if heading:
                            ttk.Label(frame, text=str(heading), font=("Arial", 18, "bold"), wraplength=900).pack(pady=6)
                        details = []
                        for key, label in (("author", "Author"), ("classification", "Category"),
                                           ("description", "Details"), ("floor", "Floor"), ("shelf", "Shelf")):
                            if body.get(key):
                                details.append(f"{label}: {body[key]}")
                        location = body.get("location")
                        if isinstance(location, dict):
                            for key, label in (("floor", "Floor"), ("aisle", "Aisle"), ("shelf", "Shelf"), ("mapName", "Map")):
                                if location.get(key):
                                    details.append(f"{label}: {location[key]}")
                        if details:
                            ttk.Label(frame, text="\n".join(details), font=("Arial", 15), justify="left").pack(pady=8)
                    elif isinstance(body, str):
                        ttk.Label(frame, text=body, font=("Arial", 16), wraplength=900).pack(pady=8)
                    for label, value in item.get("options", []):
                        callback = item.get("callback")
                        event_type = item.get("event_type")
                        if callback is None or not event_type:
                            ttk.Label(frame, text=label, font=("Arial", 15)).pack(pady=4)
                            continue
                        ttk.Button(
                            frame,
                            text=label,
                            command=lambda event=event_type, selected=value, sink=callback: sink(event, selected),
                        ).pack(fill="x", padx=30, pady=6, ipady=8)
                root.after(100, render)

            root.after(100, render)
            root.mainloop()
        except Exception as exc:
            print(f"Touchscreen UI unavailable: {exc}", flush=True)

    @staticmethod
    def _screen_options(screen: str, data: Any) -> tuple[Optional[str], list[tuple[str, Any]]]:
        if not isinstance(data, dict):
            return None, []
        normalized = screen.upper()
        if normalized == "BOOK_SEARCH_RESULTS":
            return "BOOK_SELECTED", UIController._record_options(data, ("books", "results"), ("bookId", "id"), ("title", "name"))
        if normalized == "FACILITY_SEARCH_RESULTS":
            return "FACILITY_SELECTED", UIController._record_options(data, ("facilities", "results"), ("facilityId", "id"), ("name", "title"))
        if normalized == "BOOK_ACTION":
            return "BOOK_ACTION", [("Show location", {"action": "VIEW_LOCATION"}), ("Start guidance", {"action": "START_GUIDANCE"}), ("Rate", {"action": "RATE"})]
        if normalized == "RATING":
            return "RATING_SUBMIT", [(str(value), {"rating": value}) for value in range(1, 6)]
        options = data.get("options")
        if isinstance(options, list):
            event_type = data.get("event_type")
            return str(event_type) if event_type else None, [
                (str(value), {"value": value}) for value in options
            ]
        return None, []

    @staticmethod
    def _record_options(
        data: Dict[str, Any],
        list_keys: tuple[str, ...],
        id_keys: tuple[str, ...],
        label_keys: tuple[str, ...],
    ) -> list[tuple[str, Any]]:
        records: Any = data
        for key in list_keys:
            if isinstance(data.get(key), list):
                records = data[key]
                break
        if not isinstance(records, list):
            return []
        options = []
        for record in records:
            if not isinstance(record, dict):
                continue
            selected_id = next((record[key] for key in id_keys if key in record), None)
            label = next((record[key] for key in label_keys if key in record), selected_id)
            if selected_id is not None:
                options.append((str(label), {"value": selected_id}))
        return options
