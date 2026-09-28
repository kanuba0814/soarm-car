"""Conservative natural-language navigation intent gateway."""
from __future__ import annotations
import re

PLACE_IDS = frozenset(("room-1", "room-2", "room-3", "mian"))
_DESTINATION = re.compile(r"(?:room[- ]?[123]|mian)", re.I)
_UNSUPPORTED = ("探索", "explore", "找一下", "朝向", "heading", "巡航", "patrol")
_NEGATION = ("不要", "别", "不去", "不要去", "don't", "do not")


def parse_navigation_intent(text: str, source: str = "text") -> dict:
    if source not in ("text", "voice_transcript"):
        return {"status": "rejected", "reason": "unsupported_input_source", "motion_authorized": False}
    normalized = " ".join(str(text).strip().split())
    if not normalized:
        return {"status": "rejected", "reason": "empty_input", "motion_authorized": False}
    lowered = normalized.lower()
    if any(token in lowered for token in _UNSUPPORTED) or any(token in lowered for token in _NEGATION):
        return {"status": "rejected", "reason": "unsupported_or_unsafe_intent", "motion_authorized": False}
    matches = _DESTINATION.findall(normalized)
    ids = {re.sub(r"room[ -]", "room-", item.lower()) for item in matches}
    if len(ids) != 1:
        return {"status": "rejected", "reason": "destination_missing_or_ambiguous", "motion_authorized": False}
    place_id = next(iter(ids))
    if place_id not in PLACE_IDS:
        return {"status": "rejected", "reason": "destination_not_allowlisted", "motion_authorized": False}
    return {"status": "awaiting_confirmation", "reason": "explicit_confirmation_required",
            "motion_authorized": False, "input_source": source, "transcript": normalized,
            "request": {"place_id": place_id, "frame_id": "map", "require_heading": False}}
