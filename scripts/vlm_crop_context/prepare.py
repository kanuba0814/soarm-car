"""CPU image preparation for bounded candidate crop/context VLM experiments."""
import hashlib
import math

from PIL import Image, ImageDraw, ImageFont, ImageOps


def rgb_content_sha256(image):
    image = image.convert("RGB")
    return hashlib.sha256(image.tobytes()).hexdigest()


def clip_box_xyxy(box_xyxy, image_size):
    """Clip a finite xyxy box and reject boxes that cannot yield pixels."""
    if len(box_xyxy) != 4 or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in box_xyxy):
        raise ValueError("box_xyxy must contain four finite numeric values")
    left, top, right, bottom = box_xyxy
    width, height = image_size
    clipped = (max(0.0, left), max(0.0, top), min(float(width), right), min(float(height), bottom))
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        raise ValueError("box_xyxy is empty after clipping")
    return clipped


def _font():
    return ImageFont.load_default()


def _draw_label(draw, xy, text, fill):
    box = draw.textbbox(xy, text, font=_font())
    draw.rectangle(box, fill=fill)
    draw.text(xy, text, font=_font(), fill="white")


def _annotated_full(image, candidates):
    canvas = image.copy()
    draw = ImageDraw.Draw(canvas)
    for candidate in candidates:
        draw.rectangle(candidate["clipped_box_xyxy"], outline="#00c853", width=3)
        left, top, _, _ = candidate["clipped_box_xyxy"]
        _draw_label(draw, (int(left) + 2, max(0, int(top) - 12)), candidate["display_label"], "#00695c")
    return canvas


def _fit_with_padding(image, size):
    contained = ImageOps.contain(image, size, Image.Resampling.LANCZOS)
    tile = Image.new("RGB", size, "#202020")
    tile.paste(contained, ((size[0] - contained.width) // 2, (size[1] - contained.height) // 2))
    return tile, [contained.width, contained.height]


def prepare_views(image, detections, display_labels, padding_fraction=0.18, tile_size=(256, 192)):
    """Create full, crop-sheet, and named context panels from real RGB pixels."""
    image = image.convert("RGB")
    if len(detections) != len(display_labels):
        raise ValueError("detections and display_labels must have identical lengths")
    source_hash = rgb_content_sha256(image)
    candidates = []
    for detection, display_label in zip(detections, display_labels):
        original = list(detection["box_xyxy"])
        clipped = clip_box_xyxy(original, image.size)
        left, top, right, bottom = clipped
        pad_x, pad_y = (right - left) * padding_fraction, (bottom - top) * padding_fraction
        context_box = clip_box_xyxy((left - pad_x, top - pad_y, right + pad_x, bottom + pad_y), image.size)
        crop = image.crop(tuple(round(value) for value in context_box))
        if crop.width == 0 or crop.height == 0:
            raise ValueError("crop unexpectedly empty")
        candidates.append({
            "opaque_id": f"{detection['observation_id']}:{detection['detection_index']}",
            "display_label": str(display_label),
            "original_box_xyxy": original,
            "clipped_box_xyxy": list(clipped),
            "context_box_xyxy": list(context_box),
            "source_rgb_sha256": source_hash,
            "crop_source_size_px": [crop.width, crop.height],
            "crop": crop,
        })
    full = _annotated_full(image, candidates)
    columns = 2
    rows = max(1, math.ceil(len(candidates) / columns))
    sheet = Image.new("RGB", (columns * tile_size[0], rows * (tile_size[1] + 18)), "#101010")
    draw = ImageDraw.Draw(sheet)
    for index, candidate in enumerate(candidates):
        x, y = (index % columns) * tile_size[0], (index // columns) * (tile_size[1] + 18)
        tile, resized = _fit_with_padding(candidate.pop("crop"), tile_size)
        sheet.paste(tile, (x, y + 18))
        _draw_label(draw, (x + 3, y + 3), candidate["display_label"], "#00695c")
        candidate["rendered_crop_size_px"] = resized
    panel_width = max(full.width, sheet.width)
    panels = Image.new("RGB", (panel_width, full.height + sheet.height + 36), "#101010")
    panels.paste(full, ((panel_width - full.width) // 2, 18))
    panels.paste(sheet, ((panel_width - sheet.width) // 2, full.height + 36))
    panel_draw = ImageDraw.Draw(panels)
    _draw_label(panel_draw, (3, 3), "FULL", "#424242")
    _draw_label(panel_draw, (3, full.height + 20), "CROPS", "#424242")
    return {
        "full_annotated": full,
        "crop_contact_sheet": sheet,
        "full_plus_crop_panels": panels,
        "candidates": candidates,
        "source_rgb_sha256": source_hash,
    }
