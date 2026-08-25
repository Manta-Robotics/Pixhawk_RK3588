#!/usr/bin/env python3
"""Download a China-only offline NASA Blue Marble XYZ tile package."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
import time
import urllib.request
from pathlib import Path


BOUNDS = {"west": 70.0, "south": 15.0, "east": 140.0, "north": 55.0}
MAX_ZOOM = 8
TILE_URL = (
    "https://gibs.earthdata.nasa.gov/wmts/epsg3857/best/"
    "BlueMarble_ShadedRelief_Bathymetry/default/"
    "GoogleMapsCompatible_Level8/{z}/{y}/{x}.jpg"
)


def tile_x(longitude: float, zoom: int) -> int:
    return max(0, min((1 << zoom) - 1, math.floor((longitude + 180.0) / 360.0 * (1 << zoom))))


def tile_y(latitude: float, zoom: int) -> int:
    latitude = max(-85.05112878, min(85.05112878, latitude))
    radians = math.radians(latitude)
    value = (1.0 - math.asinh(math.tan(radians)) / math.pi) / 2.0 * (1 << zoom)
    return max(0, min((1 << zoom) - 1, math.floor(value)))


def enumerate_tiles() -> list[tuple[int, int, int]]:
    tiles: list[tuple[int, int, int]] = []
    for zoom in range(MAX_ZOOM + 1):
        x0, x1 = tile_x(BOUNDS["west"], zoom), tile_x(BOUNDS["east"], zoom)
        y0, y1 = tile_y(BOUNDS["north"], zoom), tile_y(BOUNDS["south"], zoom)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                tiles.append((zoom, x, y))
    return tiles


def download_tile(output: Path, tile: tuple[int, int, int]) -> str:
    zoom, x, y = tile
    target = output / str(zoom) / str(x) / f"{y}.jpg"
    if target.exists() and target.stat().st_size > 512:
        return "cached"
    target.parent.mkdir(parents=True, exist_ok=True)
    url = TILE_URL.format(z=zoom, x=x, y=y)
    request = urllib.request.Request(url, headers={"User-Agent": "MANTA-offline-map-builder/1.0"})
    last_error: Exception | None = None
    for attempt in range(4):
        temporary = target.with_suffix(".jpg.part")
        try:
            with urllib.request.urlopen(request, timeout=30) as response, temporary.open("wb") as stream:
                stream.write(response.read())
            if temporary.stat().st_size <= 512:
                raise RuntimeError(f"empty tile {url}")
            os.replace(temporary, target)
            return "downloaded"
        except Exception as error:
            last_error = error
            temporary.unlink(missing_ok=True)
            time.sleep(0.5 * (attempt + 1))
    raise RuntimeError(f"failed {url}: {last_error}")


def write_manifest(output: Path) -> None:
    manifest = {
        "version": 1,
        "name": "China offline satellite overview",
        "bounds": BOUNDS,
        "center": [105.0, 35.0],
        "tileSize": 256,
        "minZoom": 0,
        "maxZoom": MAX_ZOOM,
        "initialZoom": 3.2,
        "source": "NASA Blue Marble Shaded Relief and Bathymetry",
        "sourceResolution": "500 m/pixel source; offline tiles through zoom 8",
        "attribution": "NASA Earth Observatory / NASA GIBS",
        "notForNavigation": True,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=12)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    tiles = enumerate_tiles()
    completed = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
        futures = [executor.submit(download_tile, args.output, tile) for tile in tiles]
        for future in concurrent.futures.as_completed(futures):
            future.result()
            completed += 1
            if completed % 100 == 0 or completed == len(tiles):
                print(f"{completed}/{len(tiles)} tiles", flush=True)
    write_manifest(args.output)


if __name__ == "__main__":
    main()
