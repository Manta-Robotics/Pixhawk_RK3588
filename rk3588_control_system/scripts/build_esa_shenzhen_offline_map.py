#!/usr/bin/env python3
"""Build a board-hosted Shenzhen XYZ package from ESA Sentinel-2 10 m COGs.

Build dependency: rasterio (Pillow and NumPy are installed with it). The source
COGs are public and support HTTP range reads, so no Copernicus account or API key
is required.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np
import rasterio
from PIL import Image
from rasterio.enums import Resampling
from rasterio.transform import from_bounds
from rasterio.windows import Window, from_bounds as window_from_bounds
from rasterio.warp import reproject


BOUNDS = {"west": 113.72, "south": 22.38, "east": 114.65, "north": 22.90}
CENTER = [114.0579, 22.5431]
MIN_ZOOM = 9
MAX_ZOOM = 14
TILE_SIZE = 256
HALF_WORLD_M = math.pi * 6378137.0
SOURCE_URLS = (
    "https://esa-worldcover-s2.s3.eu-central-1.amazonaws.com/rgbnir/2021/N22/"
    "ESA_WorldCover_10m_2021_v200_N22E113_S2RGBNIR.tif",
    "https://esa-worldcover-s2.s3.eu-central-1.amazonaws.com/rgbnir/2021/N22/"
    "ESA_WorldCover_10m_2021_v200_N22E114_S2RGBNIR.tif",
)


def tile_x(longitude: float, zoom: int) -> int:
    return max(0, min((1 << zoom) - 1, math.floor((longitude + 180.0) / 360.0 * (1 << zoom))))


def tile_y(latitude: float, zoom: int) -> int:
    latitude = max(-85.05112878, min(85.05112878, latitude))
    value = (1.0 - math.asinh(math.tan(math.radians(latitude))) / math.pi) / 2.0 * (1 << zoom)
    return max(0, min((1 << zoom) - 1, math.floor(value)))


def tile_bounds_3857(zoom: int, x: int, y: int) -> tuple[float, float, float, float]:
    count = 1 << zoom
    west = x / count * 2 * HALF_WORLD_M - HALF_WORLD_M
    east = (x + 1) / count * 2 * HALF_WORLD_M - HALF_WORLD_M
    north = HALF_WORLD_M - y / count * 2 * HALF_WORLD_M
    south = HALF_WORLD_M - (y + 1) / count * 2 * HALF_WORLD_M
    return west, south, east, north


def enumerate_tiles() -> list[tuple[int, int, int]]:
    result: list[tuple[int, int, int]] = []
    for zoom in range(MIN_ZOOM, MAX_ZOOM + 1):
        x0, x1 = tile_x(BOUNDS["west"], zoom), tile_x(BOUNDS["east"], zoom)
        y0, y1 = tile_y(BOUNDS["north"], zoom), tile_y(BOUNDS["south"], zoom)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                result.append((zoom, x, y))
    return result


def crop_sources(cache: Path) -> list[Path]:
    cache.mkdir(parents=True, exist_ok=True)
    outputs: list[Path] = []
    for url in SOURCE_URLS:
        target = cache / Path(url).name.replace(".tif", "_shenzhen.tif")
        local_source = cache / Path(url).name
        source_location = str(local_source) if local_source.exists() else url
        outputs.append(target)
        if target.exists() and target.stat().st_size > 1_000_000:
            print(f"source cache ready: {target.name}", flush=True)
            continue
        last_error: Exception | None = None
        for attempt in range(5):
            try:
                with rasterio.Env(
                    GDAL_HTTP_MULTIRANGE="YES",
                    GDAL_HTTP_MERGE_CONSECUTIVE_RANGES="YES",
                    GDAL_HTTP_MAX_RETRY="5",
                    GDAL_HTTP_RETRY_DELAY="1",
                ):
                    with rasterio.open(source_location) as source:
                        west = max(BOUNDS["west"], source.bounds.left)
                        south = max(BOUNDS["south"], source.bounds.bottom)
                        east = min(BOUNDS["east"], source.bounds.right)
                        north = min(BOUNDS["north"], source.bounds.top)
                        if west >= east or south >= north:
                            continue
                        window = window_from_bounds(west, south, east, north, transform=source.transform)
                        window = Window(
                            math.floor(window.col_off), math.floor(window.row_off),
                            math.ceil(window.width), math.ceil(window.height),
                        )
                        # Source order is B02, B03, B04, B08. Store natural RGB.
                        data = source.read([3, 2, 1], window=window)
                        profile = source.profile.copy()
                        profile.update(
                            driver="GTiff", width=data.shape[2], height=data.shape[1], count=3,
                            transform=source.window_transform(window), dtype="uint16", nodata=0,
                            compress="DEFLATE", predictor=2, tiled=True, blockxsize=256, blockysize=256,
                        )
                        temporary = target.with_suffix(".part.tif")
                        with rasterio.open(temporary, "w", **profile) as output:
                            output.write(data)
                        os.replace(temporary, target)
                        print(f"source crop ready: {target.name} ({target.stat().st_size / 1048576:.1f} MiB)", flush=True)
                        break
            except Exception as error:
                last_error = error
                target.with_suffix(".part.tif").unlink(missing_ok=True)
                if attempt == 4:
                    raise
                print(f"source read retry {attempt + 1}/5 for {target.name}: {error}", flush=True)
                time.sleep(2 * (attempt + 1))
    return [path for path in outputs if path.exists()]


def color_limits(sources: list[Path]) -> tuple[np.ndarray, np.ndarray]:
    samples: list[np.ndarray] = []
    for path in sources:
        with rasterio.open(path) as source:
            sample = source.read(out_shape=(3, max(1, source.height // 24), max(1, source.width // 24)))
            samples.append(sample.reshape(3, -1))
    merged = np.concatenate(samples, axis=1)
    low = np.array([np.percentile(band[band > 0], 2) for band in merged], dtype=np.float32)
    high = np.array([np.percentile(band[band > 0], 98.5) for band in merged], dtype=np.float32)
    print(f"RGB stretch low={low.round(1).tolist()} high={high.round(1).tolist()}", flush=True)
    return low, high


def render_tile(
    output: Path,
    source_paths: list[Path],
    low: np.ndarray,
    high: np.ndarray,
    tile: tuple[int, int, int],
) -> str:
    zoom, x, y = tile
    target = output / str(zoom) / str(x) / f"{y}.jpg"
    if target.exists() and target.stat().st_size > 512:
        return "cached"
    bounds = tile_bounds_3857(zoom, x, y)
    transform = from_bounds(*bounds, TILE_SIZE, TILE_SIZE)
    raw = np.zeros((3, TILE_SIZE, TILE_SIZE), dtype=np.float32)
    for path in source_paths:
        with rasterio.open(path) as source:
            for band_index in range(3):
                reproject(
                    source=rasterio.band(source, band_index + 1),
                    destination=raw[band_index],
                    src_transform=source.transform,
                    src_crs=source.crs,
                    src_nodata=0,
                    dst_transform=transform,
                    dst_crs="EPSG:3857",
                    dst_nodata=0,
                    resampling=Resampling.bilinear,
                    init_dest_nodata=False,
                )
    scaled = np.empty_like(raw, dtype=np.uint8)
    for band_index in range(3):
        normalized = np.clip((raw[band_index] - low[band_index]) / (high[band_index] - low[band_index]), 0, 1)
        normalized = np.power(normalized, 0.88)
        scaled[band_index] = np.round(normalized * 255).astype(np.uint8)
    image = Image.fromarray(np.moveaxis(scaled, 0, 2), mode="RGB")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(".jpg.part")
    image.save(temporary, "JPEG", quality=86, optimize=True, progressive=True)
    os.replace(temporary, target)
    return "written"


def write_manifest(output: Path, tile_count: int) -> None:
    manifest = {
        "version": 2,
        "name": "Shenzhen Sentinel-2 offline satellite map",
        "bounds": BOUNDS,
        "center": CENTER,
        "tileSize": TILE_SIZE,
        "minZoom": MIN_ZOOM,
        "maxZoom": MAX_ZOOM,
        "initialZoom": 10.2,
        "source": "ESA WorldCover Sentinel-2 RGBNIR 2021 annual cloudless composite",
        "sourceResolution": "approximately 10 m/pixel source; offline XYZ through zoom 14",
        "shortResolution": "Sentinel-2 10 m",
        "attribution": "ESA WorldCover / Copernicus Sentinel-2",
        "license": "CC BY 4.0",
        "coverage": "Shenzhen municipality and nearby coastal waters",
        "tileCount": tile_count,
        "notForNavigation": True,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cache", type=Path, default=Path(tempfile.gettempdir()) / "manta-sentinel2-cache")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()
    if args.clean and args.output.exists():
        shutil.rmtree(args.output)
    args.output.mkdir(parents=True, exist_ok=True)
    sources = crop_sources(args.cache)
    if len(sources) != 2:
        raise RuntimeError(f"expected two Shenzhen source crops, found {len(sources)}")
    low, high = color_limits(sources)
    tiles = enumerate_tiles()
    completed = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
        futures = [executor.submit(render_tile, args.output, sources, low, high, tile) for tile in tiles]
        for future in concurrent.futures.as_completed(futures):
            future.result()
            completed += 1
            if completed % 100 == 0 or completed == len(tiles):
                print(f"{completed}/{len(tiles)} tiles", flush=True)
    write_manifest(args.output, len(tiles))


if __name__ == "__main__":
    main()
