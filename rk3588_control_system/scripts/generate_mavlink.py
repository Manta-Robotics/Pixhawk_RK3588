#!/usr/bin/env python3
"""Generate the official C protocol implementation; Python is build-time only."""
from pathlib import Path
import argparse
import pymavlink
from pymavlink.generator import mavgen

parser = argparse.ArgumentParser()
parser.add_argument("output", type=Path)
args = parser.parse_args()
xml = Path(pymavlink.__file__).parent / "dialects" / "v20" / "ardupilotmega.xml"
if not xml.is_file():
    raise SystemExit(f"MAVLink dialect XML missing: {xml}")
if not mavgen.mavgen(mavgen.Opts(str(args.output), wire_protocol="2.0", language="C"), [str(xml)]):
    raise SystemExit("MAVLink generation failed")
