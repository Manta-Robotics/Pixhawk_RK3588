# MANTA Field Console V5

## Why this is a new direction

V5 removes the previous cinematic landing page, vertical sidebar, ambient camera
motion, scan-line texture, and dashboard-card composition. The new interface is
organized around two product states:

1. **Device Dock** — product view, nearby-device identity, transport topology,
   secure pairing, and one clear connection action.
2. **Mission Surface** — a continuous four-region control surface that preserves
   the confirmed IMU / joystick / gimbal / log layout and its swap control.

The visual language uses a quiet grid, strong negative space, precise edge
labels, restrained ice blue, and status-only green / orange / red. Motion is
limited to feedback, state changes, and route progress; there is no ambient
cinematic animation.

## Research translated into interface rules

- Kimi's official workflow treats a polished website as a requirement-analysis,
  planning, asset, implementation, and iterative preview process rather than a
  one-shot style prompt.
- Moonshot's own refactor write-up recommends project-specific rules first,
  dependency tracing, property-level comparison, and explicit validation of
  overlapping layers and state transitions.
- High-view showcase patterns used here are structural, not decorative: Swiss
  grid clarity, restrained chrome, generous whitespace, one dominant content
  surface, and interactive controls tied to a real product variable.

References:

- https://www.kimi.com/help/websites/websites-overview
- https://www.kimi.com/resources/shipping-a-refactor-of-moonshot-ai-with-kimi-code-cli
- https://www.kimi.com/showcases/websites/editorial-commission-desk
- https://www.kimi.com/showcases/websites/editorial-reader
- https://www.kimi.com/zh-cn/showcases/websites/memories-of-sky
- `docs/kimi-ui-research-matrix-2026-07-20.md`

## Product constraints retained from discovery

| Area | V5 contract |
| --- | --- |
| Product | Surface photography robot for open-water swimmers and professionals |
| Tone | Professional, minimal, outdoor; DJI-clean with restrained soft glass |
| Orientation | Landscape only on iPhone and iPad |
| Primary navigation | Device, Map, Media; settings, logs, help remain in a drawer |
| Home | Product promotional image plus nearby-device connection dock |
| Pairing | Six-digit first-pair code; model and serial number are visible |
| Transport | BLE discovery/control, 5 GHz Wi-Fi video/media, internet-preserving topology |
| Control layout | Left top IMU, left bottom 360-degree joystick, right top gimbal video, right bottom logs; layout can swap |
| Persistent telemetry | Connection, latency, speed, FCU temperature, recording duration |
| Safety | Slide-to-stop immediately stops all movement; gimbal fault stops gimbal only and produces a diagnostic report |
| IMU | Guided FCU-driven six-position calibration with motion lock |
| Media | Large featured recording; first release downloads only; navigation and cancellation locked until completion |
| Map | GPS connection and map provider are always visible at bottom left |
| Language | Simplified Chinese default with complete English runtime-state coverage |
| Simulation | Preview can simulate link, video loss, gimbal fault, temperature, emergency stop, offline state, latency, and speed; battery is excluded |

## Visual tokens

- Background: `#070A0C`
- Surface: `#0E1316`
- Text: `#F4F7F7`
- Ice blue: `#8ADFFF`
- Healthy: `#63DCA7`
- Warning: `#EFAA5A`
- Critical: `#EF5E62`
- Primary radii: 1–3 px; circles are reserved for instruments and status points
- Feedback motion: 100–280 ms; no ambient zoom or decorative particles

## Preview verification target

The browser prototype is a visual and interaction proof. BLE, Bonjour, Wi-Fi,
video decoding, app-sandbox downloads, Photos export, landscape enforcement, and
hardware failsafe behavior still require the native SwiftUI build and physical
iPhone/iPad plus MANTA validation.
