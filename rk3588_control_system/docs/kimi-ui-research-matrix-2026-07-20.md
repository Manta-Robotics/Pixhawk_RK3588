# MANTA UI research matrix — Kimi showcase study

Date: 2026-07-20  
Scope: 62 unique showcase entries collected from the Chinese and international Kimi showcase galleries. The 50 highest-viewed cases are recorded below; 28 are technology, system, data, or tool interfaces. View counts are a point-in-time snapshot, not a permanent ranking.

## What was reviewed

This study does not treat “high-end UI” as one visual style. It separates six useful families:

1. **Editorial / cinematic:** a strong opening frame, controlled typography, asymmetric composition, and slow non-critical reveals.
2. **System workbench:** persistent navigation, compact rows, split panes, 1 px hierarchy, and direct manipulation.
3. **Data narrative:** a clear question, executive summary, progressive evidence, charts, legends, and conclusions.
4. **Spatial / temporal:** maps, timelines, scroll sequences, season or time controls, and a visible sense of place.
5. **Direct-play interaction:** drag, flip, draw, reveal, shuffle, or collection mechanics with immediate feedback.
6. **Quiet utility:** restrained monochrome surfaces, generous reading space, and minimal chrome around the task.

The MANTA app uses different families for different jobs: editorial for discovery, workbench for live control, data/spatial logic for map and telemetry, quiet utility for settings, and guided narrative for six-face IMU calibration.

## 2026-07-20 visual correction after product review

The first transfer leaned too heavily on the workbench family and made MANTA
feel like industrial ground equipment. A second review focused on Kimi cases
whose quality comes from atmosphere, composition, restrained color, and motion:

- **Memories of Sky:** color is an environmental state, changed continuously by
  one meaningful time control rather than added as decorative neon.
- **Letter to Grandma museum:** a low-chroma base, one accent color, authentic
  typography, and generous whitespace create emotion without UI ornament.
- **High-end architecture studio:** one dominant image and asymmetric negative
  space carry the composition; secondary information does not compete.
- **Minimalist Reader and MUJI analysis:** tools stay quiet around the content;
  low-chroma surfaces and whitespace can remain functional without feeling cold.
- **Kouji interactive scroll:** motion is synchronized with content progression,
  not applied uniformly to every container.

Revised MANTA rule: the live camera, product image, map, and recordings are the
content. UI chrome should float above those surfaces with soft depth and a deep
ocean palette. Borders, grids, indices, and monospaced labels are reduced to the
few places where they communicate operational state. Micro-motion is attached
to connect, playback progress, selected mode, map location, recording, and
warnings. It is not used as a generic "technology" effect.

## Top 50 audit matrix

| # | Case | Views | Family | Tech | Transferable lesson for MANTA |
|---:|---|---:|---|:---:|---|
| 1 | High-end architecture studio | 119,686 | Editorial |  | One dominant frame, copper/black restraint, bilingual type hierarchy, staggered project reveal. |
| 2 | Opulence Video Hero Pages | 103,912 | Cinematic |  | Full-bleed motion works for product discovery, but must stop before operational controls begin. |
| 3 | Godfather trilogy fan page | 97,394 | Editorial |  | A narrow palette and material texture create identity more effectively than generic glow. |
| 4 | Web Linux desktop | 92,017 | Workbench | ✓ | Persistent system chrome, task switching, real applications, and visible state form a coherent operating model. |
| 5 | Kouji interactive scroll | 67,160 | Spatial narrative |  | Audio, image, and text can progress as one authored sequence; useful for onboarding, not live control. |
| 6 | VS Code Web replica | 59,800 | Workbench | ✓ | Dense panes stay readable through alignment, tonal separation, compact labels, and a persistent status bar. |
| 7 | MACD NASDAQ backtest | 53,448 | Data narrative | ✓ | Lead with the answer, then expose metrics, charts, drawdown, and evidence progressively. |
| 8 | NVIDIA stock analysis | 51,029 | Data narrative | ✓ | Interactive charting needs stable legends, comparable scales, and annotations tied to events. |
| 9 | Shan Hai Jing gacha | 43,247 | Direct play |  | Animated reveal is rewarding when it confirms a user action; collection state remains visible. |
| 10 | British Museum website | 34,206 | Editorial timeline |  | A featured object plus a chronological spine gives long content a memorable structure. |
| 11 | Addams Family Polaroids | 23,637 | Direct play |  | Drag, shuffle, and flip create tactile feedback; physics should remain bounded and purposeful. |
| 12 | Letter to Grandma museum | 22,467 | Editorial archive |  | Warm material cues and authentic type can create emotion without excessive interface decoration. |
| 13 | Shein supply-chain analysis | 21,735 | Data narrative | ✓ | Separate mechanism, risk, timeline, and competitive comparison rather than flattening everything into cards. |
| 14 | Zotero Web clone | 16,766 | Workbench | ✓ | Three panes, draggable splitters, compact rows, and 1 px dividers are premium when information density is the product. |
| 15 | Minimalist Reader | 13,814 | Quiet utility |  | Content gets the largest surface; tools appear only when the reading task needs them. |
| 16 | Automated restaurant system | 13,675 | System control | ✓ | A central controller must show command, acknowledgement, fault, manual takeover, degradation, and recovery as separate states. |
| 17 | I Have a Dream audiobook | 12,969 | Sequential media |  | Synchronized narration and scenes demonstrate a clear time-based progress model. |
| 18 | PyTorch multimodal loading | 12,494 | Technical report | ✓ | Trace the pipeline end-to-end, separate observed bottlenecks from hypotheses, and keep rollback criteria visible. |
| 19 | Paris and Switzerland itinerary | 11,515 | Timeline utility |  | Day-by-day structure makes complex plans scannable; confirmations and unresolved items need different emphasis. |
| 20 | Thailand fuel-price analysis | 10,968 | Data comparison | ✓ | Faceted comparison needs explicit dimensions, source labels, and a stable reading order. |
| 21 | Fountain-pen industrial history | 10,526 | Comparative editorial |  | A common comparison framework makes many regions and eras legible without identical cards. |
| 22 | A Brush with Time | 10,502 | Editorial research |  | Blend image-led storytelling with evidence sections; do not let visual drama obscure conclusions. |
| 23 | Amazon evolution timeline | 9,868 | Timeline / system | ✓ | A dated spine distinguishes proven history from forecast and connects many product eras. |
| 24 | 30 LA small-business sites | 9,399 | Multi-brand system |  | Reusable structure can still preserve distinct identity when content and type treatment vary. |
| 25 | Memories of Sky | 9,098 | Spatial / temporal |  | A time slider can change the whole environment smoothly while keeping controls stable. |
| 26 | AI conversation-turn analysis | 9,044 | Data narrative | ✓ | Avoid a false universal average; show context, distributions, and resolution metrics together. |
| 27 | AI and graduate education | 8,779 | Research synthesis | ✓ | Policy, practice, limitations, and ethics need linked but visibly distinct sections. |
| 28 | Security Agent Architecture | 8,645 | System architecture | ✓ | Tool limits, approvals, rollback, audit records, and human ownership must remain visible in a safety-sensitive UI. |
| 29 | Prefill–Decode model serving | 8,229 | System performance | ✓ | Correlate resource use, latency, transfer, and operational cost rather than reporting isolated metrics. |
| 30 | FastMCP implementation path | 7,946 | Developer map | ✓ | A relationship map is more useful than a feature list when tools, resources, prompts, clients, and APIs interact. |
| 31 | Stochastic magnetic devices | 7,800 | Technical comparison | ✓ | Evidence strength and research gaps deserve equal visual weight beside headline performance. |
| 32 | European–Chinese soup comparison | 7,536 | Comparative editorial |  | Shared dimensions and regional exceptions create a more natural comparison than uniform tiles. |
| 33 | AGI learning roadmap | 7,296 | Roadmap | ✓ | Multi-year progression benefits from milestones, prerequisites, and a visible current position. |
| 34 | 30 Shanghai shop landing pages | 7,274 | Directory / map |  | Gallery overview, individual detail, and external map action form a clear three-level journey. |
| 35 | Ugly-cute consumer economy | 6,982 | Data editorial |  | Visual identity can be expressive while pricing, scarcity, and risk stay analytically structured. |
| 36 | Multi-currency pricing | 6,958 | Utility | ✓ | A single global switch can update all comparable values; the selected context must stay obvious. |
| 37 | Editorial Commission Desk | 6,955 | Workbench | ✓ | Swiss-grid discipline, drafts, briefs, and status tracking show how a quiet dashboard can feel authored. |
| 38 | AI Agent Embodiment | 6,739 | System architecture | ✓ | Perception, grounding, action, oversight, privacy, and evaluation should read as one loop. |
| 39 | MUJI business analysis | 6,718 | Quiet data | ✓ | Whitespace and low-chroma charts can carry dense business information without feeling sterile. |
| 40 | Zilong Ding history | 5,014 | Evidence narrative |  | Provenance, ownership, debate, and uncertainty require visibly different evidence states. |
| 41 | 300-agent World Cup prediction | 4,998 | Data heatmap | ✓ | Aggregate many agents into probability and confidence views; avoid showing raw complexity by default. |
| 42 | European applied learning | 3,533 | Comparative research |  | A consistent country framework makes structural differences easier to compare. |
| 43 | DAPO, LoRA, synchronous RL | 3,467 | Technical explainer | ✓ | Move from mechanism to experiment to scaling result, with mathematical depth available on demand. |
| 44 | Modified gravity report | 3,104 | Scientific comparison | ✓ | Keep theory, claim, constraint, and evidence gap separate so visual polish does not imply certainty. |
| 45 | ArcGIS Pro 3.5 guide | 3,055 | Spatial workbench | ✓ | Organize changes around real workflows—data, modeling, mapping, and projects—not release-note chronology alone. |
| 46 | Lamborghini Huracán report | 2,975 | Product evolution | ✓ | A technical product story works best when architecture, variants, and changes share a common timeline. |
| 47 | Physical meaning in equations | 2,921 | Guided learning | ✓ | Stage complexity, pair symbols with phenomena, and confirm understanding after each step. |
| 48 | K2 agent model field guide | 2,823 | Technical field guide | ✓ | Architecture, training, benchmarks, tool use, licensing, and access need distinct layers and provenance. |
| 49 | Atelier Veil perfume page | 2,816 | Cinematic |  | Ambient material shifts can express product mood; keep them outside frequent operational actions. |
| 50 | Lipstick Queen brand site | 2,709 | Editorial commerce |  | Bold type and an asymmetric grid can carry a small catalogue without generic repeated cards. |

## Deep technology study (12 cases)

### 1. Web Linux desktop

- Visual inspection: the default login is almost monochrome and centered; after sign-in the desktop keeps global navigation, task state, clock, and app entry points persistent.
- MANTA adoption: treat the connected screen as a dedicated operating environment, not a marketing dashboard. Device state and safety controls stay anchored while tools change.

### 2. VS Code Web replica

- Visual inspection: nearly black canvas, narrow activity rail, explorer pane, large task surface, and a high-contrast bottom status rail. There are many controls, but each region has one job.
- MANTA adoption: use fixed chrome and continuous panes. Reserve ice blue for selection/current state, not decorative outlines around every object.

### 3. Zotero Web clone

- Source specification: pixel-accurate three-pane layout, compact rows, 1 px dividers, draggable splitters, toolbar, status, filtering, sorting, and search; explicitly avoids flashy animation.
- MANTA adoption: logs, telemetry, and media metadata should feel precise through alignment and density, with resizable or switchable spatial priority.

### 4. MACD strategy analysis

- Visual inspection: a restrained report cover leads into an executive summary, then metric blocks and evidence. The reading order is obvious even before charts are reached.
- MANTA adoption: map and diagnostic reports should begin with current answer/status, then reveal data and details instead of presenting equal-priority tiles.

### 5. NVIDIA analysis

- Source specification: financial events, valuation comparison, strategy backtest, and Plotly output are tied to the same time axis.
- MANTA adoption: telemetry charts must keep event markers, status changes, and time synchronized.

### 6. Automated restaurant system

- Source specification: one central control layer connects order flow, equipment, robots, safety monitoring, fault isolation, degraded operation, manual takeover, safe stop, and recovery verification.
- MANTA adoption: command-sent, acknowledged, applied, faulted, stopped, and recovered are different UI states. The log panel must preserve that chain.

### 7. PyTorch multimodal pipeline

- Source specification: baseline first; then worker memory, decoding, transfer, GPU idle time, tail latency, restart behavior, controlled A/B tests, and rollback criteria.
- MANTA adoption: the gimbal diagnostic should compare measurements with a baseline, label hypotheses, and require recovery validation before clearing a fault.

### 8. Security Agent Architecture

- Source specification: seven defensive agents retain visible tool limits, approvals, rollback, logs, and failure states; high-impact actions require human approval.
- MANTA adoption: emergency stop, motor commands, calibration, and hardware protection must use distinct authority and confirmation patterns.

### 9. Prefill–Decode model serving

- Source specification: resource allocation, latency, cache transfer, and operational complexity are evaluated together.
- MANTA adoption: do not show “signal” alone. Connection quality is a combined view of BLE command latency, Wi-Fi video/media path, packet continuity, and device readiness.

### 10. FastMCP implementation path

- Source specification: tools, resources, prompts, server, client, FastAPI, remote proxy, OpenAPI, and tests are shown as implementation relationships.
- MANTA adoption: communication settings should explain discovery, BLE control, local Wi-Fi media, and internet coexistence as a topology, not a list of toggles.

### 11. ArcGIS Pro workflow guide

- Source specification: updates are grouped by data, modeling, mapping, and project workflows.
- MANTA adoption: map controls should be grouped around locate, source/status, route, and mission context rather than a miscellaneous tool shelf.

### 12. AI Agent Embodiment

- Source specification: perception connects to action through grounding, causal reasoning, oversight, privacy, and evaluation.
- MANTA adoption: video target acquisition, joystick intent, device acknowledgement, and safety supervision form a visible control loop.

## Design rules derived for MANTA

### Composition

- **Discovery:** full-bleed product photograph, restrained editorial copy, one primary connect action, device availability close to the action.
- **Connected control:** continuous split-pane workbench. Video receives the largest operational surface; IMU and joystick remain immediately reachable; logs keep a compact live strip.
- **Map:** full-bleed spatial canvas with a persistent lower-left GPS/provider legend and a small task-oriented command cluster.
- **Media:** one large recent recording plus a compact chronological rail; downloading is a clear modal task and blocks conflicting actions.
- **Calibration:** six faces form a spatial sequence with current face, stability, samples, pass criteria, and recovery/abort.

### Micro-motion

- 80–160 ms: press, latch, command acknowledgement, OSD toggle, record state.
- 220–320 ms: pane switch, status tray, map/source reveal.
- 280–420 ms: connect, calibration step, download sheet, fault escalation.
- 4–16 s: only ambient discovery-page movement; never on joystick, emergency stop, live telemetry, or fault UI.
- Honor `prefers-reduced-motion`; no safety-critical feedback may depend on motion alone.

### Explicit exclusions

- No universal glass-card grid.
- No decorative neon frame around every panel.
- No autonomous particle field in the control console.
- No hover-only information on touch devices.
- No cinematic scroll or long entrance animation after connection.
- No color-only safety state: every warning includes text/icon/status semantics.

## Source set

- Kimi Chinese showcase gallery: https://www.kimi.com/zh-cn/showcases/
- Kimi international showcase gallery: https://www.kimi.com/showcases/
- Kimi Websites feature: https://www.kimi.com/zh-cn/features/websites
- Kimi K2.5 visual coding notes: https://www.kimi.com/blog/kimi-k2-5
- Representative technology details: Web Linux, VS Code Web, Zotero, MACD, NVIDIA, automated restaurant, PyTorch multimodal loading, security agents, prefill–decode serving, FastMCP, ArcGIS Pro, and AI agent embodiment.
