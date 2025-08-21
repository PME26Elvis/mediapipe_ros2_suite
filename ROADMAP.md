# Roadmap

This roadmap prioritizes **event-level APIs**, **reproducibility**, and **developer experience** over raw model breadth.

## Near term (v0.1.x)
- **CI coverage**: keep Humble green; improve Jazzy from experimental → stable.
- **DX polish**: clearer “model not found” errors; `MP_MODELS_DIR`; multi-camera launch example.
- **Lightweight metrics**: per-pipeline FPS log (1 line/sec) for sanity checks.

## Short-mid term (v0.2.x)
- **Sensors**: example launch for RealSense (color only first).
- **Examples**: bag-replay example; minimal rqt graph & rviz snapshots.
- **Docs**: Jazzy notes; troubleshooting (camera topics, encodings, common errors).

## Mid term (v0.3.x)
- **Pipelines**: optional Holistic (hand+pose+face) when stable in Tasks API.
- **Testing**: smoke tests on recorded images/bags; keep contracts stable.
- **Packaging** (optional): container recipe purely for reproducible demos.

## Principles
- Stable message contracts; additive evolution.
- Parameterized nodes > ad-hoc scripts.
- No redistribution of third-party model binaries; document sources instead.
