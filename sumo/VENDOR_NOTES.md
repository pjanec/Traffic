# Vendored SUMO source — trim notes

- Upstream: https://github.com/eclipse-sumo/sumo
- Tag: `v1_20_0` (matches `SUMO_VERSION=1.20.0` at repo root)
- Commit: `96efa4d36d6c9af50710015bb725fc51730b582a`

The full upstream checkout at this tag is ~1.4G, almost entirely `tests/` (1.1G) and
binary wiki assets under `docs/web` (~46M of screenshots). Neither is needed to port
the simulation core, so this vendor tree keeps only:

- `src/` — full and byte-identical to upstream (this is what the porting tasks read).
- `docs/web/docs/**/*.md` — the wiki text (algorithm descriptions, model docs), images
  dropped.
- `docs/sumo.bib` — the bibliography (papers cited by the algorithm docs).
- `LICENSE`, `NOTICE.md`, `AUTHORS`, `README.md`, `CITATION.cff`.

Dropped entirely: `tests/`, `tools/`, `data/`, `build_config/`, `CMakeLists.txt`,
`bin/`, `unittest/`, non-markdown `docs/web` assets. None of these are read by the
porting tasks; if a later task needs one, re-vendor it from the same tag/commit above.
