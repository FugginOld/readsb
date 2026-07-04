# Architecture backlog

Follow-up candidates from an architecture review + deepening pass (2026-07-03).
Read `CONTEXT.md` first for the vocabulary and constraints already established.

## Already done this session

1. **struct aircraft** — chunk/persistence trace fields moved behind an opaque
   `struct traceHistory` (globe_index.c). Recent buffer stayed inline (byte-range
   rollback in `trackUpdateFromMessage` depends on it). See CONTEXT.md.
2. **net_io.c split** — 6466 lines → net_io.c (owner: client/service lifecycle,
   epoll loop, ping/pong, flush) + 6 protocol adapters (net_beast.c, net_sbs.c,
   net_asterix.c, net_uat.c, net_planefinder.c, net_gpsd.c). See CONTEXT.md.
3. **Modes god-struct, SDR slice** — ~20 SDR fields moved from `struct _Modes`
   into `struct sdrConfig SdrConfig` (sdr.h/sdr.c). 4 fields deleted as dead
   (`input_format`, `converter_function`, `enable_agc`, `ppm_error` — each
   backend already had its own local copy of these, the Modes ones were orphaned).
4. **trackUpdateFromMessage decompose, partial** — the ~281-line field-merge
   block (category/opstatus/altitude/squawk/heading/nav/etc, track.c lines
   ~2489-2768 before extraction) pulled into `static void mergeValidatedFields(a,
   mm, now, message_version)`. Signature of trackUpdateFromMessage unchanged.
   Function: 965 → 673 lines.

## Backlog, ranked by risk (lowest first)

### 1. Modes god-struct, api_ cluster (recommended next)

Same pattern as the SDR extraction — smaller, even cleaner.

- **Fields (~15-19):** `api`, `apiUpdate`, `apiBufferInitDone`, `apiThreadCount`,
  `apiWorkerCpuMicro`, `apiRequestCounter`, `apiService`, `apiListeners`,
  `apiBuffer[2]`, `apiFlip`, `apiThread`, `apiFlipMutex`, `apiShutdownDelay`,
  `api_fds_per_thread`, `max_fds_api`. `net_output_api_ports` straddles this
  and the net_ cluster — decide ownership when you get there.
- **Files touching `Modes.api*`:** aircraft.c, api.c, json_out.c, readsb.c,
  stats.c (5 files, matches SDR's 5).
- **Dead field found:** `Modes.apiFlipMutex` — declared, zero reads/writes
  anywhere outside its own declaration. Delete alongside the extraction,
  same as SDR's 4 dead fields.
- **CLI parsing:** only 5 `case Opt*` sites in readsb.c assign into these
  fields (`apiShutdownDelay` x2, `api`, `apiThreadCount`, `apiUpdate`).
- **No name collisions:** `struct apiCon/apiOptions/apiCircle/apiEntry/
  apiBuffer/apiThread` in api.h are already distinctly named from a
  prospective `ApiConfig` global — unlike SDR's local-struct gain-field
  near-collisions.
- **Estimated diff:** ~5 files, ~20-25 rename sites — roughly half the SDR
  extraction's footprint. Follow the exact same recipe: new `struct
  apiConfig` + `ApiConfig` global in api.h/api.c, mechanical
  `Modes.apiX` → `ApiConfig.X` rename, delete `apiFlipMutex`.

### 2. api.c HTTP/query split

- **api.c** is 2314 lines, api.h declares 6 public functions.
- **Query engine** (~1200 lines, ~26 functions): `findInBox`, `findInCircle`,
  `findHexList`/`findRegList`/`findCallsignList`, hash lookups, `apiReq`
  (line 479-832, 354 lines — the request→JSON/binCraft encoder), `apiAdd`,
  `apiGenerateJson`, `apiUpdate`, `apiBufferInit/Cleanup`,
  `apiGenerateAircraftJson/GlobeJson`.
- **HTTP transport** (~1100 lines, ~21 functions): `parseFetch`,
  `parseHalfUUID`, `parseDoubles`, `sendStatus`/`send200`/`send400`/
  `send405`/`send505`/`send503`/`send500`, `apiReadRequest`, `apiSendData`,
  `apiCloseCon`, `apiResetCon`, `acceptCon`, `apiThreadEntryPoint`,
  `apiUpdateEntryPoint`, `apiInit`, `apiCleanup`, `shutClose`, `apiShutdown`.
- **Entanglement — one seam:** transport calls query exactly once
  (`parseFetch` → `apiReq`, api.c:1518). Query never calls transport.
  Much closer to the SDR split's shape than net_io.c's tangle.
- **Shared types:** already centralized in api.h (`struct apiCon`,
  `apiOptions`, `apiEntry`, `apiBuffer`, `apiThread`, `apiCircle`, `struct
  range`, macros `API_REQ_PADSTART`/`API_REQ_LIST_MAX`/`API_ZSTD_LVL`) —
  nothing to relocate. Two small macros (`memWrite`, `EPOLLEXCLUSIVE`) and
  one forward decl are transport-local, trivial to move.
- **Global state already partitioned:** `Modes.apiBuffer[]`/`apiFlip[]` are
  query-only; `Modes.apiThread`/`apiThreadCount`/`apiListeners`/
  `apiService`/`api_fds_per_thread` are transport-only. No field is heavily
  touched by both halves.
- **Verdict:** SDR-shaped, not net_io.c-shaped. Expect ~1.5-2x the SDR
  effort (bigger file, one function — `apiReq` — to export across the new
  boundary), no call-graph script needed.
- **Note:** do this *after* the api_ Modes cluster above if both are in
  scope — they touch the same file and fields, less rebasing if sequenced.

### 3. trackUpdateFromMessage, CPR/position block (partial)

- Current location: track.c, `trackUpdateFromMessage` starts at line 2287
  (line numbers will have shifted further if items 1-2 above land first —
  re-grep before touching).
- CPR/position section: lines ~2491-2852 (~362 lines), inside the same
  function, adjacent to the byte-range scratch/rollback
  (`memcpy(&scratch, a, offsetof(struct aircraft, traceCache))` /
  the matching rollback memcpy near the end).
- **No `goto`/early-return inside this range** — confirmed safe to extract
  from a control-flow perspective. All the function's `goto exit;` sites are
  earlier, before the scratch snapshot.
- **Two genuinely separable sub-blocks**, recommended as the actual scope
  for this pass (not the whole 362 lines):
  - SBS/MLAT position handling, lines ~2673-2746 (~74 lines) — touches
    only `a`, `mm`, `now` (all pointer/value-passable).
  - Rough-receiver-location DF11 block, lines ~2748-2782 (~35 lines) —
    same, no scratch/`cpr_new` interaction.
- **Leave in place:** the CPR-accept/dup-check (~2491-2497), the
  airground/`updatePosition` block that sets `cpr_new` (~2624-2671), and
  the final rollback (~2846-2852) — these have real ordering dependencies
  (`cpr_new`, `haveScratch`, `scratch` all get read/written across this
  span, and the rollback must run strictly last, after every sub-block that
  can set `mm->garbage`/`mm->pos_bad`/`mm->duplicate`).
- **Verdict:** tractable for the two sub-blocks named above; the rest is a
  second, harder pass — don't attempt it in the same sitting as the easy
  part.

### 4. struct client encapsulation (do last, scoped subset only)

- `struct client` (net_io.h:94-157): **56 fields**, touched by 8 files —
  net_io.c (owner), the 6 protocol adapters, plus track.c and json_out.c
  (read-only, via `mm->client->…` and `service->clients` walks for
  stats/display). api.c never touches it.
- **Read-many-write-few pattern, same as struct aircraft** — most fields
  are legitimately read broadly; full opacity isn't warranted for those.
- **~13 fields are net_io.c-only**: `buflen`, `bufmax`, `bufferToProcess`,
  `discard`, `processing`, `acceptSocket`, `net_connector_dummyClient`,
  `dropHalfDrop`, `dropHalfUntil`, `last_read_flush`, `dropHalfAntiSpam`,
  `epollEvent`, `proxy_string`, `modeac_requested`. Plus `orphaned_bytes`/
  `orphaned_bytes_ts` are net_beast.c-only.
- **No persistence/serialization surprise** — confirmed no memcpy/fwrite of
  `struct client` anywhere (the one `sizeof(struct client)` use is a normal
  array allocation, not serialization).
- **Real prerequisite before any opacity work**: `struct net_connector`
  embeds `struct client dummyClient` *by value*, and `readsb.c:1502-1509`
  sizes `struct net_connector` with `sizeof()` for a realloc'd array. This
  means struct client's full layout must stay visible wherever
  net_connector is sized, even though `dummyClient.*` access itself is
  confined to net_io.c. **Fix this first**: change `dummyClient` to a
  heap-allocated pointer, verify readsb.c's realloc math still holds, before
  attempting to opaque-ify anything.
- **Verdict:** scoped subset only (the ~13 net_io.c-only fields, after the
  `dummyClient` prerequisite), not full encapsulation — same reasoning as
  struct aircraft's recent buffer staying inline. Highest total effort of
  the four backlog items; do it last.

## Deferred indefinitely (found during research, not in original 4)

- **Modes net_ cluster** (~28-30 config fields, 7 files: api.c, mode_s.c,
  net_beast.c, net_io.c, readsb.c, stats.c, track.c) — 3-4x the SDR/api_
  cluster's blast radius. 70 of readsb.c's 138 `case Opt` CLI branches
  assign into `Modes.net_*` fields — nearly half the entire CLI parser.
  Also mixes true config with epoll runtime state (`net_epfd`,
  `net_events`, `net_maxEvents`, `net_event_count`) that probably shouldn't
  move into a "config" struct at all. Real candidate eventually, but size
  it as its own multi-session effort, not a quick follow-on.
- **Modes stats_ cluster** — 11 files including every hot decode path
  (demod_2400.c, net_planefinder.c, net_sbs.c, mode_s.c, json_out.c,
  track.c). `struct stats` is already used as a local pointer-variable type
  name (`struct stats *st`) throughout stats.c/track.c, which would make a
  mechanical `Modes.stats_X` rename actively confusing to execute safely.
  Worst risk/reward of anything found. Leave alone unless something forces
  the issue.

## How to pick this back up

Each item above has enough detail (field lists, file lists, line ranges,
call counts) to skip re-deriving it from scratch — but line numbers will
have drifted if earlier items in this list already landed. Re-verify with
grep before editing, the same way this session caught `decodeSbsLine`'s
missed direct reference and the `ieee754_binary32_le_to_float` return-type
gap during the net_io.c split. Recommended order is the ranking above:
api_ cluster → api.c split → CPR/position sub-blocks → struct client subset.
