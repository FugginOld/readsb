# Domain glossary

Started during the struct-aircraft deepening review (2026-07-03). Add terms here as they're sharpened; don't backfill the whole codebase speculatively.

## aircraft record
One tracked aircraft's state, `struct aircraft` (track.h). Owned by track.c: all writes to core fields go through track.c. Reads stay direct (`const struct aircraft *`) — no invariant risk, no need to route through an accessor.

## trace
Per-aircraft position history. Two parts, split by a real constraint (see `struct aircraft`'s scratch/rollback below), not just concept:

- **recent buffer** (`trace_current`, `trace_current_len`, `trace_current_max`, `trace_len`, `trace_write`, `trace_writeCounter`) — uncompressed, most recent points. Stays inline in the aircraft record: `trackUpdateFromMessage` mutates it (via `setPosition`/`traceAdd`) inside a byte-range rollback that undoes rejected messages, so it can't move behind a pointer without reimplementing that rollback.
- **history** (`trace_chunks`, `trace_chunk_len`, `trace_chunk_overall_bytes`, disk-write timestamps, `traceCache`, `traceLock`, `traceLast`) — compressed chunks and persistence bookkeeping. Owned by globe_index.c via `struct traceHistory`, held as an opaque pointer (`a->traceHistory`) — no other module sees its members. None of it is touched inside the rollback window, so it's free to encapsulate.

Queries against history are exposed as named functions (`traceHasRecentDuplicate`), not raw field access.

## aircraft record persistence
`struct aircraft` is also the on-disk state format (`save_blob`/`load_blob` in globe_index.c, raw `memcpy` of the struct plus separately-serialized trace data). Layout changes are an already-supported, self-healing event: `load_aircraft` copies `imin(oldSize, newSize)` bytes, logs when the size changed, and forces an immediate re-save in the new format. Pointer-typed fields (`trace_current`, `trace_chunks`, `traceHistory`) are never trusted from the raw copy — they're reconstructed from the trailing serialized data after load.

## active list
`onActiveList` — whether an aircraft is currently included in aircraft.json output. Lives in the aircraft record but is a track.c concern, not part of `trace`; track.c writes it directly as the owning module.

## net protocol adapter

A wire format net_io.c speaks: Beast, SBS, Asterix, UAT, Planefinder, GPSD/HULC. Each is one `read_fn` (net_io.h) wired into a `struct net_service` at `serviceInit` time — the seam already existed pre-split, this just gives each adapter its own file (`net_beast.c`, `net_sbs.c`, etc.) instead of sharing net_io.c's namespace. net_io.c stays the owner module: client/service lifecycle, the epoll loop, heartbeat/ping-pong, and `modesQueueOutput`'s output fan-out (hardcoded, not a dispatch table — every enabled writer gets called every message, there's no runtime "pick one" the way input has, so a table would be indirection with nothing to select). Beast is a container format and calls directly into the UAT and GPSD/HULC adapters for embedded frames — real wire-protocol nesting, not a layering violation. `struct client` itself is not part of this seam; adapters still read/write its fields directly (deepening that is a separate, later candidate).
