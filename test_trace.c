// Smoke test for the struct-aircraft trace deepening (track.c / globe_index.c).
// Not a framework, just assert() -- run via `make test`.

#include "readsb.h"
#include <assert.h>

struct _Modes Modes;
struct _Threads Threads;
const char *argp_program_version = "test_trace";
const char *argp_program_credits = "";
const char *argp_program_bug_address = "";

// stubs for symbols normally provided by readsb.c, which this test doesn't link against
void setExit(int arg) { (void) arg; }
int priorityTasksPending(void) { return 0; }
void priorityTasksRun(void) { }
void receiverPositionChanged(float lat, float lon, float alt) { (void) lat; (void) lon; (void) alt; }

int main(void) {
    struct aircraft a;
    memset(&a, 0, sizeof(a));
    a.addr = 0xabcdef;
    a.traceHistory = traceHistoryCreate();

    int n = 3;
    a.trace_current = cmalloc(stateBytes(n));
    memset(a.trace_current, 0, stateBytes(n));
    a.trace_current_max = n;
    a.trace_current_len = n;
    a.trace_len = n;

    int32_t lats[3] = { 100, 200, 300 };
    int32_t lons[3] = { 10, 20, 30 };
    for (int i = 0; i < n; i++) {
        struct state *s = getState(a.trace_current, i);
        s->lat = lats[i];
        s->lon = lons[i];
    }

    assert(traceHasRecentDuplicate(&a, 200, 20) == 1);
    assert(traceHasRecentDuplicate(&a, 999, 999) == 0);

    int64_t before = a.seen;
    trackTouchSeen(&a, 123456789);
    assert(a.seen == 123456789);
    assert(a.seen != before);

    assert(traceChunkBytes(&a) == 0);

    sfree(a.trace_current);
    traceHistoryDestroy(a.traceHistory);

    fprintf(stderr, "test_trace: OK\n");
    return 0;
}
