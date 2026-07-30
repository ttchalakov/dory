#ifndef __PERF_UTILS_H__
#define __PERF_UTILS_H__
#include <stddef.h>
#include <stdint.h>

typedef struct {
  unsigned int L3_input;
  unsigned int L3_output;
  unsigned int L3_after_weights;
  unsigned int L2_input;
  unsigned int bypass;
  unsigned int L2_output;
  unsigned int L2_weights;
  unsigned int L1_buffer;
  unsigned int ram;
  unsigned int out_mult;
  unsigned int out_shift;
  unsigned int layer_id;
} layer_args_t;

void print_perf(const char *name, const int cycles, const int macs);
void checksum(const char *name, const uint8_t *d, size_t size, uint32_t sum_true);

/* Per-layer inspection hook.
 *
 * Called once after every layer completes, while that layer's output is still
 * live in L2 and before the allocator can hand the space to the next layer. The
 * default is an empty weak symbol, so it costs a call and nothing else; an
 * application that wants to see intermediate activations -- to checksum them, to
 * diff them against a reference, to stream them off-chip -- just defines
 * dory_layer_done() itself and the strong definition wins at link time.
 *
 * This exists because there was previously no way to observe a layer boundary
 * from outside the generated code: VERBOSE prints to stdout, which is unusable
 * on a target whose console is a radio link, and it cannot be redirected.
 */
void dory_layer_done(int layer_id, const char *name, void *l2_output, size_t size);

/* Weight-staging inspection hook -- the 3-level target only.
 *
 * Called once per layer whose weights are read from L3 into L2 as a whole blob
 * (allocate_layer[i] == 1), immediately after the cl_ram_read() that stages them
 * and before the layer runs. `size` is the number of bytes that read asked for.
 *
 * This is the L3 counterpart of dory_dma_probe(). On the 2-level target weights
 * are resident C arrays and cannot arrive wrong; on this target every layer's
 * weights cross the HyperBus first, and a short read there is indistinguishable
 * downstream from bad arithmetic -- the kernel simply computes on whatever else
 * was in that L2 buffer. The generated network.h already carries
 * weights_checksum[i] for each blob, so an application that defines this hook can
 * confirm the bytes that arrived are the bytes DORY emitted, which is the one
 * check that separates "the staging path is broken" from "the layer is broken".
 *
 * Weak and empty by default. Note it does NOT fire for a layer that streams its
 * own weight tiles from L3 (allocate_layer[i] == 0): there is no single staged
 * blob to check in that case, and those tile reads go through the layer's own
 * dory_dma path, which dory_dma_probe() already covers.
 */
void dory_weights_staged(int layer_id, const char *name, void *l2_weights, size_t size);

/* Incremented by execute_layer_fork() whenever pmsis_l1_malloc() for a layer's
 * L1 working buffer fails. DORY sizes that buffer assuming it is the sole user of
 * cluster L1, so any application that also puts something there -- a JPEG encoder
 * sharing the cluster, say -- can exhaust it. The layer is then skipped rather
 * than run against address 0, which means the network silently produces a stale
 * output: exposing the count is what lets an application notice. */
extern volatile unsigned int dory_l1_alloc_failed;
#endif
