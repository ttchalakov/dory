#include "dory_dma.h"

#include "pmsis.h"

#ifndef MCHAN_BASE_ADDR
// FIXME: For GAP9, this must point to ARCHI_MCHAN_EXT_ADDR!!!
// In PULP-SDK for Kraken, this is fixed.
// GAP8 hardware to be tested...
#define MCHAN_BASE_ADDR (ARCHI_MCHAN_DEMUX_ADDR)  // CLUSTER_MCHAN_ADDR
#endif
#define MCHAN_EVENT
//#define MCHAN_POLLED
#ifdef MCHAN_EVENT
#define MCHAN_EVENT_BIT (ARCHI_CL_EVT_DMA0)  // 8
#endif
#include "mchan.h"


#if   defined(MCHAN_POLLED)
#define MCHAN_FLAGS    (MCHAN_CMD_FLAG_INCREMENTAL)
#elif defined(MCHAN_EVENT)
#define MCHAN_FLAGS    (MCHAN_CMD_FLAG_EVENT_ENABLE | MCHAN_CMD_FLAG_INCREMENTAL)
#elif defined(MCHAN_INTERRUPT)
#define MCHAN_FLAGS    (MCHAN_CMD_FLAG_INTERRUPT_ENABLE | MCHAN_CMD_FLAG_INCREMENTAL)
#endif

#define MCHAN_FLAGS_1D (MCHAN_FLAGS)
#define MCHAN_FLAGS_2D (MCHAN_FLAGS | MCHAN_CMD_FLAG_2D_TRANSFER_EXTERNAL)

#define MIN(a,b) ((a)<(b)?(a):(b))

/* Largest byte count a single MCHAN command can express on this target.
 *
 * MEASURED on GAP8 (AI-deck, 8 cores): a 34816-byte contiguous L2->L1 transfer
 * lands only its first 2048 bytes -- 34816 & 0x7FFF -- and MCHAN then reports
 * the transfer *complete*, so neither dory_dma_barrier() nor a status poll can
 * detect it. The rest of the destination keeps whatever the previous layer left
 * there, and the kernel computes on it. The command's length field is therefore
 * 15 bits wide, not the 16 that archi/dma/mchan_v6.h advertises
 * (MCHAN_CMD_CMD_LEN_WIDTH) -- note that the 2D line-length field in that same
 * header is already documented as 15 (PLP_DMA_2D_LEN_WIDTH).
 *
 * GVSOC's MCHAN model extracts the length with PLP_DMA_SIZE_GET, i.e. all 16
 * bits, so oversized transfers complete perfectly in simulation. Any network
 * whose tiler emits a >=32 KB transfer is bit-exact in GVSOC and silently wrong
 * on the board. A fully connected layer reaches this at 128 output channels x
 * 256 inputs; DORY's tiler sizes tiles against the L1 budget and has no notion
 * of a per-transfer limit, so it emits them freely.
 *
 * Kept a multiple of 4 so every chunk after the first stays word aligned. */
#ifndef MCHAN_MAX_TRANSFER_SIZE
#define MCHAN_MAX_TRANSFER_SIZE (32764)
#endif

/* Instrumentation hooks. Both are weak and both compile out unless
 * DORY_DMA_PROBE is defined: they sit in the innermost loop of every tiled layer
 * and reading the MCHAN status is a peripheral access.
 *
 *   dory_dma_probe_issue()  immediately before a transfer is pushed
 *   dory_dma_probe()        from dory_dma_barrier(), once the wait has returned
 *
 * The completion hook alone yields a timestamp with no duration behind it, so
 * MCHAN arbitration and L2 port contention cannot be told apart -- both present
 * as "the barrier took a while". Pairing the two gives per-transfer duration and
 * therefore effective bandwidth, and the status word read at issue says whether
 * the queue was already occupied. Below the MCHAN peak with an empty queue is
 * port contention; at the peak with a non-empty queue is arbitration.
 *
 * Both take only already-computed values, so an override costs a handful of
 * cycles and does not perturb the race being measured. An override may store
 * what it sees; it must not print. */
#ifdef DORY_DMA_PROBE
__attribute__((weak)) void dory_dma_probe(DMA_copy *copy, unsigned int mchan_status) {
  (void) copy;
  (void) mchan_status;
}
__attribute__((weak)) void dory_dma_probe_issue(DMA_copy *copy, unsigned int mchan_status) {
  (void) copy;
  (void) mchan_status;
}
#define DORY_DMA_PROBE_CALL(copy) dory_dma_probe((copy), MCHAN_READ_STATUS())
#define DORY_DMA_PROBE_ISSUE_CALL(copy) dory_dma_probe_issue((copy), MCHAN_READ_STATUS())
#else
#define DORY_DMA_PROBE_CALL(copy) ((void) 0)
#define DORY_DMA_PROBE_ISSUE_CALL(copy) ((void) 0)
#endif

static void dory_dma_push_lines(int dir, unsigned char *loc, unsigned char *ext,
                                unsigned int size_1d, unsigned int n_lines,
                                unsigned int stride);

void dory_dma_memcpy_hwc_to_chw(DMA_copy *copy){
#ifdef SINGLE_CORE_DMA
  if (pi_core_id() == 0) {
#endif
  int start_pixel, stop_pixel; // "pixel" is a misnomer; the CHANNELS are divided between the cores
  // this function assumes that a DW tile is always as wide as the complete feature map (this is enforced by DORY's tiler)
  // if there is only 1 DMA control unit for the cluster (e.g., Kraken), we can't execute DMA calls on multiple clusters.
#ifndef SINGLE_CORE_DMA
  int core_id = pi_core_id();
  int Log2Core = log2(NUM_CORES);
  int number_of_copies_per_core = (copy->length_1d_copy >> Log2Core) + ((copy->length_1d_copy & (NUM_CORES-1))!=0);
  start_pixel = MIN(number_of_copies_per_core * core_id, copy->length_1d_copy);
  stop_pixel = MIN(start_pixel + number_of_copies_per_core, copy->length_1d_copy);
#else
  start_pixel = 0;
  stop_pixel = copy->length_1d_copy;
#endif
  void * loc = copy->loc + copy->number_of_1d_copies*copy->number_of_2d_copies*start_pixel;
  void * ext = copy->ext + start_pixel;
  const int size_2d = copy->number_of_1d_copies * copy->number_of_2d_copies;

  for (int i=start_pixel; i<stop_pixel; i++) {
    DORY_DMA_PROBE_ISSUE_CALL(copy);
    // one byte at a time, so size_2d "lines" of 1 byte. Chunked because one
    // command cannot express more than MCHAN_MAX_TRANSFER_SIZE bytes.
    dory_dma_push_lines(copy->dir, (unsigned char *) loc, (unsigned char *) ext,
                        1, (unsigned int) size_2d, (unsigned int) copy->stride_1d);
#ifdef ALWAYS_BLOCK_DMA_TRANSFERS // needed on GAP8 board
    dory_dma_barrier(copy);
#endif
    ext += 1; // next channel
    loc += copy->number_of_1d_copies * copy->number_of_2d_copies;
  }
#ifdef SINGLE_CORE_DMA
  }
#endif
}

void dory_dma_memcpy_1d_async(DMA_copy *copy) {
  if (pi_core_id() == 0) {
    DORY_DMA_PROBE_ISSUE_CALL(copy);
    // Split anything past the command's length field into several commands. They
    // all land on the same counter -- an allocated counter stays active until
    // another is allocated -- so the existing barrier still waits for all of
    // them. See MCHAN_MAX_TRANSFER_SIZE.
    unsigned int remaining = (unsigned int) copy->length_1d_copy * copy->number_of_1d_copies * copy->number_of_2d_copies;
    unsigned char *loc = (unsigned char *) copy->loc;
    unsigned char *ext = (unsigned char *) copy->ext;
    while (remaining > 0) {
      unsigned int chunk = MIN(remaining, MCHAN_MAX_TRANSFER_SIZE);
      mchan_transfer_t trans = {
        .cmd = chunk | (copy->dir << MCHAN_CMD_SHIFT_DIRECTION) | MCHAN_FLAGS_1D,
        .size = chunk,
        .ext = ext,
        .loc = loc
      };
      mchan_transfer_push_1d(trans);
      remaining -= chunk;
      loc += chunk;
      ext += chunk;
    }
  }
}

/* Push a strided transfer of n_lines x size_1d bytes, splitting it into as many
 * commands as the length field needs (see MCHAN_MAX_TRANSFER_SIZE). Splits on
 * whole lines, which keeps the 2D descriptor valid; if a single line is itself
 * too long, that line is contiguous on both sides and so can be moved with plain
 * 1D commands. Must be called from one core only. */
static void dory_dma_push_lines(int dir, unsigned char *loc, unsigned char *ext,
                                unsigned int size_1d, unsigned int n_lines,
                                unsigned int stride) {
  if (size_1d == 0 || n_lines == 0) return;

  if (size_1d > MCHAN_MAX_TRANSFER_SIZE) {
    for (unsigned int i = 0; i < n_lines; i++) {
      unsigned int remaining = size_1d;
      unsigned char *l = loc + (unsigned int) i * size_1d;
      unsigned char *e = ext + (unsigned int) i * stride;
      while (remaining > 0) {
        unsigned int chunk = MIN(remaining, MCHAN_MAX_TRANSFER_SIZE);
        mchan_transfer_t t = {
          .cmd = chunk | (dir << MCHAN_CMD_SHIFT_DIRECTION) | MCHAN_FLAGS_1D,
          .size = chunk, .ext = e, .loc = l
        };
        mchan_transfer_push_1d(t);
        remaining -= chunk; l += chunk; e += chunk;
      }
    }
    return;
  }

  const unsigned int lines_per_cmd = MCHAN_MAX_TRANSFER_SIZE / size_1d;
  unsigned int done = 0;
  while (done < n_lines) {
    unsigned int lines = MIN(n_lines - done, lines_per_cmd);
    mchan_transfer_t t = {
      .cmd = (lines * size_1d) | (dir << MCHAN_CMD_SHIFT_DIRECTION) | MCHAN_FLAGS_2D,
      .size = lines * size_1d,
      .ext = ext + (unsigned int) done * stride,
      .loc = loc + (unsigned int) done * size_1d,
      .ext_size_1d = size_1d,
      .ext_stride_1d = stride
    };
    mchan_transfer_push_2d(t);
    done += lines;
  }
}

void dory_dma_memcpy_2d_async(DMA_copy *copy) {
  if (pi_core_id() == 0) {
    DORY_DMA_PROBE_ISSUE_CALL(copy);
    const int size_2d = copy->number_of_1d_copies * copy->length_1d_copy * copy->number_of_2d_copies;
    const int stride = (copy->number_of_2d_copies == 1) ? copy->stride_1d : copy->stride_2d;
    const int size_1d = (copy->number_of_2d_copies == 1) ? copy->length_1d_copy : copy->length_1d_copy * copy->number_of_1d_copies;

    dory_dma_push_lines(copy->dir, (unsigned char *) copy->loc,
                        (unsigned char *) copy->ext, (unsigned int) size_1d,
                        size_1d ? (unsigned int) (size_2d / size_1d) : 0,
                        (unsigned int) stride);
  }
}

void dory_dma_memcpy_3d_async(DMA_copy *copy) {
#ifdef SINGLE_CORE_DMA
  if (pi_core_id() == 0) {
#endif
  int start_pixel, stop_pixel;
#ifndef SINGLE_CORE_DMA
  int core_id = pi_core_id();
  int Log2Core = log2(NUM_CORES);
  int number_of_2d_copies_per_core = (copy->number_of_2d_copies >> Log2Core) + ((copy->number_of_2d_copies & (NUM_CORES-1))!=0);
  start_pixel = MIN(number_of_2d_copies_per_core * core_id, copy->number_of_2d_copies);
  stop_pixel = MIN(start_pixel + number_of_2d_copies_per_core, copy->number_of_2d_copies);
#else
  start_pixel = 0;
  stop_pixel = copy->number_of_2d_copies;
#endif
  void *ext = copy->ext + copy->stride_2d*start_pixel;
  void *loc = copy->loc + copy->length_1d_copy*copy->number_of_1d_copies*start_pixel;
  const int size_2d = copy->number_of_1d_copies * copy->length_1d_copy;
  for (int i = start_pixel; i < stop_pixel; i++) {
    DORY_DMA_PROBE_ISSUE_CALL(copy);
    // Chunked for the same reason as the 1D/2D paths: one command cannot express
    // more than MCHAN_MAX_TRANSFER_SIZE bytes, and exceeding it truncates
    // silently rather than failing.
    dory_dma_push_lines(copy->dir, (unsigned char *) loc, (unsigned char *) ext,
                        (unsigned int) copy->length_1d_copy,
                        (unsigned int) copy->number_of_1d_copies,
                        (unsigned int) copy->stride_1d);
#ifdef ALWAYS_BLOCK_DMA_TRANSFERS // needed on GAP8 board
    dory_dma_barrier(copy);
#endif
    loc += size_2d;
    ext += copy->stride_2d;
  }
#ifdef SINGLE_CORE_DMA
  }
#endif
}

void dory_dma_memcpy_async(DMA_copy *copy) {
  if (copy->hwc_to_chw == 1) {
    dory_dma_memcpy_hwc_to_chw(copy);
  }
  else if ((copy->number_of_2d_copies == 1 && copy->number_of_1d_copies == 1) || (copy->stride_1d == copy->length_1d_copy &&  copy->number_of_1d_copies * copy->length_1d_copy == copy->stride_2d) || (copy->number_of_2d_copies == 1 && copy->length_1d_copy == copy->stride_1d)) {
    dory_dma_memcpy_1d_async(copy);
  } else if ((copy->number_of_2d_copies == 1) || (copy->length_1d_copy == copy->stride_1d)) {// wrong!
    dory_dma_memcpy_2d_async(copy);
  } else {
    dory_dma_memcpy_3d_async(copy);
  }
}

void dory_dma_free(DMA_copy *copy) {
  mchan_transfer_free(copy->tid);
}

void dory_dma_barrier(DMA_copy *copy) {
#ifdef SINGLE_CORE_DMA
  // if DMA is only used by a single core (only 1 ctrl interface), other cores must not access its register file. Instead, they should all wait for core 0 to confirm the transfer is over.
  if (pi_core_id() == 0)
    mchan_transfer_wait(copy->tid);
  DORY_DMA_PROBE_CALL(copy);
  pi_cl_team_barrier(0);
#else
  mchan_transfer_wait(copy->tid);
  DORY_DMA_PROBE_CALL(copy);
#endif
}

int dory_dma_allocate() {
  return mchan_transfer_get_id();
}
