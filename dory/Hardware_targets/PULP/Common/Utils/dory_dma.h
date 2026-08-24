/*
 * dory.h
 * Alessio Burrello <alessio.burrello@unibo.it>
 *
 * Copyright (C) 2019-2020 University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _DORY_DMA_H
#define _DORY_DMA_H

typedef struct
{
  void *ext;
  void *loc;
  unsigned short hwc_to_chw;
  unsigned short stride_2d;
  unsigned short number_of_2d_copies;
  unsigned short stride_1d;
  unsigned short number_of_1d_copies;
  unsigned short length_1d_copy;
  int dir; // 0 l1->l2, 1 l2->l1
  int tid;
} DMA_copy;

void dory_dma_memcpy_hwc_to_chw(DMA_copy *copy);

void dory_dma_memcpy_1d_async(DMA_copy *copy);

void dory_dma_memcpy_2d_async(DMA_copy *copy);

void dory_dma_memcpy_3d_async(DMA_copy *copy);

void dory_dma_memcpy_async(DMA_copy *copy);

void dory_dma_free(DMA_copy *copy);

void dory_dma_barrier(DMA_copy *copy);

int dory_dma_allocate();

#ifdef DORY_DMA_PROBE
/* Weak hook, called by dory_dma_barrier() once the wait has returned, with the
 * MCHAN status word read at that instant. Bits 0..15 are the per-counter
 * "transfer still pending" flags, so a non-zero low half here means the barrier
 * let the core past while data was still moving -- something no amount of
 * inspecting the generated code can tell you, and the difference between a
 * tiling bug and a synchronization bug. Default is an empty function; an
 * application overrides it to record what it sees. Deliberately takes only
 * already-computed values so that overriding it costs a handful of cycles and
 * does not perturb the very race it is measuring. */
void dory_dma_probe(DMA_copy *copy, unsigned int mchan_status);

/* Issue-side counterpart, called immediately before each transfer is pushed,
 * from whichever core does the pushing. Its reason to exist is that the
 * completion hook above records *when* a transfer finished and never how long it
 * took, which leaves the two candidate causes of a slow barrier -- MCHAN
 * arbitration and L2 port contention -- indistinguishable. Subtracting this
 * stamp from the completion stamp gives a duration, and with copy's dimensions a
 * bandwidth; the status word here says whether the queue was already occupied
 * when this transfer was pushed, which is what separates the two.
 *
 * Note the pairing is not always one to one. The 1D and 2D paths push once per
 * call from core 0. The 3D and HWC paths push once per 2D slice, from every core
 * unless SINGLE_CORE_DMA, and under ALWAYS_BLOCK_DMA_TRANSFERS they call
 * dory_dma_barrier() inside that same loop -- so an override must treat a burst
 * of issues followed by one completion as normal and report the depth rather
 * than assume it is one. */
void dory_dma_probe_issue(DMA_copy *copy, unsigned int mchan_status);
#endif // DORY_DMA_PROBE
#endif
