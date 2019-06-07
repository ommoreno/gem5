/*
 * Copyright (c) 2011, 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 */

#include "cpu/pred/gshare.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "debug/GSHARE.hh"

GShareBP::GShareBP(const GShareBPParams *params)
    : BPredUnit(params),
      PHTCtrBits(params->PHTCtrBits),
      PHTPredictorSize(params->PHTPredictorSize),
      globalHistoryBits(params->globalPredictorSize),
      globalHistory(params->numThreads, 0)
{
//    if (!isPowerOf2(globalHistoryBits)) {
//        fatal("Invalid global history size!\n");
//    }

//    if (!isPowerOf2(PHTPredictorSize)) {
//        fatal("Invalid gShare predictor table size!\n");
//    }

    DPRINTF(GSHARE, "In constructor");

    PHTCtrs.resize(PHTPredictorSize);
    for (int i = 0; i < PHTPredictorSize; ++i)        
            PHTCtrs[i].setBits(PHTCtrBits);

    historyRegisterMask = mask(globalHistoryBits);
    gShareMask = PHTPredictorSize - 1;

    // Check that predictors don't use more bits than they have available
    if (gShareMask > historyRegisterMask) {
        fatal("Gshare predictor too large for global history bits!\n");
    }

    // Check if there is more history bits than needed by predictor
    if (historyRegisterMask > gShareMask) {
        inform("More global history bits than required by predictors\n");
    }

    // Set thresholds for predictor's counters
    // This is equivalent to (2^(Ctr))/2 - 1
    gShareThreshold = (ULL(1) << (PHTCtrBits  - 1)) - 1;
}

inline
unsigned
GShareBP::calcGShareIdx(Addr &branch_addr, ThreadID tid)
{
    // Get low order bits after removing instruction offset. XOR addr with branch history 
    return ((branch_addr >> instShiftAmt) & gShareMask) ^ (globalHistory[tid] & historyRegisterMask);
}

inline
void
GShareBP::updateGlobalHistTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1) | 1;
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
GShareBP::updateGlobalHistNotTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1);
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

void
GShareBP::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    unsigned gshare_idx = calcGShareIdx(branch_addr, tid);
    //Update Global History to Not Taken (clear LSB)
    globalHistory[tid] &= (historyRegisterMask & ~ULL(1));
    //Update Gshare table to Not Taken
    PHTCtrs[gshare_idx] =
       PHTCtrs[gshare_idx].read() & (gShareMask & ~ULL(1));
}

bool
GShareBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    unsigned gshare_idx;
    bool gshare_prediction;

    DPRINTF(GSHARE, "In lookup");

    gshare_idx = calcGShareIdx(branch_addr, tid);
    gshare_prediction = PHTCtrs[gshare_idx].read() > gShareThreshold;
    
    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->gShareIdx = gshare_idx;
    history->gSharePredTaken = gshare_prediction;
    bp_history = (void *)history;

    if (gshare_prediction) {
        updateGlobalHistTaken(tid);
        return true;
    } else {
        updateGlobalHistNotTaken(tid);
        return false;
    }
}

void
GShareBP::uncondBranch(ThreadID tid, Addr pc, void * &bp_history)
{
    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->gShareIdx = invalidPredictorIndex;
    bp_history = static_cast<void *>(history);

    updateGlobalHistTaken(tid);
}

void
GShareBP::update(ThreadID tid, Addr branch_addr, bool taken,
                     void *bp_history, bool squashed)
{
    assert(bp_history);
    BPHistory *history = static_cast<BPHistory *>(bp_history);

    if (squashed) {
        globalHistory[tid] = (history->globalHistory << 1) | taken;
        globalHistory[tid] &= historyRegisterMask;
        return;
    }

    unsigned gShare_predictor_idx = history->gShareIdx;
    if (taken) {
        PHTCtrs[gShare_predictor_idx].increment();
    } else {
        PHTCtrs[gShare_predictor_idx].decrement();
    }

    // We're done with this history, now delete it.
    delete history;
}

void
GShareBP::squash(ThreadID tid, void *bp_history)
{
    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // Restore global history to state prior to this branch.
    globalHistory[tid] = history->globalHistory;

    // Delete this BPHistory now that we're done with it.
    delete history;
}

GShareBP*
GShareBPParams::create()
{
    return new GShareBP(this);
}

unsigned        
GShareBP::getGHR(ThreadID tid, void *bp_history) const      
{       
    return static_cast<BPHistory *>(bp_history)->globalHistory;     
}

#ifdef DEBUG
int
GShareBP::BPHistory::newCount = 0;
#endif
