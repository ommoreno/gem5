/**
 * Copyright (c) 2018 Inria
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
 * Authors: Daniel Carvalho, Orlando Moreno
 */

#include "mem/cache/replacement_policies/lru_ipv.hh"

#include <cassert>
#include <memory>

#include "params/LRUIPV.hh"
#include "debug/LRUDEBUG.hh"

LRUIPV::LRUIPV(const Params *p)
    : BaseReplacementPolicy(p)
{
}

void
LRUIPV::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
const
{
	DPRINTF(LRUDEBUG, "Inside Invalidate\n");

    std::static_pointer_cast<LRUIPVReplData>(replacement_data)->position = 16;

    DPRINTF(LRUDEBUG, "Exiting Invalidate\n");
}

void
LRUIPV::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
	DPRINTF(LRUDEBUG, "Inside touch\n");

    std::shared_ptr<LRUIPVReplData> replEntry = std::static_pointer_cast<LRUIPVReplData>(replacement_data);
    int newPosition = lruIPV[replEntry->position];

    for (auto& entry : entries) {
        if (std::static_pointer_cast<LRUIPVReplData>(entry)->position >= newPosition) {
            std::static_pointer_cast<LRUIPVReplData>(entry)->position += 1;
        }
    }

    replEntry->position = newPosition;

    DPRINTF(LRUDEBUG, "Exiting touch\n");
}

void
LRUIPV::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
	DPRINTF(LRUDEBUG, "Inside reset\n");

    insertionPoint = lruIPV.back();
    // Push entries down the vector from 13 to 15. This will invalidate the last element
    for (auto& entry : entries) {
        if (std::static_pointer_cast<LRUIPVReplData>(entry)->position >= insertionPoint) {
            std::static_pointer_cast<LRUIPVReplData>(entry)->position += 1;
        }
    }

    // Insert entry into defined position, i.e. 13
    std::shared_ptr<LRUIPVReplData> newEntry = std::static_pointer_cast<LRUIPVReplData>(replacement_data);
    newEntry->position = insertionPoint;

    entries.push_back(newEntry);

    DPRINTF(LRUDEBUG, "Exiting reset\n");
}

ReplaceableEntry*
LRUIPV::getVictim(const ReplacementCandidates& candidates) const
{
    DPRINTF(LRUDEBUG, "Inside getVictim\n");
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    ReplaceableEntry* victim = candidates[0];
    int victimIndex = -1;
    for (const auto& candidate : candidates) {
        if (std::static_pointer_cast<LRUIPVReplData>(candidate->replacement_data)->position > 15) {
            victim = candidate;
        }
        victimIndex++;
    }

    assert(victimIndex > -1);
    entries.erase(entries.begin() + victimIndex);

    DPRINTF(LRUDEBUG, "Exiting getVictim\n");
    return victim;
}

std::shared_ptr<ReplacementData>
LRUIPV::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new LRUIPVReplData());
}

LRUIPV*
LRUIPVParams::create()
{
    return new LRUIPV
(this);
}
