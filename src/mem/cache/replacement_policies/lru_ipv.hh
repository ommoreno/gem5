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

/**
 * @file
 * Declaration of a Least Recently Used (LRU) Insertion/Promotion Vector 
 * (IPV) replacement policy.
 * The victim is chosen using the IPV defined in Figure 3 of the paper in
 * http://taco.cse.tamu.edu/pdfs/p284-jimenez.pdf.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_LRU_IPV_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_LRU_IPV_HH__

#include "mem/cache/replacement_policies/base.hh"

struct LRUIPVParams;

class LRUIPV : public BaseReplacementPolicy
{
  private:
    /** 
     * Values in IPV indicate which position to go to after a hit.
     * i.e., The replacement data at node 15 gets moved to node 11 after a hit.
     * If the position of a replacement data is not 0-15, then it is invalid.
     * The last value indicates which node to use for data that is inserted.
     */
    std::vector<int> lruIPV{0, 0, 1, 0, 3, 0, 1, 2, 1, 0, 5, 1, 0, 0, 1, 11, 13};

  protected:
    /** LRUIPV-specific implementation of replacement data. */
    struct LRUIPVReplData : ReplacementData
    {
        /** Position in IPV */
        int position;

        /**
         * Default constructor. Invalidate data.
         */
        LRUIPVReplData() : position(16) {}
    };

  public:
    /** Convenience typedef. */
    typedef LRUIPVParams Params;

    /**
     * Construct and initiliaze this replacement policy.
     */
    LRUIPV(const Params *p);

    /**
     * Destructor.
     */
    ~LRUIPV() {}

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Set its position to 16 on the vector.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                              const override;

    /**
     * Touch an entry to update its replacement data.
     * Set its position according to the IPV.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Set its position to 13 according to the IPV.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using last position in IPV.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_LRU_IPV_HH__
