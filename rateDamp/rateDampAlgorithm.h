/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _RATE_DAMP_ALGORITHM_H
#define _RATE_DAMP_ALGORITHM_H

#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include <stdint.h>

/*! @brief Rate damp algorithm class */
class RateDampAlgorithm {
   public:
    CmdTorqueBodyMsgPayload update(uint64_t currentSimNanos,
                                   NavAttMsgPayload& attNavInMsg);  //!< Algorithm update method
    void setRateGain(float p);                                     //!< Setter method for rate feedback gain
    float getRateGain() const;                                     //!< Getter method for rate feedback gain

   private:
    float P{};  //!< [N*m*s] Rate feedback gain
};

#endif
