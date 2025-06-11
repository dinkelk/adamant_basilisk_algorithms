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

#ifndef _ATT_TRACKING_ERROR_ALGORITHM_H
#define _ATT_TRACKING_ERROR_ALGORITHM_H
#include "Eigen/src/Freestanding/freestanding_fpclassify.h"

#include <Eigen/Core>

#include "AttGuidMsgPayload.h"
#include "AttRefMsgPayload.h"
#include "NavAttMsgPayload.h"

class AttTrackingErrorAlgorithm {
   public:
    void reset(uint64_t callTime);  //!< Method for algorithm reset
    AttGuidMsgPayload update(uint64_t callTime,
                             AttRefMsgPayload& attRefInMsg,
                             NavAttMsgPayload& attNavInMsg);  //!< Algorithm update method
    void setSigma_R0R(const Eigen::Vector3f& sigma_R0R);      //!< Setter for sigma_R0R variable
    const Eigen::Vector3f& getSigma_R0R() const;              //!< Getter for sigma_R0R variable

   private:
    Eigen::Vector3f sigma_R0R{};  //!< MRP from corrected reference frame to original reference frame R0.
};

#endif
