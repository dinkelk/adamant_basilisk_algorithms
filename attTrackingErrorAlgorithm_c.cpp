/* ISC License
 *
 * Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
 * University of Colorado at Boulder
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "AttTrackingErrorAlgorithm_c.h"
#include "AttTrackingErrorAlgorithm.h"  // the original C++ class
#include <Eigen/Core>

AttTrackingErrorAlgorithm*
AttTrackingErrorAlgorithm_create(void)
{
    return reinterpret_cast<AttTrackingErrorAlgorithm*>(
        new ::AttTrackingErrorAlgorithm());
}

void
AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithm* self)
{
    delete reinterpret_cast<::AttTrackingErrorAlgorithm*>(self);
}

void
AttTrackingErrorAlgorithm_reset(AttTrackingErrorAlgorithm* self,
                                uint64_t callTime)
{
    reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)
        ->reset(callTime);
}

AttGuidMsgPayload
AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithm* self,
                                 uint64_t callTime,
                                 AttRefMsgPayload* attRefInMsg,
                                 NavAttMsgPayload* attNavInMsg)
{
    return reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)
        ->update(callTime, *attRefInMsg, *attNavInMsg);
}

void
AttTrackingErrorAlgorithm_setSigma_R0R(AttTrackingErrorAlgorithm* self,
                                       Vector3f_c sigma_R0R)
{
    Eigen::Vector3f vec;
    vec << sigma_R0R.data[0],
           sigma_R0R.data[1],
           sigma_R0R.data[2];
    reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)
        ->setSigma_R0R(vec);
}

Vector3f_c
AttTrackingErrorAlgorithm_getSigma_R0R(AttTrackingErrorAlgorithm* self)
{
    Eigen::Vector3f vec =
        reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)
            ->getSigma_R0R();
    Vector3f_c out;
    out.data[0] = vec[0];
    out.data[1] = vec[1];
    out.data[2] = vec[2];
    return out;
}

