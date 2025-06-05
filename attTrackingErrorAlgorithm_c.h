#ifndef ATT_TRACKING_ERROR_C_API_H
#define ATT_TRACKING_ERROR_C_API_H

/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University
 of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER
 IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include <stdint.h>   // for uint64_t
#include <stddef.h>   // for size_t

#include "AttGuidMsgPayload.h"
#include "AttRefMsgPayload.h"
#include "NavAttMsgPayload.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the C++ class
typedef struct AttTrackingErrorAlgorithm AttTrackingErrorAlgorithm;

/*
 * Allocate a new AttTrackingErrorAlgorithm on the heap.
 * Returns a pointer which Ada can store as System.Address.
 */
AttTrackingErrorAlgorithm* attea_new();

/*
 * Delete an instance previously created with attea_new().
 */
void attea_delete(AttTrackingErrorAlgorithm* ptr);

/*
 * Invoke AttTrackingErrorAlgorithm::reset(uint64_t).
 */
void attea_reset(AttTrackingErrorAlgorithm* ptr,
                 uint64_t callTime);

/*
 * Invoke AttTrackingErrorAlgorithm::update(uint64_t, AttRefMsgPayload&, NavAttMsgPayload&).
 * Returns the resulting AttGuidMsgPayload by value.
 */
AttGuidMsgPayload attea_update(AttTrackingErrorAlgorithm* ptr,
                               uint64_t callTime,
                               AttRefMsgPayload* attRefIn,
                               NavAttMsgPayload* attNavIn);

/*
 * Invoke AttTrackingErrorAlgorithm::setSigma_R0R(const Eigen::Vector3f&).
 * Ada should pass a float array of length 3.
 */
void attea_set_sigma_R0R(AttTrackingErrorAlgorithm* ptr,
                         float sigma[3]);

/*
 * Invoke AttTrackingErrorAlgorithm::getSigma_R0R() const.
 * Writes three floats into out_sigma[0..2].
 */
void attea_get_sigma_R0R(AttTrackingErrorAlgorithm* ptr,
                         float out_sigma[3]);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ATT_TRACKING_ERROR_C_API_H
