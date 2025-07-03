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

#ifndef ATTTRACKINGERRORALGORITHM_C_H
#define ATTTRACKINGERRORALGORITHM_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"

/** Opaque handle for an AttTrackingErrorAlgorithm instance. */
typedef struct AttTrackingErrorAlgorithm AttTrackingErrorAlgorithm;

/** @brief Flattened 3D vector for C interoperability */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief Construct a new AttTrackingErrorAlgorithm.
 * @return Pointer to the new AttTrackingErrorAlgorithm.
 */
AttTrackingErrorAlgorithm* AttTrackingErrorAlgorithm_create(void);

/**
 * @brief Destroy an AttTrackingErrorAlgorithm.
 * @param self The instance to destroy.
 */
void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithm* self);

/**
 * @brief Reset the algorithm state.
 * @param self     The algorithm instance.
 * @param callTime The clock time at which the function was called (nanoseconds).
 */
void AttTrackingErrorAlgorithm_reset(AttTrackingErrorAlgorithm* self, uint64_t callTime);

/**
 * @brief Run the update step of the attitude tracking error algorithm.
 * @param self         The algorithm instance.
 * @param callTime     The clock time at which the function was called (nanoseconds).
 * @param attRefInMsg  Pointer to the attitude reference message payload.
 * @param attNavInMsg  Pointer to the navigation attitude message payload.
 * @return Computed attitude guidance message payload.
 */
AttGuidMsgPayload AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithm* self,
                                                   uint64_t callTime,
                                                   AttRefMsgPayload* attRefInMsg,
                                                   NavAttMsgPayload* attNavInMsg);

/**
 * @brief Set the sigma_R0R parameter (MRP from corrected reference frame to original reference frame R0).
 * @param self      The algorithm instance.
 * @param sigma_R0R The 3-vector in flattened POD format.
 */
void AttTrackingErrorAlgorithm_setSigma_R0R(AttTrackingErrorAlgorithm* self, Vector3f_c sigma_R0R);

/**
 * @brief Get the current sigma_R0R parameter.
 * @param self The algorithm instance.
 * @return The current sigma_R0R vector in flattened POD format.
 */
Vector3f_c AttTrackingErrorAlgorithm_getSigma_R0R(const AttTrackingErrorAlgorithm* self);

#ifdef __cplusplus
}
#endif

#endif  // ATTTRACKINGERRORALGORITHM_C_H
