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

#include <stdint.h>
#include "AttGuidMsgPayload.h"
#include "AttRefMsgPayload.h"
#include "NavAttMsgPayload.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ AttTrackingErrorAlgorithm instance.
 */
typedef struct AttTrackingErrorAlgorithm AttTrackingErrorAlgorithm;

/**
 * @brief POD representation of a 3-vector (Eigen::Vector3f).
 */
typedef struct {
    float data[3];
} Vector3f_c;

/**
 * @brief Construct a new AttTrackingErrorAlgorithm instance.
 * @return Pointer to a new AttTrackingErrorAlgorithm (must be destroyed).
 */
AttTrackingErrorAlgorithm* AttTrackingErrorAlgorithm_create(void);

/**
 * @brief Destroy a previously created AttTrackingErrorAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithm* self);

/**
 * @brief Reset the algorithm state.
 * @param self     Pointer to the instance.
 * @param callTime Time stamp for reset.
 */
void AttTrackingErrorAlgorithm_reset(AttTrackingErrorAlgorithm* self,
                                     uint64_t callTime);

/**
 * @brief Run the update step.
 * @param self         Pointer to the instance.
 * @param callTime     Time stamp for update.
 * @param attRefInMsg  Pointer to reference-frame message payload.
 * @param attNavInMsg  Pointer to navigation attitude message payload.
 * @return AttGuidMsgPayload  The computed guidance message.
 */
AttGuidMsgPayload
AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithm* self,
                                 uint64_t callTime,
                                 AttRefMsgPayload* attRefInMsg,
                                 NavAttMsgPayload* attNavInMsg);

/**
 * @brief Set the σ_R0R three-vector.
 * @param self      Pointer to the instance.
 * @param sigma_R0R 3-vector in flattened POD format.
 */
void AttTrackingErrorAlgorithm_setSigma_R0R(AttTrackingErrorAlgorithm* self,
                                            Vector3f_c sigma_R0R);

/**
 * @brief Get the current σ_R0R three-vector.
 * @param self Pointer to the instance.
 * @return Vector3f_c  Flattened POD containing the vector.
 */
Vector3f_c AttTrackingErrorAlgorithm_getSigma_R0R(AttTrackingErrorAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ATTTRACKINGERRORALGORITHM_C_H
