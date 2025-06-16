/* ISC License
 *
 * Copyright (c) 2025, Laboratory for Atmospheric and Space Physics,
 * University of Colorado at Boulder
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies.
 *
 * THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef RATE_DAMP_ALGORITHM_C_H
#define RATE_DAMP_ALGORITHM_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"

/** Opaque handle for a RateDampAlgorithm instance. */
typedef struct RateDampAlgorithm RateDampAlgorithm;

/**
 * @brief Construct a new RateDampAlgorithm.
 * @return Pointer to the new RateDampAlgorithm.
 */
RateDampAlgorithm* RateDampAlgorithm_create(void);

/**
 * @brief Destroy a RateDampAlgorithm.
 * @param self The instance to destroy.
 */
void RateDampAlgorithm_destroy(RateDampAlgorithm* self);

/**
 * @brief Run the update step of the rate-damp algorithm.
 * @param self               The algorithm instance.
 * @param currentSimNanos    Current simulation time (ns).
 * @param attNavInMsg        Pointer to input attitude navigation payload.
 * @return Computed torque command payload.
 */
CmdTorqueBodyMsgPayload RateDampAlgorithm_update(RateDampAlgorithm* self,
                                                 uint64_t currentSimNanos,
                                                 NavAttMsgPayload* attNavInMsg);

/**
 * @brief Set the rate-feedback gain.
 * @param self The algorithm instance.
 * @param p    New feedback gain [N·m·s].
 */
void RateDampAlgorithm_setRateGain(RateDampAlgorithm* self, float p);

/**
 * @brief Get the current rate-feedback gain.
 * @param self The algorithm instance.
 * @return The current feedback gain [N·m·s].
 */
float RateDampAlgorithm_getRateGain(const RateDampAlgorithm* self);

#ifdef __cplusplus
}
#endif

#endif  // RATE_DAMP_ALGORITHM_C_H

