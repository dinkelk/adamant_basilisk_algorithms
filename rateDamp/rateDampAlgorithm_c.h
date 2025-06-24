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

#ifndef RATEDAMPALGORITHM_C_H
#define RATEDAMPALGORITHM_C_H

#include <stdint.h>
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RateDampAlgorithm instance.
 */
typedef struct RateDampAlgorithm RateDampAlgorithm;

/**
 * @brief Construct a new RateDampAlgorithm instance.
 * @return Pointer to a new RateDampAlgorithm (must be destroyed).
 */
RateDampAlgorithm* RateDampAlgorithm_create(void);

/**
 * @brief Destroy a previously created RateDampAlgorithm.
 * @param self Pointer to the instance to destroy.
 */
void RateDampAlgorithm_destroy(RateDampAlgorithm* self);

/**
 * @brief Execute the update step of the algorithm.
 * @param self            Pointer to the instance.
 * @param currentSimNanos Current simulation time in nanoseconds.
 * @param attNavInMsg     Pointer to navigation attitude message payload.
 * @return CmdTorqueBodyMsgPayload  Computed torque command.
 */
CmdTorqueBodyMsgPayload RateDampAlgorithm_update(
    RateDampAlgorithm* self,
    uint64_t currentSimNanos,
    NavAttMsgPayload* attNavInMsg);

/**
 * @brief Set the rate feedback gain.
 * @param self Pointer to the instance.
 * @param p    New gain value.
 */
void RateDampAlgorithm_setRateGain(RateDampAlgorithm* self, float p);

/**
 * @brief Get the current rate feedback gain.
 * @param self Pointer to the instance.
 * @return float Current gain value.
 */
float RateDampAlgorithm_getRateGain(RateDampAlgorithm* self);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // RATEDAMPALGORITHM_C_H
