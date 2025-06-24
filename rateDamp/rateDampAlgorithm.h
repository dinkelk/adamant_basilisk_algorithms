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
#ifndef _RATE_DAMP_ALGORITHM_C_H_
#define _RATE_DAMP_ALGORITHM_C_H_

#include <stdint.h>
#include "architecture/utilities/avsEigenSupport.h"
#include "RateDampAlgorithm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle to the C++ RateDampAlgorithm instance.
 */
typedef struct RateDampAlgorithm RateDampAlgorithm;

/*!
 * @brief Create a new RateDampAlgorithm instance and return an opaque handle.
 *
 * This function creates a new C++ RateDampAlgorithm object and returns
 * an opaque pointer that can be used in the C interface. The caller is responsible for
 * destroying this handle when done using it, typically by calling rate_damp_algorithm_destroy().
 *
 * @return Pointer to a newly created RateDampAlgorithm instance.
 */
RateDampAlgorithm* rate_damp_algorithm_create(void);

/*!
 * @brief Destroy a RateDampAlgorithm instance.
 *
 * This function frees the memory associated with the given RateDampAlgorithm handle.
 *
 * @param self Handle to the RateDampAlgorithm instance to destroy.
 */
void rate_damp_algorithm_destroy(RateDampAlgorithm* self);

/*!
 * @brief Update method for the RateDampAlgorithm.
 *
 * This function computes and returns a command torque message payload based on
 * the navigation attitude measurements. It should be called periodically with updated data.
 *
 * @param[in] currentSimNanos Current simulation time in nanoseconds since Unix epoch.
 * @param[in] attNavInMsg Navigation attitude message payload containing vehicle state information.
 * @return Command torque body message payload.
 */
RateDampAlgorithm_CmdTorqueBodyMsgPayload rate_damp_algorithm_update(
        RateDampAlgorithm* self,
        uint64_t currentSimNanos,
        const NavAttMsgPayload_c* attNavInMsg);

/*!
 * @brief Set the rate feedback gain for the algorithm.
 *
 * This function sets a parameter of the RateDampAlgorithm that controls
 * how aggressively rates are damped. The value must be positive and typically small.
 *
 * @param self Handle to the RateDampAlgorithm instance.
 * @param[in] p Gain value (positive).
 */
void rate_damp_algorithm_set_rate_gain(RateDampAlgorithm* self, float p);

/*!
 * @brief Get the current rate feedback gain setting for the algorithm.
 *
 * This function retrieves the rate feedback gain parameter that was set via
 * rate_damp_algorithm_set_rate_gain().
 *
 * @param[in] self Handle to the RateDampAlgorithm instance.
 * @return Current gain value (positive).
 */
float rate_damp_algorithm_get_rate_gain(RateDampAlgorithm* self);

#ifdef __cplusplus
}
#endif

#endif /* _RATE_DAMP_ALGORITHM_C_H_ */
#include "RateDampAlgorithm_c.h"
#include <cstring>
#include <cassert>

RateDampAlgorithm* rate_damp_algorithm_create(void) {
    RateDampAlgorithm* self = new RateDampAlgorithm;
    // Initialize the algorithm state
    self->reset(0);
    return self;
}

void rate_damp_algorithm_destroy(RateDampAlgorithm* self) {
    assert(self != nullptr);
    delete self;
}

RateDampAlgorithm_CmdTorqueBodyMsgPayload rate_damp_algorithm_update(
        RateDampAlgorithm* self,
        uint64_t currentSimNanos,
        const NavAttMsgPayload_c* attNavInMsg) {

    // Convert C struct to Eigen vector for sigma_BN and omega_BN_B
    Eigen::Vector3f sigma_BN_vec(attNavInMsg->sigma_BN[0], attNavInMsg->sigma_BN[1], attNavInMsg->sigma_BN[2]);
    Eigen::Vector3f omega_BN_B_vec(attNavInMsg->omega_BN_B[0], attNavInMsg->omega_BN_B[1], attNavInMsg->omega_BN_B[2]);

    // Update the C++ algorithm
    auto cmdTorqueOutBuffer = self->update(currentSimNanos, *attNavInMsg);

    // Copy the result to a C struct
    RateDampAlgorithm_CmdTorqueBodyMsgPayload out;
    for (int i = 0; i < 3; ++i) {
        out.torqueRequestBody[i] = cmdTorqueOutBuffer.torqueRequestBody[i];
    }

    return out;
}

void rate_damp_algorithm_set_rate_gain(RateDampAlgorithm* self, float p) {
    assert(p > 0.0);
    self->setRateGain(abs(p));
}

float rate_damp_algorithm_get_rate_gain(RateDampAlgorithm* self) {
    return self->getRateGain();
}
