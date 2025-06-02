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

#include "fswAlgorithms/attGuidance/attTrackingError/attTrackingErrorAlgorithm.h"

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingErrorAlgorithm::reset(uint64_t callTime) {
    // Reset the algorithm
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
AttGuidMsgPayload AttTrackingErrorAlgorithm::update(uint64_t callTime,
                                                    AttRefMsgPayload& attRefInMsg,
                                                    NavAttMsgPayload& attNavInMsg) {
    AttGuidMsgPayload attGuidOut;

    // Compute MRP from the original reference frame R0 to the corrected reference frame R
    Eigen::Vector3f sigma_RR0 = -1.0 * this->sigma_R0R;

    // Compute MRP from inertial to updated reference frame sigma_RN
    Eigen::Vector3f sigma_RN = cArray2EigenVector3f(attRefInMsg.sigma_RN);
    sigma_RN = addMrp(sigma_RN, sigma_RR0);

    // Compute attitude error sigma_BR
    Eigen::Vector3f sigma_BN = cArray2EigenVector3f(attNavInMsg.sigma_BN);
    Eigen::Vector3f sigma_BR = subMrp(sigma_BN, sigma_RN);

    // Compute angular velocity reference body frame components omega_RN_B
    Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    Eigen::Vector3f omega_RN_N = cArray2EigenVector3f(attRefInMsg.omega_RN_N);
    Eigen::Vector3f omega_RN_B = dcm_BN * omega_RN_N;

    // Compute angular velocity error omega_BR
    Eigen::Vector3f omega_BN_B = cArray2EigenVector3f(attNavInMsg.omega_BN_B);
    Eigen::Vector3f omega_BR_B = omega_BN_B - omega_RN_B;

    // Compute reference angular velocity rate in body frame components domega_RN_B
    Eigen::Vector3f domega_RN_N = cArray2EigenVector3f(attRefInMsg.domega_RN_N);
    Eigen::Vector3f domega_RN_B = dcm_BN * domega_RN_N;

    // Write attitude guidance output message
    eigenVector3f2CArray(omega_RN_B, attGuidOut.omega_RN_B);
    eigenVector3f2CArray(omega_BR_B, attGuidOut.omega_BR_B);
    eigenVector3f2CArray(sigma_BR, attGuidOut.sigma_BR);
    eigenVector3f2CArray(domega_RN_B, attGuidOut.domega_RN_B);

    return attGuidOut;
}

/*! Setter method for sigma_R0R.
 @return void
 @param sigma_R0R
*/
void AttTrackingErrorAlgorithm::setSigma_R0R(const Eigen::Vector3f& sigma_R0R) { this->sigma_R0R = sigma_R0R; }

/*! Getter method for sigma_R0R.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3f& AttTrackingErrorAlgorithm::getSigma_R0R() const { return this->sigma_R0R; }
