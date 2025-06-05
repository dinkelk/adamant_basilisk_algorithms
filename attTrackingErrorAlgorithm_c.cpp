#include "attTrackingErrorAlgorithm_c.h"
#include "attTrackingErrorAlgorithm.h"  // Original C++ header
#include <Eigen/Core>                   // For Eigen::Vector3f

extern "C" {

/*
 * Allocate a new C++ AttTrackingErrorAlgorithm on the heap.
 */
AttTrackingErrorAlgorithm* attea_new() {
    return new AttTrackingErrorAlgorithm();
}

/*
 * Delete a C++ AttTrackingErrorAlgorithm previously created with attea_new().
 */
void attea_delete(AttTrackingErrorAlgorithm* ptr) {
    delete ptr;
}

/*
 * Call the C++ reset(uint64_t) method.
 */
void attea_reset(AttTrackingErrorAlgorithm* ptr, uint64_t callTime) {
    ptr->reset(callTime);
}

/*
 * Call the C++ update(uint64_t, AttRefMsgPayload&, NavAttMsgPayload&).
 * Returns a copy of AttGuidMsgPayload by value.
 */
AttGuidMsgPayload
attea_update(AttTrackingErrorAlgorithm* ptr,
             uint64_t callTime,
             AttRefMsgPayload* attRefIn,
             NavAttMsgPayload* attNavIn)
{
    // Dereference the C pointers back into C++ references:
    return ptr->update(callTime, *attRefIn, *attNavIn);
}

/*
 * Call the C++ setSigma_R0R(const Eigen::Vector3f&).
 * Ada passes a float[3]; convert to Eigen::Vector3f.
 */
void attea_set_sigma_R0R(AttTrackingErrorAlgorithm* ptr, float sigma[3]) {
    Eigen::Vector3f v;
    v << sigma[0], sigma[1], sigma[2];
    ptr->setSigma_R0R(v);
}

/*
 * Call the C++ getSigma_R0R() const.
 * Convert the returned Eigen::Vector3f into three floats.
 */
void attea_get_sigma_R0R(AttTrackingErrorAlgorithm* ptr, float out_sigma[3]) {
    Eigen::Vector3f v = ptr->getSigma_R0R();
    out_sigma[0] = v[0];
    out_sigma[1] = v[1];
    out_sigma[2] = v[2];
}

}  // extern "C"
