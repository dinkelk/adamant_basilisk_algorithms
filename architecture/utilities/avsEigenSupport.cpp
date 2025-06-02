/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "avsEigenSupport.h"

/*! This function provides a direct conversion between a 3-float-vector and an
output C float array. We are providing this function to save on the inline conversion
and the transpose that would have been performed by the general case.
@return void
@param inMat The source Eigen 3-float-vector
@param outArray The destination array we copy into
*/

void eigenVector3f2CArray(Eigen::Vector3f &inMat, float *outArray) {
    memcpy(outArray, inMat.data(), 3 * sizeof(float));
}
/*! This function performs the conversion between an input C array
3-float-vector and an output Eigen vector3f. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Vector3f
@param inArray The input float array (row-major)
*/
Eigen::Vector3f cArray2EigenVector3f(float *inArray) { return Eigen::Map<Eigen::Vector3f>(inArray, 3, 1); }
