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

#ifndef _AVSEIGENSUPPORT_
#define _AVSEIGENSUPPORT_
#include "Eigen/src/Freestanding/freestanding_fpclassify.h"
#include "avsEigenMRP.h"

#include <Eigen/Core>

//!@brief Conversion between 3-float-vector and output array
void eigenVector3f2CArray(Eigen::Vector3f &inMat, float *outArray);

//!@brief Specific conversion between a float C array and an Eigen float 3-vector
Eigen::Vector3f cArray2EigenVector3f(float *inArray);

#endif /* _AVSEIGENSUPPORT_ */
