/*
 * turbopixel_python.cpp
 *
 *  Created on: Sept 01, 2011
 *      Author: Andreas Mueller (amueller@ais.uni-bonn.de)
 *
 *
 * Software License Agreement (BSD License)
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

// python bindings for gpu turbopixels


#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/overloads.hpp>
#include <pyublas/numpy.hpp>
#include "../include/Turbopixels.h"

using namespace boost::python;
pyublas::numpy_matrix<int>
apply_turbopixels(const pyublas::numpy_vector<unsigned char> & image, const int & nSuperpixels, const int & device){
    const size_t ndims = image.ndim();
    if (ndims != 3)
        throw("Only RGB images supported.");
    const npy_intp * const dims = image.dims();
    const int height = dims[0];
    const int width = dims[1];
    pyublas::numpy_matrix<unsigned int> result(height, width);

    tpix::Turbopixels turbopixels(device, width, height, nSuperpixels);
    turbopixels.process(result.data().begin(), image.data().begin());
    return result;
    
}

BOOST_PYTHON_MODULE(turbopixel_python){
    def("turbopixels", apply_turbopixels, (arg("image"), arg("nSuperpixels"), arg("device")=0));
}
