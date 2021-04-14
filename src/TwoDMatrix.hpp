/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//TODO: Put doxygen in header

#ifndef TWODMATRIX_HPP_INCLUDE
#define TWODMATRIX_HPP_INCLUDE

#include <vector>

//TODO: Have functions return when they're actually calculating something 
//TODO: Have functions explicitly feed in vectors/matrices
//Could have this be stateless and define the vectors/matrices 

class TwoDMatrix
{
    public:
        TwoDMatrix();
        
        void print_fit();
        void invert_matrix();
        double det();
        void scale_matrix(double scalar);
        void scale_vector(double scalar);
        void update_vector(double ctrl, double measure, double history_discount);
        std::vector<double> multiply_vector_by_matrix();
        std::vector<double> multiply_vector_by_inverse_matrix();
        void update_fit(double ctrl, double measure, double history_discount);
        double estimate_measure(double ctrl_value);
        double estimate_ctrl(double desired_measure);

    private:
        std::vector<std::vector<double> > fit_matrix;
        std::vector<std::vector<double> > fit_inverse;
        std::vector<double> fit_vector;
};

#endif
