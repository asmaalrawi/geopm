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

#include <cmath>
#include <cassert>
#include <algorithm>

#include "TwoDMatrix.hpp"

TwoDMatrix::TwoDMatrix():
     fit_matrix(2, std::vector<double>(1.0))
    , fit_inverse(2, std::vector<double>(1.0))
    , fit_vector(2, 1.0)
{
}


void TwoDMatrix::invert_matrix()
{
    //TODO: Replace--if debug is not enabled they're ignored
//        GEOPM_DEBUG_ASSERT(!std::isnan(matrix_determinant),
//                           "TwoDMatrix::" + std::string(__func__) +
//                           "(): Matrix Determinant is nan");
//
//        GEOPM_DEBUG_ASSERT(matrix_determinant!=0,
//                           "TwoDMatrix::" + std::string(__func__) +
//                           "(): Attempting to invert a non-invertable matrix");
//
    
    double matrix_determinant = det();

    if(matrix_determinant == 0){
        fit_inverse[0][0] = 1.0;
        fit_inverse[0][1] = 0.0;
        fit_inverse[1][0] = 0.0;
        fit_inverse[1][1] = 1.0;
    }
        double inv_det = 1.0/matrix_determinant;
        fit_inverse[0][0] = inv_det*fit_matrix[1][1];
        fit_inverse[0][1] = -1.0*inv_det*fit_matrix[0][1];
        fit_inverse[1][0] = -1.0*inv_det*fit_matrix[1][0];
        fit_inverse[1][1] = inv_det*fit_matrix[0][0];
}

void TwoDMatrix::print_fit()
{
    std::vector<double> state_vector = multiply_vector_by_inverse_matrix();

    for(int first_idx=0; first_idx<2; first_idx++){
        printf("| %5.2f | | ", fit_vector[first_idx]);
        for(int sec_idx=0; sec_idx<2; sec_idx++){
            printf("%5.2f ", fit_inverse[first_idx][sec_idx]);
        }
        printf("| = | %5.2f |\n", state_vector[first_idx]);
    }
    return;
}

double TwoDMatrix::det(){
    return fit_matrix[0][0]*fit_matrix[1][1] - fit_matrix[0][1]*fit_matrix[1][0];
}

void TwoDMatrix::scale_matrix(double scalar){
    for(int first_idx=0; first_idx<2; first_idx++){
        for(int sec_idx=0; sec_idx<2; sec_idx++){
            fit_matrix[first_idx][sec_idx] *= scalar;
        }
    }
    return;
}

void TwoDMatrix::scale_vector(double scalar)
{
    for(int first_idx=0; first_idx<2; first_idx++){
        fit_vector[first_idx] *= scalar;
    }
    return;
}

void TwoDMatrix::update_vector(double ctrl, double measure, double history_discount)
{
    scale_vector(history_discount);
    
    fit_vector[0] += ctrl*measure;
    fit_vector[1] += measure;
    return;
}

std::vector<double> TwoDMatrix::multiply_vector_by_matrix()
{
    std::vector<double> state_vector (2, 0.0);

    for(int first_idx=0; first_idx<2; first_idx++){
        for(int sec_idx=0; sec_idx<2; sec_idx++){
            state_vector[first_idx] += fit_matrix[first_idx][sec_idx]*fit_vector[sec_idx];
        }
    }
    return state_vector;
}

std::vector<double> TwoDMatrix::multiply_vector_by_inverse_matrix()
{
    std::vector<double> state_vector (2, 0.0);

    for(int first_idx=0; first_idx<2; first_idx++){
        for(int sec_idx=0; sec_idx<2; sec_idx++){
            state_vector[first_idx] += fit_inverse[first_idx][sec_idx]*fit_vector[sec_idx];
        }
    }
    return state_vector;
}

void TwoDMatrix::update_fit(double ctrl, double measure, double history_discount)
{
    scale_matrix(history_discount);

    fit_matrix[0][0] += ctrl*ctrl;
    fit_matrix[0][1] += ctrl;
    fit_matrix[1][0] += ctrl;
    fit_matrix[1][1] += 1.0;

    invert_matrix();

}

double TwoDMatrix::estimate_measure(double ctrl_value)
{
    std::vector<double> state_vector = multiply_vector_by_inverse_matrix();

    return ctrl_value * state_vector[0] + state_vector[1];
}

double TwoDMatrix::estimate_ctrl(double desired_measure)
{
    std::vector<double> state_vector = multiply_vector_by_inverse_matrix();


    return (desired_measure - state_vector[1])/state_vector[0];
}
