/*
 *  ft_calib.cpp
 *
 *
 *  Created on: Sep 26, 2012
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <force_torque_sensor_calib/ft_calib.h>

namespace Calibration{

FTCalib::FTCalib()
{

    // set the process model
    MatrixWrapper::Matrix A(10,10);
    for (unsigned int i = 0; i < 10; i++)
    {
        for (unsigned int j = 0; j < 10; j++)
        {
            if (i == j) {
                A(i + 1, j + 1) = 1.0;
            }

            else {
                A(i + 1, j + 1) = 0.0;
            }
        }
    }

    MatrixWrapper::ColumnVector mu(10);
    for(unsigned int i=0; i<10; i++) mu(i+1) = 0.0;

    MatrixWrapper::SymmetricMatrix sigma(10);
    sigma = 0.0;
    for(unsigned int i=0; i<10; i++) sigma(i+1,i+1) = 1.0;

    BFL::Gaussian sys_uncertainty(mu, sigma);
    m_sys_pdf = new BFL::LinearAnalyticConditionalGaussian(A, sys_uncertainty);

    m_sys_model = new BFL::LinearAnalyticSystemModelGaussianUncertainty(m_sys_pdf);


    // initialize the measurement model
    MatrixWrapper::Matrix H(6,10);
    H = 0.0;

    MatrixWrapper::ColumnVector ft_meas_mu(6);
    ft_meas_mu = 0.0;

    MatrixWrapper::SymmetricMatrix ft_meas_cov(6);
    ft_meas_cov = 0.0;

    for(unsigned int i=0; i<3; i++)
    {
        ft_meas_cov(i+1, i+1) = 0.2;
        ft_meas_cov(i+4, i+4) = 0.05;
    }

    BFL::Gaussian ft_meas_uncertainty(ft_meas_mu, ft_meas_cov);

    m_ft_meas_pdf = new BFL::LinearAnalyticConditionalGaussian(H, ft_meas_uncertainty);

    m_ft_meas_model = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(m_ft_meas_pdf);


    // set the prior
    MatrixWrapper::ColumnVector prior_mu(10);
    for(unsigned int i=0; i<10; i++) prior_mu(i+1) = 0.0;
    prior_mu(1) = 3.0;

    MatrixWrapper::SymmetricMatrix prior_sigma(10);
    prior_sigma = 0.0;
    for(unsigned int i=0; i<10; i++) prior_sigma(i+1,i+1) = 1.0;

    m_prior = new BFL::Gaussian(prior_mu, prior_sigma);

    // construct the filter
    m_filter = new BFL::ExtendedKalmanFilter(m_prior);

}

FTCalib::~FTCalib(){
    delete m_sys_model;
    delete m_sys_pdf;
    delete m_prior;
    delete m_filter;
    delete m_ft_meas_pdf;
    delete m_ft_meas_model;
}

// Least squares to estimate the FT sensor parameters
// The estimated parameters are :
// [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
// m: mass of the gripper
// [cx, cy, cz] are the coordinates of the center of mass of the gripper
// FB : force bias
// TB: torque bias
// All expressed in the FT sensor frame
// The inputs are the FT sensor acceleration (including gravity) and (raw) FT measurements
// expressed in FT sensor frame.
Eigen::Matrix<double, 10, 1> FTCalib::calibrate(const KDL::FrameAcc &ft_sensor_acc, const KDL::Vector &gravity,
                          const KDL::Wrench &ft_raw_meas)
{
    Eigen::Matrix<double, 10, 1> ft_sensor_params;

    MatrixWrapper::Matrix H = getMeasurementMatrix(ft_sensor_acc, gravity);

    // update the measurement model

    MatrixWrapper::ColumnVector ft_meas_mu(6);
    ft_meas_mu = 0.0;

    MatrixWrapper::SymmetricMatrix ft_meas_cov(6);
    ft_meas_cov = 0.0;

    for(unsigned int i=0; i<3; i++)
    {
        ft_meas_cov(i+1, i+1) = 0.2;
        ft_meas_cov(i+4, i+4) = 0.05;
    }

    BFL::Gaussian ft_meas_uncertainty(ft_meas_mu, ft_meas_cov);

    *m_ft_meas_pdf = BFL::LinearAnalyticConditionalGaussian(H, ft_meas_uncertainty);

    *m_ft_meas_model = BFL::LinearAnalyticMeasurementModelGaussianUncertainty(m_ft_meas_pdf);

    MatrixWrapper::ColumnVector ft_meas(6);
    for(unsigned int i=0; i<6; i++) ft_meas(i+1) = ft_raw_meas(i);


    // update the Kalman filter
    m_filter->Update(m_sys_model, m_ft_meas_model, ft_meas);

    MatrixWrapper::ColumnVector posterior_mean = m_filter->PostGet()->ExpectedValueGet();

    for(unsigned int i=0; i<10; i++) ft_sensor_params(i,0) = posterior_mean(i+1);

    return ft_sensor_params;
}


MatrixWrapper::Matrix FTCalib::getMeasurementMatrix(const KDL::FrameAcc &ft_sensor_acc,
                                                    const KDL::Vector &gravity)
{
    KDL::Vector w = ft_sensor_acc.M.w;
    KDL::Vector alpha = ft_sensor_acc.M.dw;
    KDL::Vector a = ft_sensor_acc.p.dv;

    KDL::Vector g = gravity;

    MatrixWrapper::Matrix H(6,10);
    H = 0.0;

	for(unsigned int i=0; i<3; i++)
	{
		for(unsigned int j=4; j<10; j++)
		{
			if(i==j-4)
			{
                H(i+1,j+1) = 1.0;
			}
			else
			{
                H(i+1,j+1) = 0.0;
			}
		}
	}

	for(unsigned int i=3; i<6; i++)
	{
        H(i+1,1) = 0.0;
	}

    H(4,2) = 0.0;
    H(5,3) = 0.0;
    H(6,4) = 0.0;

	for(unsigned int i=0; i<3; i++)
	{
        H(i+1,1) = a(i) - g(i);
	}

    H(1,2) = -w(1)*w(1) - w(2)*w(2);
    H(1,3) = w(0)*w(1) - alpha(2);
    H(1,4) = w(0)*w(2) + alpha(1);

    H(2,2) = w(0)*w(1) + alpha(2);
    H(2,3) = -w(0)*w(0) - w(2)*w(2);
    H(2,4) = w(1)*w(2) - alpha(0);

    H(3,2) = w(0)*w(2) - alpha(1);
    H(3,3) = w(1)*w(2) + alpha(0);
    H(3,4) = -w(1)*w(1) - w(0)*w(0);

    H(4,3) = a(2) - g(2);
    H(4,4) = -a(1) + g(1);


    H(5,2) = -a(2) + g(2);
    H(5,4) = a(0) - g(0);

    H(6,2) = a(1) - g(1);
    H(6,3) = -a(0) + g(0);

	for(unsigned int i=3; i<6; i++)
	{
		for(unsigned int j=4; j<10; j++)
		{
			if(i==(j-4))
			{
                H(i+1,j+1) = 1.0;
			}
			else
			{
                H(i+1,j+1) = 0.0;
			}
		}
	}


	return H;
}

}
