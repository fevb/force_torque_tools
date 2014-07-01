/*
 *  ft_calib.h
 *
 *  Least squares calibration of:
 *  - Bias of F/T sensor
 *  - Mass of attached gripper
 *  - Location of the center of mass of the gripper
 *
 *  Requires calibrated accelerometer readings
 *  (calibrated with respect to the robot).
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



#ifndef FTCALIB_H_
#define FTCALIB_H_
#include <kdl/frames.hpp>
#include <kdl/frameacc.hpp>
#include <eigen3/Eigen/Core>
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

// Least Squares calibration of bias of FT sensor and the mass and location of the COM of the gripper
namespace Calibration{
class FTCalib
{
public:

	FTCalib();
	virtual ~FTCalib();

	// Least squares to estimate the F/T sensor parameters
	// The estimated parameters are :
	// [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
	// m: mass of the gripper
	// [cx, cy, cz] are the coordinates of the center of mass of the gripper
	// FB : force bias
	// TB: torque bias
	// All expressed in the FT sensor frame
    // The inputs are the FT sensor acceleration (including gravity) and (raw) FT measurements
    // expressed in FT sensor frame.
    virtual Eigen::Matrix<double, 10, 1> calibrate(const KDL::FrameAcc &ft_sensor_acc, const KDL::Vector &gravity,
                                      const KDL::Wrench &ft_raw_meas);

protected:

    BFL::LinearAnalyticConditionalGaussian* m_sys_pdf;
    BFL::LinearAnalyticSystemModelGaussianUncertainty* m_sys_model;

    BFL::LinearAnalyticConditionalGaussian *m_ft_meas_pdf;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *m_ft_meas_model;

    BFL::Gaussian *m_prior;
    BFL::ExtendedKalmanFilter *m_filter;


	// measurement matrix based on "On-line Rigid Object Recognition and Pose Estimation
	//  Based on Inertial Parameters", D. Kubus, T. Kroger, F. Wahl, IROS 2008
    virtual MatrixWrapper::Matrix getMeasurementMatrix(const KDL::FrameAcc &ft_sensor_acc,
                                                       const KDL::Vector &gravity);

};
}


#endif /* INERTIALPARAMESTIMATOR_H_ */
