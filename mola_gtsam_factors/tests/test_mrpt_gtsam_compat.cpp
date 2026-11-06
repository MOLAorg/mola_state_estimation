/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

#include <mola_gtsam_factors/mrpt_gtsam_compat.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        throw std::runtime_error("Test assertion failed: " + msg);
    }
}

gtsam::Pose3 pose_from_ypr(const gtsam::Vector6& p)
{
    const auto P = mrpt::poses::CPose3D::FromXYZYawPitchRoll(p[0], p[1], p[2], p[3], p[4], p[5]);
    return gtsam::Pose3(
        gtsam::Rot3(P.getRotationMatrix().asEigen()), gtsam::Point3(P.x(), P.y(), P.z()));
}

// Some full covariance in MRPT (x,y,z,yaw,pitch,roll) coordinates.
mrpt::poses::CPose3DPDFGaussian test_pdf()
{
    mrpt::poses::CPose3DPDFGaussian pdf;
    pdf.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, -2.0, 3.0, 2.5, 0.3, -0.4);
    gtsam::Matrix6 A;
    // clang-format off
    A << 0.3,  0.1, 0.0,  0.02, 0.0,  0.01,
         0.0,  0.2, 0.05, 0.0,  0.01, 0.0,
         0.1,  0.0, 0.4,  0.0,  0.0,  0.03,
         0.0,  0.02, 0.0, 0.05, 0.01, 0.0,
         0.01, 0.0, 0.0,  0.0,  0.04, 0.01,
         0.0,  0.0, 0.02, 0.01, 0.0,  0.06;
    // clang-format on
    pdf.cov.asEigen() = A * A.transpose();
    return pdf;
}

// The GTSAM covariance must be J*C*J^T, with J the numerical Jacobian of
// Logmap(mean^-1 * pose) with respect to (x,y,z,yaw,pitch,roll).
void test_to_gtsam_vs_numerical_jacobian()
{
    const auto pdf = test_pdf();

    gtsam::Vector6 p0;
    p0 << pdf.mean.x(), pdf.mean.y(), pdf.mean.z(), pdf.mean.yaw(), pdf.mean.pitch(),
        pdf.mean.roll();
    const gtsam::Pose3 mean = pose_from_ypr(p0);

    const double   h = 1e-6;
    gtsam::Matrix6 J;
    for (int i = 0; i < 6; i++)
    {
        gtsam::Vector6 pp = p0;
        gtsam::Vector6 pm = p0;
        pp[i] += h;
        pm[i] -= h;
        J.col(i) = (gtsam::Pose3::Logmap(mean.between(pose_from_ypr(pp))) -
                    gtsam::Pose3::Logmap(mean.between(pose_from_ypr(pm)))) /
                   (2 * h);
    }
    const gtsam::Matrix6 expected = J * pdf.cov.asEigen() * J.transpose();

    gtsam::Pose3   pose;
    gtsam::Matrix6 cov;
    mola::gtsam_compat::to_gtsam_se3_cov6(pdf, pose, cov);

    expect(pose.equals(mean, 1e-9), "to_gtsam_se3_cov6() pose");
    const double err = (cov - expected).cwiseAbs().maxCoeff();
    expect(err < 1e-6, "to_gtsam_se3_cov6() covariance, max error=" + std::to_string(err));
    std::cout << "[Test Passed] to_gtsam_se3_cov6() vs numerical Jacobian\n";
}

void test_round_trip()
{
    const auto pdf = test_pdf();

    gtsam::Pose3   pose;
    gtsam::Matrix6 cov;
    mola::gtsam_compat::to_gtsam_se3_cov6(pdf, pose, cov);
    const auto back = mola::gtsam_compat::to_mrpt_se3_pdf(pose, cov);

    expect(
        (back.mean.asVectorVal() - pdf.mean.asVectorVal()).asEigen().cwiseAbs().maxCoeff() < 1e-9,
        "to_mrpt_se3_pdf() mean");
    const double err = (back.cov.asEigen() - pdf.cov.asEigen()).cwiseAbs().maxCoeff();
    expect(err < 1e-9, "to_mrpt_se3_pdf() covariance, max error=" + std::to_string(err));
    std::cout << "[Test Passed] to_mrpt_se3_pdf(to_gtsam_se3_cov6()) round trip\n";
}
}  // namespace

int main()
{
    try
    {
        test_to_gtsam_vs_numerical_jacobian();
        test_round_trip();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
