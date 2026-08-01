/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Rishibrata Biswas, Adnan Munawar
  Created on: 2019-11-11

  (C) Copyright 2019-2026 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorMTM.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

const double TWO_PI = 2.0 * cmnPI;

void AddAngleLifts(std::vector<double> & values,
                   const double & angle,
                   const double & minimum,
                   const double & maximum)
{
    const int first = static_cast<int>(std::ceil((minimum - angle) / TWO_PI - 1e-12));
    const int last = static_cast<int>(std::floor((maximum - angle) / TWO_PI + 1e-12));
    for (int turn = first; turn <= last; ++turn) {
        values.push_back(angle + turn * TWO_PI);
    }
}

double SquaredLimitPenalty(const double & value,
                           const double & minimum,
                           const double & maximum)
{
    // This is only a tie breaker: continuity with the supplied joint vector
    // remains the primary selection criterion.
    const double range = maximum - minimum;
    const double margin = std::min(value - minimum, maximum - value);
    const double safeMargin = std::max(margin, 1e-6 * range);
    return 1e-6 * range * range / (safeMargin * safeMargin);
}

double SquaredPlatformPreferencePenalty(const double & platform,
                                        const double & preferredPlatform)
{
    const double difference = platform - preferredPlatform;
    return difference * difference;
}

double SquaredNeutralJointPenalty(const double & value)
{
    // Joint 5 (wrist pitch) should remain near its neutral position whenever
    // the redundant platform and alternate Euler branch make that possible.
    // This deliberately outweighs continuity with the supplied reference;
    // e.g., it selects the alternate branch instead of retaining a 146 deg
    // wrist-pitch reference solely because it has zero motion cost.
    const double wristPitchPreferenceWeight = 10.0;
    return wristPitchPreferenceWeight * value * value;
}

}

robManipulatorMTM::robManipulatorMTM(const std::vector<robKinematics *> linkParms,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0)
{
}

robManipulatorMTM::robManipulatorMTM(const std::string &robotfilename,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0)
{
}

robManipulatorMTM::robManipulatorMTM(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0)
{
}

robManipulator::Errno
robManipulatorMTM::InverseKinematics(vctDynamicVector<double> & q,
                                     const vctFrame4x4<double> & Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        mLastError = "robManipulatorMTM::InverseKinematics: expected "
            + std::to_string(links.size()) + " joints values but received "
            + std::to_string(q.size());
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        mLastError = "robManipulatorMTM::InverseKinematics: the manipulator has no links";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    // if we encounter a joint limit, keep computing a solution but at
    // the end return failure
    bool hasReachedJointLimit = false;

    // take Rtw0 into account
    vctFrm4x4 Rt07;
    Rtw0.ApplyInverseTo(Rts, Rt07);

    q[0] = atan2l(Rt07.Translation().X(),
                  -Rt07.Translation().Y());

    // The position solution is specific to the MTM standard-DH chain, but
    // its lengths come from the robot model loaded by robManipulator.
    const double l1 = links[1].PStar().Norm();
    const double l1_sqr = l1 * l1;

    // Create the triangle "above" the forearm to find position.  For the
    // MTM, these are respectively A of link 3 and D of link 4.
    const double forarmBase = links[2].PStar().Norm();
    const double forarmHeight = links[3].PStar().Norm();
    const double l2_sqr = forarmBase * forarmBase + forarmHeight * forarmHeight;
    const double l2 = sqrt(l2_sqr) ;
    const double angleOffset = asinl(forarmHeight / l2);

    // project in plane formed by links 2 & 3 to find q2 and q3 (joint[1] and joint[2])
    const double x = -Rt07.Translation().Z();
    const double y = sqrt(Rt07.Translation().X() * Rt07.Translation().X()
                          + Rt07.Translation().Y() * Rt07.Translation().Y());

    // 2 dof IK in plane
    const double d_sqr = x * x + y * y;
    const double d = sqrt(d_sqr);
    const double a1 = atan2l(y, x);
    const double a2 = acosl((l1_sqr - l2_sqr + d_sqr) / (2.0 * l1 * d));
    const double q1 = a1 - a2;
    const double q2 = -acosl((l1_sqr + l2_sqr - d_sqr) / (2.0 * l1 * l2));

    q[1] = q1;
    q[2] = q2 - angleOffset + cmnPI_2;

    // check joint limits for first 3 joints
    for (size_t joint = 0; joint < 3; joint++) {
        if (ClampJointValueAndUpdateError(joint, q[joint], 1e-5)) {
            hasReachedJointLimit = true;
        }
    }

    // Joint 3 (the platform) is redundant for a Cartesian pose.  Resolve
    // this one-dimensional redundancy by evaluating the analytic wrist IK at
    // platform angles throughout its physical range.  The caller-provided q
    // is the continuity reference, so include it explicitly before sampling.
    const vctDynamicVector<double> reference(q);
    const double preferredPlatform = FindOptimalPlatformAngle(reference, Rt07);
    const double platformMin = links[3].GetKinematics()->PositionMin();
    const double platformMax = links[3].GetKinematics()->PositionMax();
    const double platformStep = cmnPI / 90.0; // two degrees
    std::vector<double> platformCandidates;
    platformCandidates.push_back(std::min(platformMax, std::max(platformMin, reference[3])));
    platformCandidates.push_back(platformMin);
    platformCandidates.push_back(platformMax);
    for (double platform = platformMin; platform < platformMax; platform += platformStep) {
        platformCandidates.push_back(platform);
    }

    bool foundWristSolution = false;
    double bestCost = std::numeric_limits<double>::infinity();
    vctDynamicVector<double> bestQ(q);
    const double wristMin[3] = {
        links[4].GetKinematics()->PositionMin(),
        links[5].GetKinematics()->PositionMin(),
        links[6].GetKinematics()->PositionMin()
    };
    const double wristMax[3] = {
        links[4].GetKinematics()->PositionMax(),
        links[5].GetKinematics()->PositionMax(),
        links[6].GetKinematics()->PositionMax()
    };

    for (size_t platformIndex = 0; platformIndex < platformCandidates.size(); ++platformIndex) {
        const double platform = platformCandidates[platformIndex];
        q[3] = platform;

        // Convert the MTM tool frame to the frame used by the DH wrist.
        vctEulerYZXRotation3 eulerOffset;
        eulerOffset.Assign(cmnPI_2, 0.0, -cmnPI_2);
        vctMatrixRotation3<double, true> Rt8;
        vctEulerToMatrixRotation3(eulerOffset, Rt8);
        vctFrm4x4 Rt78, Rt08;
        Rt78.Rotation().Assign(Rt8);
        Rt08 = Rt07 * Rt78;

        const vctFrm4x4 Rt04 = ForwardKinematics(q, 4);
        vctFrm4x4 Rt48;
        Rt04.ApplyInverseTo(Rt08, Rt48);
        const vctEulerZYXRotation3 euler(Rt48.Rotation());

        // ZYX has two non-singular families:
        // (alpha, beta, gamma) and (alpha + pi, pi - beta, gamma + pi).
        // The DH offsets map gamma to wrist roll as pi - gamma.
        const double branches[2][3] = {
            {euler.alpha(),             euler.beta(),              cmnPI - euler.gamma()},
            {euler.alpha() + cmnPI,     cmnPI - euler.beta(),      -euler.gamma()}
        };
        for (size_t branch = 0; branch < 2; ++branch) {
            std::vector<double> pitch, yaw, roll;
            AddAngleLifts(pitch, branches[branch][0], wristMin[0], wristMax[0]);
            AddAngleLifts(yaw,   branches[branch][1], wristMin[1], wristMax[1]);
            AddAngleLifts(roll,  branches[branch][2], wristMin[2], wristMax[2]);
            for (size_t pitchIndex = 0; pitchIndex < pitch.size(); ++pitchIndex) {
                for (size_t yawIndex = 0; yawIndex < yaw.size(); ++yawIndex) {
                    for (size_t rollIndex = 0; rollIndex < roll.size(); ++rollIndex) {
                        const double wrist[3] = {pitch[pitchIndex], yaw[yawIndex], roll[rollIndex]};
                        double cost = (platform - reference[3]) * (platform - reference[3]);
                        cost += SquaredPlatformPreferencePenalty(platform, preferredPlatform);
                        cost += SquaredNeutralJointPenalty(wrist[0]); // MTM joint 5: wrist pitch
                        for (size_t joint = 0; joint < 3; ++joint) {
                            const double difference = wrist[joint] - reference[joint + 4];
                            cost += difference * difference;
                            cost += SquaredLimitPenalty(wrist[joint], wristMin[joint], wristMax[joint]);
                        }
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestQ.Assign(q);
                            bestQ[4] = wrist[0];
                            bestQ[5] = wrist[1];
                            bestQ[6] = wrist[2];
                            foundWristSolution = true;
                        }
                    }
                }
            }
        }
    }

    if (!foundWristSolution) {
        mLastError = "robManipulatorMTM::InverseKinematics: no platform and wrist solution within joint limits";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }
    q.Assign(bestQ);

    if (hasReachedJointLimit) {
        return robManipulator::EFAILURE;
    }

    return robManipulator::ESUCCESS;
}

double robManipulatorMTM::FindOptimalPlatformAngle(const vctDynamicVector<double> & q,
                                                   const vctFrame4x4<double> & Rt07) const
{
    const vctFrm4x4 Rt03 = ForwardKinematics(q, 3);
    vctFrm4x4 Rt37;
    Rt03.ApplyInverseTo(Rt07, Rt37);

    const double x = Rt37.Element(0, 2);
    const double y = Rt37.Element(1, 2);
    const double projectionNorm = std::sqrt(x * x + y * y);
    if (projectionNorm <= 1e-12) {
        return q[3];
    }
    double angleDifference = std::acos(std::max(-1.0, std::min(1.0, -x / projectionNorm)));
    if (y > 0.0) {
        angleDifference = -angleDifference;
    }

    const double option1 = angleDifference;
    double option2 = option1 - cmnPI;
    if (option2 > cmnPI) {
        option2 -= TWO_PI;
    } else if (option2 < -3.0 * cmnPI_2) {
        option2 += TWO_PI;
    }
    if ((option2 < -cmnPI) && (option2 > -3.0 * cmnPI_2) && (q[3] > 0.0)) {
        option2 += TWO_PI;
    }
    double normalizedOption1 = option1;
    if ((normalizedOption1 > cmnPI_2) && (normalizedOption1 < cmnPI) && (q[3] < 0.0)) {
        normalizedOption1 -= TWO_PI;
    }

    const double solution = (std::abs(q[3] - option2) < std::abs(q[3] - normalizedOption1))
        ? option2 : normalizedOption1;
    const double projectionWeight = std::abs(std::cos(q[4]));
    const double platform = solution * projectionWeight + q[3] * (1.0 - projectionWeight);
    return std::min(links[3].GetKinematics()->PositionMax(),
                    std::max(links[3].GetKinematics()->PositionMin(), platform));
}
