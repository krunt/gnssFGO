//
// Created by lars on 08.12.21.
//

#ifndef ONLINE_FGO_GPINTERPOLATEDDDCARPHAFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDDDCARPHAFACTOR_H

//#include "model/gp_interpolator/GPInterpolatorPose3WNOA.h"
/*Inputs:
 * Keys: pose of time i&j X(i)&X(j), velocity of time i&j V(i)&V(j), integer double differenced ambiguity of time i M(i)
 * Double Difference CarrierPhase measurement: ddphi (already calculated) or carPhaMR,carPhaMB,carPhaIR,carPhaIB
 * Position of the satellite Master (got chosen) and the other satellite I: pointMaster ,pointSatI
 * Position of the Base station we double differentiate to: pointBase
 * Integer to find the right ambiguity in the vector: nAmbiguity (maybe it can be done in a more elegant way)
 * Position of sensor with respect to the Body: lb
 * Wavelength: lambda
 * delta_t: timedifference of state i & j tau: timedifference of state i & time of measurement
 * omega1&2: angularvelocity of time i and j
 * Covariance Matrix of the measurement/s: model and QcModel*/
/* measurement equation used:
 * Wavelength * DDCarrierPhase = Double Differenced Distance of Satellite and Receiver -(+) lambda * DDAmbiguityCycles + noise
 * somtimes the AmbiguityCycles get added, but its just a definition in DD measurements it doesnt matter because it can be neg anyway*/
/*Jacobian: (e_RM - e_RI) * (R * skew(t), I) * GPSLAMJacobian (Pose of measurement derived by vel or pose or omega)
 * */
//Dong, Boots, Dellaert - Sparse Gaussian Processes for Continuous-Time Trajectory Estimation on Matrix Lie Groups
//Double-Difference starts at p. 323 in Farrell - Aided Navigation (Differenced at p. 313)
namespace fgo {
  namespace factors {
    class GPInterpolatedDDCarPhaFactor : public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3,
            gtsam::Vector, gtsam::Pose3, gtsam::Vector3> {
    private:
      //measurements
      gtsam::Vector3 omega1_; //angular Rate of time i already corrected
      gtsam::Vector3 omega2_; // angular rate of time j already corrected
      double ddphi_; //DD measurement
      int nAmbiguity_; //position of ambiguity in vector
      //constants
      double lambda_; //wavelength
      gtsam::Point3 lb_; //position of antenna in body frame
      gtsam::Point3 lb2_ = gtsam::Point3(0,0,0);
      //points
      gtsam::Point3 pointMaster_; //position Master Satellite
      gtsam::Point3 pointSatI_;  //position Satellite I
      gtsam::Point3 pointBase_; //position of Base

      fgo::models::GaussianProcessInterpolatorPose3WNOA GPbase_;

      typedef GPInterpolatedDDCarPhaFactor This;
      typedef gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3,
              gtsam::Vector, gtsam::Pose3, gtsam::Vector3> Interpolator;


    public:
      GPInterpolatedDDCarPhaFactor() {}

      //normal measurements known when constructed
      GPInterpolatedDDCarPhaFactor(gtsam::Key point_i, gtsam::Key vel_i, gtsam::Key ambiguity_i,
                                   gtsam::Key point_j, gtsam::Key vel_j,
                                   const double &carPhaMR, const double &carPhaMB,
                                   const double &carPhaIR, const double &carPhaIB,
                                   const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                   const gtsam::Point3 &pointBase, const int nAmbiguity,
                                   const gtsam::Point3 &lb, const double &lambda,
                                   const double &delta_t, const double &tau,
                                   const gtsam::Vector3 &omega1, const gtsam::Vector3 &omega2,
                                   const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model) :
              Interpolator(model, point_i, vel_i, ambiguity_i, point_j, vel_j),
              omega1_(omega1), omega2_(omega2),
              nAmbiguity_(nAmbiguity), lambda_(lambda), lb_(lb), pointMaster_(pointMaster),
              pointSatI_(pointSatI), pointBase_(pointBase), GPbase_(Qc_model, delta_t, tau) {
        ddphi_ = (carPhaMR - carPhaMB) - (carPhaIR - carPhaIB);
      }

      // ddmeasurements not known when constructed
      GPInterpolatedDDCarPhaFactor(gtsam::Key point_i, gtsam::Key vel_i, gtsam::Key ambiguity_i,
                                   gtsam::Key point_j, gtsam::Key vel_j, const double &ddCarPha,
                                   const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                   const gtsam::Point3 &pointBase, const int nAmbiguity,
                                   const gtsam::Point3 &lb, const double &lambda, const double &tau,
                                   const double &delta_t, const gtsam::Vector3 &omega1, const gtsam::Vector3 &omega2,
                                   const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model) :
              Interpolator(model, point_i, vel_i, ambiguity_i, point_j, vel_j), omega1_(omega1),
              omega2_(omega2), ddphi_(ddCarPha), nAmbiguity_(nAmbiguity), lambda_(lambda), lb_(lb),
              pointMaster_(pointMaster), pointSatI_(pointSatI), pointBase_(pointBase), GPbase_(Qc_model, delta_t, tau) {
      }

      GPInterpolatedDDCarPhaFactor(gtsam::Key point_i, gtsam::Key vel_i, gtsam::Key ambiguity_i,
                                   gtsam::Key point_j, gtsam::Key vel_j, const double &ddCarPha,
                                   const gtsam::Point3 &pointMaster, const gtsam::Point3 &pointSatI,
                                   const int nAmbiguity, const gtsam::Point3 &lb, const gtsam::Point3 &lb2,
                                   const double &lambda, const double &tau,
                                   const double &delta_t, const gtsam::Vector3 &omega1, const gtsam::Vector3 &omega2,
                                   const gtsam::SharedNoiseModel &model, const gtsam::SharedNoiseModel &Qc_model) :
              Interpolator(model, point_i, vel_i, ambiguity_i, point_j, vel_j), omega1_(omega1),
              omega2_(omega2), ddphi_(ddCarPha), nAmbiguity_(nAmbiguity), lambda_(lambda), lb_(lb),
              lb2_(lb2), pointMaster_(pointMaster)
              , pointSatI_(pointSatI), GPbase_(Qc_model, delta_t, tau) {
      }

      virtual ~GPInterpolatedDDCarPhaFactor() {}

      gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
      }

      gtsam::Vector evaluateError(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1,
                                  const gtsam::Vector &ddambiguity_j,
                                  const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2,
                                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                                  boost::optional<gtsam::Matrix &> H5 = boost::none) const override {

        gtsam::Pose3 poseBody; //interpolated position
        gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P; //matrices for interpolate position
        gtsam::Vector6 omVel1 = (gtsam::Vector6() << omega1_, vel1).finished();
        gtsam::Vector6 omVel2 = (gtsam::Vector6() << omega2_, vel2).finished();
        if (H1 || H2 || H4 || H5) {
          poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2,
                                             Hint1_P, Hint2_P, Hint3_P, Hint4_P);
        } else {
          poseBody = GPbase_.interpolatePose(pose1, omVel1, pose2, omVel2);
        }

        double rangeRM, rangeRI, rangeBM, rangeBI;
        gtsam::Matrix13 e_RM, e_RI; //unity vector pointing from receiver to satellite (1x3)
        gtsam::Matrix13 e_RM2, e_RI2 = gtsam::Matrix13(0,0,0);
        gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(0, 0, 0), lb_); //pose_Antenna wrt to Body
        gtsam::Point3 positionReceiver = poseBody.compose(body_P_sensor).translation(); //pose_Antenna wrt to Earth

        //calculate distances
        if (lb2_.norm() == 0){
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeBM = gtsam::distance3(pointBase_, pointMaster_);
          rangeBI = gtsam::distance3(pointBase_, pointSatI_);
        } else {
          gtsam::Point3 positionReceiver2 = poseBody.compose(gtsam::Pose3(gtsam::Rot3(), lb2_)).translation();
          rangeRM = gtsam::distance3(positionReceiver, pointMaster_, e_RM);
          rangeRI = gtsam::distance3(positionReceiver, pointSatI_, e_RI);
          rangeBM = gtsam::distance3(positionReceiver2, pointMaster_, e_RM2);
          rangeBI = gtsam::distance3(positionReceiver2, pointSatI_, e_RI2);
        }

        if (H1 || H2 || H4 || H5) {
          gtsam::Matrix66 correction1 = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose1.rotation().matrix().transpose()).finished();
          gtsam::Matrix66 correction2 = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose2.rotation().matrix().transpose()).finished();
          gtsam::Matrix66 correction = (gtsam::Matrix66() << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, poseBody.rotation().matrix()).finished();
          //CORRECT MATRICES
          Hint1_P = correction * Hint1_P * correction1;
          Hint3_P = correction * Hint3_P * correction2;
          // we just need derivation of vel
          Hint2_P = Hint2_P.block<6,3>(0,3);
          Hint4_P = Hint4_P.block<6,3>(0,3);
          
          gtsam::Matrix rotation = poseBody.rotation().matrix(); // eRb
          gtsam::Matrix16 jacobian =
                  (gtsam::Matrix16() << (e_RM - e_RI) * rotation * gtsam::skewSymmetric(-lb_) -
                          (e_RM2 - e_RI2) * rotation * gtsam::skewSymmetric(-lb2_),
                          e_RM - e_RI - (e_RM2 - e_RI2)).finished(); //calculate Jacobian for poseReceiver
          if (H1) *H1 = (gtsam::Matrix16() << jacobian * Hint1_P).finished(); //pose1
          if (H2) *H2 = (gtsam::Matrix13() << jacobian * Hint2_P).finished(); //vel1
          if (H4) *H4 = (gtsam::Matrix16() << jacobian * Hint3_P).finished(); //pose2
          if (H5) *H5 = (gtsam::Matrix13() << jacobian * Hint4_P).finished(); //vel2
        }

        if (H3) *H3 = (gtsam::Matrix11() << lambda_).finished(); //ddAmbiguity

        return (gtsam::Vector1() << (rangeRM - rangeBM) - (rangeRI - rangeBI) +
                                    lambda_ * (ddambiguity_j(nAmbiguity_) - ddphi_)).finished();
      }

      /** return the measured */

      double measured() const {
        return ddphi_;
      }

      /** number of variables attached to this factor */
      size_t size() const {
        return 5;
      }

      /** equals specialized to this factor */
      bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
        const This *e = dynamic_cast<const This *> (&expected);
        return e != NULL && Base::equals(*e, tol)
               && gtsam::equal_with_abs_tol((gtsam::Vector1() << this->ddphi_).finished(),
                                            (gtsam::Vector1() << e->ddphi_).finished(), tol);
      }

      /** print contents */
      void print(const std::string &s = "",
                 const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GPInterDDPseudorange:" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;

      template<class ARCHIVE>
      void serialize(ARCHIVE &ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("GPInterDDPseudorange",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(ddphi_);
      }


    };
  }
}


#endif //ONLINE_FGO_GPINTERPOLATEDDDCARPHAFACTOR_H

namespace gtsam {
  template<>
  struct traits<fgo::factors::GPInterpolatedDDCarPhaFactor> :
          public Testable<fgo::factors::GPInterpolatedDDCarPhaFactor> {
  };
}