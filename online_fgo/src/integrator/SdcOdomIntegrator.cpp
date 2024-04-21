//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//


#include "integrator/SdcOdomIntegrator.h"

namespace fgo::integrator
{

    void SdcOdomIntegrator::initialize(rclcpp::Node &node, graph::GraphBase &graphPtr, const std::string& integratorName, bool isPrimarySensor) {
      integratorName_ = integratorName;
      isPrimarySensor_ = isPrimarySensor;
      rosNodePtr_ = &node;
      graphPtr_ = &graphPtr;
      this->initIntegratorBaseParams();

      paramPtr_ = std::make_shared<IntegratorSdcOdomParams>(integratorBaseParamPtr_);
      preIntergratorParams_ = std::make_shared<fgo::factor::VelocityPreintegrationParams>();

      integratorBaseParamPtr_->noiseModelVelocity = data_types::NoiseModel::GAUSSIAN;
      integratorBaseParamPtr_->robustParamVelocity = 0.5;
      integratorBaseParamPtr_->velocityFrame = fgo::factor::MeasurementFrame::BODY;

      bufferOdomVel_.resize_buffer(1000);

      subSdcOdometer_ = rosNodePtr_->create_subscription<sdc_msgs::msg::Odometry>("/odometry",
                                                                               rclcpp::SystemDefaultsQoS(),
                                                                               [this](const sdc_msgs::msg::Odometry::ConstSharedPtr msg) -> void
                                                                                {
        // static double sumVelocity = 0;
        // static uint calcZeroVelocityCounter = 1;
        const auto ts = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        auto data = OdomVelItem();
        data.timestamp = ts.seconds();
        data.vel_x_odom = 0.5 * (msg->wheel_velocities.rear_left + msg->wheel_velocities.rear_right);
        bufferOdomVel_.update_buffer(data, ts);

      //   sumVelocity += msg->vel_correvit;
      //   if(calcZeroVelocityCounter > 6)
      //   {
      //     const auto avgVelocity = abs(sumVelocity) / calcZeroVelocityCounter;

      //     if(avgVelocity < paramPtr_->zeroVelocityThreshold)
      //     {
      //       RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), integratorName_ << " reported near zero velocity: " << data.angle_correvit);
      //       bufferCorrevit_.clean();
      //       zeroVelocity_ = true;
      //     }
      //     else
      //       zeroVelocity_ = false;
      //     calcZeroVelocityCounter = 0;
      //     sumVelocity = 0.;
      //  }
      //  calcZeroVelocityCounter ++;

                                                                               });
    }

    bool
    SdcOdomIntegrator::factorize(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                                  const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
                                  const solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                                  std::vector<std::pair<rclcpp::Time, fgo::data_types::State>>& timePredStates,
                                  gtsam::Values &values,
                                  solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
                                  gtsam::KeyVector& relatedKeys) {

      // static auto firstCallTime = rosNodePtr_->now();
      // static auto fixedVelVar = paramPtr_->velVar * paramPtr_->velVarScale;

      if(!paramPtr_->integrateVelocity)
      {
        return true;
      }

      // if((rosNodePtr_->now() - firstCallTime).seconds() < paramPtr_->factorizeDelay)
      // {
      //   RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << "Correvit: delaying ");
      //   bufferCorrevit_.clean();
      //   return true;
      // }

      static gtsam::Key pose_key_j, vel_key_j, bias_key_j, omega_key_j,
             pose_key_i, vel_key_i,  bias_key_i, omega_key_i,
             pose_key_sync, vel_key_sync,  bias_key_sync;
      static uint64_t last_key_index = 0;
      static boost::circular_buffer<OdomVelItem> restOdom(1000);

      auto dataOdom = bufferOdomVel_.get_all_buffer_and_clean();

      if(!restOdom.empty())
      {
        dataOdom.insert(dataOdom.begin(), restOdom.begin(), restOdom.end());
        restOdom.clear();
      }

      /*
      if (integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ)
      {
        interpolator_ = std::make_shared<fgo::model::GPWNOJInterpolatorPose3>(
            gtsam::noiseModel::Gaussian::Covariance(integratorParamPtr_->QcGPInterpolator * gtsam::I_6x6), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOA)
      {
        interpolator_ = std::make_shared<fgo::model::GPWNOAInterpolatorPose3>(
            gtsam::noiseModel::Gaussian::Covariance(integratorParamPtr_->QcGPInterpolator * gtsam::I_6x6), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else
      {
        RCLCPP_WARN(appPtr_->get_logger(), "NO gpType chosen. Please choose.");
        return false;
      }*/

      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << "SdcOdom: integrating with " << dataOdom.size() << " sdc odom data");

      //std::vector<StateMeasSyncResult> stateMeasSyncResults_;

      auto dataOdomIter = dataOdom.begin();
      while(dataOdomIter != dataOdom.end())
      {
        const auto corrected_time = dataOdomIter->timestamp  - 0.005;
        const auto vel = (gtsam::Vector3() << dataOdomIter->vel_x_odom, 0, 0).finished();

        //stateMeasSyncResults_.emplace_back(syncResult);

        // if(dataOdomIter->vel_x_correvit < paramPtr_->nearZeroVelocityThreshold)
        // {
        //   RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "[Correvit] near zero velocity with vx: " << dataCorrevitIter->vel_x_correvit  <<" not integrating ... ");
        //   dataOdomIter ++;
        //   continue;
        // }

        auto syncResult = findStateForMeasurement(currentKeyIndexTimestampMap, corrected_time, paramPtr_);
        if (syncResult.status == StateMeasSyncStatus::DROPPED ||syncResult.status == StateMeasSyncStatus::INTERPOLATED )
        {
          dataOdomIter ++;
          continue;
        }

        //RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Found State I: " << std::fixed <<
        //                                                                syncResult.keyIndexI << " : " << gtsam::symbolIndex(syncResult.keyIndexI) << " at: " << syncResult.timestampI << " and J: "
        //                                                                << syncResult.keyIndexJ << " : " << gtsam::symbolIndex(syncResult.keyIndexJ) << " at: " << syncResult.timestampJ
        //                                                                << " DurationToI: " << syncResult.durationFromStateI);
        const auto currentPredState = timePredStates.back().second; //graph::querryCurrentPredictedState(timePredStates, timeSync);

        if(paramPtr_->verbose) {
          std::cout << "Sdc Odom Measured X: " << dataOdomIter->vel_x_odom << std::endl;
        }

        pose_key_i = X(syncResult.keyIndexI);
        vel_key_i = V(syncResult.keyIndexI);
        bias_key_i = B(syncResult.keyIndexI);
        omega_key_i = W(syncResult.keyIndexI);

        pose_key_j = X(syncResult.keyIndexJ);
        vel_key_j = V(syncResult.keyIndexJ);
        bias_key_j = B(syncResult.keyIndexJ);
        omega_key_j = W(syncResult.keyIndexJ);

        if (!syncResult.stateJExist()){
          RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "GP Odom: NO state J found !!! ");
        }
        double timeSync;
        if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I || syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J)
        {

          // now we found a state which is synchronized with the GNSS obs
          if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
            pose_key_sync = pose_key_i;
            vel_key_sync = vel_key_i;
            bias_key_sync = bias_key_i;
            timeSync = syncResult.timestampI;
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
           //                    "[Correvit] Found time synchronized state at I " << gtsam::symbolIndex(pose_key_sync)
           //                                                                    <<
           //                                                                    " with time difference: "
            //                                                                   << syncResult.durationFromStateI);
          }
          else {
            pose_key_sync = pose_key_j;
            vel_key_sync = vel_key_j;
            bias_key_sync = bias_key_j;
            timeSync = syncResult.timestampJ;
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
            //                   "[Correvit] Found time synchronized state at J " << gtsam::symbolIndex(pose_key_sync)
             //                                                                  <<
             //                                                                  " with time difference: "
             //                                                                  << syncResult.durationFromStateI);
          }
          const auto [fountGyro, this_gyro] = findOmegaToMeasurement(corrected_time, timestampGyroMap);
          const auto this_gyro_unbiased = currentPredState.imuBias.correctGyroscope(this_gyro);
          //this_gyro = currentPredState.imuBias.correctGyroscope(this_gyro);
          //std::cout << "Correvit currentKey: " << gtsam::symbolIndex(pose_key_sync) << std::endl;
          //std::cout << "Correvit lastKey: " << last_key_index << std::endl;
          //std::cout << "Correvit found omega: " << this_gyro << std::endl;
          //std::cout << "Correvit found omega: " << this_gyro_unbiased << std::endl;
          if(last_key_index == gtsam::symbolIndex(pose_key_sync))
          {
            dataOdomIter ++;
            //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
            //                   "[Correvit] Not integrating of  " << last_key_index);
            continue;
          }
          //RCLCPP_WARN_STREAM(appPtr_->get_logger(),
         //                    "[Correvit] Integrating of  " << last_key_index);
          this->addNavVelocityFactor(pose_key_sync, vel_key_sync, vel, 0.01 * gtsam::Vector3::Ones(), 
                                     this_gyro_unbiased,
                                     paramPtr_->transIMUToOdom, 
                                     fgo::factor::VelocityType::VEL3D);

        }
/*
        else if (syncResult.status == StateMeasSyncStatus::INTERPOLATED && integratorParamPtr_->useGPInterpolatedFactor)
        {
          RCLCPP_WARN_STREAM(appPtr_->get_logger(),
                             "[Correvit] Found not synchronized between " << syncResult.keyIndexI << " and " << syncResult.keyIndexJ <<
                                                                          " with time difference: "
                                                                         << syncResult.durationFromStateI);

          const double delta_t = syncResult.timestampJ - syncResult.timestampI;
          const double taui = syncResult.durationFromStateI;
          //recalculate interpolator // set up interpolator
          //corrected_time_gnss_meas - timestampI;
          RCLCPP_INFO_STREAM(appPtr_->get_logger(), integratorName_ + ": GP delta: " << delta_t << " tau: " << taui);
          //ALSO NEEDED FOR DDCP TDCP, AND THATS IN SYNCED CASE AND IN NOT SYNCED CASE

          if(integratorParamPtr_->gpType == fgo::data_types::GPModelType::WNOJ) {
            const auto [foundAccI, accI, foundAccJ, accJ] = findAccelerationToState(syncResult.keyIndexI, stateIDAccMap);

            if(!foundAccI || !foundAccJ) {
              dataCorrevitIter ++;
              continue;
            }

            interpolator_->recalculate(delta_t, taui, accI, accJ);
          }
          else
            interpolator_->recalculate(delta_t, taui);
          RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Integrating GP interpolated Correvit ...");
          this->addGPInterpolatedNavVelocityFactor(pose_key_i, vel_key_i, omega_key_i,
                                                   pose_key_j, vel_key_j, omega_key_j,
                                                   vel, fixedVelVar,
                                                   integratorParamPtr_->transIMUToCorrevit, interpolator_, integratorParamPtr_->velocityType);
        }*/
        else if(syncResult.status == StateMeasSyncStatus::CACHED)
        {
          restOdom.push_back(*dataOdomIter);
        }
        last_key_index = gtsam::symbolIndex(pose_key_sync);
        dataOdomIter ++;
      }

      return true;
    }

    bool SdcOdomIntegrator::fetchResult(const gtsam::Values &result, const gtsam::Marginals &martinals,
                                         const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                         data_types::State &optState) {
      return true;
    }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::integrator::SdcOdomIntegrator, fgo::integrator::IntegratorBase)