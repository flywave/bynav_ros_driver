#ifndef GNSS_SPS_HPP_
#define GNSS_SPS_HPP_

#include <bynav_gps_driver/gnss_constant.h>
#include <eigen3/Eigen/Dense>

namespace bynav_gps_driver {

void filter_L1(const std::vector<ObsPtr> &obs,
               const std::vector<EphemBasePtr> &ephems,
               std::vector<ObsPtr> &L1_obs,
               std::vector<EphemBasePtr> &L1_ephems);

std::vector<SatStatePtr> sat_states(const std::vector<ObsPtr> &obs,
                                    const std::vector<EphemBasePtr> &ephems);

void psr_res(const Eigen::Matrix<double, 7, 1> &rcv_state,
             const std::vector<ObsPtr> &obs,
             const std::vector<SatStatePtr> &all_sv_states,
             const std::vector<double> &iono_params, Eigen::VectorXd &res,
             Eigen::MatrixXd &J, std::vector<Eigen::Vector2d> &atmos_delay,
             std::vector<Eigen::Vector2d> &all_sv_azel);

Eigen::Matrix<double, 7, 1> psr_pos(const std::vector<ObsPtr> &obs,
                                    const std::vector<EphemBasePtr> &ephems,
                                    const std::vector<double> &iono_params);

void dopp_res(const Eigen::Matrix<double, 4, 1> &rcv_state,
              const Eigen::Vector3d &rcv_ecef, const std::vector<ObsPtr> &obs,
              const std::vector<SatStatePtr> &all_sv_states,
              Eigen::VectorXd &res, Eigen::MatrixXd &J);

Eigen::Matrix<double, 4, 1> dopp_vel(const std::vector<ObsPtr> &obs,
                                     const std::vector<EphemBasePtr> &ephems,
                                     Eigen::Vector3d &ref_ecef);
} // namespace bynav_gps_driver

#endif