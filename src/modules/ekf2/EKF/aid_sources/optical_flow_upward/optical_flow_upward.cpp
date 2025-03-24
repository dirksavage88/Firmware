/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ekf.h"

#include "aid_sources/optical_flow_upward/optical_flow_upward.hpp"

#include "ekf_derivation/generated/compute_body_vel_innov_var_h.h"
#include "ekf_derivation/generated/compute_body_vel_y_innov_var.h"
#include "ekf_derivation/generated/compute_body_vel_z_innov_var.h"

#if defined(CONFIG_EKF2_OPTICAL_FLOW_UPWARD) && defined(MODULE_NAME)

void OpticalFlowUpward::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_sensor_optical_flow_sub.updated()) {

		sensor_optical_flow_s sensor_optical_flow{};
		_sensor_optical_flow_sub.copy(&sensor_optical_flow);

		float range_m = NAN;

		{
			distance_sensor_s distance_sensor{};

			if (_distance_sensor_sub.copy(&distance_sensor)) {
				if (PX4_ISFINITE(distance_sensor.current_distance)) {
					range_m = distance_sensor.current_distance;
				}
			}
		}

		// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
		// is produced by a RH rotation of the image about the sensor axis.
		Vector2f flow_xy_rad = Vector2f(-sensor_optical_flow.pixel_flow[0], -sensor_optical_flow.pixel_flow[1]);
		Vector3f gyro_integral = Vector3f(-sensor_optical_flow.delta_angle[0], -sensor_optical_flow.delta_angle[1],
						  -sensor_optical_flow.delta_angle[2]);

		const float flow_dt = 1e-6f * (float)sensor_optical_flow.integration_timespan_us;

		// correct timestamp to midpoint of integration interval as the data is converted to rates
		const int64_t time_us = sensor_optical_flow.timestamp_sample
					- sensor_optical_flow.integration_timespan_us / 2
					- static_cast<int64_t>(_param_ekf2_ofu_delay.get() * 1000);

		if (time_us > 0 && PX4_ISFINITE(range_m)) {
			OpticalFlowUpwardSample sample{
				.time_us = (uint64_t)time_us,
				.flow_dt = flow_dt,
				.flow_xy_rad = flow_xy_rad,
				.gyro_integral = gyro_integral,
				.range_m = range_m,
				.flow_quality = sensor_optical_flow.quality
			};

			_ringbuffer.push(sample);

			_time_last_buffer_push = imu_delayed.time_us;
		}
	}

#endif // MODULE_NAME

	OpticalFlowUpwardSample sample;

	if (_ringbuffer.pop_first_older_than(imu_delayed.time_us, &sample)) {

		if (!_param_ekf2_ofu_ctrl.get()) {
			return;
		}

		estimator_aid_source3d_s &aid_src = _aid_src_optical_flow_upward;

		// compensate for body motion to give a LOS rate
		const Vector2f flow_compensated_xy_rad = sample.flow_xy_rad - sample.gyro_integral.xy();

		// vel body
		Vector3f vel_sensor;
		vel_sensor(0) = - sample.range_m * flow_compensated_xy_rad(1) / sample.flow_dt;
		vel_sensor(1) =   sample.range_m * flow_compensated_xy_rad(0) / sample.flow_dt;
		vel_sensor(2) = 0.f;

		const matrix::Dcmf R_to_body(matrix::Eulerf(math::radians(180.f), 0.f, 0.f));

		const Vector3f vel_body = R_to_body * vel_sensor;

		const Vector3f ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - ekf.getGyroBias());

		// const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - ekf._state.gyro_bias;
		// Vector3f position_offset_body = ekf._params.ev_pos_body - ekf._params.imu_pos_body;
		// _velocity_offset_body = angular_velocity % position_offset_body;

		//float quality_ratio = static_cast<float>(sample.flow_quality) / 255.f;
		const float R = math::max(_param_ekf2_ofu_noise.get(), 0.01f);

		const Vector3f measurement{vel_body};
		const Vector3f measurement_var{R, R, R};

		// vel NE
		// Vector3f vel_body;
		Ekf::VectorState H[3];
		Vector3f innov_var;
		Vector3f innov = ekf._R_to_earth.transpose() * ekf._state.vel - vel_body;
		const auto state_vector = ekf._state.vector();
		sym::ComputeBodyVelInnovVarH(state_vector, ekf.P, measurement_var, &innov_var, &H[0], &H[1], &H[2]);

		float innovation_gate = 1.f;

		ekf.updateAidSourceStatus(aid_src,
					  sample.time_us,        // sample timestamp
					  vel_body,              // observation
					  measurement_var,       // observation variance
					  innov,                 // innovation
					  innov_var,             // innovation variance
					  innovation_gate);      // innovation gate

		// && ekf.control_status_flags().yaw_align;
		const bool continuing_conditions = ekf.control_status_flags().tilt_align
						   && sample.flow_quality > 10
						   && PX4_ISFINITE(sample.range_m);

		const bool starting_conditions = continuing_conditions
						 && (sample.flow_quality > 100);

		switch (_state) {
		case State::stopped:

		/* FALLTHROUGH */
		case State::starting:
			if (starting_conditions) {
				_state = State::starting;

				if ((_test_ratio_filtered > 0.f) && (_test_ratio_filtered < 0.5f)) {
					// TODO: conservative start

					bool fused = true;
					bool reset = false;

					if (fused || reset) {
						ekf.enableControlStatusOpticalFlowUpward();
						// _reset_counters.lat_lon = sample.lat_lon_reset_counter;
						_state = State::active;
					}

				}
			}

			break;

		case State::active:
			if (continuing_conditions) {

				if (!aid_src.innovation_rejected) {
					for (uint8_t index = 0; index <= 2; index++) {
						if (index == 1) {
							sym::ComputeBodyVelYInnovVar(state_vector, ekf.P, measurement_var(index), &aid_src.innovation_variance[index]);

						} else if (index == 2) {
							// skip z
							// sym::ComputeBodyVelZInnovVar(state_vector, P, measurement_var(index), &aid_src.innovation_variance[index]);
						}

						aid_src.innovation[index] = Vector3f(ekf._R_to_earth.transpose().row(index)) * ekf._state.vel - measurement(index);

						Ekf::VectorState Kfusion = ekf.P * H[index] / aid_src.innovation_variance[index];
						ekf.measurementUpdate(Kfusion, H[index], aid_src.observation_variance[index], aid_src.innovation[index]);
					}

					aid_src.fused = true;
					aid_src.time_last_fuse = ekf._time_delayed_us;

					//_time_last_hor_vel_fuse = _time_delayed_us;
					//_time_last_ver_vel_fuse = _time_delayed_us;
				}

				if (isTimedOut(aid_src.time_last_fuse, imu_delayed.time_us, ekf._params.no_aid_timeout_max)) {

					if (ekf.isOnlyActiveSourceOfHorizontalPositionAiding(ekf.control_status_flags().optical_flow_upward)) {
						// TODO
						//ekf.resetAidSourceStatusZeroInnovation(aid_src);

					} else {
						ekf.disableControlStatusOpticalFlowUpward();
						_state = State::stopped;
					}
				}

			} else {
				ekf.disableControlStatusOpticalFlowUpward();
				_state = State::stopped;
			}

			break;

		default:
			break;
		}

#if defined(MODULE_NAME)
		aid_src.timestamp = hrt_absolute_time();
		_estimator_aid_src_optical_flow_upward_pub.publish(aid_src);

		// _estimator_optical_flow_upward_vel_pub.publish(aid_src);

		// publish estimator_optical_flow_upward_vel
		{
			vehicle_optical_flow_vel_s flow_vel{};
			flow_vel.timestamp_sample = sample.time_us;

			vel_sensor.copyTo(flow_vel.vel_sensor);
			vel_body.copyTo(flow_vel.vel_body);

			const matrix::Vector3f vel_ned{ekf._R_to_earth * vel_body};
			vel_ned.copyTo(flow_vel.vel_ne);

			// TODO
			//ekf.getFilteredFlowVelBody().copyTo(flow_vel.vel_body_filtered);
			//ekf.getFilteredFlowVelNE().copyTo(flow_vel.vel_ne_filtered);

			sample.flow_xy_rad.copyTo(flow_vel.flow_rate_uncompensated);
			flow_compensated_xy_rad.copyTo(flow_vel.flow_rate_compensated);

			const Vector3f gyro_rate = sample.gyro_integral / sample.flow_dt;
			gyro_rate.copyTo(flow_vel.gyro_rate);

			// TODO:
			//ekf.getFlowGyroBias().copyTo(flow_vel.gyro_bias);
			ref_body_rate.copyTo(flow_vel.ref_gyro);

			flow_vel.timestamp = hrt_absolute_time();

			_estimator_optical_flow_vel_pub.publish(flow_vel);
		}


		_test_ratio_filtered = math::max(fabsf(aid_src.test_ratio_filtered[0]), fabsf(aid_src.test_ratio_filtered[1]));
#endif // MODULE_NAME

	} else if ((_state != State::stopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
		ekf.disableControlStatusOpticalFlowUpward();
		_state = State::stopped;
		ECL_WARN("Optical flow upward data stopped");
	}
}

#endif // CONFIG_EKF2_OPTICAL_FLOW_UPWARD
