#pragma once

#include <lib/mixer/MixerBase/Mixer.hpp> // Airmode
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_motor_encoder.h>
#include <uORB/topics/actuator_controls.h>
#include "SLMixer.hpp"

#define MAX_SL_MOTOR_NUM 2
using namespace time_literals;

/**
 * SwashplatelessMixer app start / stop handling function
 */
extern "C" __EXPORT int swashplateless_mixer_main(int argc, char *argv[]);

class SwashplatelessMixer : public ModuleBase<SwashplatelessMixer>, public ModuleParams,
	public px4::WorkItem
{
public:
	SwashplatelessMixer();
	~SwashplatelessMixer() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();
	SLMixer * sl_mixer[MAX_SL_MOTOR_NUM] = {nullptr};
	float calibration_offset[MAX_SL_MOTOR_NUM] = {0};

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _actuators_sub{ORB_ID(actuator_controls_0)};	/**< actuator controls subscription */
	uORB::SubscriptionCallbackWorkItem  _sensor_motor_encoder_sub{this, ORB_ID(sensor_motor_encoder)};	/**< manual control setpoint subscription */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<actuator_controls_s>		_actuators_pub{ORB_ID(actuator_controls_5)}; //Publish to group 2, and pass mixer run on group 5

	actuator_controls_s v_actuator_controls{};
	actuator_controls_s v_actuator_controls_output{};
	manual_control_setpoint_s _manual_control_setpoint{};
	uint64_t first_mix_time {0};
	int64_t mix_count {0};
	float mix_freq {0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */
	enum DebugMode {
		DEBUG_DISABLE = 0,
		DEBUG_PASSTHROUGH_RC = 1,
		DEBUG_ROLL_TORQUE = 2,
		DEBUG_PITCH_TORQUE = 3,
		DEBUG_THRUST_ONLY = 4
	} _debug_mode;


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SL_GAIN>) _param_sl_gain,
		(ParamFloat<px4::params::SL_CALIB_0>) _param_calib_0,
		(ParamFloat<px4::params::SL_CALIB_1>) _param_calib_1,
		(ParamFloat<px4::params::SL_PHASE_OFF_0>) _param_phase_offset_0,
		(ParamFloat<px4::params::SL_PHASE_OFF_1>) _param_phase_offset_1,
		(ParamInt<px4::params::SL_DIR_0>) _param_dir_0,
		(ParamInt<px4::params::SL_DIR_1>) _param_dir_1,
		(ParamFloat<px4::params::SL_GAIN_FORCE>) _param_gain_force,
		(ParamFloat<px4::params::SL_GAIN_TORQUE>) _param_gain_torque,
		(ParamFloat<px4::params::SL_PROP_POS_0>) _param_prop_pos_0,
		(ParamFloat<px4::params::SL_PROP_POS_1>) _param_prop_pos_1,
		(ParamInt<px4::params::SL_DEBUG_MOD>) _param_debug_mode
	)
};

