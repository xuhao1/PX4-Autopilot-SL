#include "swashplateless.hpp"

#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

SwashplatelessMixer::SwashplatelessMixer() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::sl_mixer),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	for (int i = 0; i < MAX_SL_MOTOR_NUM; i++) {
		sl_mixer[i] = new SLMixer();
	}
	parameters_updated();
}

SwashplatelessMixer::~SwashplatelessMixer()
{
	for (int i = 0; i < MAX_SL_MOTOR_NUM; i++) {
		delete sl_mixer[i];
	}
	perf_free(_loop_perf);
}

bool
SwashplatelessMixer::init()
{
	if (!_sensor_motor_encoder_sub.registerCallback()) {
		PX4_ERR("sensor_motor_encoder callback registration failed!");
		return false;
	}
	for (int i = 0; i < 8; i++) {
		v_actuator_controls_output.control[i] = 0;
	}
	return true;
}

void
SwashplatelessMixer::parameters_updated()
{
	float amp = _param_sl_gain.get();
	for (int i = 0; i < MAX_SL_MOTOR_NUM; i++) {
		sl_mixer[i]->set_amp(amp);
		sl_mixer[i]->set_amp_force(_param_gain_force.get());
		sl_mixer[i]->set_amp_torque(_param_gain_torque.get());
	}
	calibration_offset[0] = _param_calib_0.get()*M_DEG_TO_RAD_F;
	calibration_offset[1] = _param_calib_1.get()*M_DEG_TO_RAD_F;
	sl_mixer[0]->set_direction(_param_dir_0.get());
	sl_mixer[1]->set_direction(_param_dir_1.get());
	sl_mixer[0]->set_phrase_offset(_param_phase_offset_0.get());
	sl_mixer[1]->set_phrase_offset(_param_phase_offset_1.get());
	sl_mixer[0]->set_prop_pos(_param_prop_pos_0.get());
	sl_mixer[1]->set_prop_pos(_param_prop_pos_1.get());
	_debug_mode = (DebugMode) _param_debug_mode.get();
}

void SwashplatelessMixer::Run()
{
	if (should_exit()) {
		_sensor_motor_encoder_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	sensor_motor_encoder_s v_motor_enc;
	if (_sensor_motor_encoder_sub.update(&v_motor_enc)) {
		uint64_t ts = hrt_absolute_time();
		if (first_mix_time == 0) {
			first_mix_time = ts;
		} else {
			mix_freq = (float)mix_count/(float)(ts - first_mix_time)*1000000.0f;
		}

		uint8_t motor_id = v_motor_enc.motor_id;
		float calibed_angle = v_motor_enc.motor_abs_angle - calibration_offset[motor_id];
		if (_actuators_sub.updated()) {
			_actuators_sub.copy(&v_actuator_controls);
		}
		if (_manual_control_setpoint_sub.updated()) {
			_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
		}
		Vector3f thrust{0, 0, v_actuator_controls.control[3]};
		Vector3f torque{v_actuator_controls.control[0], v_actuator_controls.control[1], v_actuator_controls.control[2]};
		if (_debug_mode > DEBUG_DISABLE) {
			if (_debug_mode == DEBUG_PASSTHROUGH_RC) {
				thrust(2) = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);
				torque(0) = math::constrain(_manual_control_setpoint.x, -1.0f, 1.0f);
				torque(1) = math::constrain(_manual_control_setpoint.y, -1.0f, 1.0f);
				torque(2) = math::constrain(_manual_control_setpoint.r, -1.0f, 1.0f);
			} else if (_debug_mode == DEBUG_ROLL_TORQUE) {
				torque(0) = 1.0;
				torque(1) = 0.0;
				torque(2) = 0.0;
			} if (_debug_mode == DEBUG_PITCH_TORQUE) {
				torque(0) = 0.0;
				torque(1) = 1.0;
				torque(2) = 0.0;
			}
		}
		float output = sl_mixer[motor_id]->mix(calibed_angle, v_motor_enc.motor_rpm, thrust, torque);
		if (_debug_mode == DEBUG_THRUST_ONLY) {
		    output = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);
		}
		v_actuator_controls_output.control[motor_id] = output;
		v_actuator_controls_output.timestamp = ts;
		_actuators_pub.publish(v_actuator_controls_output);
		mix_count += 1;
		// sl_mixer[motor_id]->print_status();
		// PX4_INFO("motor %d ctrl:%.1f ABS pos:%.1f caled %.1f rpm:%.1f actuator: %.2f %.2f %.2f %.2f",
		// 	v_motor_enc.motor_id, (double) output, (double) (v_motor_enc.motor_abs_angle*M_RAD_TO_DEG_F),
		// 	(double) (calibed_angle*M_RAD_TO_DEG_F), (double) v_motor_enc.motor_rpm, (double) v_actuator_controls.control[0],
		// 	(double) v_actuator_controls.control[1], (double) v_actuator_controls.control[2], (double) v_actuator_controls.control[3]);
	}
	perf_end(_loop_perf);
}

int SwashplatelessMixer::task_spawn(int argc, char *argv[])
{
	SwashplatelessMixer *instance = new SwashplatelessMixer();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SwashplatelessMixer::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SwashplatelessMixer::print_status()
{
	PX4_INFO("SwashplatelessMixer total freq %.1f", (double) mix_freq);
	for (int i = 0; i < MAX_SL_MOTOR_NUM; i++) {
		PX4_INFO("Mixer %d:", i);
		sl_mixer[i]->print_status();
	}
	//Print loop perf
	perf_print_counter(_loop_perf);
	return 0;
}

int SwashplatelessMixer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Mixer for swashplateless helicoper

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("swashplateless_mixer", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int swashplateless_mixer_main(int argc, char *argv[])
{
	return SwashplatelessMixer::main(argc, argv);
}
