#include "SLMixer.hpp"
#include <mathlib/mathlib.h>

SLMixer::SLMixer() {

}

SLMixer::~SLMixer() {

}

float SLMixer::mix(float calibed_angle, float rpm, const Vector3f & thrust, const Vector3f & torque) {
//Mixer of swashplateless
	target_thrust = thrust;
	target_torque = torque;
	motor_angle = calibed_angle;
	float thrust_z = math::max(thrust(2), 0.05f);
	n_x = -torque(1)*_amp_torque/_prop_pos + thrust(0)*_amp_force/thrust_z;
	n_y = torque(0)*_amp_torque/_prop_pos + thrust(1)*_amp_force/thrust_z;
	cyclic_pharse = atan2f(n_x, n_y);
	cyclic_amp = sqrtf(n_x*n_x + n_y*n_y);
	output = _amp * cyclic_amp * sinf(calibed_angle -_phrase_offset - _dir*cyclic_pharse) + thrust(2) - torque(2)*_dir;
	output = math::max(output, 0.0f);
	return output;
}


void SLMixer::set_direction(int direction) {
	_dir =(float) direction;
}

void SLMixer::set_amp(float amp) {
	_amp = amp;
}

void SLMixer::set_phrase_offset(float phrase_offset) {
	_phrase_offset = phrase_offset;
}

void SLMixer::set_amp_force(float amp_force) {
	_amp_force = amp_force;
}

void SLMixer::set_prop_pos(float prop_pos) {
	_prop_pos = prop_pos;
}

void SLMixer::set_amp_torque(float amp_torque) {
	_amp_torque = amp_torque;
}

void SLMixer::print_status() {
	//print target force and torque
	printf("SLMixer: target_thrust: %f %f %f\n", (double) target_thrust(0), (double) target_thrust(1),
		(double) target_thrust(2));
	printf("SLMixer: target_torque: %f %f %f\n", (double) target_torque(0), (double) target_torque(1),
		(double) target_torque(2));
	PX4_INFO("SLMixer: motor_angle :%.1fdeg n_x: %.3f, n_y: %.3f, cyclic_pharse: %.2fdeg, cyclic_amp: %.3f, output: %.3f",
		(double) motor_angle*M_RAD_TO_DEG, (double) n_x, (double) n_y, (double) cyclic_pharse*M_RAD_TO_DEG, (double) cyclic_amp,
		(double) output);
}
