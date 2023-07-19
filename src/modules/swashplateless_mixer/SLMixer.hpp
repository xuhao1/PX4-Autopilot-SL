#pragma once

#include <matrix/matrix/math.hpp>
#include <matrix/matrix/Matrix.hpp>

using matrix::Vector3f;
class SLMixer {
	float _dir = 1; //1 clockwise, -1 counterclockwise
	float _amp = 0.2; //amplitude of the sin wave
	float _phrase_offset = 0; //phrase offset with accuacy calib
	float _prop_pos = 0.06; //+1 above fc, -1 below fc. bigger the output response to torque smaller.
	float _amp_force = 1.0f;//amplitude of the TPP response to side forces
	float _amp_torque = 1.0f;//amplitude of the TPP response to torque

	float n_x{0};
	float n_y{0};
	float cyclic_pharse{0};
	float cyclic_amp{0};
	float output{0};
	Vector3f target_thrust;
	Vector3f target_torque;
	float motor_angle{0};
public:
	SLMixer();
	~SLMixer();
	float mix(float calibed_angle, float rpm, const Vector3f & thrust, const Vector3f &torque);
	void set_direction(int direction);
	void set_amp(float amp);
	void set_amp_force(float amp_force);
	void set_prop_pos(float prop_pos);
	void set_phrase_offset(float phrase_offset);
	void set_amp_torque(float amp_torque);
	void print_status();
};
