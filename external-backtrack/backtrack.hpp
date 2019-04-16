#pragma once
#define PI 3.14159265358979323846f
#include "vector.hpp"
#include "offsets.hpp"
#include <thread>
#include <future>

class c_backtrack
{
public:
	static bool is_ingame()
	{
		return ( g_ptr_memory->read_memory<int>( offsets::dw_clientstate + netvars::i_sigonstate ) == 6 );
	}
	static void send_packet( const bool status )
	{
		const BYTE val = status ? 1 : 0;
		g_ptr_memory->write_memory_protected<BYTE>( engine_module->get_image_base() + offsets::dw_sendpacket, val );
	}
	static int local_player_index()
	{
		return g_ptr_memory->read_memory<int>( offsets::dw_clientstate + netvars::i_local );
	}
	void set_localplr() {
		local_ = c_entity(local_player_index());
	}
	bool can_shoot() const
	{
		const auto next_primary_attack = g_ptr_memory->read_memory< float >( local_.current_weapon_base() + netvars::f_next_primary_attack );
		const auto server_time = local_.tickbase() * get_globalvars().interval_per_tick;

		return ( !( next_primary_attack > server_time ) );
	}
	static usercmd_t fix_cmd(usercmd_t current, const usercmd_t old) {
		current.m_vecViewAngles = old.m_vecViewAngles;
		current.m_vecAimDirection = old.m_vecAimDirection;
		current.m_flForwardmove = old.m_flForwardmove;
		current.m_flSidemove = old.m_flSidemove;
		current.m_flUpmove = old.m_flUpmove;
		current.m_iButtons = old.m_iButtons;
		current.m_siMouseDx = old.m_siMouseDx;
		current.m_siMouseDy = old.m_siMouseDy;

		return current;
	}

	void do_backtrack()
	{
		if (!can_shoot())
			return;

		std::shared_future<float> future_simtime = std::async(std::launch::async, &c_backtrack::best_simtime, this);

		const auto current_sequence_number = g_ptr_memory->read_memory<int>(offsets::dw_clientstate + offsets::dw_last_outgoing_command) + 1;

		send_packet(false);

		const auto input = g_ptr_memory->read_memory<input_t>(client_module->get_image_base() + offsets::dw_input);

		const auto ptr_usercmd = input.m_pCommands + (current_sequence_number % 150) * sizeof(usercmd_t);
		const auto ptr_verified_usercmd = input.m_pVerifiedCommands + (current_sequence_number % 150) * sizeof(verified_usercmd_t);

		const auto ptr_old_usercmd = input.m_pCommands + ((current_sequence_number - 1) % 150) * sizeof(usercmd_t);

		while (g_ptr_memory->read_memory<int32_t>(ptr_usercmd + 0x4) < current_sequence_number) {
			std::this_thread::yield();
		}

		auto old_usercmd = g_ptr_memory->read_memory<usercmd_t>(ptr_old_usercmd);
		auto usercmd = g_ptr_memory->read_memory<usercmd_t>(ptr_usercmd);

		usercmd = fix_cmd(usercmd, old_usercmd);

		while (future_simtime.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout) {

		}

		if (future_simtime.get() == -1) {
			send_packet(true);
			return;
		}

		usercmd.m_iButtons |= IN_ATTACK;
		usercmd.m_iTickCount = time_to_ticks(future_simtime.get());

		g_ptr_memory->write_memory<usercmd_t>(ptr_usercmd, usercmd);
		g_ptr_memory->write_memory<usercmd_t>(ptr_verified_usercmd, usercmd);

		send_packet(true);

	}
	float best_simtime()
	{
		if (best_target_ == -1)
			return -1;

		auto temp = FLT_MAX;
		float best = -1;
		for (auto t = 0; t < 12; ++t) {

			if (max_backtrack_ms_ > 0 && !is_valid_tick(time_to_ticks(backtrack_positions_[best_target_][t].simtime)))
				continue;

			const auto fov_distance = get_fov(get_viewangles() + (local_.punch_angles() * 2.0f), local_.eye_postition(), backtrack_positions_[best_target_][t].hitboxpos);

			if (temp > fov_distance && backtrack_positions_[best_target_][t].simtime > local_.simulation_time() - 1) {
				temp = fov_distance;
				best = backtrack_positions_[best_target_][t].simtime;
			}
		}
		return best;
	}
	void update()
	{
		best_target_ = -1;

		const auto local_index = local_player_index();

		auto best_fov = FLT_MAX;
		if( local_.health() < 1 )
			return;

		for( auto i = 0; i < get_globalvars().maxClients; i++ )
		{
			const c_entity entity( i );

			if( i == local_index )
				continue;

			if( entity.dormant() )
				continue;

			if( entity.team() == local_.team() )
				continue;

			if( entity.health() > 0 )
			{
				const auto simtime = entity.simulation_time();
				const auto head_position = entity.bone_position( 8 );

				backtrack_positions_[ i ][ get_globalvars().tickcount % 13 ] = backtrack_data_t{ simtime, head_position };
				const auto fov_distance = get_fov(get_viewangles() - (local_.punch_angles() * 2.0f), local_.eye_postition(), head_position);

				if( best_fov > fov_distance )
				{
					best_fov = fov_distance;
					best_target_ = i;
				}
			}
		}
	}
private:
	static netchannel_t get_netchannel()
	{
		return g_ptr_memory->read_memory<netchannel_t>( g_ptr_memory->read_memory<ptrdiff_t>( offsets::dw_clientstate + netvars::dw_netchannel ) );
	}
	bool is_valid_tick( const int tick ) const
	{
		const auto gvars = get_globalvars();
		const auto delta = gvars.tickcount - tick;
		const auto delta_time = delta * gvars.interval_per_tick;
		const auto max = static_cast< float >( static_cast < float >( max_backtrack_ms_ ) / static_cast < float >( 1000 ) );
		return ( fabs( delta_time ) <= max );

	}
	static double get_nextcmdtime()
	{
		return g_ptr_memory->read_memory<double>( offsets::dw_clientstate + netvars::dw_next_cmd );
	}
	static globalvars_t get_globalvars()
	{
		return g_ptr_memory->read_memory<globalvars_t>( engine_module->get_image_base() + offsets::dw_globalvars );
	}
	static void set_tick_count( const int tick )
	{
		g_ptr_memory->write_memory<int>( engine_module->get_image_base() + offsets::dw_globalvars + 0x1C, tick );
	}
	static Vector get_viewangles()
	{
		return g_ptr_memory->read_memory<Vector>( offsets::dw_clientstate + netvars::vec_view_angles );
	}
	static int time_to_ticks( float time )
	{
		return static_cast< int >( static_cast< float >( 0.5f ) + static_cast< float >( time ) / static_cast< float >( get_globalvars().interval_per_tick ) );
	}
	static Vector angle_vector( const Vector in )
	{
		const auto sin_y = sin( in.y / 180.f * static_cast< float >( PI ) );
		const auto sin_x = sin( in.x / 180.f * static_cast< float >( PI ) );

		const auto cos_y = cos( in.y / 180.f * static_cast< float >( PI ) );
		const auto cos_x = cos( in.x / 180.f* static_cast< float >( PI ) );

		return Vector( cos_x*cos_y, cos_x*sin_y, -sin_x );
	}
	/*static float distance_point_to_line( Vector point, Vector line, Vector direction )
	{
		auto point_direction = point - line;

		const auto temp = point_direction.Dot( direction ) / ( direction.x*direction.x + direction.y*direction.y + direction.z*direction.z );
		if( temp < 0.000001f )
			return FLT_MAX;

		const auto perpen_point = line + ( direction * temp );

		return ( point - perpen_point ).Length();
	}*/
	static void make_vector(Vector angle, Vector& vector) {
		const auto pitch = float(angle[0] * PI / 180);
		const auto yaw = float(angle[1] * PI / 180);
		const auto tmp = float(cos(pitch));
		vector[0] = float(-tmp * -cos(yaw));
		vector[1] = float(sin(yaw)*tmp);
		vector[2] = float(-sin(pitch));
	}
	static Vector calc_angle(Vector src, Vector dst) {
		Vector q_angles;
		auto delta = Vector((src[0] - dst[0]), (src[1] - dst[1]), (src[2] - dst[2]));
		const double hyp = sqrtf(delta[0] * delta[0] + delta[1] * delta[1]);
		q_angles[0] = static_cast<float>(atan(delta[2] / hyp) * (180.0 / PI));
		q_angles[1] = static_cast<float>(atan(delta[1] / delta[0]) * (180.0 / PI));
		q_angles[2] = 0.f;
		if (delta[0] >= 0.f)
			q_angles[1] += 180.f;

		return q_angles;
	}

	static float get_fov(Vector q_angles, Vector v_source, Vector v_destination) {

		Vector aim;

		auto ang = calc_angle(v_source, v_destination);
		make_vector(q_angles, aim);
		make_vector(ang, ang);

		const auto mag_s = sqrt((aim[0] * aim[0]) + (aim[1] * aim[1]) + (aim[2] * aim[2]));
		const auto mag_d = sqrt((aim[0] * aim[0]) + (aim[1] * aim[1]) + (aim[2] * aim[2]));
		const auto u_dot_v = aim[0] * ang[0] + aim[1] * ang[1] + aim[2] * ang[2];

		double fov = acos(u_dot_v / (mag_s*mag_d)) * (180.f / PI);


		fov *= 1.4;
		const auto x_dist = abs(v_source[0] - v_destination[0]);
		const auto y_dist = abs(v_source[1] - v_destination[1]);
		auto distance = sqrt((x_dist * x_dist) + (y_dist * y_dist));

		distance /= 650.f;

		if (distance < 0.7f)
			distance = 0.7f;

		if (distance > 6.5)
			distance = 6.5;

		fov *= distance;

		return static_cast<float>(fov);
	}

	backtrack_data_t backtrack_positions_[ 64 ][ 12 ] = { 0.0f, Vector(0.0f, 0.0f, 0.0f) };
	int best_target_ = -1;
	int max_backtrack_ms_ = 200;
	c_entity local_ = c_entity(-1);
};

extern std::unique_ptr<c_backtrack> g_ptr_backtrack;