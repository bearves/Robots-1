﻿

#ifdef UNIX
#include "rtdk.h"
#endif
#ifdef WIN32
#define rt_printf printf
#endif

#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

#include <aris.h>

#include "Robot_Gait.h"
#include "Robot_Base.h"


using namespace Aris::Dynamic;

namespace Robots
{
	auto basicParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void
	{
		Aris::Server::BasicFunctionParam param;

		for (auto &i : params)
		{
			if (i.first == "all")
			{
				std::fill_n(param.active_motor, 18, true);
			}
			else if (i.first == "first")
			{
				std::fill_n(param.active_motor, 18, false);
				std::fill_n(param.active_motor + 0, 3, true);
				std::fill_n(param.active_motor + 6, 3, true);
				std::fill_n(param.active_motor + 12, 3, true);
			}
			else if (i.first == "second")
			{
				std::fill_n(param.active_motor, 18, false);
				std::fill_n(param.active_motor + 3, 3, true);
				std::fill_n(param.active_motor + 9, 3, true);
				std::fill_n(param.active_motor + 15, 3, true);
			}
			else if (i.first == "motor")
			{
				std::fill_n(param.active_motor, 18, false);
				int id = { stoi(i.second) };
				param.active_motor[id] = true;
			}
		}

		msg_out.copyStruct(param);
	}

	auto fakeHomeParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void
	{
		FakeHomeParam param;
		msg_out.copyStruct(param);
	}
	auto fakeHomeGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)->int
	{
		auto &param = static_cast<const FakeHomeParam&>(plan_param);
		auto &robot = static_cast<Robots::RobotBase&>(model);

		auto &controller = Aris::Server::ControlServer::instance().controller();
		for (auto i = 0; i < robot.motionPool().size(); ++i)
		{
			robot.motionPool().at(i).update();
			
			controller.motionAtAbs(i).setPosOffset(static_cast<std::int32_t>(
				robot.motionPool().at(i).motPos()*controller.motionAtAbs(i).pos2countRatio() - param.motion_raw_data->at(i).feedback_pos
				));
		}

		rt_printf("feedback:\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
			, param.motion_raw_data->at(0).feedback_pos
			, param.motion_raw_data->at(1).feedback_pos
			, param.motion_raw_data->at(2).feedback_pos
			, param.motion_raw_data->at(3).feedback_pos
			, param.motion_raw_data->at(4).feedback_pos
			, param.motion_raw_data->at(5).feedback_pos
			, param.motion_raw_data->at(6).feedback_pos
			, param.motion_raw_data->at(7).feedback_pos
			, param.motion_raw_data->at(8).feedback_pos
			, param.motion_raw_data->at(9).feedback_pos
			, param.motion_raw_data->at(10).feedback_pos
			, param.motion_raw_data->at(11).feedback_pos
			, param.motion_raw_data->at(12).feedback_pos
			, param.motion_raw_data->at(13).feedback_pos
			, param.motion_raw_data->at(14).feedback_pos
			, param.motion_raw_data->at(15).feedback_pos
			, param.motion_raw_data->at(16).feedback_pos
			, param.motion_raw_data->at(17).feedback_pos);

		rt_printf("pos_offset:\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
			, controller.motionAtAbs(0).posOffset()
			, controller.motionAtAbs(1).posOffset()
			, controller.motionAtAbs(2).posOffset()
			, controller.motionAtAbs(3).posOffset()
			, controller.motionAtAbs(4).posOffset()
			, controller.motionAtAbs(5).posOffset()
			, controller.motionAtAbs(6).posOffset()
			, controller.motionAtAbs(7).posOffset()
			, controller.motionAtAbs(8).posOffset()
			, controller.motionAtAbs(9).posOffset()
			, controller.motionAtAbs(10).posOffset()
			, controller.motionAtAbs(11).posOffset()
			, controller.motionAtAbs(12).posOffset()
			, controller.motionAtAbs(13).posOffset()
			, controller.motionAtAbs(14).posOffset()
			, controller.motionAtAbs(15).posOffset()
			, controller.motionAtAbs(16).posOffset()
			, controller.motionAtAbs(17).posOffset());

		return 0;
	}

	auto recoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg_out)->void
	{
		RecoverParam param;

		for (auto &i : params)
		{
			if (i.first == "all")
			{
				std::fill_n(param.active_leg, 6, true);
			}
			else if (i.first == "first")
			{
				param.active_leg[0] = true;
				param.active_leg[1] = false;
				param.active_leg[2] = true;
				param.active_leg[3] = false;
				param.active_leg[4] = true;
				param.active_leg[5] = false;
			}
			else if (i.first == "second")
			{
				param.active_leg[0] = false;
				param.active_leg[1] = true;
				param.active_leg[2] = false;
				param.active_leg[3] = true;
				param.active_leg[4] = false;
				param.active_leg[5] = true;
			}
			else if(i.first == "leg")
			{
				auto leg_id = std::stoi(i.second);

				if (leg_id<0 || leg_id>5)throw std::runtime_error("invalide param in parseRecover func");

				std::fill_n(param.active_leg, 6, false);
				param.active_leg[leg_id] = true;
			}
			else
			{
				throw std::runtime_error("unknown param in parseRecover func");
			}
		}

		msg_out.copyStruct(param);
	}
	auto recoverGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase & plan_param)->int
	{
		auto &robot = static_cast<Robots::RobotBase &>(model);
		auto &param = static_cast<const RecoverParam &>(plan_param);

		static double beginPin[18];
		if (param.count == 0)std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);

		const double pe[6]{ 0 };
		robot.SetPeb(pe);
		robot.SetPee(param.alignPee);
		double alignPin[18]{ 0 };
		robot.GetPin(alignPin);

		int leftCount = param.count < param.align_count ? 0 : param.align_count;
		int rightCount = param.count < param.align_count ? param.align_count : param.align_count + param.recover_count;

		double s = -(PI / 2)*cos(PI * (param.count - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

		for (int i = 0; i < 6; ++i)
		{
			if (param.active_leg[i])
			{
				if (param.count < param.align_count)
				{
					for (int j = 0; j < 3; ++j)
					{
						robot.motionPool().at(i * 3 + j).setMotPos(beginPin[i * 3 + j] * (cos(s) + 1) / 2 + alignPin[i * 3 + j] * (1 - cos(s)) / 2);
					}
				}
				else
				{
					double pEE[3];
					for (int j = 0; j < 3; ++j)
					{
						pEE[j] = param.alignPee[i * 3 + j] * (cos(s) + 1) / 2 + param.recoverPee[i * 3 + j] * (1 - cos(s)) / 2;
					}

					robot.pLegs[i]->SetPee(pEE);
				}
			}
		}

		return param.align_count + param.recover_count - param.count - 1;
	}

	auto walkParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)->void
	{
		Robots::WalkParam param;

		for (auto &i : params)
		{
			if (i.first == "totalCount")
			{
				param.totalCount = std::stoi(i.second);
			}
			else if (i.first == "n")
			{
				param.n = stoi(i.second);
			}
			else if (i.first == "distance")
			{
				param.d = stod(i.second);
			}
			else if (i.first == "height")
			{
				param.h = stod(i.second);
			}
			else if (i.first == "alpha")
			{
				param.alpha = stod(i.second);
			}
			else if (i.first == "beta")
			{
				param.beta = stod(i.second);
			}
		}
		msg.copyStruct(param);
	}
	auto walkGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param_in)->int
	{
		auto &robot = static_cast<Robots::RobotBase &>(model);
		auto &param = static_cast<const Robots::WalkParam &>(param_in);
		
		//初始化
		static Aris::Dynamic::FloatMarker beginMak{ robot.ground() };
		static double beginPee[18];

		if (param.count%param.totalCount == 0)
		{
			beginMak.setPrtPm(*robot.body().pm());
			beginMak.update();
			robot.GetPee(beginPee, beginMak);
		}
		
		//以下设置各个阶段的身体的真实初始位置
		const double a = param.alpha;
		const double b = param.beta;
		const double d = param.d;
		const double h = param.h;

		const double front[3]{ -std::sin(a),0,-std::cos(a) };
		const double left[3]{ -std::cos(a),0,std::sin(a) };
		const double up[3]{ 0,1,0 };

		int period_count = param.count%param.totalCount;
		const double s = -(PI / 2)*cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI. 

		double Peb[6], Pee[18];
		std::fill(Peb, Peb + 6, 0);
		std::copy(beginPee, beginPee + 18, Pee);


		double pq_b[7]{ 0,0,0,std::sin(b / 2)*up[0],std::sin(b / 2)*up[1],std::sin(b / 2)*up[2],std::cos(b / 2) };
		double pq_b_half[7]{ 0,0,0,std::sin(b / 4)*up[0],std::sin(b / 4)*up[1],std::sin(b / 4)*up[2],std::cos(b / 4) };
		double pq_b_quad[7]{ 0,0,0,std::sin(b / 8)*up[0],std::sin(b / 8)*up[1],std::sin(b / 8)*up[2],std::cos(b / 8) };
		double pq_b_eighth[7]{ 0,0,0,std::sin(b / 16)*up[0],std::sin(b / 16)*up[1],std::sin(b / 16)*up[2],std::cos(b / 16) };
		double pm_b[16], pm_b_half[16], pm_b_quad[16], pm_b_eighth[16];
		
		s_pq2pm(pq_b, pm_b);
		s_pq2pm(pq_b_half, pm_b_half);
		s_pq2pm(pq_b_quad, pm_b_quad);
		s_pq2pm(pq_b_eighth, pm_b_eighth);

		const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;

		if ((param.count / param.totalCount) == 0)//加速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

				s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
				s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
				s_daxpy(3, d/2, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double body_forward_dir[3], body_left_dir[3];
			s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
			s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * s_interp(param.totalCount, period_count+1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 4))
					+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 4));
			}

			//规划身体姿态
			double s_acc = Aris::Dynamic::acc_even(param.totalCount, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_acc*b / 8)*up[0],std::sin(s_acc*b / 8)*up[1] ,std::sin(s_acc*b / 8)*up[2],std::cos(s_acc*b / 8) };
			double pe[6];
			s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}
		else if ((param.count / param.totalCount) == (param.n * 2 - 1))//减速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

				s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
				s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
				s_daxpy(3, d / 2, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double body_forward_dir[3], body_left_dir[3];
			s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
			s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * s_interp(param.totalCount, period_count+1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, 0)
					+ front[i] * s_interp(param.totalCount, period_count+1, 0, d / 4 / std::cos(b / 4), d/2 / param.totalCount / std::cos(b / 2),0);
			}

			//规划身体姿态
			double s_dec = Aris::Dynamic::dec_even(param.totalCount, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_dec*b / 8)*up[0],std::sin(s_dec*b / 8)*up[1] ,std::sin(s_dec*b / 8)*up[2],std::cos(s_dec*b / 8) };
			double pe[6];
			s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}
		else//匀速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				s_pm_dot_v3(pm_b_half, front, leg_forward_dir);
				
				s_pm_dot_v3(pm_b, beginPee + i, forward_d);
				s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
				s_daxpy(3, d, leg_forward_dir, 1, forward_d, 1);
				
				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double d2 = d / 2 / std::cos(b / 4);
			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * s_interp(param.totalCount, period_count + 1, 0, d2*std::sin(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 2))
					+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d/2, d / 2 / param.totalCount / std::cos(b / 2), d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b/2));
			}

			//规划身体姿态
			double s_even = even(param.totalCount, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_even*b / 4)*up[0],std::sin(s_even*b / 4)*up[1] ,std::sin(s_even*b / 4)*up[2],std::cos(s_even*b / 4) };
			double pe[6];
			s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}

		robot.SetPeb(Peb, beginMak);
		robot.SetPee(Pee, beginMak);
		
		return 2 * param.n * param.totalCount - param.count - 1;
	}
}
