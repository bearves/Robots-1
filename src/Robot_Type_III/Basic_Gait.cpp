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

#include "Basic_Gait.h"


using namespace aris::dynamic;

namespace Robots
{
	namespace Gait
	{
		auto basicParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			aris::server::BasicFunctionParam param;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill_n(param.active_motor, MOTOR_NUM, true);
				}
				else if (i.first == "first")
				{
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor + 0, 3, true);
					std::fill_n(param.active_motor + 6, 3, true);
					std::fill_n(param.active_motor + 12, 3, true);
				}
				else if (i.first == "second")
				{
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor + 3, 3, true);
					std::fill_n(param.active_motor + 9, 3, true);
					std::fill_n(param.active_motor + 15, 3, true);
				}
				else if (i.first == "motor")
				{
                    int id = { std::stoi(i.second) };
                    if (id < 0 || id > MOTOR_NUM - 1)throw std::runtime_error("invalid param in basicParse func");

					std::fill_n(param.active_motor, MOTOR_NUM, false);
					param.active_motor[id] = true;
				}
				else if (i.first == "physical_motor")
				{
                    int id = { std::stoi(i.second) };
                    if (id < 0 || id > MOTOR_NUM - 1)throw std::runtime_error("invalid param in basicParse func");

					std::fill_n(param.active_motor, MOTOR_NUM, false);
					param.active_motor[aris::server::ControlServer::instance().controller().motionAtPhy(id).absID()] = true;
				}
				else if (i.first == "leg")
				{
					auto leg_id = std::stoi(i.second);
                    if (leg_id < 0 || leg_id > 5)throw std::runtime_error("invalid param in parseRecover func");

					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor + leg_id * 3, 3, true);
				}
				else if (i.first == "waist")
				{
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					param.active_motor[18] = true;
				}
			}

			msg_out.copyStruct(param);
		}

		auto recoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			RecoverParam param;

			param.if_check_pos_min = false;
			param.if_check_pos_max = false;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill_n(param.active_leg, 6, true);
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor, 18, true);
				}
				else if (i.first == "first")
				{
					param.active_leg[0] = true;
					param.active_leg[1] = false;
					param.active_leg[2] = true;
					param.active_leg[3] = false;
					param.active_leg[4] = true;
					param.active_leg[5] = false;
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor + 0, 3, true);
					std::fill_n(param.active_motor + 6, 3, true);
					std::fill_n(param.active_motor + 12, 3, true);
				}
				else if (i.first == "second")
				{
					param.active_leg[0] = false;
					param.active_leg[1] = true;
					param.active_leg[2] = false;
					param.active_leg[3] = true;
					param.active_leg[4] = false;
					param.active_leg[5] = true;
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor + 3, 3, true);
					std::fill_n(param.active_motor + 9, 3, true);
					std::fill_n(param.active_motor + 15, 3, true);
				}
				else if (i.first == "leg")
				{
					auto leg_id = std::stoi(i.second);

					if (leg_id < 0 || leg_id>5)throw std::runtime_error("invalide param in parseRecover func");

					std::fill_n(param.active_leg, 6, false);
					param.active_leg[leg_id] = true;
					std::fill_n(param.active_motor, MOTOR_NUM, false);
					std::fill_n(param.active_motor + leg_id * 3, 3, true);
				}
				else if (i.first == "t1")
				{
					param.recover_count = std::stoi(i.second);
				}
				else if (i.first == "t2")
				{
					param.align_count = std::stoi(i.second);
				}
				else if (i.first == "margin_offset")
				{
					param.margin_offset = std::stod(i.second);
				}
                else if (i.first == "require_zero")
                {
                    param.is_zeroing_required = std::stoi(i.second) == 0 ? false : true;
                }
				else
				{
					throw std::runtime_error("unknown param in parseRecover func");
				}
			}

			msg_out.copyStruct(param);
		}
        auto recoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
        {
            auto &robot = static_cast<Robots::RobotTypeIII &>(model);
            auto &param = static_cast<const RecoverParam &>(plan_param);

            static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

            static double beginPin[18], beginPee[18], alignPin[18];
            static double beginWa;

            if (param.count == 0)
            {
                std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
                robot.GetPee(beginPee, robot.body());
                robot.GetWa(beginWa);

                const double pe[6]{ 0 };
                robot.SetPeb(pe);
                robot.SetWa(beginWa);
                robot.SetPee(param.alignPee);

                robot.GetPin(alignPin);
                robot.SetPee(beginPee, robot.body());

                //for test
                rt_printf("beginWa: %f\n", beginWa);
            }

            int leftCount = param.count < param.recover_count ? 0 : param.recover_count;
            int rightCount = param.count < param.recover_count ? param.recover_count : param.recover_count + param.align_count;

            double s = -(PI / 2)*cos(PI * (param.count - leftCount + 1) / (rightCount - leftCount)) + PI / 2;

            for (int i = 0; i < 6; ++i)
            {
                if (param.active_leg[i])
                {
                    if (param.count < param.recover_count)
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
                        robot.SetWa(beginWa);
                        robot.pLegs[i]->SetPee(pEE);
                    }
                    if (param.is_zeroing_required && param.count == param.recover_count)
                    {
                        param.force_data->at(i).isZeroingRequested = true;
                    }
                }
            }

            // recover 自己做检查 //
            for (int i = 0; i < 18; ++i)
            {
                if (param.active_motor[i] && (param.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN))
                {

                    if (param.motion_raw_data->at(i).target_pos > (cs.controller().motionAtAbs(i).maxPosCount() + param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
                    {
                        rt_printf("Motor %i's target position is bigger than its MAX permitted value in recover, you might forget to GO HOME\n", i);
                        rt_printf("The min, max and current count are:\n");
                        for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                        {
                            rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                        }
                        rt_printf("recover failed\n");
                        return 0;
                    }
                    if (param.motion_raw_data->at(i).target_pos < (cs.controller().motionAtAbs(i).minPosCount() - param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
                    {
                        rt_printf("Motor %i's target position is smaller than its MIN permitted value in recover, you might forget to GO HOME\n", i);
                        rt_printf("The min, max and current count are:\n");
                        for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                        {
                            rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                        }
                        rt_printf("recover failed\n");
                        return 0;
                    }
                }
            }

            //for test
            if (param.count == param.align_count + param.recover_count - 1)
            {
                double recoverWa;
                robot.GetWa(recoverWa);
                rt_printf("recoverWa: %f\n", recoverWa);
                double recoverPee[18];
                robot.GetPee(recoverPee);
                rt_printf("recoverPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
                    recoverPee[0], recoverPee[1], recoverPee[2], recoverPee[3], recoverPee[4], recoverPee[5], recoverPee[6], recoverPee[7], recoverPee[8],
                    recoverPee[9], recoverPee[10], recoverPee[11], recoverPee[12], recoverPee[13], recoverPee[14], recoverPee[15], recoverPee[16], recoverPee[17]);
            }

            return param.align_count + param.recover_count - param.count - 1;
        }

        auto recoverWaistParse(const std::string & cmd, const std::map<std::string, std::string>& params, aris::core::Msg & msg_out) -> void
        {
            RecoverWaistParam param;

            param.if_check_pos_min = false;
            param.if_check_pos_max = false;

            for (auto &i : params)
            {
                if (i.first == "totalCount")
                {
                    param.totalCount = std::stoi(i.second);
                }
                else if (i.first == "angle")
                {
                    param.angle = std::stod(i.second);
                }
            }
            std::fill_n(param.active_motor, 18, false);
            param.active_motor[18] = true;

            msg_out.copyStruct(param);
        }
        auto recoverWaistGait(aris::dynamic::Model & model, const aris::dynamic::PlanParamBase & param_in) -> int
        {
            auto &robot = static_cast<Robots::RobotTypeIII &>(model);
            auto &param = static_cast<const RecoverWaistParam &>(param_in);

            static double beginWin, recoverWin;

            if (param.count == 0)
            {
                beginWin = param.motion_feedback_pos->at(18);
                double beginWa;
                robot.GetWa(beginWa);

                robot.SetWa(param.angle);
                robot.GetWin(recoverWin);

                robot.SetWa(beginWa);
            }
            else if(param.count == param.totalCount - 1)
            {
                robot.SetWa(param.angle);
            }
            const double s = -0.5*cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1.
            robot.motionPool().at(18).setMotPos(beginWin * (1 - s) + recoverWin * s);

            return param.totalCount - param.count - 1;
        }

        auto extendChainParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out) -> void
        {
            ExtendChainParam param;

            param.if_check_pos_min = false;
            param.if_check_pos_max = false;

            for (auto &i : params)
            {
                if (i.first == "totalCount")
                {
                    param.totalCount = std::stoi(i.second);
                }
                else if (i.first == "motor")
                {
                    int id = { std::stoi(i.second) };
                    if (id < 0 || id > MOTOR_NUM - 1)throw std::runtime_error("invalid param in basicParse func");
                    std::fill_n(param.active_motor, MOTOR_NUM, false);
                    param.active_motor[id] = true;

                    param.motor_id = std::stoi(i.second);
                }
                else if (i.first == "length")
                {
                    param.length = std::stod(i.second);
                }
            }
            msg_out.copyStruct(param);
        }
        auto extendChainGait(aris::dynamic::Model & model, const aris::dynamic::PlanParamBase & param_in) -> int
        {
            auto &robot = static_cast<Robots::RobotTypeIII &>(model);
            auto &param = static_cast<const ExtendChainParam &>(param_in);

            const double s = -0.5*cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1.

            static double beginPin[MOTOR_NUM];
            if (param.count == 0)
            {
                std::copy_n(param.motion_feedback_pos->data(), MOTOR_NUM, beginPin);
                //for test
                rt_printf("beginPin: \n");
                for (int i = 0; i < MOTOR_NUM; ++i)
                {
                    rt_printf("%f ", beginPin[i]);
                }
                rt_printf("\n");
            }

            robot.motionPool().at(param.motor_id).setMotPos(beginPin[param.motor_id] + param.length * s);

            return param.totalCount - param.count - 1;
        }

		auto walkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			WalkParam param;

			for (auto &i : params)
			{
				if (i.first == "totalCount")
				{
					param.totalCount = std::stoi(i.second);
				}
				else if (i.first == "n")
				{
					param.n = std::stoi(i.second);
				}
				else if (i.first == "distance")
				{
					param.d = std::stod(i.second);
				}
				else if (i.first == "height")
				{
					param.h = std::stod(i.second);
				}
				else if (i.first == "alpha")
				{
					param.alpha = std::stod(i.second);
				}
				else if (i.first == "beta")
				{
					param.beta = std::stod(i.second);
				}
			}
			msg_out.copyStruct(param);
		}
		auto walkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
		{
			auto &robot = static_cast<Robots::RobotTypeIII &>(model);
			auto &param = static_cast<const WalkParam &>(param_in);

			//初始化
			static aris::dynamic::FloatMarker beginMak{ robot.ground() };
			static double beginPee[18];
			static double beginWa;

			if (param.count%param.totalCount == 0)
			{
				beginMak.setPrtPm(*robot.body().pm());
				beginMak.update();
				robot.GetPee(beginPee, beginMak);
				robot.GetWa(beginWa);
                rt_printf("beginWa: %f\n", beginWa);
                rt_printf("beginPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
                    beginPee[0], beginPee[1], beginPee[2], beginPee[3], beginPee[4], beginPee[5], beginPee[6], beginPee[7], beginPee[8],
                    beginPee[9], beginPee[10], beginPee[11], beginPee[12], beginPee[13], beginPee[14], beginPee[15], beginPee[16], beginPee[17]);
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
					Peb[i] = left[i] * s_interp(param.totalCount, period_count + 1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 4))
						+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 4));
				}

				//规划身体姿态
				double s_acc = aris::dynamic::acc_even(param.totalCount, period_count + 1);
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
					Peb[i] = left[i] * s_interp(param.totalCount, period_count + 1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, 0)
						+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), d / 2 / param.totalCount / std::cos(b / 2), 0);
				}

				//规划身体姿态
				double s_dec = aris::dynamic::dec_even(param.totalCount, period_count + 1);
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
						+ front[i] * s_interp(param.totalCount, period_count + 1, 0, d / 2, d / 2 / param.totalCount / std::cos(b / 2), d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 2));
				}

				//规划身体姿态
				double s_even = even(param.totalCount, period_count + 1);
				double pq[7] = { 0,0,0,std::sin(s_even*b / 4)*up[0],std::sin(s_even*b / 4)*up[1] ,std::sin(s_even*b / 4)*up[2],std::cos(s_even*b / 4) };
				double pe[6];
				s_pq2pe(pq, pe);
				std::copy(pe + 3, pe + 6, Peb + 3);
			}

			double Wa = beginWa;

			robot.SetPeb(Peb, beginMak);
			robot.SetWa(Wa);
			robot.SetPee(Pee, beginMak);

			return 2 * param.n * param.totalCount - param.count - 1;
		}

		auto resetOriginParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			Robots::ResetOriginParam param;
			msg_out.copyStruct(param);
		}
		auto resetOriginGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
		{
			auto &robot = static_cast<Robots::RobotTypeIII &>(model);
			auto &param = static_cast<const Robots::ResetOriginParam &>(param_in);

			double Pee[18];
			robot.GetPee(Pee, robot.body());
			double Wa;
			robot.GetWa(Wa);

			double Peb[6]{ 0,0,0,0,0,0 };
			robot.SetPeb(Peb);
			robot.SetWa(Wa);
			robot.SetPee(Pee);

            double Pin[19];
            robot.GetAllPin(Pin);

			if (param.imu_data)rt_printf("imu(pitch roll yaw): %f  %f  %f\n", param.imu_data->pitch, param.imu_data->roll, param.imu_data->yaw);

			double force_on_body[6], force_on_marker[6];

			// s_f2f 用来转换坐标系 // 
			std::copy(param.force_data->at(0).fce, param.force_data->at(0).fce + 6, force_on_marker);
			aris::dynamic::s_f2f(*robot.forceSensorMak().prtPm(), force_on_marker, force_on_body);

			rt_printf("waist angle: %f \n", Wa);
			rt_printf("Pee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
				Pee[0], Pee[1], Pee[2], Pee[3], Pee[4], Pee[5], Pee[6], Pee[7], Pee[8],
				Pee[9], Pee[10], Pee[11], Pee[12], Pee[13], Pee[14], Pee[15], Pee[16], Pee[17]);
            rt_printf("Pin: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f %f \n",
                Pin[0], Pin[1], Pin[2], Pin[3], Pin[4], Pin[5], Pin[6], Pin[7], Pin[8],
                Pin[9], Pin[10], Pin[11], Pin[12], Pin[13], Pin[14], Pin[15], Pin[16], Pin[17], Pin[18]);
            rt_printf("force on marker: %f  %f  %f  %f  %f  %f \n", force_on_marker[0], force_on_marker[1], force_on_marker[2], force_on_marker[3], force_on_marker[4], force_on_marker[5]);
			rt_printf("force on body: %f  %f  %f  %f  %f  %f \n", force_on_body[0], force_on_body[1], force_on_body[2], force_on_body[3], force_on_body[4], force_on_body[5]);

			return 0;
		}

		auto waistPreHomeParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			WaistPreHomeParam param;

			std::fill_n(param.active_motor, MOTOR_NUM, false);
			for (auto &i : params)
			{
				if (i.first == "offset")
				{
					param.offset = std::stod(i.second);
				}
				if (i.first == "velocity")
				{
					param.vel = std::stod(i.second);
				}
			}

			msg_out.copyStruct(param);
		}
		auto waistPreHomeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
		{
			auto &robot = static_cast<Robots::RobotTypeIII &>(model);
			auto &param = static_cast<const WaistPreHomeParam &>(param_in);

			static aris::server::ControlServer &cs = aris::server::ControlServer::instance();
			int input2count = cs.controller().motionAtAbs(18).pos2countRatio();
			std::int32_t remain_count = static_cast<std::int32_t>(std::fabs(param.offset / param.vel) * 1000);

			//trigger
			static bool is_homed;
			static std::int32_t offset_count;
			if (param.count == 0)
			{
				is_homed = false;
				offset_count = 0;
			}

			double fdback_pos = static_cast<double>(param.motion_raw_data->at(18).feedback_pos) / input2count;
			double mot_pos = fdback_pos + param.vel*0.001; //单位m
			robot.motionPool().at(18).setMotPos(mot_pos);

			auto fdback_dgi = param.motion_raw_data->at(18).feedback_dgi;
			if (is_homed)
			{
				++offset_count;
				if (offset_count > remain_count)
				{
					return 0;
				}
			}
			else
			{
				if ((fdback_dgi & 0x00300000) == 0x00200000)
				{
					is_homed = true;
					rt_printf("inversely homed");
				}
			}
			//test
			if (param.count % 100 == 0)
			{
				rt_printf("condition:%x\t", fdback_dgi & 0x00300000);
				rt_printf("feedback_dgi:%x\t", fdback_dgi);
				rt_printf("feedback_pos:%f\n", fdback_pos);
			}
			return 1;
		}

		auto adjustWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out) -> void
		{
			AdjustWaistParam param;

			for (auto &i : params)
			{
				if (i.first == "totalCount")
				{
					param.totalCount = std::stoi(i.second);
				}
				else if (i.first == "angle")
				{
					param.angle = std::stod(i.second);
				}
			}
			msg_out.copyStruct(param);
		}
		auto adjustWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
		{
			auto &robot = static_cast<Robots::RobotTypeIII &>(model);
			auto &param = static_cast<const AdjustWaistParam &>(param_in);
			//初始化
			static aris::dynamic::FloatMarker beginMak{ robot.ground() };
			static double beginPee[18];
			static double beginWa;

			if (param.count == 0)
			{
				beginMak.setPrtPm(*robot.body().pm());
				beginMak.update();
				robot.GetPee(beginPee, beginMak);
				robot.GetWa(beginWa);
				//for test
				double beginPin[19];
				robot.GetAllPin(beginPin);
				rt_printf("beginPin: \n");
				for (std::size_t i = 0; i < 19; ++i)
				{
					rt_printf("%f ", beginPin[i]);
				}
				rt_printf("\n");

			}

			double Peb[6]{ 0 };
			double Pee[18]{ 0 };
			std::copy_n(beginPee, 18, Pee);

			const double s = -0.5 * cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1. 
			double targetWa = param.angle;
			double Wa = targetWa * s + beginWa * (1 - s);

			robot.SetPeb(Peb, beginMak);
			robot.SetWa(Wa);
			robot.SetPee(Pee, beginMak);

            if (param.count == param.totalCount - 1)
            {
                double Pin[18];
                robot.GetPin(Pin);
                rt_printf("Final Pee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
                    Pee[0], Pee[1], Pee[2], Pee[3], Pee[4], Pee[5], Pee[6], Pee[7], Pee[8],
                    Pee[9], Pee[10], Pee[11], Pee[12], Pee[13], Pee[14], Pee[15], Pee[16], Pee[17]);
                rt_printf("Final Pin: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f \n",
                    Pin[0], Pin[1], Pin[2], Pin[3], Pin[4], Pin[5], Pin[6], Pin[7], Pin[8],
                    Pin[9], Pin[10], Pin[11], Pin[12], Pin[13], Pin[14], Pin[15], Pin[16], Pin[17]);
            }

			return param.totalCount - param.count - 1;
		}
	}
}
