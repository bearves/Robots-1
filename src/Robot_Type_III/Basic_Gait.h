#ifndef BASIC_GAIT_H
#define BASIC_GAIT_H

#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <Robot_Type_III.h>

namespace Robots
{
	namespace Gait
	{
		enum { MOTOR_NUM = 19 };

		auto basicParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;

		struct RecoverParam final :public aris::server::GaitParamBase
		{
			std::int32_t recover_count{ 3000 };
			std::int32_t align_count{ 3000 };
			bool active_leg[6]{ true,true,true,true,true,true };
			double margin_offset{ 0.01 };//meter
			double alignPee[18]
            {   -0.3,   -0.85,   -0.65,
                -0.45,  -0.85,   0,
                -0.3,   -0.85,   0.65,
                0.3,    -0.85,   -0.65,
                0.45,   -0.85,   0,
                0.3,    -0.85,   0.65 };
			double recoverPee[18]
            {   -0.3,   -0.99,   -0.65,
                -0.45,  -0.99,   0,
                -0.3,   -0.99,   0.65,
                0.3,    -0.99,   -0.65,
                0.45,   -0.99,   0,
                0.3,    -0.99,   0.65 };
            bool is_zeroing_required {false};
		};
		auto recoverParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto recoverGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

        struct RecoverWaistParam final :public aris::server::GaitParamBase
        {
            std::int32_t totalCount{ 3000 };
            double angle{ 0 };
        };
        auto recoverWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
        auto recoverWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

        struct WalkParam final :public aris::server::GaitParamBase
		{
			std::int32_t totalCount{ 3000 };
			std::int32_t n{ 2 };
			double d{ 0.5 };
			double h{ 0.05 };
			double alpha{ 0.3 };
			double beta{ 0.3 };
		};
		auto walkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto walkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

		struct ResetOriginParam final :public aris::server::GaitParamBase {};
		auto resetOriginParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto resetOriginGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

		struct WaistPreHomeParam final :public aris::server::GaitParamBase
		{
			double vel{ 0.002 }; //速度
			double offset{ 0.005 }; //触发行程开关后再走5mm
		};
		auto waistPreHomeParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto waistPreHomeGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

		struct AdjustWaistParam final :public aris::server::GaitParamBase
		{
			std::int32_t totalCount{ 3000 };
			double angle{ 0 };
		};
		auto adjustWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto adjustWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

		struct ExtendChainParam final :public aris::server::GaitParamBase
		{
			std::int32_t totalCount{ 3000 };
			int motor_id{ 0 };
			double length{ 0.01 }; 
		};
		auto extendChainParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
		auto extendChainGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
	}
}

#endif
