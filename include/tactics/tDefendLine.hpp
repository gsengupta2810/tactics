#ifndef TDEFENDLINE_HPP
#define TDEFENDLINE_HPP
#include "tactic.h"
#include "skills/skillSet.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "ssl_common/config.h"
#include <sys/time.h>
#include <unistd.h>
#include "tactic_factory.h"
#include <ssl_common/geometry.hpp>

namespace Strategy
{
	class TDefendLine : public Tactic
	{
	public:
		TDefendLine(int botID);
		~TDefendLine();

		virtual bool isCompleted(const BeliefState& bs) const;
		virtual bool isActiveTactic(void) const;
		virtual int chooseBestBot(const BeliefState& bs, std::list<int>& freeBots, const Param& tParam, int prevID = -1) const;
		virtual gr_Robot_Command execute(const BeliefState& bs, const Param& tParam);
		virtual Tactic::Param paramFromJSON(string json);
		virtual string paramToJSON(Tactic::Param p);

		//some local functions
		void getbotdest(const Param& tParam,float *x,float *y,float *slope);

	private:
		enum InternalState{
			GO_TO_DEST,
			CLOSE_TO_BALL
		}iState;

	};//class TDefendLine
    //registering the tactic class in the factory
	REGISTER_TACTIC(TDefendLine)
}//namespace Strategy
#endif