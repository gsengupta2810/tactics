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

		/* some local functions */
		//gives the distance of a line from the point (x0, y0)
		float getDistanceFromLine(const Param& tParam, const BeliefState& state, float x0, float y0) const;

	private:
		//MUL_FACTOR of 1 equals 60 frames per oscillation along the line
		static const int MUL_FACTOR = 2;
		enum InternalState{
			GO_TO_LINE,
			OSCILLATE,
			GO_TO_BALL,
			TURN_TO_POINT,
			KICK
		}iState;

	};//class TDefendLine
    //registering the tactic class in the factory
	REGISTER_TACTIC(TDefendLine)
}//namespace Strategy
#endif