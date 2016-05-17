#include <list>
#include "tDefendLine.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>

namespace Strategy
{
	TDefendLine::TDefendLine(int botID) : Tactic(botID) {}
	TDefendLine::~TDefendLine()
	{

	}

	bool TDefendLine::isCompleted(const BeliefState &bs) const {
		return false;
	}//isCompleted

	inline bool TDefendLine::isActiveTactic(void ) const {
		return true;
	}//isActiveTactic

	int TDefendLine::chooseBestBot(const BeliefState &state, std::list<int> &freeBots, const Param &tParam, int prevID) const {

		int SelectedBot;
		float MinDis = 100000000.0f;

		Vector2D<int> P1(tParam.DefendLineP.x1, tParam.DefendLineP.y1);
		Vector2D<int> P2(tParam.DefendLineP.x2, tParam.DefendLineP.y2);
		//calculate the midpoint of the line segment
		Vector2D<int> mid((P1.x + P2.x)/2,(P1.y + P2.y)/2);

		for(std::list<int>::iterator itr = freeBots.begin(); itr != freeBots.end(); ++itr) {

			float dist = getDistanceFromLine(tParam, state, *itr);

			if(dist < MinDis){
				MinDis = dist;
				SelectedBot = *itr;
			}
		}
		//FIXME: Make sure that I'm not goalkeeper
		return SelectedBot;
	}//chooseBestBot



	gr_Robot_Command TDefendLine::execute(const BeliefState &state,const Param &tParam) {

		Strategy::SkillSet::SkillID sID;
        SkillSet::SParam sParam;

        Vector2D<float> P1(tParam.DefendLineP.x1, tParam.DefendLineP.y1);
		Vector2D<float> P2(tParam.DefendLineP.x2, tParam.DefendLineP.y2);
		Vector2D<float> mid(float((P1.x + P2.x)/2.0),float((P1.y + P2.y)/2.0));
		Vector2D<float> homePos(state.homePos[botID].x, state.homePos[botID].y);
        Vector2D<float> ballPos(state.ballPos.x, state.ballPos.y);

        //calculate the distance of the nearest bot from the line
        float dis = getDistanceFromLine(tParam, state, botID);

        //just for testing the tactic
        P1.x = 0.0f;
        P1.y = 0.0f;
        P2.x = HALF_FIELD_MAXX - BOT_POINT_THRESH;
        P2.y = HALF_FIELD_MAXY - BOT_POINT_THRESH;

        /*
        If the perpendicular distance between the bot and the line is greater than threshold
        go towards the line
        Maybe midpoint??
        */
        static  int count = 0;
        if(dis > BOT_POINT_THRESH * 1.1) {
        	sID = SkillSet::GoToPoint;

        	sParam.GoToPointP.x             = mid.x;
        	sParam.GoToPointP.y             = mid.y;
        	sParam.GoToPointP.align         = tParam.PositionP.align;
        	sParam.GoToPointP.finalslope    = tParam.PositionP.finalSlope ;
            sParam.GoToPointP.finalVelocity = tParam.PositionP.finalVelocity;
        }

        /*
        Move along the line and guard it
        Dividing sixty frames into four equal parts and executing the GoToPoint skill
        */
     
        else if((homePos - ballPos).absSq() > BOT_BALL_THRESH * BOT_BALL_THRESH * 1.1 * 1.1) {

        	sID = SkillSet::GoToPoint;

        	sParam.GoToPointP.align         = tParam.PositionP.align;
        	sParam.GoToPointP.finalslope    = tParam.PositionP.finalSlope ;
            sParam.GoToPointP.finalVelocity = tParam.PositionP.finalVelocity;

            if(count > 60 * MUL_FACTOR)
            	count = 0;

        	if(count <= 15 * MUL_FACTOR){
        		sParam.GoToPointP.x = mid.x + count * ((P1.x - mid.x) / (15 * MUL_FACTOR)) ;
        	    sParam.GoToPointP.y = mid.y + count * ((P1.y - mid.y) / (15 * MUL_FACTOR));
        	}
        	else if((count > 15 * MUL_FACTOR) && (count <= 30 * MUL_FACTOR)){
        		sParam.GoToPointP.x = P1.x - (count - 1 * 15 * MUL_FACTOR) * ((P1.x - mid.x) / (15 * MUL_FACTOR));
        	    sParam.GoToPointP.y = P1.y - (count - 1 * 15 * MUL_FACTOR) * ((P1.y - mid.y) / (15 * MUL_FACTOR));
        	}
        	else if((count > 30 * MUL_FACTOR) && (count <= 45 * MUL_FACTOR)){
        		sParam.GoToPointP.x = mid.x + (count - 2 * 15 * MUL_FACTOR) * ((P2.x - mid.x) / (15 * MUL_FACTOR));
        	    sParam.GoToPointP.y = mid.y + (count - 2 * 15 * MUL_FACTOR) * ((P2.y - mid.y) / (15 * MUL_FACTOR));
        	}
        	else{
        		sParam.GoToPointP.x = P2.x - (count - 3 * 15 * MUL_FACTOR) * ((P2.x - mid.x) / (15 * MUL_FACTOR));
        	    sParam.GoToPointP.y = P2.y - (count - 3 * 15 * MUL_FACTOR) * ((P2.y - mid.y) / (15 * MUL_FACTOR));
        	}
        	count++;
        }
       
        /*
        Now that the ball is close enough towards the line spin and deflect the ball off 
        from the line
        */

        else {
        	sID = SkillSet::Spin;
        	sParam.SpinP.radPerSec = MAX_BOT_OMEGA;
        }
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
	}//execute function


	Tactic::Param TDefendLine::paramFromJSON(string json) {
		using namespace rapidjson;
		Tactic::Param tParam;
		Document d;

		d.Parse(json.c_str());
		tParam.DefendLineP.x1 = d["x1"].GetDouble();
		tParam.DefendLineP.x2 = d["x2"].GetDouble();
		tParam.DefendLineP.y1 = d["y1"].GetDouble();
		tParam.DefendLineP.y2 = d["y2"].GetDouble();
		tParam.DefendLineP.radius = d["radius"].GetDouble();

		return tParam;
	}//paramFromJSON

	string TDefendLine::paramToJSON(Tactic::Param tParam) {
		using namespace rapidjson;
		StringBuffer buffer;
		Writer<StringBuffer> w(buffer);

		w.StartObject();
		w.String("x1");
		w.Double(tParam.DefendLineP.x1);
		w.String("x2");
		w.Double(tParam.DefendLineP.x2);
		w.String("y1");
		w.Double(tParam.DefendLineP.y1);
		w.String("y2");
		w.Double(tParam.DefendLineP.y2);
		w.String("radius");
		w.Double(tParam.DefendLineP.radius);
		w.EndObject();

		return buffer.GetString();
	}//paramToJSON


	float TDefendLine::getDistanceFromLine(const Param& tParam, const BeliefState& state, int idx) const{

		Vector2D<float> P1(tParam.DefendLineP.x1, tParam.DefendLineP.y1);
		Vector2D<float> P2(tParam.DefendLineP.x2, tParam.DefendLineP.y2);
		Vector2D<float> mid(float((P1.x + P2.x)/2.0),float((P1.y + P2.y)/2.0));
		Vector2D<float> homePos(state.homePos[idx].x, state.homePos[idx].y);
        Vector2D<float> ballPos(state.ballPos.x, state.ballPos.y);

        float x0 = state.homePos[idx].x;
        float y0 = state.homePos[idx].y;
        float x1 = P1.x;
        float x2 = P2.x;
        float y1 = P1.y;
        float y2 = P2.y;
        float dis = float(abs((y2-y1)*x0 + (x1-x2)*y0 + (x2*y1 - x1*y2))/sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)));

        return dis;
	}

}//namespace strategy