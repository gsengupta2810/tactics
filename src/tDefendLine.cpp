#include <list>
#include "tDefendLine.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
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

	int TDefendLine::chooseBestBot(const BeliefState &bs, std::list<int> &freeBots, const Param &tParam, int prevID) const {

		int SelectedBot;
		float MinDis = 100000000.0f;

		Vector2D<int> P1(tParam.DefendLineP.x1, tParam.DefendLineP.y1);
		Vector2D<int> P2(tParam.DefendLineP.x2, tParam.DefendLineP.y2);
		//calculate the midpoint of the line segment
		Vector2D<int> mid((P1.x + P2.x)/2,(P1.y + P2.y)/2);

		for(std::list<int>::iterator itr = freeBots.begin(); itr != freeBots.end(); ++itr) {

			Vector2D<int> HomePos(bs.homePos[*itr].x, bs.homePos[*itr].y);
			float dist_from_mid_point = (mid - HomePos).absSq();

			if(dist_from_mid_point < MinDis){
				MinDis = dist_from_mid_point;
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
		Vector2D<float> dPoint(mid.x, state.ballPos.y);
		Vector2D<float> homePos(state.homePos[botID].x, state.homePos[botID].y);
        Vector2D<float> ballPos(state.ballPos.x, state.ballPos.y);

        //calculate the perpendicular distance between the line and the bot nearest to the line
        float x0 = state.homePos[botID].x;
        float y0 = state.homePos[botID].y;
        float x1 = P1.x;
        float x2 = P2.x;
        float y1 = P1.y;
        float y2 = P2.y;
        float dis = float(abs((y2-y1)*x0 + (x1-x2)*y0 + (x2*y1 - x1*y2))/sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)));

        /*
        If the perpendicular distance between the bot and the line is greater than threshold
        go towards the line
        Maybe midpoint??
        */
   
        if(dis > BOT_POINT_THRESH * 2.2) {
        	sID = SkillSet::GoToPoint;

        	sParam.GoToPointP.x             = mid.x;
        	sParam.GoToPointP.y             = mid.y;
        	sParam.GoToPointP.align         = tParam.PositionP.align;
        	sParam.GoToPointP.finalslope    = tParam.PositionP.finalSlope ;
            sParam.GoToPointP.finalVelocity = tParam.PositionP.finalVelocity;

            return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        }
        /*
        Move along the line and guard it
        Dividing sixty frames into four equal parts and executing the GoToPoint skill
        */
     
        else if((homePos - ballPos).absSq() > BOT_BALL_THRESH * BOT_BALL_THRESH ) {
        	static  int count = 0;

        	sID = SkillSet::GoToPoint;

        	sParam.GoToPointP.align         = tParam.PositionP.align;
        	sParam.GoToPointP.finalslope    = tParam.PositionP.finalSlope ;
            sParam.GoToPointP.finalVelocity = tParam.PositionP.finalVelocity;

            if(count > 60)
            	count = 0;

        	if(count <= 15){
        		sParam.GoToPointP.x = P1.x / 15.0;
        	    sParam.GoToPointP.y = P1.y / 15.0;
        	}
        	else if(count > 15 && count <= 30){
        		sParam.GoToPointP.x = mid.x / 15.0;
        	    sParam.GoToPointP.y = mid.y / 15.0;
        	}
        	else if(count > 30 && count <= 45){
        		sParam.GoToPointP.x = P2.x / 15.0;
        	    sParam.GoToPointP.y = P2.y / 15.0;
        	}
        	else{
        		sParam.GoToPointP.x = mid.x / 15.0;
        	    sParam.GoToPointP.y = mid.y / 15.0;
        	}
        	count++;
        	return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        }
       
        /*
        Now that the ball is close enough towards the line spin and deflect the ball off 
        from the line
        */

        else {
        	sID = SkillSet::Spin;
        	sParam.SpinP.radPerSec = MAX_BOT_OMEGA;

        	return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        }

        /*
		switch(iState){
			case GO_TO_DEST:

			       if((homePos - ballPos).absSq() < (BOT_BALL_THRESH * 1.1)){
			       	    iState = CLOSE_TO_BALL;
			       }
			       else{
			        //fill me up
			       }
			       	
			    break;

			case CLOSE_TO_BALL:
			        sID = SkillSet::Spin;
			        sParam.SpinP.radPerSec = MAX_BOT_OMEGA;//Assign it the max omega
			        iState = CLOSE_TO_BALL;

			        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
			        
			    break;

			default:
			        sID = SkillSet::GoToPoint;

			       	sParam.GoToPointP.x             = dPoint.x;
                    sParam.GoToPointP.y             = state.ballPos.y;
                    sParam.GoToPointP.align         = tParam.PositionP.align;
                    sParam.GoToPointP.finalslope    = tParam.PositionP.finalSlope ;
                    sParam.GoToPointP.finalVelocity = tParam.PositionP.finalVelocity;

                    return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
			        
			     break;
		}
      */
	}//execute


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

}//namespace strategy