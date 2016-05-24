#include <list>
#include <ros/ros.h>
#include "tDefendARC.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>

#define sgn(x) (((x)<0)?(-1):(1))

namespace Strategy {

	TDefendARC::TDefendARC(int botID) : Tactic(botID) {

	}

	TDefendARC::~TDefendARC() {}

	bool TDefendARC::isCompleted(const BeliefState& state) const {
		return false;
	}//isCompleted

	inline bool TDefendARC::isActiveTactic(void) const {
		return true;
	}

	int TDefendARC::chooseBestBot(const BeliefState& state, std::list<int>& freebots, const Param& tParam, int prevID) const {
	}//choosebestbot

	gr_Robot_Command TDefendARC::execute(const BeliefState& state, const Param& tParam) {
		Vector2D<float> P1;
		Vector2D<float> P2;
		Vector2D<float> C(-HALF_FIELD_MAXX, 0.0f);
		//Vector2D<float> homePos(state.homePos[botID].x, state.homePos[botID].y);
		//Vector2D<float> mid_arc((P1.x + P2.x)/2.0, (P1.y + P2.y)/2.0);
		//Vector2D<float> origin(0.0f, 0.0f);
		Vector2D<float> dPoint;
		Vector2D<float> tP1;
		Vector2D<float> tP2;
		//Vector2D<float> t_homePos;
		Vector2D<float> p_threat, sol_left, sol_right, sol_mid;
		Vector2D<float> upper_limit(-HALF_FIELD_MAXX, OUR_GOAL_MAXY / 1.2f);
		Vector2D<float> lower_limit(-HALF_FIELD_MAXX, OUR_GOAL_MINY / 1.2f);
		

		float R = HALF_FIELD_MAXY / 2.5f;

		Strategy::SkillSet::SkillID sID = SkillSet::GoToPoint;
		SkillSet::SParam sParam;

		//transform the co-ordinates into the center's reference frame 
		tP1.x = P1.x - C.x;
		tP1.y = P1.y - C.y;
		tP2.x = P2.x - C.x;
		tP2.y = P2.y - C.y;

		//calculate the open angle from the primary  threat
		int pt = primary_threat(state);
		if(pt == -1){
			p_threat.x = state.ballPos.x;
			p_threat.y = state.ballPos.y;
		}
		else {
			p_threat.x = state.awayPos[pt].x;
			p_threat.y = state.awayPos[pt].y;
		}

		//calculate the points of intersection of line and the circle
		inter_circle_and_line(p_threat, upper_limit, C, R, sol_left);
		inter_circle_and_line(p_threat, lower_limit, C, R, sol_right);
		inter_circle_and_line(p_threat, C, C, R, sol_mid);

		//go to the point and anticipate for the ball
		//just for testing purpose side = 0 //represents the left side
		if(tParam.DefendARCP.side == 0) {
			sParam.GoToPointP.x = (sol_left.x + sol_mid.x) / 2.0f;
		    sParam.GoToPointP.y = (sol_left.y + sol_mid.y) / 2.0f;
		}
		else {
			sParam.GoToPointP.x = (sol_right.x + sol_mid.x) / 2.0f;
		    sParam.GoToPointP.y = (sol_right.y + sol_mid.y) / 2.0f;
		}

		//decide to whether kick or dribble or pass the ball
		sParam.GoToPointP.finalslope = atan2((p_threat.y - state.homePos[botID].y), (p_threat.x - state.homePos[botID].x));
		sParam.GoToPointP.align = true;
		sParam.GoToPointP.finalVelocity;
		ROS_INFO("atan2 = %f", atan2((p_threat.y - state.homePos[botID].y), (p_threat.x - state.homePos[botID].x)));

		return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
	} //execute

	string TDefendARC::paramToJSON(Tactic::Param tParam) {
		using namespace rapidjson;
		StringBuffer buffer;
		Writer<StringBuffer> w(buffer);

		w.StartObject();
		w.String("x1");
		w.Double(tParam.DefendARCP.x1);
		w.String("x2");
		w.Double(tParam.DefendARCP.x2);
		w.String("y1");
		w.Double(tParam.DefendARCP.y1);
		w.String("y2");
		w.Double(tParam.DefendARCP.y2);
		w.String("xc");
		w.Double(tParam.DefendARCP.xc);
		w.String("yc");
		w.Double(tParam.DefendARCP.yc);
		w.String("side");
		w.Double(tParam.DefendARCP.side);
		w.EndObject();

		return buffer.GetString();

	}//paramToJSON

	Tactic::Param TDefendARC::paramFromJSON(string json) {
		using namespace rapidjson;
		Tactic::Param tParam;
		Document d;

		d.Parse(json.c_str());
		tParam.DefendARCP.x1 = d["x1"].GetDouble();
		tParam.DefendARCP.x2 = d["x2"].GetDouble();
		tParam.DefendARCP.y1 = d["y1"].GetDouble();
		tParam.DefendARCP.y2 = d["y2"].GetDouble();
		tParam.DefendARCP.xc = d["xc"].GetDouble();
		tParam.DefendARCP.yc = d["yc"].GetDouble();
		tParam.DefendARCP.side = d["side"].GetDouble();

		return tParam;
	}//paramFromJSON

	int TDefendARC::primary_threat(const BeliefState& state) const {
		if(sqrt(pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) <= LOW_BALL_VELOCITY_THRES)
			return -1;
		else {
			return -1;
		}
	}//primary_threat

	int TDefendARC::no_of_opponents_in_our_half(const BeliefState& state) const {
		int count = 0;
		for(int i = 0; i < state.awayDetected.size(); ++i) {
			if(state.awayPos[i].x < 0)
				++count;
		}
		return count;
	}//no_of_opponents_in_our_half

	void TDefendARC::inter_circle_and_line(Vector2D<float> P1, Vector2D<float> P2, Vector2D<float> C, float R, Vector2D<float>& P) const {
		//here P1 always represents the threat position
		float tx1, tx2, ty1, ty2;
		float dx, dy, dr, D;

		//shift the origin to the center of the circle
		tx1 = P1.x - C.x;
		tx2 = P2.x - C.x;
		ty1 = P1.y - C.y;
		ty2 = P2.y - C.y;

		//calculate the intersection of the line and circle of radius R
		//http://mathworld.wolfram.com/Circle-LineIntersection.html
		dx  = tx2 - tx1;
		dy  = ty2 - ty1;
		dr  = sqrt(pow(dx, 2) + pow(dy, 2));
		 D  = tx1 * ty2 - tx2 * ty1;
		P.x = float((D * dy + sgn(dy) * dx * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		P.y = float(((-D * dx) + abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));

		if(P.x < 0) {
			    P.x = float((D * dy - sgn(dy) * dx * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		}

		if(P1.y > 0) {
		    if(P.y < 0) {
			    P.y = float((-D * dx - abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		    }
		}
		else {
		    if(P.y > 0) {
			    P.y = float((-D * dx - abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		    }
		}

		//convert the co-ordinates back to the original system
		P.x = P.x + C.x;
		P.y = P.y + C.y;
	}//inter_circle_and_line function


}//namespace strategy