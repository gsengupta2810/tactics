#include <list>
#include "tKickToGoal.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>


namespace Strategy
{
    TKickToGoal::TKickToGoal(int botID) :
      Tactic( botID)
    { 
      goal = Vector2D<int>(HALF_FIELD_MAXX, 0);
    } // TTStop
    TKickToGoal::~TKickToGoal()
    { } // ~TTStop
    bool TKickToGoal::isCompleted(const BeliefState &bs) const {
      return false;
    }
    inline bool TKickToGoal::isActiveTactic(void) const
    {
      return true;
    }

    int TKickToGoal::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 10000;
      for (std::list<int>::const_iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        // TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
        Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
        Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

        float dis_from_ball = (homePos - ballPos).absSq();
        if(dis_from_ball < mindis)
        {
          dis_from_ball = mindis;
          minv = *it;
        }
      }
      return minv;
    } // chooseBestBot

    gr_Robot_Command TKickToGoal::execute(const BeliefState &state, const Tactic::Param& tParam)
    {
      float dist=0, finalSlope=0, turnAngleLeft=0, angleWithBall=0;
      Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
      dist = Vector2D<int>::dist(ballPos, botPos);
      finalSlope = Vector2D<int>::angle(goal, botPos);
      turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn
      angleWithBall = normalizeAngle(state.homePos[botID].theta - Vector2D<int>::angle(ballPos, botPos));
//      switch(iState)
//      {
//        case GOTOBALL:
//            if(!(dist > BOT_BALL_THRESH || fabs(angleWithBall) > DRIBBLER_BALL_ANGLE_RANGE))
//            {
//              iState=TURNING;///////////////////////////
//              printf("state changed gotoball to turning\n");
//              break;
//            }  
//            sID = SkillSet::GoToBall;
//            printf("going to ball %d, %f, %f\n", BOT_BALL_THRESH, dist, angleWithBall);
//            return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
//            break;
//        case FINDPOINT:
////            int destRadius = (int)Vector2D<int>::dist(botPos, goal)/2;
//            dribblePoint.x = (10*botPos.x+goal.x)/11;
//            dribblePoint.y = (10*botPos.y+goal.y)/11;
//            iState = DRIBBLING;
//            break;
//        case DRIBBLING:
//            if(dist > BOT_BALL_THRESH*2 /*|| fabs(angleWithBall) > DRIBBLER_BALL_ANGLE_RANGE*/)
//            {
//              iState = GOTOBALL;
//              break;
//            }
//            if(Vector2D<int>::dist(dribblePoint, botPos)< BOT_POINT_THRESH)
//            {
//              iState = TURNING;
//              break;
//            }
//            printf("dribbling..\n");
//            sID = SkillSet::DribbleToPoint;
//            sParam.DribbleToPointP.x = dribblePoint.x;
//            sParam.DribbleToPointP.y = dribblePoint.y;
//            sParam.DribbleToPointP.finalslope = Vector2D<int>::angle(goal, dribblePoint);
//            return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
//            break;
//        case TURNING:
//            if(!(fabs(turnAngleLeft) > SATISFIABLE_THETA/2))
//            {
//              iState = KICKING;
//              printf("state changed turning to kicking\n");
//              break;
//            }
//            printf("turning to balll %f %f\n", turnAngleLeft, angleWithBall);
//            sID = SkillSet::TurnToPoint;
//            sParam.TurnToPointP.x = goal.x;
//            sParam.TurnToPointP.y = goal.y;
//#ifdef SSL_COMM
//        sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA / 5;
//#else
//        sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA*3;
//#endif
//            return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
//            break;
//        case KICKING:
//            printf("should kick now %f %f %f\n", turnAngleLeft, finalSlope, angleWithBall);
//            sID = SkillSet::Kick;
//            sParam.KickP.power = 7;
//            return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
//            if(dist > BOT_BALL_THRESH || fabs(angleWithBall) > DRIBBLER_BALL_ANGLE_RANGE)
//            {
//              iState = GOTOBALL;
//              printf("state changed kicking to gotoball\n");
//              break;
//            }
//            break;
//      } 
      Strategy::SkillSet::SkillID sID;
      SkillSet::SParam sParam;
      if((dist > BOT_BALL_THRESH*1.5 || fabs(angleWithBall) > DRIBBLER_BALL_ANGLE_RANGE))
      {
        
        sID = SkillSet::GoToBall;
//        printf("going to ball %d, %f, %f\n", BOT_BALL_THRESH, dist, angleWithBall);
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
      }
      if((fabs(turnAngleLeft) > SATISFIABLE_THETA/2))
      {
//        printf("turning to balll %f %f\n", turnAngleLeft, angleWithBall);
        sID = SkillSet::TurnToPoint;
        sParam.TurnToPointP.x = goal.x;
        sParam.TurnToPointP.y = goal.y;
  // #ifdef SSL_COMM
    sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA / 5;
  // #else
    // sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA*5;
  // #endif
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
      }
      if((dist > BOT_BALL_THRESH || fabs(angleWithBall) > DRIBBLER_BALL_ANGLE_RANGE))
      {
        
        sID = SkillSet::GoToBall;
//        printf("going to ball %d, %f, %f\n", BOT_BALL_THRESH, dist, angleWithBall);
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
      }
//      printf("should kick now %f %f %f\n", turnAngleLeft, finalSlope, angleWithBall);
      sID = SkillSet::Kick;
      sParam.KickP.power = 7;
      return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
      
//      if(fabs(turnAngleLeft) > SATISFIABLE_THETA/2)
//      {

//        return;
//      }
      
      if (botPos.absSq() < BOT_BALL_THRESH * BOT_BALL_THRESH)
      {
        // tState = COMPLETED;
      }
    }

    Tactic::Param TKickToGoal::paramFromJSON(string json) {
      using namespace rapidjson;
      Tactic::Param tParam;
      
      return tParam;
    }

    string TKickToGoal::paramToJSON(Tactic::Param tParam) {
      return string("");
    }
    
} // namespace Strategy

