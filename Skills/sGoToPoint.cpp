/*
#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>

using namespace std;
namespace Strategy
{
  gr_Robot_Command SkillSet::goToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    using Navigation::obstacle;
    Vector2D<int> dpoint;
    dpoint.x = param.GoToPointP.x;
    dpoint.y = param.GoToPointP.y;
//    printf("point= %d,%d\n",dpoint.x,dpoint.y);
    vector<obstacle> obs;
    for(int i = 0; i < state.homeDetected.size(); ++i)
    {
      if (state.homeDetected[i]) {
        obstacle o;
        o.x = state.homePos[i].x;
        o.y = state.homePos[i].y;
        o.radius = 3 * BOT_RADIUS;
        obs.push_back(o);
      }
    }

    for(int i = 0; i < state.awayDetected.size(); ++i)
    {
      if (state.awayDetected[i]) {
        obstacle o;
        o.x = state.awayPos[i].x;
        o.y = state.awayPos[i].y;
        o.radius = 3 * BOT_RADIUS;
        obs.push_back(o);
      }
    }
    Vector2D<int> point, nextWP, nextNWP;
    // use the mergescurver planner
    Navigation::MergeSCurve pathPlanner;
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    pathPlanner.plan(botPos,
                      dpoint,
                      &nextWP,
                      &nextNWP,
                      &obs[0],
                      obs.size(),
                      botID,
                      true);
    return Navigation::waypointCommand(botID, state, nextWP, nextNWP, param.GoToPointP.finalslope, param.GoToPointP.align);
  } // goToPoint
}
*/
#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <cstdio>
#include <vector>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>

#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>

#define POINTPREDICTIONFACTOR 2

using namespace std;

namespace Strategy
{
  gr_Robot_Command SkillSet::goToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    using Navigation::obstacle;
    #if 1
      vector<obstacle> obs;
      for(int i = 0; i < state.homeDetected.size(); ++i) {
        if (state.homeDetected[i]) {
          obstacle o;
          o.x = state.homePos[i].x;
          o.y = state.homePos[i].y;
          o.radius = 2 * BOT_RADIUS;
          obs.push_back(o);
        }
      }

      for(int i = 0; i < state.awayDetected.size(); ++i) {
        if (state.awayDetected[i]) {
          obstacle o;
          o.x = state.awayPos[i].x;
          o.y = state.awayPos[i].y;
          o.radius = 2 * BOT_RADIUS;
          obs.push_back(o);
        }
      }
      
      Vector2D<int> pointPos;
      pointPos.x = param.GoToPointP.x;
      pointPos.y = param.GoToPointP.y;
      Vector2D<int> point, nextWP, nextNWP;
      Navigation::MergeSCurve pathPlanner;
      Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);

      pathPlanner.plan(botPos,
                        pointPos,
                        &nextWP,
                        &nextNWP,
                        &obs[0],
                        obs.size(),
                        botID,
                        true);
    #else
    
    #endif

    #if 1
    
    float dist = Vector2D<int>::dist(botPos, pointPos);
    float maxDisToTurn = dist - 3.5f * BOT_BALL_THRESH;
    float angleToTurn = normalizeAngle(param.GoToPointP.finalslope - (state.homePos[botID].theta)); 
    
    float minReachTime = maxDisToTurn / MAX_BOT_SPEED;
    float maxReachTime = maxDisToTurn / MIN_BOT_SPEED;
    
    float minTurnTime = angleToTurn / MAX_BOT_OMEGA;
    float maxTurnTime = angleToTurn / MIN_BOT_OMEGA;
    
    float speed = 0.0f;
    float omega = angleToTurn * MAX_BOT_OMEGA / (2 * PI);
    
    if (omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA) {
      if (omega < 0) omega = -MIN_BOT_OMEGA;
      else omega = MIN_BOT_OMEGA;
    }

    if(maxDisToTurn > 0) {
      if(minTurnTime > maxReachTime) {
        speed = MIN_BOT_SPEED;
      }
      else if (minReachTime > maxTurnTime) {
        speed = MAX_BOT_SPEED;
      }
      else if(minReachTime < minTurnTime) {
        speed =  maxDisToTurn / minTurnTime;
      }
      else if (minTurnTime < minReachTime) {
        speed = MAX_BOT_SPEED;
      }
    }
    else {
      speed = dist / MAX_FIELD_DIST * MAX_BOT_SPEED;
    }

    float motionAngle = Vector2D<int>::angle(nextWP, botPos);
    float theta =  motionAngle - state.homePos[botID].theta;

    if(param.GoToPointP.align == false) {
      if (dist < DRIBBLER_BALL_THRESH) {
        if(dist < BOT_BALL_THRESH) {          
          return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
        }
        else {
          return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), omega, 0, true);
        }
      }
      else {
        return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), omega, 0, false);
      }
    }
    else {
      if (dist > BOT_BALL_THRESH) {
        return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), 0, 0, false);
      }
      else {
        return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
      }

    }
    
       
  #else
  
  #endif
  }
}
