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
#include <fstream>

#define THRES (0.6f)
#define TEAMSIZE (6)
#define FACTOR (1.5f)

namespace Strategy
{
  TKickToGoal::TKickToGoal(int botID) : Tactic( botID) { 
   
    goal = Vector2D<int>(HALF_FIELD_MAXX, 0);
  } 
  
  TKickToGoal::~TKickToGoal() { } 

  bool TKickToGoal::isCompleted(const BeliefState &bs) const {
    return iState == FINISHED;
  }
  
  inline bool TKickToGoal::isActiveTactic(void) const {
    return iState != FINISHED;
  }

  int TKickToGoal::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const {
  
    int minv = *(freeBots.begin());
    float mindis = -1;
    for (std::list<int>::const_iterator it = freeBots.begin(); it != freeBots.end(); ++it)
    {
      Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

      float dis_from_ball = (homePos - ballPos).absSq();
      if (mindis < 0) {
        mindis = dis_from_ball;
        minv = *it; 
      }
      else if(dis_from_ball < mindis) {
        mindis = dis_from_ball;
        minv = *it;
      }
    }
    assert(mindis >= 0.0f);
    return minv;
  }

  float getRandom() {
    float randValue = (float)rand() / RAND_MAX + 0.000001f;
    if (randValue >= 1.0f) {
      randValue = 0.999999f;
    }
    return randValue;
  }
    
  gr_Robot_Command TKickToGoal::execute(const BeliefState &state, const Tactic::Param& tParam) {
    //fstream f;
    //f.open("/home/shivanshu05/catkin_ws/log.txt", fstream::app);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    float dist = Vector2D<int>::dist(ballPos, botPos);
    
    Vector2D<int> goalMax(HALF_FIELD_MAXX, OPP_GOAL_MAXY);
    Vector2D<int> goalMin(HALF_FIELD_MAXX, OPP_GOAL_MINY);
    float angleMax = Vector2D<int>::angle(goalMax, botPos);
    float angleMin = Vector2D<int>::angle(goalMin, botPos);

    Vector2D<int> destination;
    float angleUp = angleMax, angleDown = angleMin;
    if (dist >= DRIBBLER_BALL_THRESH) {
      iState=GOTOBALL;
      //f << "GO tTO BALL\n";
    }
    else if(state.homePos[botID].theta >= angleMax || state.homePos[botID].theta <= angleMin) {
    	iState = TURNING;
      //f<< "TURN TO FACE GOAL\n";
    	destination = Vector2D<int>(HALF_FIELD_MAXX, 0);
    }    
    else {
      std::vector<float> obsAngle;
    	std::vector<bool> angleHighInShootRange;
    	obsAngle.push_back(angleMin);
      angleHighInShootRange.push_back(false);
      obsAngle.push_back(angleMax);
      angleHighInShootRange.push_back(true); 	
    	//f << "angle max = " << angleMax << ", angle min = " << angleMin << endl; 
      for(int bID = 0; bID < TEAMSIZE; bID++) {
        Vector2D<int> objPos(state.awayPos[bID].x, state.awayPos[bID].y);
        float objDist = Vector2D<int>::dist(botPos, objPos);
        float alpha = asin(FACTOR * BOT_RADIUS / objDist);
        float objAngle = Vector2D<int>::angle(objPos, botPos);
        float angle1 = objAngle + alpha; 
        float angle2 = objAngle - alpha;
        //f << "bot ID = " << bID << ", angle1 = " << angle1 << ", angle2 = " << angle2 << "\t";
        if(angle1 > angleMax && angle2 < angleMin) {
          //ABORT!!! CANT SHOOT.
          //f << "CANT SHOOT!" << endl;
        }
        else if (angle1 < angleMin || angle2 > angleMax) {
          //NOT AN OBSTACLE
          //f << "not an obstacle" << endl;
        }
        else {
          if(angle1 <= angleMax && angle2 >= angleMin) {
            //f << "add both angles" << endl;
            obsAngle.push_back(angle1);
            angleHighInShootRange.push_back(false);
            obsAngle.push_back(angle2);
            angleHighInShootRange.push_back(true);
          }
          else if(angle1 >= angleMax && angle2 >= angleMin) {
            //f << "add lower value, manipulate angle max" << endl;
            angleHighInShootRange[1] = false;
            obsAngle.push_back(angle2);
            angleHighInShootRange.push_back(true);
          }
          else if(angle2 <= angleMin && angle1 <= angleMax) {
            //f << "add higher value, manipulate angle min" << endl;
            obsAngle.push_back(angle1);
            angleHighInShootRange.push_back(false);
            angleHighInShootRange[0] = true;
          }
        }       
      }

      
      for(int bID = 0; bID < TEAMSIZE && bID != botID; bID++) {
        Vector2D<int> objPos(state.homePos[bID].x, state.homePos[bID].y);
        float objDist = Vector2D<int>::dist(botPos, objPos);
        float alpha = asin(FACTOR * BOT_RADIUS / objDist);
        float objAngle = Vector2D<int>::angle(objPos, botPos);
        float angleUp = objAngle + alpha, angleDown = objAngle - alpha;

        if(angleUp >= angleMax && angleDown <= angleMin) {
          //ABORT!!! CANT SHOOT.
        }
        else if (angleUp < angleMin || angleDown > angleMax) {
          //NOT AN OBSTACLE
        }
        else {
          if(angleUp <= angleMax && angleDown >= angleMin) {
            //OBJECT FULLY INSIDE SHOOTING ANGLE, ADD BOTH ANGLES
            obsAngle.push_back(angleUp);
            angleHighInShootRange.push_back(false);
            obsAngle.push_back(angleDown);
            angleHighInShootRange.push_back(true);
          }
          else if(angleUp >= angleMax && angleDown >= angleMin) {
            //OBJECT HIDING THE UPPER GOAL POST, ADD LOWER VALUE, MANIPULATE angleMax
            angleHighInShootRange[1] = false;
            obsAngle.push_back(angleDown);
            angleHighInShootRange.push_back(true);
          }
          else if(angleDown <= angleMin && angleUp <= angleMax) {
            //OBJECT HIDING THE LOWER GOAL POST, ADD UPPER VALUE, MANIPULATE angleMin
            obsAngle.push_back(angleUp);
            angleHighInShootRange.push_back(false);
            angleHighInShootRange[0] = true;
          }
        }   
      }

      for (int i = 0; i < obsAngle.size(); i++) {
        for(int j = 1; j < obsAngle.size(); j++) {
          if(obsAngle[j] < obsAngle[j-1]) {
            float temp = obsAngle[j];
            obsAngle[j] = obsAngle[j-1];
            obsAngle[j-1] = temp;

            bool tmp = angleHighInShootRange[j];
            angleHighInShootRange[j] = angleHighInShootRange[j-1];
            angleHighInShootRange[j-1] = tmp;
          }
        }
      }
      //f << "ANGLEMAX = " << angleMax << ", ANGLEMIN = " << angleMin << endl;
      //for(int i = 0; i < obsAngle.size(); i++) {
      //  f << i << "]\t" << obsAngle[i] << endl;
      //}

      float MAX_INTERVAL = -1.0f, MAX_INTERVAL_UP = 0.0f, MAX_INTERVAL_DOWN = 0.0f;
      for (int i = 1; i < obsAngle.size(); i++) {
        if(!angleHighInShootRange[i-1] && angleHighInShootRange[i]) {
          float angleRange = obsAngle[i] - obsAngle[i-1];
          if(angleRange > MAX_INTERVAL) {
            angleUp = obsAngle[i];
            angleDown = obsAngle[i-1];
          }
        }
      }                          
      float ballBotAngle = Vector2D<int>::angle(ballPos, botPos);
      //f << "angle up = " << angleUp << ", angle down = " << angleDown << endl;  
      float randv = getRandom();
      randv = randv > THRES ? 1 - randv : randv;
      //randv = randv < 1-THRES ? 1 - randv : randv;
      float angle1 = angleUp * randv + (1-randv) * angleDown;
      float angle2 = angleUp * (1-randv) + randv * angleDown;
      if(state.homePos[botID].theta <=angle1 && state.homePos[botID].theta >= angle2) {
          iState=KICKING;
          //for(int i = 0; i < obsAngle.size(); i++) {
          //  f << i << "]\t" << obsAngle[i] << endl;
          //}
      }
      else if (state.homePos[botID].theta < (angleDown*0.5 + angleUp*0.5)){
      		iState=TURNING;
          destination.x = HALF_FIELD_MAXX;
          destination.y = OPP_GOAL_MAXY;
          //f << "TURN TO DESIRED ANGLE" << endl;
    	}
      else if(state.homePos[botID].theta > (angleUp*0.5 + angleDown*0.5)) {
        iState = TURNING;
        destination.x = HALF_FIELD_MAXX;
        destination.y = OPP_GOAL_MINY;
        //f << "TURN TO DESIRED ANGLE" << endl;
      }
    	
    }
    
    Strategy::SkillSet::SkillID sID;
    SkillSet::SParam sParam;
    
    switch(iState)
    {
      case GOTOBALL:
      {
        sID = SkillSet::GoToBall;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }
      case TURNING:
      {
        sID = SkillSet::TurnToPoint;
        sParam.TurnToPointP.x = destination.x;
        sParam.TurnToPointP.y = destination.y;
        sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }  
      case KICKING:
      {
        sID = SkillSet::Kick;
        sParam.KickP.power = 7.0f;
        iState = FINISHED;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }        

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
    
} 
