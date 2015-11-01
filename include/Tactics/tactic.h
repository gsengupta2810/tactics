#ifndef TACTIC_H
#define TACTIC_H
#include <list>
// #include <cstring>
#include <string>
#include <krssg_ssl_msgs/BeliefState.h>

using namespace std;
using krssg_ssl_msgs::BeliefState;
namespace Strategy
{
  class TParam;
  class Tactic
  {
  public:
    inline Tactic() {}
    inline ~Tactic()
    {
    } // ~Tactic
    // True if tactic execution looks completed
    virtual bool isCompleted(const BeliefState &bs) const = 0;
    // True if the tactic is active, i.e., it involves ball manipulation
    virtual bool isActiveTactic() const = 0;
    // TODO As per the design, this function must be static but it cannot be both virtual as well as static
    /* This function assigns a score to all the free bots (bots that have not been
     * assigned any role yet) depending on how suitable they are in being assigned
     * the tactic. It then chooses the bot with the highest score and assigns it the tactic.
     */
    virtual int chooseBestBot(const BeliefState &bs, std::list<int>& freeBots, const TParam& tParam, int prevID = -1) const = 0;
    /* This function takes in the current tactic parameter for a robot and
     * using the belief state info and the skill transition rules,
     * it either decides to transit to another skill or continues to run
     * the current skill.
     */
    virtual void execute(const TParam& tParam) = 0;
   
  
  }; // class Tactic

  // base class for tactic param.
  class TParam {
  public:
    // function to initialize param from json string
    virtual void fromJSON(string json) = 0;
    // function that returns a string with has its params in JSON format
    virtual string toJSON() = 0;
  };
} // namespace Strategy
#endif // TACTIC_H
