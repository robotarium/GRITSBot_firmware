#ifndef _CONTROLLER_TARGET_H_
#define _CONTROLLER_TARGET_H_

#include "include/controller/controllerBase.h"
#include <math.h>

class ControllerTarget : public ControllerBase {
  public: 
    /* Constructors */
    ControllerTarget(float k = ctrl::K) {
      k_ = k;
      targetPosition_.x = 1E6;
      targetPosition_.y = 1E6;

      /* Set refresh rate */
      updateRate_ = 30;
    }

    ControllerTarget(float k, State targetPosition) {
      k_ = k;
      targetPosition_ = targetPosition;

      /* Set refresh rate */
      updateRate_ = 30;
    }

    /* Getters */
    State getTargetPosition() {return targetPosition_;}
    
    /* Setters */
    void  setTargetPosition(State targetPosition) {
      targetPosition_ = targetPosition;
    }

    /* Controller update method */
    void update(State state) {
      /* NOTE: v ... [m/sec]
       *       w ... [rad/sec]
       */
    
      /* Allow WiFi stack to execute */
      yield();

      if( (millis() - lastUpdate_) > 1.0/updateRate_) {
        /* Calculate linear velocities vx, vy */
        float vx = k_ * (targetPosition_.x - state.x);
        float vy = k_ * (targetPosition_.y - state.y);

        /* Translate to unicycle model */
        yield();
        v_  = ( cos(-state.theta) * vx - sin(-state.theta) * vy );
        w_  = ( sin(-state.theta) * vx + cos(-state.theta) * vy ) / l_;
      }
    }
    
    float distanceToTarget(State state) {
    	return (state - targetPosition_).norm();
    }

  private:
    State targetPosition_;
};
#endif
