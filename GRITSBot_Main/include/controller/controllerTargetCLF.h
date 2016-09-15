#ifndef _CONTROLLER_TARGET_CLF_H_
#define _CONTROLLER_TARGET_CLF_H_

#include "include/controller/controllerBase.h"
#include <math.h>

class ControllerTargetCLF : public ControllerBase {
  public: 
    /* Constructors */
    ControllerTargetCLF(float Kv = ctrl::Kv, float Kw = ctrl::Kw) {
      Kv_ = Kv;
      Kw_ = Kw;

      targetPosition_.x = 1E6;
      targetPosition_.y = 1E6;

      /* Set refresh rate */
      updateRate_ = 30;
    }

    ControllerTargetCLF(State targetPosition, 
    										float Kv = ctrl::Kv, 
    										float Kw = ctrl::Kw) {
      Kv_ = Kv;
      Kw_ = Kw;

      targetPosition_ = targetPosition;

      /* Set refresh rate */
      updateRate_ = 30;
    }

    /* Getters */
    State getTargetPosition() {return targetPosition_;}

    /* Setters */
    void setTargetPosition(State targetPosition) {
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
        State dPos     = targetPosition_ - state; 
        float thetaDes = dPos.angle(); 
        float distance = dPos.norm();
        float dTheta   = thetaDes - state.theta;

        yield();
        v_ = distance * cos(dTheta) * Kv_;
        w_ = distance * sin(dTheta) * Kw_;
      }
    }
    
    float distanceToTarget(State state) {
    	return (state - targetPosition_).norm();
    }

  private:
    State targetPosition_;

    float Kv_;
    float Kw_;
};
#endif
