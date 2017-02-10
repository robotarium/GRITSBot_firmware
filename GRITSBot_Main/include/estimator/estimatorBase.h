#ifndef _EST_BASE_H_
#define _EST_BASE_H_

#include <GRITSBot_Main/include/utilities/constants.h>
#include <GRITSBot_Main/include/utilities/state.h>

class EstimatorBase {
  public:
    /* Constructor */
    EstimatorBase() {
    	state_ 			= State();
    	lastUpdate_ = millis();
    	updateRate_ = 30;
    }

    /* Update method */
    virtual void update(float v, float w, State z = State()) {
      float dt = float(millis() - lastUpdate_);

      if(dt > 1.0 / updateRate_) {
		    state_.x  		+= v * cos(state_.theta) * dt;
		    state_.y  		+= v * sin(state_.theta) * dt;
		    state_.theta 	+= w * dt;

		    lastUpdate_ = millis();
      }
    };

    /* Getters/setters */
    void setState(State state) { state_ = state; }
    State getState() {return state_; }

	protected:
    State state_;
    unsigned long lastUpdate_;        /* Time stamp of last update */
    uint16_t      updateRate_;        /* Refresh rate in [Hz] */
};

#endif
