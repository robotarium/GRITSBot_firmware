#ifndef _CONTROLLER_BASE_H_
#define _CONTROLLER_BASE_H_

#include <include/utilities/constants.h>
#include <include/utilities/state.h>

class ControllerBase {
  public:
    /* Constructor */
    ControllerBase() {
    	v_ = 0; 
    	w_ = 0; 
    	l_ = ctrl::L;
    }

    /* Getters and Setters */
    float getV() const { return v_; }
    float getW() const { return w_; }
    float getL() {return l_;}
    float getGain() const {return k_;}

    void setL(float l) { l_ = l; }
    void setV(float v) { v_ = v; }
    void setW(float w) { w_ = w; }
    void setGain(float k) {k_ = k;}

    /* Arena boundary check */
    bool isWithinBoundaries(State x) {
      if(x.x < arena::minX || x.x > arena::maxX ||
         x.y < arena::minY || x.y > arena::maxY) {
        return false;
      } else {
        return true;
      }
    }

    /* Virtual update function to be implemented in 
     * every derived class
     */
    virtual void update(State state) {}

    /* Virtual functions for other types of controller */
    /*virtual void setGain(float k) = 0;*/
    virtual void setTargetPosition(State targetPosition) = 0;
    virtual State getTargetPosition() = 0;
    virtual float distanceToTarget(State state) = 0;

  protected:
    unsigned long lastUpdate_;        /* Time stamp of last update */
    uint16_t      updateRate_;        /* Refresh rate in [Hz] */

    float v_, w_;	/* Linear and rotational velocities */
    float l_;		  /* Linearization point for velocity controller */
    float k_;     /* Position controller gain */
};
#endif
