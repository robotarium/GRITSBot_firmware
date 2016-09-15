#ifndef _STATE_
#define _STATE_

class State{
	public:
		State(float x_ = 0, float y_ = 0, float theta_ = 0) { 
      x = x_; 
      y = y_; 
      theta = theta_;
    }

    /* Status functions */
    bool isEmpty() {
      float prec = 0.001;
      if(abs(x) < prec && abs(y) < prec && abs(theta) < prec) {
        return true;
      } else {
        return false;
      }
    }

    /* Arithmetic functions */
    float norm() {
      return sqrt(x*x + y*y);
    }

    float angle() {
      return atan2(y, x);
    }

    /* Overloaded operators */
    State operator-(const State& b) {
      State s;

      s.x     = this->x - b.x;
      s.y     = this->y - b.y;
      s.theta = this->theta - b.theta;

      return s;
    }

    State operator+(const State& b) {
      State s;

      s.x     = this->x + b.x;
      s.y     = this->y + b.y;
      s.theta = this->theta + b.theta;

      return s;
    }

    State& operator*=(const float k) {
      this->x     *= k;
      this->y     *= k;
      this->theta *= k;
      return *this;
    }

	public:
		float x;
		float y;
		float theta;
};
#endif 
