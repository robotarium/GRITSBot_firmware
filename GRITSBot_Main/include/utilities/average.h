/*
 -------------------------------------------------------------------------------
 Definition of class 'Average'

 DESCRIPTION: This class faciliates computing running averages.
 
 METHODS:
    void addData(float d);
      Adds a data point to the running average stored in this instance.

    float getAndKeepAverage();
      Returns the current average value stored in this instance WITHOUT
      resetting the class variables storing the data. 

    float getAverage();
      Returns the current average value stored in this instance AND 
      resets the class variables storing the data. 

    void reset();
      Resets the class variables storing the data in this class instance
      without returnin any data.

 NOTES: This implementation is not templated and stores only float values.
 
 EXAMPLES:
    Example 1: Instantiating an instance of class 'Average', adding and 
                getting data
    -----------------------------------------------------------------------------
    #include "include/average.h"

    Average current_;

    current_.addData(0.45);
    current.getAverage();
    -----------------------------------------------------------------------------
 
 Initially created by Daniel Pickem 8/2/15.
 -------------------------------------------------------------------------------
 */

#ifndef _AVERAGE_H_
#define _AVERAGE_H_

class Average{
  public:
    //--------------------------------------------------------------------------
    // Lifecycle
    //--------------------------------------------------------------------------
    /* Constructor */
    Average() {data_ = 0; count_ = 0;}

    /* Destructor */
    ~Average() {}

    //--------------------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------------------
    /* Input functions */ 
    void addData(float data) {
      data_ += data;
      count_++;
    }

    /* Output functions */
    float getAndKeepAverage() {
      if(count_ > 0) {
        return data_ / count_;
      } else {
        return 0.0;
      }
    }
        
    float getAverage() {
      if(count_ > 0) {
        float result = data_ / count_;
        reset();
        return result;
      } else {
        return 0.0;
      }
    }

  private:
    //--------------------------------------------------------------------------
    // Private Member Functions
    //--------------------------------------------------------------------------
    /* Reset data and count member variable */
    void reset() {
      data_   = 0.0;
      count_  = 0;
    }

    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    float data_;
    uint32_t count_;
};
#endif
