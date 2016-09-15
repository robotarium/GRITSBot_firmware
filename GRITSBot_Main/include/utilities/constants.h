#ifndef _CONSTANTS_
#define _CONSTANTS_

namespace rb {
    const float RADIUS 		  = 0.015;
    const float WIDTH  		  = 0.03;
    const float V_WHEEL_MAX = 0.1;
}

namespace ctrl {
    const float K 	= 1.0;
    const float Kv  = 1.0;
    const float Kw  = 1.0;
    const float L   = 0.02;
}

namespace arena {
  const float minX = -0.6;
  const float maxX =  0.6;
  const float minY = -0.4;
  const float maxY =  0.4;
}
#endif 
