#ifndef Avatar_params_h
#define Avatar_params_h

// Safety Limit Parameters

#define VELOCITY_LIMIT 10000              // Minimum velocity that will trigger STANDBY mode
#define H_LINK_ENABLE_DISTANCE 0.01       // Maximum distance that will trigger ACTIVE mode
#define H_LINK_DISABLE_DISTANCE 0.1       // Minimum distance that will trigger STANDBY mode 
#define V_LINK_ENABLE_DISTANCE 0.01       // Maximum distance that will trigger ACTIVE mode
#define V_LINK_DISABLE_DISTANCE 0.1       // Minimum distance that will trigger STANDBY mode 
#define LINK_TIMEOUT                      // Minimum time between link updates that will trigger STANDBY mode  NOT IMPLEMENTED
#define ENCODER_TIMEOUT                   // Minimum time between encoder updates that will trigger ERROR mode MOT IMPLEMENTED


// Peformance Parameters

#define H_HIGH_STIFFNESS 100
#define H_LOW_STIFFNESS 10
#define H_HIGH_DAMPING 0
#define H_LOW_DAMPING 0
#define H_STANDBY_DAMPING 0

#define V_HIGH_STIFFNESS 100
#define V_LOW_STIFFNESS 10
#define V_HIGH_DAMPING 0
#define V_LOW_DAMPING 0
#define V_STANDBY_DAMPING 0

#define POS_TC (float) 100.0 // Microseconds

#endif
