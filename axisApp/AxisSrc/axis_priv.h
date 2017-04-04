/* axis_priv.h: private fields, which could be outside the record */

#ifndef INC_axis_priv_H
#define INC_axis_priv_H

#include "epicsTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

  struct axis_priv {
    struct {
      double position;           /**< Commanded motor position */
      double encoderPosition;    /**< Actual encoder position */
    } readBack;
  };
  
#ifdef __cplusplus
}
#endif
#endif /* INC_axis_priv_H */