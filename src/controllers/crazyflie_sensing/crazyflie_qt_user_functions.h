#ifndef CRAZYFLIE_QT_USER_FUNCTIONS_H
#define CRAZYFLIE_QT_USER_FUNCTIONS_H

/* Allow to draw objects in the arena */
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/crazyflie/simulator/crazyflie_entity.h>

using namespace argos;

class CCrazyflieQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CCrazyflieQTUserFunctions() {}

   virtual ~CCrazyflieQTUserFunctions() {}

   virtual void DrawInWorld();
   
};

#endif