#ifndef UPO_RRT_STATE_
#define UPO_RRT_STATE_

//#include <ros/ros.h>

namespace upo_RRT {

	class State
	{
		public:
			State();
			State(float x, float y, float yaw = 0.0, float lv = 0.0, float av = 0.0);
		    ~State();
	
			void getState(float &x, float &y, float &yaw, float &lv, float &av);
			float getX();
			float getY();
			float getYaw();
			float getLinVel();
			float getAngVel();

			void setX(float x);
			void setY(float y);
			void setYaw(float yaw);
			void setLv(float lv);
			void setAv(float av);

		private:
		
			float 		x_;

			float 		y_;

			float 		yaw_;

			float 		lin_vel_;

			float 		ang_vel_;

	};
}
#endif
