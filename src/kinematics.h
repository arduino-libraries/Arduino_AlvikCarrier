#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"

class Kinematics{
    private:
        float left_velocity;
        float right_velocity;
        float linear_velocity;
        float angular_velocity;
        float wheel_track;
        float wheel_diameter;
        float wheel_radius;


    public:
        Kinematics(const float _wheel_track, const float _wheel_diameter){
            left_velocity=0.0;
            right_velocity=0.0;
            linear_velocity=0.0;
            angular_velocity=0.0;
            wheel_track=_wheel_track;
            wheel_diameter=_wheel_diameter;
            wheel_radius=wheel_diameter/2.0;
        }

        void forward(const float linear, const float angular){
            left_velocity=(2*linear_velocity-angular_velocity*wheel_track)/(wheel_diameter);
            right_velocity=(2*linear_velocity+angular_velocity*wheel_track)/(wheel_diameter);
        }

        void inverse(const float left_velocity, const float right_velocity){
            linear_velocity=(left_velocity+right_velocity)*wheel_radius/2.0;
            angular_velocity=(-left_velocity+right_velocity)*wheel_radius/wheel_track;
        }

        float getLeftVelocity(){
            return left_velocity;
        }

        float getRightVelocity(){
            return right_velocity;
        }

        float getLinearVelocity(){
            return linear_velocity;
        }

        float getAngularVelocity(){
            return angular_velocity;
        }


};



#endif