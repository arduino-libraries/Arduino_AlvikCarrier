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
        float rads_to_rpm;
        float rpm_to_rads;
        float rads_to_degs;
        float degs_to_rads;
        float w;


    public:
        Kinematics(const float _wheel_track, const float _wheel_diameter){
            w=0.0;
            left_velocity=0.0;
            right_velocity=0.0;
            linear_velocity=0.0;
            angular_velocity=0.0;
            wheel_track=_wheel_track;
            wheel_diameter=_wheel_diameter;
            wheel_radius=wheel_diameter/2.0;

            rads_to_rpm=60.0/(2.0*PI);
            rpm_to_rads=2.0*PI/60.0;

            rads_to_degs=180.0/PI;
            degs_to_rads=PI/180.0;

        }

        void forward(const float linear, const float angular){
            w = angular*degs_to_rads;
            left_velocity=(2*linear-w*wheel_track)/(wheel_diameter);
            left_velocity=rads_to_rpm*left_velocity;
            right_velocity=(2*linear+w*wheel_track)/(wheel_diameter);
            right_velocity=rads_to_rpm*right_velocity;
        }

        void inverse(const float left_vel, const float right_vel){
            linear_velocity=(left_vel+right_vel)*wheel_radius/2.0;
            angular_velocity=(-left_vel+right_vel)*wheel_radius/wheel_track;
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