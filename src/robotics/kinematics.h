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
        float wheel_circumference;
        float rads_to_rpm;
        float rpm_to_rads;
        float rads_to_degs;
        float degs_to_rads;
        float w;

        float theta, delta_theta;
        float x, y;
        float delta_left, delta_right;
        float delta_travel;
        float delta_x, delta_y;
        float travel;



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
            wheel_circumference=wheel_diameter*PI;

            rads_to_rpm=60.0/(2.0*PI);
            rpm_to_rads=2.0*PI/60.0;

            rads_to_degs=180.0/PI;
            degs_to_rads=PI/180.0;

            theta=0.0;
            x=0.0;
            y=0.0;

            delta_x=0.0;
            delta_y=0.0;
            delta_theta=0.0;
            delta_left=0.0;
            delta_right=0.0;
            delta_travel=0.0;
            travel=0.0;

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

        void updatePose(const float left_rotation, const float right_rotation){
            delta_left=left_rotation*wheel_circumference;
            delta_right=right_rotation*wheel_circumference;
            delta_travel=(delta_left+delta_right)/2.0;
            delta_theta=(-delta_left+delta_right)/(2.0*wheel_track);
            delta_x=delta_travel*cos(theta+delta_theta/2.0);
            delta_y=delta_travel*sin(theta+delta_theta/2.0);
            x+=delta_x;
            y+=delta_y;
            travel+=delta_travel;
        }

        void resetPose(const float initial_x=0.0, const float initial_y=0.0, const float initial_theta=0.0){
            x=initial_x;
            y=initial_y;
            theta=degs_to_rads*initial_theta;
            travel=0.0;
            delta_x=0.0;
            delta_y=0.0;
            delta_theta=0.0;
            delta_left=0.0;
            delta_right=0.0;
            delta_travel=0.0;

        }

        float getX(){
            return x;
        }

        float getY(){
            return y;
        }

        float getTheta(){
            return rads_to_degs*theta;
        }

        float getTravel(){
            return travel;
        }

        float getDeltaX(){
            return delta_x;
        }

        float getDeltaY(){
            return delta_y;
        }

        float getDeltaTheta(){
            return rads_to_degs*delta_theta;
        }




};



#endif