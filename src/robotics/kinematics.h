/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"
#include "../definitions/robot_definitions.h"

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

        float v, w;
        float left_vel, right_vel;

        float theta, delta_theta;
        float x, y;
        float delta_left, delta_right;
        float delta_travel;
        float delta_x, delta_y;
        float travel;

        float control_period;


        float mm_to_m(const float mm){
            return mm*0.001;
        }

        float m_to_mm(const float m){
            return m*1000.0;
        }

        float rads_to_degs(const float rads){
            return rads*180.0/PI;
        }

        float degs_to_rads(const float degs){
            return degs*PI/180.0;
        }

        float rads_to_rpm(const float rads){
            return rads*60.0/(2.0*PI);
        }

        float rpm_to_rads(const float rpm){
            return rpm*2.0*PI/60.0;
        }

        float rads_to_rotations(const float rads){
            return rads/(2.0*PI);
        }

        float rotations_to_rads(const float rotations){
            return rotations*2.0*PI;
        }


    public:
        Kinematics(const float _wheel_track, const float _wheel_diameter, const float _control_period=MOTOR_CONTROL_PERIOD){
            w=0.0;
            left_velocity=0.0;
            right_velocity=0.0;
            linear_velocity=0.0;
            angular_velocity=0.0;

            wheel_track=mm_to_m(_wheel_track);
            wheel_diameter=mm_to_m(_wheel_diameter);

            wheel_radius=wheel_diameter/2.0;
            wheel_circumference=wheel_diameter*PI;

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

            control_period = _control_period;
        }

        void forward(const float linear, const float angular){
            v = mm_to_m(linear);
            w = degs_to_rads(angular);

            left_velocity=(2*v-w*wheel_track)/(wheel_diameter);
            right_velocity=(2*v+w*wheel_track)/(wheel_diameter);
        }

        void inverse(const float left, const float right){
            left_vel=rpm_to_rads(left);
            right_vel=rpm_to_rads(right);

            linear_velocity=(left_vel+right_vel)*wheel_radius/2.0;
            angular_velocity=(-left_vel+right_vel)*wheel_radius/wheel_track;
        }

        float getLeftVelocity(){
            return rads_to_rpm(left_velocity);
        }

        float getRightVelocity(){
            return rads_to_rpm(right_velocity);
        }

        float getLinearVelocity(){
            return m_to_mm(linear_velocity);
        }

        float getAngularVelocity(){
            return rads_to_degs(angular_velocity);
        }

        /*
        void updatePose(const float left_rotation, const float right_rotation){
            delta_left=left_rotation*wheel_circumference;
            delta_right=right_rotation*wheel_circumference;
            delta_travel=(delta_left+delta_right)/2.0;
            delta_theta=(-delta_left+delta_right)/(wheel_track);
            
            delta_x=delta_travel*cos(theta+delta_theta/2.0);
            delta_y=delta_travel*sin(theta+delta_theta/2.0);
            
            delta_x=delta_travel*cos(delta_theta);
            delta_y=delta_travel*sin(delta_theta);
            x+=delta_x;
            y+=delta_y;
            theta+=delta_theta;
            travel+=delta_travel; 
        }
        */

        void updatePose(){
            delta_theta=angular_velocity*control_period;
            delta_x=linear_velocity*cos(theta)*control_period;
            delta_y=linear_velocity*sin(theta)*control_period;
            delta_travel=sqrt(delta_x*delta_x+delta_y*delta_y);
            x+=delta_x;
            y+=delta_y;
            theta+=delta_theta;
            travel+=delta_travel;
        }

        void resetPose(const float initial_x=0.0, const float initial_y=0.0, const float initial_theta=0.0){
            x=mm_to_m(initial_x);
            y=mm_to_m(initial_y);
            theta=degs_to_rads(initial_theta);
            travel=0.0;
            delta_x=0.0;
            delta_y=0.0;
            delta_theta=0.0;
            delta_left=0.0;
            delta_right=0.0;
            delta_travel=0.0;
        }

        void resetTravel(){
            travel=0.0;
        }

        float getX(){
            return m_to_mm(x);
        }

        float getY(){
            return m_to_mm(y);
        }

        float getTheta(){
            return rads_to_degs(theta);
        }

        float getTravel(){
            return m_to_mm(travel);
        }

        float getDeltaX(){
            return m_to_mm(delta_x);
        }

        float getDeltaY(){
            return m_to_mm(delta_y);
        }

        float getDeltaTheta(){
            return rads_to_degs(delta_theta);
        }




};



#endif