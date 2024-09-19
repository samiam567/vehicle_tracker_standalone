/*
 Copyright 2023 Alec Pannunzio <apun189@gmail.com>
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License. 
*/

using System;
using ROS2;

using blackandgold_msgs.msg;
using autoware_auto_perception_msgs.msg;
using geometry_msgs.msg;
using visualization_msgs.msg;

using System.Numerics;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;

namespace planning.multivehicle_awareness {
    
    public class Vehicle {

    
        BoundingBox bbox;
        private TrackedObject tracked_object_message = new TrackedObject();
        Marker marker = new Marker();

        Stopwatch lastUpdated = new Stopwatch();
        
        double lastUpdatedSecs = 0.0;

        private int scansSinceLastUpdate = 0; // How many lidar detections have passed since we have been detected
        private int successfulScanScore = 0; // +1 for successful scan -1 for missed scan

        protected VehicleState state; 

        public bool isValid = false; // whether we are confident that this vehicle actually exists
        
        UInt64 id;

        public Vehicle(BoundingBox box, UInt64 id) {
            this.bbox = box;

            this.id = id;
            
            state = new ComponentKalmanLinearState(Point32ToVector3(box.Centroid));
            lastUpdated.Start(); // record when we initialized this box

           
        }
 

        private System.Numerics.Vector3 Point32ToVector3(Point32 point) {
            System.Numerics.Vector3 pose = new System.Numerics.Vector3();
            pose.X = point.X;
            pose.Y = point.Y;
            pose.Z = point.Z;

            return pose;
        }

        public void UpdateState(BoundingBox box, double detectTime, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCovarianceMatrix, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCMatrix, bool onupdate) {
            // continuously update our non-centroid box data with the latest detection
            this.bbox = box;


            if (box == null) return;

          
            if (onupdate) successfulScanScore+=2; // we add 2 here since the Clean() method will subtract 1, making a new of +1 for this scan

            if (isValid) {
                // refresh scansSinceLastUpdate
                scansSinceLastUpdate = 0;
            }else{
                // check whether or not we are now a valid vehicle
                scansSinceLastUpdate -= 2; // keep subracting until we are smaller than SCANS_BEFORE_VALID
                if (scansSinceLastUpdate <= -VehicleTracker.SCANS_BEFORE_VALID*2 ) {
                    isValid = true;
                }
            }
            isValid = true; // upgrade to a valid box after we have been detected more than once

            

            System.Numerics.Vector3 pose = Point32ToVector3(box.Centroid);
          
         
            //float timeDiff = lastUpdated.Elapsed.Ticks / TimeSpan.TicksPerSecond;
            float timeDiff = (float) (detectTime - lastUpdatedSecs);
            //Console.WriteLine("Tdiff: " + timeDiff);
            lastUpdated.Restart();
            lastUpdatedSecs = detectTime;


            state.Update(pose, timeDiff, sensorCovarianceMatrix, sensorCMatrix);
            // Console.WriteLine("id: " + this.id + "  Velocity: " + this.state.GetVelocity().Length());
        }

    
        
        /**
        * Predicts what the state is right now
        * stepIterations is the number of iterations we will use to perterbate the state
        * timeDiff is the difference in time since the state was last updated
        **/
        public void PredictState(int stepIterations, float timeDiff) {
            
            float timeStep = timeDiff/stepIterations;

            state.Predict(stepIterations,timeStep, timeDiff);

            Clean(false);
        }

        /**
        * Predicts what the state is right now
        */
        public void PredictState() {
            PredictState(VehicleTracker.STD_PREDICT_ITERATIONS, (float)(((double)lastUpdated.Elapsed.Ticks) / TimeSpan.TicksPerSecond));
        }

        /**
         * <summary>Predicts what the position is right now</summary>
         **/
        public System.Numerics.Vector3 GetNowPosition() {
            PredictState();
            return state.GetNowPosition();
        }

        // <summary>Will get the position at the specified time</summary>
        public System.Numerics.Vector3 GetNowPosition(double nowTime) {
            PredictState(VehicleTracker.STD_PREDICT_ITERATIONS,(float) (nowTime-lastUpdatedSecs) );
            return state.GetNowPosition();
        }

       
        // a GetNow___ call or PredictState() MUST be called before any of these methods to generate CachedState
        public System.Numerics.Vector3 GetCachedPosition() {
            return state.GetNowPosition();
        }
        public System.Numerics.Vector3 GetCachedVelocity() {
            return state.GetNowVelocity();
        }
        public System.Numerics.Vector3 GetCachedAcceleration() {
            return state.GetNowAcceleration();
        }

        public UInt64 GetId() {
            return id;
        }

        public float GetLastUpdatedElapsed() {
            //Console.WriteLine("Time: " + ((double)lastUpdated.Elapsed.Ticks)/TimeSpan.TicksPerSecond);
            return (float)(((double)lastUpdated.Elapsed.Ticks) / TimeSpan.TicksPerSecond);
        }

        
        public TrackedObject getTrackedObjectMessage() {

            

            // calculate and cach our state at this time
            PredictState();

            System.Numerics.Vector3 position = GetCachedPosition();
            System.Numerics.Vector3 velocity = GetCachedVelocity();
            System.Numerics.Vector3 acceleration = GetCachedAcceleration();
            System.Numerics.Vector3 orientation = velocity / velocity.Length();

            if (velocity.Length() > uint.MaxValue) {
                // do nothing (reject this value) // this.bbox.Velocity = uint.MaxValue; 
            }else{
                this.bbox.Velocity = velocity.Length();
            }

            // Update bounding box message
            this.bbox.Centroid.X = position.X;
            this.bbox.Centroid.Y = position.Y; 
            this.bbox.Centroid.Z = position.Z;

            // calculate the quaternion for the orientation
            System.Numerics.Quaternion orientationQ = ROS2.Tf2DotNet.TransformBuffer.QuaternionFromVectors(System.Numerics.Vector3.UnitY, orientation);
            
            this.bbox.Orientation.X = orientationQ.X;
            this.bbox.Orientation.Y = orientationQ.Y;
            this.bbox.Orientation.Z = orientationQ.Z;
            this.bbox.Orientation.W = orientationQ.W;


           
            
            // update trackedobject message
            // tracked_object_message.Uuid.Uuid[0] = id; // FIXME I have no idea how to fix this
            tracked_object_message.Object_id = id;
            tracked_object_message.Kinematics.Centroid_position.X = position.X;
            tracked_object_message.Kinematics.Centroid_position.Y = position.Y;
            tracked_object_message.Kinematics.Centroid_position.Z = position.Z;

            // TODO include covariance with twist
            tracked_object_message.Kinematics.Twist.Twist.Linear.X = velocity.X;
            tracked_object_message.Kinematics.Twist.Twist.Linear.Y = velocity.Y;
            tracked_object_message.Kinematics.Twist.Twist.Linear.Z = velocity.Z;
            tracked_object_message.Kinematics.Acceleration.Accel.Linear.X = acceleration.X;
            tracked_object_message.Kinematics.Acceleration.Accel.Linear.Y = acceleration.Y;
            tracked_object_message.Kinematics.Acceleration.Accel.Linear.Z = acceleration.Z;

            // compute existance_probability based on scanswithoutupdate
            // TODO this is an extreme heuristic, and probably should be reformulated
            tracked_object_message.Existence_probability = MathF.Min(1.0F, MathF.Max(0.0F, successfulScanScore) / VehicleTracker.MAX_SCANS_WOUT_UPDATE_VALID);


            return tracked_object_message;
        }




        /**
        * If it has been too long since we have been detected, remove ourself from the list of tracked vehicles
        * This should be called each detection cycle
        * @Returns whether this Vehicle has been removed or not
        **/
        public bool Clean(bool onDetection) {
            bool remove = false;

            // TODO potentially change this to take into account distance from the ego_agent

            if (scansSinceLastUpdate > VehicleTracker.MAX_SCANS_WOUT_UPDATE_DELETE) {
                remove = true; // this detection is very stale, delete it completely
            } else if (scansSinceLastUpdate > VehicleTracker.MAX_SCANS_WOUT_UPDATE_VALID) {
                this.isValid = false; // this detection is stale, don't publish it anymore
            }


            // check the time since we were last updated
            if (GetLastUpdatedElapsed() > VehicleTracker.MAX_TIME_WOUT_UPDATE_VALID) {
                this.isValid = false; // this detection is stale, don't publish it anymore
                //Console.WriteLine("you're done");
            }

            
            if (onDetection) {
                scansSinceLastUpdate++;
                successfulScanScore--;
            }
            
            return remove;
        }
    }




}
