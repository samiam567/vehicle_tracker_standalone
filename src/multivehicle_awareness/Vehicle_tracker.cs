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

// #define DEBUG // DEBUG to display the vehicles we are tracking to the console

using System;
using ROS2;

using blackandgold_msgs.msg;
using autoware_auto_perception_msgs.msg;
using autoware_auto_geometry_msgs.msg;
using visualization_msgs.msg;

using System.Numerics;
using System.Collections.Generic;
using System.Diagnostics;

using MathNet.Numerics;
using MathNet.Numerics.Random;
using Plotly.NET;



namespace planning.multivehicle_awareness
{

    public class VehicleTracker {
        
        public static readonly int SCANS_BEFORE_VALID = 3; // the number of times we must scan the object consecutively before marking it as valid & publishing as output

        public static readonly int MAX_SCANS_WOUT_UPDATE_VALID = 5; //the number of detections without detecting a given vehicle we will keep it as a valid tracked vehicle
        public static readonly int MAX_SCANS_WOUT_UPDATE_DELETE = 30; // the number of detection without detecting a given vehicle before we delete it completely
        
        public static readonly float MAX_TIME_WOUT_UPDATE_VALID = 3.0F; // the maximum time without detecting a given vehicle we will keep it as a valid tracked vehicle : Note that this is validated after calling the Predict method

        // DEPRECTED
        public static readonly int STD_PREDICT_ITERATIONS = 10;  // the standard number of iterations we will use when predicting a state
        
        public static readonly float DETECT_DIST_VEL_PERCENT = 0.2F; // __ * velocity is the max radius we will consider a new detection to be a part of a given vehicle 
        public static readonly float DETECT_DIST_HARD_RADIUS = 2.5F; // the hard distance (m) from where we expect a detection to be for us to consider it to be a part of that vehicle
        
        const string expected_frame_id = "center_of_gravity";
    
        INode node;

        List<Vehicle> tracked_vehicles = new List<Vehicle>();
        

        TrackedObjects tracked_object_array_msg = new TrackedObjects();


        // this+1 = the next unique id that we will assign a new detection
        UInt64 vehicle_id_counter = 1;


        //publishers
        Publisher<blackandgold_msgs.msg.ErrorReport> error_msg_pub;
        Publisher<TrackedObjects> tracked_vehicle_boxes_pub;
        Publisher<visualization_msgs.msg.MarkerArray> tracked_vehicle_markers_pub;


        // error report message
        blackandgold_msgs.msg.ErrorReport error_msg = new blackandgold_msgs.msg.ErrorReport();


        ROS2.Tf2DotNet.TransformBuffer tf_buffer = new ROS2.Tf2DotNet.TransformBuffer(); // create a buffer to store tf tree
        ROS2.Tf2DotNet.TransformListener tf_listener;
        static ROS2.Clock clock; // clock so we can grab rostime for messages
        
        public VehicleTracker(INode node, Publisher<TrackedObjects> tracked_vehicles_pub, Publisher<MarkerArray> tracked_vehicles_markers_pub) {
            this.node = node;

            // load publishers
            this.tracked_vehicle_boxes_pub = tracked_vehicles_pub;
            this.tracked_vehicle_markers_pub = tracked_vehicles_markers_pub;

            // create publishers
            error_msg_pub = node.CreatePublisher<blackandgold_msgs.msg.ErrorReport>("/planning/errors");

            // wait
            // System.Threading.Thread.Sleep(5000);

            error_msg.Origin = "vehicle_tracker_node";
            error_msg.Module = "planning";
            error_msg.Lifetime = 1;
            

  
            tf_listener = new ROS2.Tf2DotNet.TransformListener(tf_buffer, node); // create a tf_listener
            clock = new ROS2.Clock();


        }


        // test function for tf2_dotnet (it is safe to delete this method)
        public void testTFS(std_msgs.msg.Bool msg) {
            //  {return this -> tfBuffer_ -> canTransform(toFrame_, item.header.frame_id, tf2::TimePointZero);},
            geometry_msgs.msg.TransformStamped transform = tf_buffer.LookupTransform("base_link", "center_of_gravity");

            if (transform != null) {
                Console.WriteLine(string.Format("Found transform with translation {0} {1} {2} , quat: {3} {4} {5} {6}", 
                    transform.Transform.Translation.X,
                    transform.Transform.Translation.Y,
                    transform.Transform.Translation.Z,
                    transform.Transform.Rotation.X,
                    transform.Transform.Rotation.Y,
                    transform.Transform.Rotation.Z,
                    transform.Transform.Rotation.W
                    )
                );
        
            }
        
        }

        /**
         * determines whether a box and vehicle can be considered the same entity
         **/
        private bool closeEnough(float distance, Vehicle cVeh, BoundingBox box) {
            float cVeh_velocity = cVeh.GetCachedVelocity().Length();
            return distance < (cVeh_velocity==0 ? DETECT_DIST_HARD_RADIUS*4 : DETECT_DIST_HARD_RADIUS + DETECT_DIST_VEL_PERCENT*cVeh_velocity);
        }

        

        /**
         * @summary this method receives detections from the sensor callback methods, pairs them with an existing tracked vehicle or creates one, and then will call that vehicle's update method with the new detection. Finally it will tell the vehicles to clean themselves.
         * @params
         *      msg                    : the array of BoundingBoxes message for the received detections
         *      sensorCovarianceMatrix : the covariance matrix for the sensor that got these detections
         *      sensorCMatrix          : the C matrix for the sensor that got these detection. See kalman filter theorey
         *      contToDelete           : the tracked vehicles will become invalid and then delete themselves after a number of detection cycles. The cycles are only incremented if this parameter is true
        **/
        public void receiveBoxes(BoundingBoxArray msg, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCovarianceMatrix, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCMatrix, bool contToDelete) {
            
            if (msg.Header.Frame_id != expected_frame_id) {
  
                error_msg.Description = "Recieved detection with wrong frame id! <" + msg.Header.Frame_id + "> should be <" + expected_frame_id + ">";
                error_msg.Severity = 3;
                error_msg.Lifetime = 10;
                error_msg_pub.Publish(error_msg);
       
            }
            

            
          
            var boxes = msg.Boxes;
            double timeSecs = msg.Header.Stamp.Sec + msg.Header.Stamp.Nanosec / 1000000000.0;

            // Predict the position of the tracked_vehicles for fast use in the rest of this method
            foreach (Vehicle cVeh in tracked_vehicles){
                cVeh.GetNowPosition(timeSecs);
            }

            // seperate the tracked_vehicles between the valid and invalid ones
            List<Vehicle> valid_vehicles = new List<Vehicle>();
            List<Vehicle> invalid_vehicles = new List<Vehicle>();
            foreach (Vehicle cVeh in tracked_vehicles){
                if (cVeh.isValid) {
                    valid_vehicles.Add(cVeh);
                }else{
                    invalid_vehicles.Add(cVeh);
                }

            }


            // create a list of boxes to store the boxes we haven't paired yet
            List<BoundingBox> unpairedBoxes = new List<BoundingBox>();
            foreach (BoundingBox cBox in boxes){
                unpairedBoxes.Add(cBox);
            }


        // pair boundingboxes with tracked vehicles

            // for all the valid boxes, pair it with the most likely bounding box
            // we are assuming here that there is a high but not certain chance the valid boxes will be detected again
            foreach (Vehicle cVeh in valid_vehicles) {
                float closestDist = float.MaxValue;
                BoundingBox closestBox = null;

                foreach (BoundingBox cBox in unpairedBoxes) {
                    System.Numerics.Vector3 boxPos = new System.Numerics.Vector3(cBox.Centroid.X, cBox.Centroid.Y, cBox.Centroid.Z);
                    float distance = System.Numerics.Vector3.Distance(boxPos,cVeh.GetCachedPosition());

                    // figure out whether this box may be considered a detection of this vehicle
                    if (distance < closestDist && closeEnough(distance, cVeh,cBox) ) {
                        closestDist = distance;
                        closestBox = cBox;
                    }
                }

                if (closestBox != null){
                    // box has been paired with a tracked vehicle, so update its state
                    cVeh.UpdateState(closestBox, timeSecs, sensorCovarianceMatrix, sensorCMatrix, contToDelete);
                    unpairedBoxes.Remove(closestBox); // we have paired this veh, so don't try to pair it again
                }
            }




            // For all of the boxes that weren't paired to a valid vehicle, check if any of them are a detection of an invalid vehicle

            // add boxes that didn't pair to a vehicle
            foreach (BoundingBox cBox in unpairedBoxes) {
                System.Numerics.Vector3 boxPos = new System.Numerics.Vector3(cBox.Centroid.X, cBox.Centroid.Y, cBox.Centroid.Z);
                float closestDist = float.MaxValue;
                Vehicle closestVeh = null;

                foreach (Vehicle cVeh in invalid_vehicles) {
                    float distance = System.Numerics.Vector3.Distance(boxPos,cVeh.GetCachedPosition());

                    // figure out whether this box may be considered a detection of this vehicle
                    if (distance < closestDist && closeEnough(distance, cVeh,cBox) ) {
                        closestDist = distance;
                        closestVeh = cVeh;
                    }
                }

                if (closestVeh == null) {
                    // we could not find a Vehicle that fit this detection, so add a new vehicle for it
                    tracked_vehicles.Add(new Vehicle(cBox,GetNewVehicleId()));


                }else{
                    // box has been paired with a tracked vehicle, so update its state
                    closestVeh.UpdateState(cBox, timeSecs, sensorCovarianceMatrix, sensorCMatrix, contToDelete);
                    // unpairedBoxes.Remove(cBox); // we have paired this veh, so don't try to pair it again NOTE: this line is probably unneccesary 
                }
            }


            // clean vehicles
            if (contToDelete) {
                List<Vehicle> vehicles_to_delete = new List<Vehicle>();
                foreach (Vehicle cVeh in tracked_vehicles) {
                    if (cVeh.Clean(true)) {
                        vehicles_to_delete.Add(cVeh);
                    }
                }
                foreach (Vehicle cVeh in vehicles_to_delete) {
                    tracked_vehicles.Remove(cVeh);
                }
            }

            



        }

        // will return the next unique id to assign to a never-before-seen vehicle
        private UInt64 GetNewVehicleId() {
            if (vehicle_id_counter >= 9223372036854780000) {
                vehicle_id_counter = 1;
            }
            vehicle_id_counter++;

            return vehicle_id_counter;
        }

       


        // will publish BoundingBoxArray and MarkerArray messages
        public void publishBoundingBoxes()
        {

            // debug output
            #if DEBUG
                int i = 0;
                foreach (Vehicle cVeh in tracked_vehicles) {
                    var vehPos = cVeh.GetCachedPosition();
                    if (cVeh.isValid) Console.WriteLine($"Veh{i}, id: {cVeh.GetId()}, isValid: {cVeh.isValid}, pos: {vehPos.X}, {vehPos.Y}, {vehPos.Z} ");
                    i++;
                }
            #endif
            
            RosTime nowTime = clock.Now;

            List<Vehicle> valid_vehicles = new List<Vehicle>(); 

            foreach (Vehicle cVeh in tracked_vehicles) {
                if (cVeh.isValid) {
                    valid_vehicles.Add(cVeh);
                }
            }      
        
            MarkerArray markerArr = new MarkerArray();

            // update header
            tracked_object_array_msg.Header.Frame_id = expected_frame_id;
            tracked_object_array_msg.Header.Stamp.Sec = nowTime.sec;
            tracked_object_array_msg.Header.Stamp.Nanosec = nowTime.nanosec;


            var objects = new TrackedObject[valid_vehicles.Count];   
            var markers = new Marker[valid_vehicles.Count*2]; // twice as many as we have vehicles because we add labels
            tracked_object_array_msg.Objects = objects;
            markerArr.Markers = markers;



            int i = 0;
            foreach (Vehicle cVeh in valid_vehicles) {

                objects[i] = cVeh.getTrackedObjectMessage();
             
                
          
                // deal with markers
                
                markers[i] = new Marker();
                markers[i].Header.Stamp = tracked_object_array_msg.Header.Stamp;
                markers[i].Header.Frame_id = expected_frame_id;
                markers[i].Ns = "bbox";
                markers[i].Id = i;
                markers[i].Type = Marker.CUBE;
                markers[i].Action = Marker.ADD;

                markers[i].Pose.Position.X = objects[i].Kinematics.Centroid_position.X;
                markers[i].Pose.Position.Y = objects[i].Kinematics.Centroid_position.Y;
                markers[i].Pose.Position.Z = objects[i].Kinematics.Centroid_position.Z;


                // X and Y scale are swapped between these two message types (WARNING)
                markers[i].Scale.X = 5.0F; // boxes[i].Size.Y;
                markers[i].Scale.Y = 2.0F; // boxes[i].Size.X;
                markers[i].Scale.Z = 0.8F;  // boxes[i].Size.Z;

                // marker color
                markers[i].Color.R = 0.0F;
                markers[i].Color.G = 1.0F;
                markers[i].Color.B = 1.0F;
                markers[i].Color.A = 0.75F;

                markers[i].Lifetime.Sec = 0;
                markers[i].Lifetime.Nanosec = 500000000;



                // marker text label
                markers[i+valid_vehicles.Count] = new Marker();
                markers[i+valid_vehicles.Count].Header.Stamp = tracked_object_array_msg.Header.Stamp;
                markers[i+valid_vehicles.Count].Header.Frame_id = expected_frame_id;
                markers[i+valid_vehicles.Count].Ns = "bbox";
                markers[i+valid_vehicles.Count].Id = i+valid_vehicles.Count;
                markers[i+valid_vehicles.Count].Type = Marker.TEXT_VIEW_FACING;
                markers[i+valid_vehicles.Count].Action = Marker.ADD;

                markers[i+valid_vehicles.Count].Pose.Position.X = objects[i].Kinematics.Centroid_position.X;
                markers[i+valid_vehicles.Count].Pose.Position.Y = objects[i].Kinematics.Centroid_position.Y;
                markers[i+valid_vehicles.Count].Pose.Position.Z = objects[i].Kinematics.Centroid_position.Z;

                markers[i+valid_vehicles.Count].Text = cVeh.GetId().ToString();
                markers[i+valid_vehicles.Count].Lifetime.Sec = 0; //1.0
                markers[i+valid_vehicles.Count].Lifetime.Nanosec = 10000000;

                markers[i+valid_vehicles.Count].Scale.Z = 3;

                markers[i+valid_vehicles.Count].Color.R = 1.0F;
                markers[i+valid_vehicles.Count].Color.G = 0.0F;
                markers[i+valid_vehicles.Count].Color.B = 0.0F;
                markers[i+valid_vehicles.Count].Color.A = 1.0F;


                i++;
            }

            //tracked_object_array_msg.Boxes.Length = i;
            //Console.WriteLine($"Veh{i}, id: {cVeh.GetId()}, isValid: {cVeh.isValid}, pos: {vehPos.X}, {vehPos.Y}, {vehPos.Z} ");

            //Console.WriteLine($"Array size: {boxes.Length}");

        
            
            tracked_vehicle_boxes_pub.Publish(tracked_object_array_msg);
            tracked_vehicle_markers_pub.Publish(markerArr);

        }

        public void pubError(string error_reason, int error_severity, int error_lifetime) {
            // Console.WriteLine(error_reason);
            error_msg.Description = error_reason;
            error_msg.Severity = (sbyte) error_severity;
            error_msg.Origin = "vehicle_tracker_node";
            error_msg.Module = "planning";
            error_msg.Lifetime = (sbyte) error_lifetime;
            error_msg_pub.Publish(error_msg);
        }
    

    }

    
}
