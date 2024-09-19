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

    public class VehicleTrackerNode {
        

        const string expected_frame_id = "center_of_gravity";


        VehicleTracker vehicle_tracker;
        INode node;


        //publishers
        Publisher<TrackedObjects> tracked_vehicle_boxes_pub;
        Publisher<visualization_msgs.msg.MarkerArray> tracked_vehicle_markers_pub;
        Publisher<blackandgold_msgs.msg.ErrorReport> error_msg_pub;


        // error report message
        blackandgold_msgs.msg.ErrorReport error_msg = new blackandgold_msgs.msg.ErrorReport();


        // Covariance for our sensor readings
        readonly MathNet.Numerics.LinearAlgebra.Matrix<float> R_matrix_radar = 0.3F *
            MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.DenseOfArray(new float[,] {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1},
            }
        );
        
        readonly MathNet.Numerics.LinearAlgebra.Matrix<float> R_matrix_lidar = 0.3F *
            MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.DenseOfArray(new float[,] {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1},
            }
        );

        readonly MathNet.Numerics.LinearAlgebra.Matrix<float> R_matrix_ghost_veh = 0.1F *
            MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.DenseOfArray(new float[,] {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1},
            }
        );

        // The components of the state we are directly measuring
        readonly MathNet.Numerics.LinearAlgebra.Matrix<float> C_matrix = 
            MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.DenseOfArray(new float[,] {
                {1,0,0,0,0,0,0,0,0},
                {0,1,0,0,0,0,0,0,0},
                {0,0,1,0,0,0,0,0,0},
            }
        );


        
        
        public VehicleTrackerNode(INode node) {
            this.node = node;

            

            // Create publishers
            tracked_vehicle_boxes_pub = node.CreatePublisher<TrackedObjects>("/planning/tracked_vehicles");
            tracked_vehicle_markers_pub = node.CreatePublisher<visualization_msgs.msg.MarkerArray>("/planning/tracked_vehicles_markers");
            error_msg_pub = node.CreatePublisher<blackandgold_msgs.msg.ErrorReport>("/planning/errors");


            vehicle_tracker = new VehicleTracker(node,tracked_vehicle_boxes_pub,tracked_vehicle_markers_pub);

            // wait
            System.Threading.Thread.Sleep(5000);

            // create subscribers
            //TODO: Subscribe to on track bboxes instead of bboxes. (Needs the off map filter implementation)

            ISubscription<BoundingBoxArray> lidar_bounding_box_sub = node.CreateSubscription<BoundingBoxArray>("/perception/lidar/vehicle/bboxes",this.receiveLidarBoxes);
            ISubscription<BoundingBoxArray> radar_bounding_box_sub = node.CreateSubscription<BoundingBoxArray>("/radar_processing/vehicles_boxes", this.receiveRadarBoxes);
            //ISubscription<BoundingBoxArray> camera1_bbox = node.CreateSubscription<BoundingBoxArray>("/yolov5/f_camera1/bounding_boxes",this.receiveBoxes);
            //ISubscription<BoundingBoxArray> camera2_bbox = node.CreateSubscription<BoundingBoxArray>("/yolov5/f_camera2/bounding_boxes",this.receiveBoxes);
            ISubscription<BoundingBoxArray> ghost_veh_position_sub = node.CreateSubscription<BoundingBoxArray>("/planning/ghost_veh_position", this.receiveGhostVehiclePos);   

            // ISubscription<std_msgs.msg.Empty> clock100hz_sub = node.CreateSubscription<std_msgs.msg.Empty>("/vehicle/hz100",vehicle_tracker.publishBoundingBoxes, new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA));
            var timer_100hz = node.CreateTimer(1.0F/100, vehicle_tracker.publishBoundingBoxes);

            error_msg.Origin = "Vehicle_tracker_node";
            error_msg.Module = "Planning";
            error_msg.Lifetime = 1;
            



        }



        private void receiveRadarBoxes(BoundingBoxArray msg) {
            vehicle_tracker.receiveBoxes(msg, R_matrix_radar,C_matrix, true);
        }
        private void receiveLidarBoxes(BoundingBoxArray msg) {
            vehicle_tracker.receiveBoxes(msg, R_matrix_lidar,C_matrix, true);
        }
        private void receiveGhostVehiclePos(BoundingBoxArray msg) {
            vehicle_tracker.receiveBoxes(msg, R_matrix_ghost_veh, C_matrix, false);
        }



        public static void Main(string[] args)
        {

            Ros2cs.Init();
            INode node = Ros2cs.CreateNode("vehicle_tracker_node");
            VehicleTrackerNode Node = new VehicleTrackerNode(node);
            
        
            do {
                try {
                Ros2cs.Spin(node);
                }catch(Exception e) {
                // if it's not a publisher context exception, console out the error
                if (! e.ToString().Contains("publisher's context is invalid")){
                    Console.WriteLine("ERROR: " + e.ToString());
                }

                // publish our exception to the safety framework
                try{
                    Node.vehicle_tracker.pubError(e.ToString().Replace("\n","\\n"), 1, 1);
                    continue;
                }catch(Exception){continue;} // if the pub fails for any reason, just continue anyway (this should rarely happen)
                }
            }while(Ros2cs.Ok());

            Ros2cs.Shutdown();

        }
    }
}
