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


using System.Numerics;

using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;

using MathNet;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace planning.multivehicle_awareness {



    /**
    <summary>A state to be used for a object state machine. </summary>
    **/
    public interface VehicleState {
        

        Vector3 GetPosition();     // Returns the position from the last recorded detection (last time update was called)
        Vector3 GetVelocity();     // Returns the velocity from the last recorded detection (last time update was called)
        Vector3 GetAcceleration(); // Returns the acceleration from the last recorded detection (last time update was called)

        Vector3 GetNowPosition();     // Returns the position from the last time we called Predict
        Vector3 GetNowVelocity();     // Returns the velocity from the last time we called Predict
        Vector3 GetNowAcceleration(); // Returns the acceleration from the last time we called Predict

        //<summary>Predict will cause getNow<metric> methods to return the state timeDiff seconds since the state was last updated</summary>
        void Predict(int stepIterations,float timeStep,float totalTimeDiff);
        void Update(Vector3 pos, float timeDiff, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCovarianceMatrix, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCMatrix); // updates the state based with a detection at pos, timeDiff seconds since the last detection
    }

    public class ComponentKalmanLinearState : VehicleState {
        
        

   
        MathNet.Numerics.LinearAlgebra.Vector<float> state;

        

        
        MathNet.Numerics.LinearAlgebra.Matrix<float> aprioricovarianceMatrix;
        MathNet.Numerics.LinearAlgebra.Matrix<float> covarianceMatrix;
    
        MathNet.Numerics.LinearAlgebra.Vector<float> CachedNowState;


        // Q - how much do we trust the model vs trust the sensor
        readonly MathNet.Numerics.LinearAlgebra.Matrix<float> processNoise = 0.1F * Matrix<float>.Build.DenseIdentity(9);
        
        
        
        
        

        public ComponentKalmanLinearState(Vector3 pose) {

            state = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(new float[] {pose.X, pose.Y, pose.Z, 0,0,0, 0,0,0} );

            CachedNowState = state; // temporarially set this to state before getNowPos is called to avoid crashes incase we try to retreive cachedstate before calling getNowPos
            
            covarianceMatrix = 100.0F * Matrix<float>.Build.DenseIdentity(9);
        }

        public void Update(Vector3 pose, float timeDiff, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCovarianceMatrix, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCMatrix) {
            
            MathNet.Numerics.LinearAlgebra.Vector<float> yK = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(
                new float[] {pose.X, pose.Y, pose.Z}
            );
            
            //Console.WriteLine($"Sensor: {sensor}");
            // predict what our state is now
            Predict(0,0,timeDiff);


            // update our current state and covariance with new sensor reading
            
            MathNet.Numerics.LinearAlgebra.Matrix<float> R_matrix = sensorCovarianceMatrix;
            MathNet.Numerics.LinearAlgebra.Matrix<float> C_matrix = sensorCMatrix;
            
            // calculate K_matrix
            MathNet.Numerics.LinearAlgebra.Matrix<float> K_matrix = 
                (aprioricovarianceMatrix * C_matrix.Transpose()) * (C_matrix * aprioricovarianceMatrix * C_matrix.Transpose() + R_matrix).Inverse();
            
            // update our state
            state = CachedNowState + K_matrix * (yK - C_matrix * CachedNowState);

            // update our covariance
            covarianceMatrix = (Matrix<float>.Build.DenseIdentity(9) - K_matrix*C_matrix )*aprioricovarianceMatrix;
            
        }

      

        public void Predict(int stepIterations,float timeStep, float totalTimeDiff) {
            // NOTE that we do not use stepIterations and timestep, and thus we only use one calculation step for the integral.
            // (The integral is calculated exactly rather than approximated)


            CachedNowState = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.DenseOfVector(state);

            // update CachedNowState with the kalman prediction

        
            float[,] A_arr = new float[,] {
                {1,0,0,totalTimeDiff,0,0,(float) (0.5*Math.Pow(totalTimeDiff,2)),0,0},
                {0,1,0,0,totalTimeDiff,0,0,(float) (0.5*Math.Pow(totalTimeDiff,2)),0},
                {0,0,1,0,0,totalTimeDiff,0,0,(float) (0.5*Math.Pow(totalTimeDiff,2))},
                {0,0,0,1,0,0,totalTimeDiff,0,0},
                {0,0,0,0,1,0,0,totalTimeDiff,0},
                {0,0,0,0,0,1,0,0,totalTimeDiff},
                {0,0,0,0,0,0,1,0,0},
                {0,0,0,0,0,0,0,1,0},
                {0,0,0,0,0,0,0,0,1}};
            
            MathNet.Numerics.LinearAlgebra.Matrix<float> A_mat = MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.DenseOfArray(A_arr);

            // We are not using B_arr because we do not know the inputs to the other cars
            // float[,] B_arr = new float[,] {

            // }

            CachedNowState = A_mat * state;

            aprioricovarianceMatrix = A_mat * covarianceMatrix * A_mat.Transpose() + processNoise;


            
        }
        
        public Vector3 GetPosition() {
            return new System.Numerics.Vector3(state.At(0), state.At(1),state.At(2));
        }

        public Vector3 GetVelocity() {
            return new System.Numerics.Vector3(state.At(0+3), state.At(1+3),state.At(2+3));
        }

        public Vector3 GetAcceleration() {
            return new System.Numerics.Vector3(state.At(0+6), state.At(1+6),state.At(2+6));
        }

        public Vector3 GetNowPosition() {
            return new System.Numerics.Vector3(CachedNowState.At(0), CachedNowState.At(1),CachedNowState.At(2));
        }

        public Vector3 GetNowVelocity() {
            return new System.Numerics.Vector3(CachedNowState.At(0+3), CachedNowState.At(1+3),CachedNowState.At(2+3));
        }

        public Vector3 GetNowAcceleration() {
            return new System.Numerics.Vector3(CachedNowState.At(0+6), CachedNowState.At(1+6),CachedNowState.At(2+6));
        }


    }

    public class ComponentLinearState : VehicleState {
        protected float[,] state = new float[,]{{0,0,0},{0,0,0},{0,0,0},{0,0,0}}; // [[xpos,xvel,xaccel,],[ypos,yvel,yaccel,],[zpos,zvel,zaccel,],[(whether each metric has enough datapoints to be significant 0=false, 1=true)]]
        public float[,] CachedNowState; // will change to Cached nowPos from the last GetNowPos call

        public ComponentLinearState(Vector3 pose) {
            state[0,0] = pose.X;
            state[1,0] = pose.Y;
            state[2,0] = pose.Z;

            CachedNowState = state; // temporarially set this to state before getNowPos is called to avoid crashes incase we try to retreive cachedstate before calling getNowPos
           
        }

        public void Update(Vector3 pose, float timeDiff, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCovarianceMatrix, MathNet.Numerics.LinearAlgebra.Matrix<float> sensorCMatrix) {
             // Update the state matrix using the new pos
            float prevVal = 0;
            for (int dim = 0; dim < state.GetLength(0)-1; dim++) { // dim=dimension
                float nextDerivative = getDim(pose, dim); // gets the <dim> position
                for (int dir = 0; dir < state.GetLength(1); dir++ ) { // iterate through derivatives of position dir=derivative
                    prevVal = state[dim,dir];
                    state[dim,dir] = nextDerivative;
                    if (state[state.GetLength(0)-1,dir] == 1 && timeDiff != 0 && dir < state.GetLength(1)-1) { // if we already had a value for this derivative we can calculate the derivative of this derivative
                        nextDerivative = (state[dim,dir] - prevVal)/timeDiff; // calculate next derivative
                        nextDerivative = ( nextDerivative + state[dim,dir+1] ) / 2; // help protect against super sudden changes
                    
                    }else{
                        // this is the first time we got to this dir
                        state[state.GetLength(0)-1,dir] = 1; // we just propogated this dir so we are good for the next pass
                        break;
                    }
                }
               
            } 
        }

        private float getDim(Vector3 pos, int dim ) {
            
            switch(dim) {
                case 0:
                    return pos.X;
                case 1:
                    return pos.Y;
                case 2:
                    return pos.Z;
                default:
                    Console.WriteLine("ERROR: Invalid dim in Vehicle.getDim!");
                    return 0;
            }
            
        }

        public void Predict(int stepIterations,float timeStep,float totalTimeDiff) {
            // copy this.state to predState
            float[,] predState = new float[,] {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
            for (int dim = 0; dim < predState.GetLength(0); dim++) {
                for (int dir = 0; dir < predState.GetLength(1); dir++) {
                    predState[dim,dir] = state[dim,dir];
                }
            }

            // advance state by the timestep each iteration
            for (int i = 0; i < stepIterations; i++) {

                for (int dim = 0; dim < predState.GetLength(0); dim++) {
                    for (int dir = predState.GetLength(1)-1; dir > 0; dir--) {
                        state[dim,dir-1] += state[dim,dir] * timeStep;
                    }
                }
            }

            CachedNowState = predState;
            
        }
        
        public Vector3 GetPosition() {
            return new System.Numerics.Vector3(state[0,0], state[1,0],state[2,0]);
        }

        public Vector3 GetVelocity() {
            return new System.Numerics.Vector3(state[0,1], state[1,1],state[2,1]);
        }

        public Vector3 GetAcceleration() {
            return new System.Numerics.Vector3(state[0,2], state[1,2],state[2,2]);
        }

        public Vector3 GetNowPosition() {
            return new System.Numerics.Vector3(CachedNowState[0,0], CachedNowState[1,0],CachedNowState[2,0]);
        }

        public Vector3 GetNowVelocity() {
            return new System.Numerics.Vector3(CachedNowState[0,1], CachedNowState[1,1],CachedNowState[2,1]);
        }

        public Vector3 GetNowAcceleration() {
            return new System.Numerics.Vector3(CachedNowState[0,2], CachedNowState[1,2],CachedNowState[2,2]);
        }


    }
}