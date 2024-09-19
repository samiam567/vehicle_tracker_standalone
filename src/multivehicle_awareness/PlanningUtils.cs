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
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace planning.multivehicle_awareness
{
    /**
      * @summary this struct provides some common constants for use in the multivehicle awareness package
      **/
    public struct PlanningUtils
    {

        
        public const float SLIPSTREAM_CAR_FOLLOW_DIST = 15; // the distance we will follow a car when we are attempting to gain an overtake opportunity. This should be larger than car_follow_distance in speed_reference_generator
        public const float LEASURE_CAR_FOLLOW_DIST = SLIPSTREAM_CAR_FOLLOW_DIST + 20; // the distance we will follow a car normally

        public const float OVERTAKE_TRIGGER_DIST = LEASURE_CAR_FOLLOW_DIST + 5; // The distance below which we will attempt to trigger overtake
        

        
        public const float CAR_WIDTH = 2; // the width of the car in meters
        public const float CAR_LENGTH = 4.8768F; // length of the car in meters
        
        public const float TRACK_WIDTH = 10.0F; // the width of the track in meters // NOTE this will be deprecated with /planning/feasible_offset_range

        public const float SAFETY_DISTANCE = CAR_WIDTH/2 + 1; // the distance we will leave between our car and another car at all times
    


        // declare states (NOTE that these are not an enum so we can transmit over an int8 message)
        public const sbyte STATE__CATCHING_UP = 0; // ACTION__MAINTAIN_REGULAR_PATH, we maintain optimal race line
        public const sbyte STATE__DEFENDING = 1; // ACTION__DEFEND - we will attempt to keep the car behind us from overtaking us
        public const sbyte STATE__ATTACKING = 2; // ACTION__HOLD_SLIPSTREAM until we are ready to overtake - draft behind the car infront of us

        public const sbyte STATE__PREPARED_TO_OVERTAKE = 3; // DEPRECATED wait to send ACTION__ATTEMPT_OVERTAKE
        public const sbyte STATE__OVERTAKING = 4; // execute overtake actions, ACTION__OVERTAKE
        public const sbyte MAX_STATE = STATE__OVERTAKING;

        // declare action modes (NOTE that these are not an enum so we can transmit over an int8 message)
        public const sbyte ACTION__SUBMIT = 0;                // we will maintain position and slow down
        public const sbyte ACTION__MAINTAIN_REGULAR_PATH = 1; // we will follow the unaltered path from path_server
        public const sbyte ACTION__MAINTAIN_CENTER_PATH = 2;  // we will attempt to stay in the middle of the track boundries
        public const sbyte ACTION__ATTEMPT_OVERTAKE = 3;      // we are in place to overtake, and attempting to do so
        public const sbyte ACTION__OVERTAKE = 4;              // we are actively overtaking the car infront of us
        public const sbyte ACTION__HOLD_SLIPSTREAM = 5;       // follow closely behind the car infront of us before overtaking same as attempt_overtake but we use the slipstream
        public const sbyte ACTION__DEFEND = 6;                // mirror the car behind us's movements to keep them from overtaking
        public const sbyte ACTION__COMPLETE_OVERTAKE = 7;     // return back to our regular race line



        public static bool _isDefaultAction(int action) {
            // shorthand method to add condition to go back to default action states
            return action == ACTION__DEFEND                || 
                    action == ACTION__MAINTAIN_CENTER_PATH ||
                    action == ACTION__MAINTAIN_REGULAR_PATH;
        }
    }
}