using System;
using ROS2;


using MathNet.Numerics;

namespace Examples
{
  /// <summary> A simple listener class to illustrate Ros2cs in action </summary>
  public class Velocity_listener {

    INode node;

    public Velocity_listener(INode node) {
      this.node = node;


      ISubscription<std_msgs.msg.Float32> velocity_sub = node.CreateSubscription<std_msgs.msg.Float32>("/localization/velocity", Velocity_listener.recieveVelocity);
      // ISubscription<blackandgold_msgs.msg.CarProximityReport> carProxSub = node.CreateSubscription<blackandgold_msgs.msg.CarProximityReport>("/perception/car_proximity_report",Velocity_listener.recieveCarProximityReport);


      // example of using a nuget library (MathNet.Numerics)

      Console.WriteLine("solving linear equation:");

      // Solve a random linear equation system with 500 unknowns
      var m = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.Random(500, 500);
      var v = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Random(500);
      var y = m.Solve(v);
      Console.WriteLine("Solution:: " + y);
      
      /////

    }
    public static void recieveVelocity(std_msgs.msg.Float32 msg) {
        Console.WriteLine("I heard: " + msg.Data);
    }

    // public static void recieveCarProximityReport(blackandgold_msgs.msg.CarProximityReport msg) {
    //   Console.WriteLine("FrontDist: " + msg.Front_dist);
    // }

    

    public static void Main(string[] args)
    {

      Ros2cs.Init();
      INode node = Ros2cs.CreateNode("velocity_listener");
      Velocity_listener Node = new Velocity_listener(node);
    
      Ros2cs.Spin(node);
      Ros2cs.Shutdown();
      
    }
  }
}