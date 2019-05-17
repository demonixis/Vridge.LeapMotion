using Leap;
using System;
using System.Numerics;
using VRE.Vridge.API.Client.Messages.BasicTypes;
using VRE.Vridge.API.Client.Messages.v3.Controller;
using VRE.Vridge.API.Client.Remotes;

namespace LeapSharp
{
    //https://github.com/SDraw/driver_leap/blob/master/driver_leap/CLeapHandController.cpp
    class Program
    {
        private static VridgeRemote m_VridgeRemote;

        static void Main(string[] args)
        {
            m_VridgeRemote = new VridgeRemote("127.0.0.1", "Vridge.LeapMotion", Capabilities.Controllers | Capabilities.HeadTracking);

            Controller controller = new Controller();
            controller.Connect += OnServiceConnect;
            controller.Device += OnConnect;
            controller.FrameReady += OnFrame;

            // Keep this process running until Enter is pressed
            Console.WriteLine("Press Enter to quit...");
            Console.ReadLine();
            controller.Dispose();
        }

        public static void OnServiceConnect(object sender, ConnectionEventArgs args)
        {
            Console.WriteLine("Service Connected");
        }

        public static void OnConnect(object sender, DeviceEventArgs args)
        {
            Console.WriteLine("Connected");
        }

        public static void OnFrame(object sender, FrameEventArgs args)
        {
            if (m_VridgeRemote.Controller == null)
                return;

            var headPose = m_VridgeRemote?.Head.GetCurrentPose();
            if (headPose == null)
                return;

            var matrix = Matrix4x4.Transpose(new Matrix4x4(
                headPose[0], headPose[1], headPose[2], headPose[3],
                headPose[4], headPose[5], headPose[6], headPose[7],
                headPose[8], headPose[9], headPose[10], headPose[11],
                headPose[12], headPose[13], headPose[14], headPose[15]));

            var frame = args.frame;
            var position = matrix.Translation;

            foreach (var hand in frame.Hands)
            {
                var hPos = hand.PalmPosition;
                var hQuat = hand.Rotation;
                var left = !hand.IsLeft;

                var dir = hand.Direction;
                dir /= dir.Magnitude;

                var normal = hand.PalmNormal;
                normal /= normal.Magnitude;

                var cross = dir.Cross(normal);

                var mat = new Matrix4x4(normal.x, normal.z, normal.y, 0,
                    cross.x, cross.z, cross.y, 0,
                    dir.x, dir.z, dir.y, 0,
                    0, 0, 0, 1);

                Matrix4x4.Decompose(mat, out Vector3 scale, out Quaternion q, out Vector3 t);

                m_VridgeRemote.Controller.SetControllerState(
                     left ? 0 : 1,
                     HeadRelation.Unrelated,
                     left ? HandType.Left : HandType.Right,
                     new Quaternion(hQuat.x, hQuat.y, hQuat.z, hQuat.w),
                     position + new Vector3(hPos.x * -0.001f, -0.001f * hPos.z, -0.001f * hPos.y - 0.15f),
                     0,
                     0,
                     0,
                     false,
                     false,
                     false,
                     false,
                     false,
                     false);
            }
        }
    }
}
