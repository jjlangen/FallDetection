using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using Microsoft.Kinect;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.IO;
using System.Threading;

namespace FallDetection
{
    public partial class Form1 : Form
    {
        #region Variables
        private KinectSensor sensor;
        private DepthImagePixel[] depthPixels;
        private byte[] colorDepthPixels;
        private WriteableBitmap bitmap;
        private Skeleton[] skeletonData;
        #endregion

        public Form1()
        {
            InitializeComponent();            
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            if (KinectSensor.KinectSensors.Count == 0)
            {
                MessageBox.Show("No Kinect device detected", "Fall Detection");
                Application.Exit();
                //return;
            }
            // We will use the first connected kinect sensor
            foreach (var firstSensor in KinectSensor.KinectSensors)
            {
                if (firstSensor.Status == KinectStatus.Connected)
                {
                    sensor = firstSensor;
                    break;
                }
            }

            if (sensor != null)
            {
                // Start the DepthStream to get depthframes and allocate space to store the data
                sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                depthPixels = new DepthImagePixel[sensor.DepthStream.FramePixelDataLength];
                colorDepthPixels = new byte[sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                try { sensor.Start(); }
                catch (IOException) { sensor = null; }

                // Event handler for new depth data
                sensor.DepthFrameReady += sensor_DepthFrameReady;

                // The bitmap which is going to be displayed
                bitmap = new WriteableBitmap(sensor.DepthStream.FrameWidth, sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                sensor.SkeletonStream.Enable(); // Enable skeletal tracking

                skeletonData = new Skeleton[sensor.SkeletonStream.FrameSkeletonArrayLength]; // Allocate ST data

                sensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(sensor_SkeletonFrameReady); // Get Ready for Skeleton Ready Events

                loadStartingConfig();
            }
            else Application.Exit();
        }

        private void DrawSkeletons()
        {
            foreach (Skeleton skeleton in this.skeletonData)
            {
                if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                {
                    DrawTrackedSkeletonJoints(skeleton.Joints);
                }
                else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                {
                   // DrawSkeletonPosition(skeleton.Position);
                }
            }
        }

        private void DrawTrackedSkeletonJoints(JointCollection jointCollection)
        {
            // Render Head and Shoulders
            DrawBone(jointCollection[JointType.Head], jointCollection[JointType.ShoulderCenter]);
            DrawBone(jointCollection[JointType.ShoulderCenter], jointCollection[JointType.ShoulderLeft]);
            DrawBone(jointCollection[JointType.ShoulderCenter], jointCollection[JointType.ShoulderRight]);

            // Render Left Arm
            DrawBone(jointCollection[JointType.ShoulderLeft], jointCollection[JointType.ElbowLeft]);
            DrawBone(jointCollection[JointType.ElbowLeft], jointCollection[JointType.WristLeft]);
            DrawBone(jointCollection[JointType.WristLeft], jointCollection[JointType.HandLeft]);

            // Render Right Arm
            DrawBone(jointCollection[JointType.ShoulderRight], jointCollection[JointType.ElbowRight]);
            DrawBone(jointCollection[JointType.ElbowRight], jointCollection[JointType.WristRight]);
            DrawBone(jointCollection[JointType.WristRight], jointCollection[JointType.HandRight]);

            // Render other bones...
        }

        private void DrawBone(Joint jointFrom, Joint jointTo)
        {
            if (jointFrom.TrackingState == JointTrackingState.NotTracked ||
            jointTo.TrackingState == JointTrackingState.NotTracked)
            {
                return; // nothing to draw, one of the joints is not tracked
            }

            if (jointFrom.TrackingState == JointTrackingState.Inferred ||
            jointTo.TrackingState == JointTrackingState.Inferred)
            {
                //DrawNonTrackedBoneLine(jointFrom.Position, jointTo.Position);  // Draw thin lines if either one of the joints is inferred
            }

            if (jointFrom.TrackingState == JointTrackingState.Tracked &&
            jointTo.TrackingState == JointTrackingState.Tracked)
            {
                //DrawTrackedBoneLine(jointFrom.Position, jointTo.Position);  // Draw bold lines if the joints are both tracked
            }
        }

        private void sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame()) // Open the Skeleton frame
            {
                if (skeletonFrame != null && this.skeletonData != null) // check that a frame is available
                {
                    skeletonFrame.CopySkeletonDataTo(this.skeletonData); // get the skeletal information in this frame
                }
            }
        }

        private void sensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy pixel data from the image to an array
                    depthFrame.CopyDepthImagePixelDataTo(depthPixels);

                    int minDepth = 200;
                    int maxDepth = 1000;
                    int pixIndex = 0;
                    for (int i = 0; i < depthPixels.Length; ++i)
                    {
                        short depth = depthPixels[i].Depth;
                        // Make pixels outside of the specified depthrange black
                        byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                        // Write the B G and R bytes to our output array and skip the Alpha channel
                        colorDepthPixels[pixIndex++] = intensity;
                        colorDepthPixels[pixIndex++] = intensity;
                        colorDepthPixels[pixIndex++] = intensity;
                        ++pixIndex;
                    }
                    // Write pixel data to a WriteableBitmap
                    bitmap.WritePixels(
                        new System.Windows.Int32Rect(0, 0, bitmap.PixelWidth, bitmap.PixelHeight),
                        colorDepthPixels, bitmap.PixelWidth * sizeof(int), 0);

                    pictureBox.Image = convertToBitmap(bitmap);
                }
            }
        }

        private Bitmap convertToBitmap(WriteableBitmap wbmp)
        {
            Bitmap bmp;
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder encoder = new BmpBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create((BitmapSource)wbmp));
                encoder.Save(outStream);
                bmp = new Bitmap(outStream);
            }
            return bmp;
        }

        private void loadStartingConfig()
        {
            trackBar1.Value = sensor.ElevationAngle;
            startButton.Text = "Stop";
        }

        private void trackBar1_MouseUp(object sender, MouseEventArgs e)
        {
            try { sensor.ElevationAngle = trackBar1.Value; }
            catch (InvalidOperationException ioe) { MessageBox.Show(ioe.Message); };
        }

        private void startButton_Click(object sender, EventArgs e)
        {
            if (pictureBox.Image == null)
            {
                // Start it up
                startButton.Text = "Stop";
                sensor.DepthFrameReady += sensor_DepthFrameReady;
            }
            else
            {
                // Shut it down
                startButton.Text = "Start";
                sensor.DepthFrameReady -= sensor_DepthFrameReady;
                pictureBox.Image = null;
            }
        }
    }
}
