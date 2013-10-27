using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using Microsoft.Kinect;
using System.IO;

namespace WpfApplication1
{
    public partial class MainWindow : Window
    {
        #region Variables
        private const float RenderWidth = 640.0f;
        private const float RenderHeight = 480.0f;
        private const double JointThickness = 3;
        private const double BodyCenterThickness = 10;
        private const double ClipBoundsThickness = 10;
        private readonly Brush centerPointBrush = Brushes.Blue;
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));     
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private KinectSensor sensor;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;

        private WriteableBitmap colorBitmap, depthBitmap;
        private DepthImagePixel[] depthPixels;
        private byte[] colorPixels, colorDepthPixels;

        private float lastPosY = 0.0f;
        private Queue<float>[] dPosY = new Queue<float>[8];
        #endregion

        public MainWindow()
        {
            InitializeComponent();
        }
        
        // Draws indicators to show which edges are clipping skeleton data
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        // Execute startup tasks
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            if (KinectSensor.KinectSensors.Count == 0)
            {
                MessageBox.Show("No Kinects device detected", "Fall Detection");
                Application.Current.Shutdown();
                //return;
            }

            // Initialise array of queues
            for (int i = 0; i < dPosY.Length; i++)
                dPosY[i] = new Queue<float>();

            // Create the drawing group we'll use for drawing
            drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            SkeletalImage.Source = imageSource;

            // Look through all sensors and start the first connected one.
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the color stream to receive colored frames
                sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                sensor.ColorFrameReady += SensorColorFrameReady;

                // Turn on the skeleton stream to receive skeleton frames
                sensor.SkeletonStream.Enable();
                sensor.SkeletonFrameReady += SensorSkeletonFrameReady;

                // Configure the depth stream to receive depth frames
                configureDepthStream();
                sensor.DepthFrameReady += SensorDepthFrameReady;

                // Start the sensor
                try
                { sensor.Start(); sensor.ElevationAngle = 8; }
                catch (IOException)
                { sensor = null; }
            }
        }

        private void configureDepthStream()
        {
            int frameLength = sensor.DepthStream.FramePixelDataLength;
            // Turn on the depth stream to receive depth frames
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            // Allocate space to put the depth pixels we'll receive
            depthPixels = new DepthImagePixel[frameLength];
            // Allocate space to put the color pixels we'll create
            colorDepthPixels = new byte[frameLength * sizeof(int)];
            // Bitmap that will be displayed
            depthBitmap = new WriteableBitmap(sensor.DepthStream.FrameWidth, sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            DepthImage.Source = depthBitmap;
        }

        // Event handler for switching between skeletonframe and colorframe
        void checkBoxSkelOnlyChanged(object sender, RoutedEventArgs e)
        {
            if (checkBoxSkelOnly.IsChecked.GetValueOrDefault())
            { ColorImage.Visibility = Visibility.Hidden; }
            else { ColorImage.Visibility = Visibility.Visible; }
        }

        // Execute shutdown tasks
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            { sensor.Stop(); }
        }

        // Event handler for sensor's DepthFrameReady event
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(depthPixels);

                    // Get the min and max depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;

                    // Convert the depth to RGB
                    int depthPixelIndex = 0;
                    for (int i = 0; i < depthPixels.Length; ++i)
                    {
                        // Get the depth for this pixel
                        short depth = depthPixels[i].Depth;
                      
                        byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                        // Write out blue byte
                        colorDepthPixels[depthPixelIndex++] = intensity;

                        // Write out green byte
                        colorDepthPixels[depthPixelIndex++] = intensity;

                        // Write out red byte                        
                        colorDepthPixels[depthPixelIndex++] = intensity;

                        depthPixelIndex++;
                    }

                    // Write the pixel data into our bitmap
                    depthBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                        colorDepthPixels,
                        depthBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }
        }

        // Event handler for sensor's ColorFrameReady event
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    colorPixels = new byte[colorFrame.PixelDataLength];

                    colorFrame.CopyPixelDataTo(colorPixels);

                    colorBitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96, 96, PixelFormats.Bgr32, null);
                    colorBitmap.WritePixels(new Int32Rect(0, 0, colorFrame.Width, colorFrame.Height), colorPixels, colorFrame.Width * 4, 0);
                    ColorImage.Source = colorBitmap;
                }
            }
        }

        // Event handler for sensor's SkeletonFrameReady event
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                    foreach (Skeleton skeleton in skeletons)
                    {

                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            detectFall(skeletonFrame, skeleton);
                    }
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));
                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            centerPointBrush,
                            null,
                            SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // Prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        // Draws a skeleton's bones and joints
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                { drawBrush = trackedJointBrush; }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                { drawBrush = inferredJointBrush; }

                if (drawBrush != null)
                { drawingContext.DrawEllipse(drawBrush, null, SkeletonPointToScreen(joint.Position), JointThickness, JointThickness); }
            }
        }

        // Maps a SkeletonPoint to lie within our render space and converts to Point
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        // Draws a bone line between two joints
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            { return; }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            { return; }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            { drawPen = this.trackedBonePen; }

            drawingContext.DrawLine(drawPen, SkeletonPointToScreen(joint0.Position), SkeletonPointToScreen(joint1.Position)); 
        }

        // Makes a floor plane and uses the position of the head to calculate the length of a vector normal to the plane
        private void detectFall(SkeletonFrame sframe, Skeleton skeleton)
        {
            int fallCounter = 0;
            int j = 0;

            Joint[] joints = new Joint[] {
                skeleton.Joints[JointType.Head],
                skeleton.Joints[JointType.ShoulderCenter],
                skeleton.Joints[JointType.ShoulderLeft],
                skeleton.Joints[JointType.ShoulderRight],
                skeleton.Joints[JointType.Spine],
                skeleton.Joints[JointType.HipCenter],
                skeleton.Joints[JointType.HipLeft],
                skeleton.Joints[JointType.HipRight]
            };

            float A = sframe.FloorClipPlane.Item1;
            float B = sframe.FloorClipPlane.Item2;
            float C = sframe.FloorClipPlane.Item3;
            float D = sframe.FloorClipPlane.Item4;

            for (int i = 0; i < joints.Length; i++)
            {
                if (joints[i].TrackingState == JointTrackingState.Tracked)
                {
                    fallCounter += checkJoint(i, A, B, C, D, joints[i]);
                    j++;
                }
            }

            if (fallCounter >= (j*100/75))
            {
                label.Content = "Fall Detected";
                label.Foreground = Brushes.Red;
            }
        }

        private int checkJoint(int n, float A, float B, float C, float D, Joint joint)
        {
            float x = joint.Position.X;
            float y = joint.Position.Y;
            float z = joint.Position.Z;

            if (dPosY[n].Count == 10)
                dPosY[n].Dequeue();

            dPosY[n].Enqueue(lastPosY - y);
            lastPosY = y;

            float dPosYTotal = 0.0f;
            foreach(float dPosYStep in dPosY[n])
                dPosYTotal += dPosYStep;

            float dPosYAvg = dPosYTotal / dPosY[n].Count;            

            float num = A * x + B * y + C * z + D;
            float denum = A * A + B * B + C * C;

            float distance = num / (float)Math.Sqrt(denum);
            if (distance <= 0.50 && dPosYAvg > 0.05)
            {
                return 1;
            }

            return 0;
        }
    }
}
