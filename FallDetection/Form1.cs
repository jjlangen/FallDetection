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

                loadStartingConfig();
            }
            else Application.Exit();
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
