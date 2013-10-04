using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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

namespace KinectTest3
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];
        public MainWindow()
        {
            InitializeComponent();
        }


        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
        }

        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor oldSensor = (KinectSensor)e.OldValue;
            StopKinect(oldSensor);

            KinectSensor newSensor = (KinectSensor)e.NewValue;
            newSensor.ColorStream.Enable();
            newSensor.DepthStream.Enable();
            newSensor.SkeletonStream.Enable();
            newSensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(_sensor_AllFramesReady);
            try
            {
                newSensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
        }

        void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            byte[] pixels;
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame == null)
                {
                    return;
                }

                pixels = GenerateColoredBytes(depthFrame);

                if (closing)
                {

                    return;
                }
                Skeleton first = GetFirstSkeleton(e);

                if(first == null)
                {
                    return;
                }
                pixels = GetCameraPoint(first, e, pixels, depthFrame);
                int stride = depthFrame.Width * 4;
                image1.Source = BitmapSource.Create(depthFrame.Width, depthFrame.Height,
                    96, 96, PixelFormats.Bgr32, null, pixels, stride);
            }

        }
        private Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null;
                }

                skeletonFrameData.CopySkeletonDataTo(allSkeletons);

                Skeleton first = (from s in allSkeletons
                                  where s.TrackingState == SkeletonTrackingState.Tracked
                                  select s).FirstOrDefault();
                return first;
            }
        }
        private byte[] GetCameraPoint(Skeleton first, AllFramesReadyEventArgs e, byte[] pixels, DepthImageFrame depthFrame)
        {
            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                if (depth == null || kinectSensorChooser1.Kinect == null)
                {
                    return pixels;
                }

                DepthImagePoint headDepthPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.Head].Position);
                DepthImagePoint leftDepthPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.HandLeft].Position);
                DepthImagePoint rightDepthPoint = depth.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position);

                paintPoint(headDepthPoint, pixels, depthFrame);
                paintPoint(leftDepthPoint, pixels, depthFrame);
                paintPoint(rightDepthPoint, pixels, depthFrame);

                ColorImagePoint headColorPoint = depth.MapToColorImagePoint(headDepthPoint.X, headDepthPoint.Y, ColorImageFormat.RgbResolution640x480Fps30);
                ColorImagePoint leftColorPoint = depth.MapToColorImagePoint(leftDepthPoint.X, leftDepthPoint.Y, ColorImageFormat.RgbResolution640x480Fps30);
                ColorImagePoint rightColorPoint = depth.MapToColorImagePoint(rightDepthPoint.X, rightDepthPoint.Y, ColorImageFormat.RgbResolution640x480Fps30);
                return pixels;
            }
        }
        private byte[] paintPoint(DepthImagePoint myPoint, byte[] pixels, DepthImageFrame depthFrame)
        {
            int x = myPoint.X;
            int y = myPoint.Y;
            int j = 0;
            int width = depthFrame.Width;
            int height = depthFrame.Height;

            int paintWidth = 20;
            int paintHeight = 20;

            while (j < paintWidth && (width * 4 * y) + ((x + j) * 4) < (width * height * 4))
            {
                int k = 0;
                while (k < paintHeight && (width * 4 * (y + k)) + ((x + j) * 4) < (width * height * 4))
                {
                    int pos = (width * 4 * (y + k)) + ((x + j) * 4);
                    pixels[pos] = 0; // the blue index
                    pixels[pos + 1] = 0; // the red index
                    pixels[pos + 2] = 0; // the green index
                    k++;
                }
                j++;
            }
            j = 0;
            while (j < paintWidth && (width * 4 * y) + ((x - j) * 4) > (width * height * 4))
            {
                int k = 0;
                while (k < paintHeight && (width * 4 * (y - k)) + ((x - j) * 4) < (width * height * 4))
                {
                    int pos = (width * 4 * (y - k)) + ((x - j) * 4);
                    pixels[pos] = 0; // the blue index
                    pixels[pos + 1] = 0; // the red index
                    pixels[pos + 2] = 0; // the green index
                    k++;
                }
                j++;
            }
            return pixels;
        }
        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);
            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];
            // *4 because each pixel has 4 data entries, blue green red and empty
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;
            //index positions for the depth data
            for (int depthIndex = 0, colorIndex = 0; depthIndex < rawDepthData.Length && colorIndex < pixels.Length; depthIndex++, colorIndex += 4)
            {
                // get player (needs skeleton tracking enabled)
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                //gets actual depth value
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                //these numbers represent mm away from the camera
                if (depth <= 900)
                {
                    pixels[colorIndex + BlueIndex] = 255;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;
                }
                else if (depth > 900 && depth < 2000)
                {
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 255;
                    pixels[colorIndex + RedIndex] = 0;
                }
                else if (depth >= 2000)
                {
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 255;
                }
            }
            return pixels;
        }
        void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                sensor.Stop();
                sensor.AudioSource.Stop();
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopKinect(kinectSensorChooser1.Kinect);
        }
    }
}
