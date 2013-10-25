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
        int headtilt = 0;
        int counter = 0;
        int threshold = 5;
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
                //DEBUG
                if (counter < threshold)
                {
                    headtilt = headtilt + -kinectSensorChooser1.Kinect.ElevationAngle;
                }
                else
                {
                    if (counter == threshold)
                    {
                        headtilt = headtilt / threshold;
                    }
                    else
                    {
                        int hx, hy, hdepth, hrelX, hrelY;
                        int rx, ry, rdepth, rrelX, rrelY;
                        float thetaX, thetaY;
                        thetaX = 28.5f;
                        thetaY = 21.5f;
                        hx = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.Head].Position).X;
                        hy = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.Head].Position).Y;
                        hdepth = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.Head].Position).Depth;
                        rx = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position).X;
                        ry = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position).Y;
                        rdepth = depthFrame.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position).Depth;
                        hrelX = (int)(((hx - 320.0f) / 320.0f) * hdepth * System.Math.Tan(thetaX / 180.0f * System.Math.PI));
                        hrelY = -(int)(((hy - 240.0f) / 240.0f) * hdepth * System.Math.Tan(thetaY / 180.0f * System.Math.PI));
                        rrelX = (int)(((rx - 320.0f) / 320.0f) * rdepth * System.Math.Tan(thetaX / 180.0f * System.Math.PI));
                        rrelY = -(int)(((ry - 240.0f) / 240.0f) * rdepth * System.Math.Tan(thetaY / 180.0f * System.Math.PI));
                        int hrelXP, hrelYP, hrelZP;
                        int rrelXP, rrelYP, rrelZP;
                        hrelXP = hrelX;
                        rrelXP = rrelX;
                        hrelYP = (int)(hrelY * System.Math.Cos(headtilt / 180.0f * System.Math.PI) - hdepth * System.Math.Sin(headtilt / 180.0f * System.Math.PI));
                        hrelZP = (int)(hrelY * System.Math.Sin(headtilt / 180.0f * System.Math.PI) + hdepth * System.Math.Cos(headtilt / 180.0f * System.Math.PI));
                        rrelYP = (int)(rrelY * System.Math.Cos(headtilt / 180.0f * System.Math.PI) - rdepth * System.Math.Sin(headtilt / 180.0f * System.Math.PI));
                        rrelZP = (int)(rrelY * System.Math.Sin(headtilt / 180.0f * System.Math.PI) + rdepth * System.Math.Cos(headtilt / 180.0f * System.Math.PI));
                        //Console.WriteLine("x: " + x + "\ty: " + y);
                        //Console.WriteLine("x: " + relX + "\ty: " + relY + "\tz: " + depth + "\ttheta: " + headtilt);
                        //Console.WriteLine("x: " + rrelXP + "\ty: " + rrelYP + "\tz: " + rrelZP + "\ttheta: " + headtilt);
                        //textBox1.Text = "HEAD: x: " + hrelXP + "\ty: " + hrelYP + "\tz: " + hrelZP + "\ttheta: " + headtilt;
                        //textBox2.Text = "RHAND: x: " + rrelXP + "\ty: " + rrelYP + "\tz: " + rrelZP;
                        //Byte[] loppity = new Byte[]; TODO: figure out how to into byte array- image relations
                        
                    }
                }
                counter++;
                //DEBUG
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
            try
            {
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
            }
            catch (IndexOutOfRangeException e)
            {
                return null;
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

        private void textBox1_TextChanged_1(object sender, TextChangedEventArgs e)
        {

        }

        private void textBox2_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void meterToPixel(double x, double y, double z, Byte[] pixels)
        {
            const double RATIO = 549.46;
            int xPix, zPix;
            xPix = getXPix(x, y, z);
            zPix = (int)(z * RATIO);
            xPix = 32; zPix = 32;
            int stride = 3648 *4;
            for (int i = 0; i < 20; i++)
            {
                pixels[(zPix+i) * stride + (xPix+i) * 4] = 100;
            }
        }

        private int getXPix(double x, double y, double z)
        {
            const double h = 2.7432;
            const double k = -1.5764;
            const double r = 3.1639;
            const int numPix = 3648;
            double t0 = Math.Acos(-1 * h / r);
            double t1 = Math.Asin(-1 * k / r);
            double t = Math.Acos((x - h) / r);

            double pixX = r * ((t0 - t) / (t0 - t1)) * numPix;

            return ((int)pixX);
        }
    }
}
