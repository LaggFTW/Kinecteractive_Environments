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
using Microsoft.Kinect.Toolkit;
using Microsoft.Kinect.Toolkit.Interaction;
using InputManager;

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

            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            newSensor.SkeletonStream.Enable(parameters);
            //newSensor.SkeletonStream.Enable();
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
                        //Console.WriteLine("HEAD: x: " + hrelXP + "\ty: " + hrelYP + "\tz: " + hrelZP + "\ttheta: " + headtilt);
                        //Console.WriteLine("RHAND: x: " + rrelXP + "\ty: " + rrelYP + "\tz: " + rrelZP);
                        //Console.WriteLine("HEAD: x: " + (hrelXP / 1000.0 + 2.743) + "\ty: " + ((-hrelZP / 1000.0) + 1.087) + "\tz: " + (hrelYP / 1000.0 + 0.325));
                        //Console.WriteLine("RHAND: x: " + (rrelXP / 1000.0 + 2.743) + "\ty: " + ((-rrelZP / 1000.0) + 1.087) + "\tz: " + (rrelYP / 1000.0 + 0.325));
                        double[] intersects = intersectionPoints(hrelXP / 1000.0 + 2.743, (-hrelZP / 1000.0) + 1.087, hrelYP / 1000.0 + 0.325, rrelXP / 1000.0 + 2.743, (-rrelZP / 1000.0) + 1.087, rrelYP / 1000.0 + 0.325);
                        Console.WriteLine(intersects[0] + " " + intersects[1] + " " + intersects[2]);
                        meterToPixel(intersects[0], intersects[1], intersects[2]);
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

        private void meterToPixel(double x, double y, double z)
        {
            const double RATIO = 549.46;
            int xPix, zPix;
            xPix = getXPix(x, y, z);
            zPix = 1200-(int)(z * RATIO);
            moveCursor(xPix, zPix);
        }

        private double[] intersectionPoints(double x1, double y1, double z1, double x2, double y2, double z2)
        {
            const double X_CENTER = 2.7432;
            const double Y_CENTER = -1.5764;
            const double RADIUS = 3.1639;

            double xd = x2 - x1;
            double yd = y2 - y1;
            double a = xd * xd + yd * yd;
            double b = 2 * ((x1 - X_CENTER) * xd + (y1 - Y_CENTER) * yd);
            double c = (x1 - X_CENTER) * (x1 - X_CENTER) + (y1 - Y_CENTER) * (y1 - Y_CENTER) - RADIUS * RADIUS;

            double t = (-b + Math.Sqrt(b * b - 4 * a * c)) / (2 * a);

            double[] position = {(x1 + (x2 - x1)*t), (y1 + (y2 - y1)*t), (z1 + (z2 - z1)*t)};

            return position;
            //double xd = 
            //double m = d - a;
            //double n = e - b;
            //double u = Math.Pow(m, 2) + Math.Pow(n, 2);
            //double w = Math.Pow(a - 2.7432, 2) + Math.Pow((b + 1.5764), 2) - Math.Pow(3.1639, 2);
            //double v = 2 * ((a - 2.7432) * m + (b + 1.5764) * n);

            //double t = (((-1) * v + Math.Sqrt(Math.Pow(v, 2) - (4 * u * w))) / (2 * u));

            //double[] position = {(a + (d-a)*t), (b + (e-b)*t), -1*(c + (f-c)*t)}; // z is flipped
          
            //return position;
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

            double pixX = ((t0 - t) / (t0 - t1)) * numPix;

            return ((int)pixX);
        }

        private void moveCursor(int x, int y)
        {
            if (x > 3648)
            {
                x = 3648;
            }
            if (x < 0)
            {
                x = 0;
            }
            if (y < 0)
            {
                y = 0;
            }
            if (y > 1200)
            {
                y = 1200;
            }
            InputManager.Mouse.Move(x + 3840, y);
        }

        
    }
}
