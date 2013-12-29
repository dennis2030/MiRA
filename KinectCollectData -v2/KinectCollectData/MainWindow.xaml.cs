using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

using System.Drawing;
using System.IO;
using Coding4Fun.Kinect.Wpf;
using System.Threading;
using System.Collections.Generic;
 

namespace KinectCollectData
{
    public partial class MainWindow : Window
    {
        private KinectSensor _sensor;
        private BitmapSource _bitmap;
        List<DepthImagePoint> skeletonList = new List<DepthImagePoint>();
        List<ColorImagePoint> colorSkeletonList = new List<ColorImagePoint>();
        private byte[] _colorPixels = new byte[0];
        
      //  private short[] _depthPixels = new short[0];
        private DepthImagePixel[] _depthPixels = new DepthImagePixel[0];
        private DepthImagePixel[] _dummyPixels = new DepthImagePixel[0];
        Skeleton[] skeletonData = new Skeleton[0];
        // rectangles for different crop area
        
        Int32Rect fullbodyRect = new Int32Rect();

        bool rectBool = false;
        bool fullbodyBool = false;
        int frameCount = 0;
        BoneOrientation cHipOrientation;
        BoneRotation cHipRotation = new BoneRotation();

        private void SetSensor(KinectSensor newSensor)
        {
            if (_sensor != null)
            {
                _sensor.Stop();
            }

            _sensor = newSensor;

            if (_sensor != null)
            {
                Debug.Assert(_sensor.Status == KinectStatus.Connected, "This should only be called with Connected sensors.");
                _sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                _sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                _sensor.SkeletonStream.Enable();
                _sensor.AllFramesReady += _sensor_AllFramesReady;
                _sensor.Start();
            }
        }
        void writeColorSkeleton(List<ColorImagePoint> colorSkeList, string path)
        {
             string tmpS = "";

            using (StreamWriter writer = new StreamWriter(path, true))
            {

                for (int i = 0; i < colorSkeList.Count; i++)
                {
                    if (i != 0)
                        tmpS += ",";
                    tmpS += colorSkeList[i].X + " " + colorSkeList[i].Y;
                }
                writer.WriteLine(tmpS);
                tmpS = "";                
            }
            
            return;
        }

        void writeDepthData(DepthImagePixel[] _depthPixels, List<DepthImagePoint> skeList, BoneRotation boneR,string path,int rectX, int rectY, int width, int height, bool isHuman)
        {
            
            // save cropped text file for necktie
            string tmpS = "";

            using (StreamWriter writer = new StreamWriter(path, true))
            {            
                // write width and height to the first 2 lines
                writer.WriteLine(width + " " + height);                

                // write one line about the positions of 20 skeleton joints
                for (int i = 0; i < skeList.Count; i++)
                {
                    if (i != 0)
                        tmpS += ",";
                    tmpS += skeList[i].X + " " + skeList[i].Y;
                }
                writer.WriteLine(tmpS);

                // writes the Quaternion of hip center
                Vector4 tmpV = boneR.Quaternion;
                tmpS = tmpV.W + " " + tmpV.X + " " + tmpV.Y + " " + tmpV.Z;
                writer.WriteLine(tmpS);

                // write the selected raw depth data
                tmpS = "";
                for (int j = 0; j < height; j++)
                {
                    for (int i = 0; i < width; i++)
                    {
                        DepthImagePixel tmpPixel = _depthPixels[rectX + i + 640 * (j + rectY)];
                        if (isHuman)
                        {
                            if (tmpPixel.PlayerIndex == 0)
                            {
                                tmpS += "0 ";
                                continue;
                            }
                        }

                        tmpS += tmpPixel.Depth.ToString() + " ";
                        
                    }
                    writer.WriteLine(tmpS);
                    tmpS = "";
                }
                
            }
            
            return;
        }
        void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            string timeNow = DateTime.Now.ToString("yyyyMMddHHmmss");

            frameCount++;
            if (frameCount % 30 > 0)
                return;
            skeletonList.Clear();
            colorSkeletonList.Clear();
            // process for skeleton data
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    // copy skeleton information to skeletonData
                    skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletonData);    
                    
                }
                foreach (Skeleton skeleton in skeletonData)
                {
                    if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        cHipOrientation = skeleton.BoneOrientations[JointType.HipCenter];
                        cHipRotation = cHipOrientation.AbsoluteRotation;
                    }
                }
            }

            // color part
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    Debug.Assert(colorFrame.Width == 640 && colorFrame.Height == 480, "This app only uses 640x480.");

                    if (_colorPixels.Length != colorFrame.PixelDataLength)
                    {
                        _bitmap = colorFrame.ToBitmapSource();
                        this.testImage.Source = _bitmap;
                    }
                    foreach (Skeleton skeleton in skeletonData)
                    {
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            //map skeleton point to color coordinate
                            ColorImagePoint head = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.Head].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint shoulderCenter = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ShoulderCenter].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint hipCenter = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.HipCenter].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint hipLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.HipLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint hipRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.HipRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint shoulderRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ShoulderRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint shoulderLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ShoulderLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint spine = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.Spine].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint handRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.HandRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint handLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.HandLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint elbowRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ElbowRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint elbowLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ElbowLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint wristRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.WristRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint wristLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.WristLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint kneeRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.KneeRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint kneeLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.KneeLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint footRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.FootRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint footLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.FootLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                            ColorImagePoint ankleRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.AnkleRight].Position, ColorImageFormat.RgbResolution640x480Fps30);                            
                            ColorImagePoint ankleLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.AnkleLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);

                            colorSkeletonList.Add(head);
                            colorSkeletonList.Add(shoulderCenter);
                            colorSkeletonList.Add(hipCenter);
                            colorSkeletonList.Add(hipLeft);
                            colorSkeletonList.Add(hipRight);
                            colorSkeletonList.Add(shoulderRight);
                            colorSkeletonList.Add(shoulderLeft);
                            colorSkeletonList.Add(spine);
                            colorSkeletonList.Add(handRight);
                            colorSkeletonList.Add(handLeft);
                            colorSkeletonList.Add(elbowRight);
                            colorSkeletonList.Add(elbowLeft);
                            colorSkeletonList.Add(wristRight);
                            colorSkeletonList.Add(wristLeft);
                            colorSkeletonList.Add(kneeRight);
                            colorSkeletonList.Add(kneeLeft);
                            colorSkeletonList.Add(footRight);
                            colorSkeletonList.Add(footLeft);
                            colorSkeletonList.Add(ankleRight);
                            colorSkeletonList.Add(ankleLeft);

                            int height = Math.Abs(head.Y - hipCenter.Y);
                            int width = Math.Abs(shoulderLeft.X - shoulderRight.X);
                            int headHeight = Math.Abs(head.Y - shoulderCenter.Y);
                            
                            fullbodyRect = new Int32Rect((int)(shoulderLeft.X / 1.1), (int)(head.Y / 1.2), (int)(width * 1.7), (int)((ankleRight.Y - head.Y) * 1.1));
                            if (shoulderLeft.X + width * 1.3 < 640 && (int)((ankleRight.Y - head.Y) * 1.1) < 480)
                                fullbodyBool = true;
                            if (width > 50 && height > 50 && headHeight > 10 && height < 400 && width < 600 && shoulderLeft.X > 0 && shoulderCenter.Y > 0 && (head.Y - headHeight / 3 * 2) > 0 && (head.Y + shoulderCenter.Y) > 0)
                                rectBool = true;

                        }
                        else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                        {

                        }
                    }                                       
                }
            }
            if (fullbodyBool)
            {               
                try
                {
                    CroppedBitmap cropBody = new CroppedBitmap(_bitmap, fullbodyRect);
                    cropBody.Save("C:\\KinectFullBodyImage\\" + timeNow + ".jpg", ImageFormat.Jpeg);
                }
                catch
                {
                }
                    
                fullbodyBool = false;
            }
            

            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    Debug.Assert(depthFrame.Width == 640 && depthFrame.Height == 480, "This app only uses 640x480.");

                    if (_depthPixels.Length != depthFrame.PixelDataLength)
                    {                        
                        _depthPixels = new DepthImagePixel[depthFrame.PixelDataLength];                        
                    }
                    
                    depthFrame.CopyDepthImagePixelDataTo(_depthPixels);                    

                    foreach (Skeleton skeleton in skeletonData)
                    {
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            
                            DepthImagePoint head = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.Head].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint shoulderCenter = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ShoulderCenter].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint hipCenter = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HipCenter].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint hipLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HipLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint hipRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HipRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint shoulderRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ShoulderRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint shoulderLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ShoulderLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint spine = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.Spine].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint handRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint handLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint elbowRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ElbowRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint elbowLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ElbowLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint wristRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.WristRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint wristLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.WristLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint kneeRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.KneeRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint kneeLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.KneeLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint footRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.FootRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint footLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.FootLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint ankleRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.AnkleRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint ankleLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.AnkleLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                            
                            skeletonList.Add(head);
                            skeletonList.Add(shoulderCenter);
                            skeletonList.Add(hipCenter);
                            skeletonList.Add(hipLeft);
                            skeletonList.Add(hipRight);
                            skeletonList.Add(shoulderRight);
                            skeletonList.Add(shoulderLeft);
                            skeletonList.Add(spine);
                            skeletonList.Add(handRight);
                            skeletonList.Add(handLeft);
                            skeletonList.Add(elbowRight);
                            skeletonList.Add(elbowLeft);
                            skeletonList.Add(wristRight);
                            skeletonList.Add(wristLeft);
                            skeletonList.Add(kneeRight);
                            skeletonList.Add(kneeLeft);
                            skeletonList.Add(footRight);
                            skeletonList.Add(footLeft);
                            skeletonList.Add(ankleRight);
                            skeletonList.Add(ankleLeft);

                            int height = Math.Abs(head.Y - hipCenter.Y);
                            int width = Math.Abs(shoulderLeft.X - shoulderRight.X);
                            int headHeight = Math.Abs(head.Y - shoulderCenter.Y);

                            if (width > 50 && height > 50 && headHeight > 10 && height < 400 && width < 600 && shoulderLeft.X > 0 && shoulderCenter.Y > 0 && head.Y - headHeight / 3 * 2 > 0 && (head.Y + shoulderCenter.Y) > 0)
                                rectBool = true;
                            // filter the body that is too close or too far away
                            if (shoulderCenter.Depth < 400 || shoulderCenter.Depth > 2500)
                                rectBool = false;

                        }
                        else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                        {

                        }

                    }
                    if (rectBool)
                    {
                        _bitmap.Save("C:\\KinectNewColor\\" + timeNow.ToString() + ".jpg", ImageFormat.Jpeg);

                        Thread colorSkeThread = new Thread(() => writeColorSkeleton(colorSkeletonList, "C:\\KinectNewColor\\" + timeNow + ".txt"));
                        colorSkeThread.Start();


                        Thread rawDataThread = new Thread(() => writeDepthData(_depthPixels, skeletonList, cHipRotation, "C:\\KinectNewDepth\\" + timeNow + ".txt", 0, 0, 640, 480, false));
                        rawDataThread.Start();

                        Thread humanThread = new Thread(() => writeDepthData(_depthPixels, skeletonList, cHipRotation, "C:\\KinectNewDepth\\" + timeNow + "_human.txt", 0, 0, 640, 480, true));
                        humanThread.Start(); 
                        rectBool = false;
                    }                    
                }
            }
            
            rectBool = false;
        }

        public MainWindow()
        {
            InitializeComponent();

            KinectSensor.KinectSensors.StatusChanged += (object sender, StatusChangedEventArgs e) =>
            {
                if (e.Sensor == _sensor)
                {
                    if (e.Status != KinectStatus.Connected)
                    {
                        SetSensor(null);
                    }
                }
                else if ((_sensor == null) && (e.Status == KinectStatus.Connected))
                {
                    SetSensor(e.Sensor);
                }
            };

            foreach (var sensor in KinectSensor.KinectSensors)
            {
                if (sensor.Status == KinectStatus.Connected)
                {
                    SetSensor(sensor);
                }
            }
        }
    }
}
