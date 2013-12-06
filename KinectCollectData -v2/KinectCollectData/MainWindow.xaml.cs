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
 

namespace KinectCollectData
{
    public partial class MainWindow : Window
    {
        private KinectSensor _sensor;
        private BitmapSource _bitmap;
        private WriteableBitmap _depthMap;
        private byte[] _bitmapBits;
        private ColorImagePoint[] _mappedDepthLocations;
        private byte[] _colorPixels = new byte[0];
      //  private short[] _depthPixels = new short[0];
        private DepthImagePixel[] _depthPixels = new DepthImagePixel[0];
        Skeleton[] skeletonData = new Skeleton[0];
        // rectangles for different crop area
        Int32Rect necktieColorRect = new Int32Rect();
        Int32Rect necktieDepthRect = new Int32Rect();
        Int32Rect collarColorRect = new Int32Rect();
        Int32Rect collarDepthRect = new Int32Rect();
        Int32Rect glassColorRect = new Int32Rect();
        Int32Rect glassDepthRect = new Int32Rect();
        Int32Rect fullbodyRect = new Int32Rect();

        bool rectBool = false;
        bool fullbodyBool = false;
        int frameCount = 0;
        BoneOrientation cHipOrientation;

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

        void writeDepthData(DepthImagePixel[] _depthPixels, string path,int rectX, int rectY, int width, int height)
        {
            
            // save cropped text file for necktie
            string tmpS = "";
            using (StreamWriter writer = new StreamWriter(path, true))
            {                
                writer.WriteLine(width);
                writer.WriteLine(height);
                for (int j = 0; j < height; j++)
                {
                    for (int i = 0; i < width; i++)
                        tmpS += _depthPixels[rectX + i + 640 * (j + rectY)].Depth.ToString() + " ";
                    writer.WriteLine(tmpS);
                    tmpS = "";
                }
                
            }
            
            return;
        }
        void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            string timeNow = DateTime.Now.ToString("yyyyMMddHHmmss");

            
            // process for skeleton data
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    // copy skeleton information to skeletonData
                    skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletonData);                    
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
                    if (frameCount % 30 == 0)
                    {
                        foreach (Skeleton skeleton in skeletonData)
                        {
                            if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            {
                                //map skeleton point to color coordinate
                                ColorImagePoint head = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.Head].Position, ColorImageFormat.RgbResolution640x480Fps30);
                                ColorImagePoint shoulderCenter = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ShoulderCenter].Position, ColorImageFormat.RgbResolution640x480Fps30);
                                ColorImagePoint hipCenter = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.HipCenter].Position, ColorImageFormat.RgbResolution640x480Fps30);
                                ColorImagePoint shoulderRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ShoulderRight].Position, ColorImageFormat.RgbResolution640x480Fps30);
                                ColorImagePoint shoulderLeft = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.ShoulderLeft].Position, ColorImageFormat.RgbResolution640x480Fps30);
                                ColorImagePoint ankleRight = this._sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeleton.Joints[JointType.AnkleRight].Position,ColorImageFormat.RgbResolution640x480Fps30);

                                int height = Math.Abs(head.Y - hipCenter.Y);
                                int width = Math.Abs(shoulderLeft.X - shoulderRight.X);
                                int headHeight = Math.Abs(head.Y - shoulderCenter.Y);
                                // for necktie
                                necktieColorRect = new Int32Rect(shoulderLeft.X, (head.Y + shoulderCenter.Y) / 2, width, height);
                                collarColorRect = new Int32Rect(shoulderLeft.X, (head.Y + shoulderCenter.Y) / 2, width, height *3 / 7);
                                glassColorRect = new Int32Rect(shoulderLeft.X, (head.Y - headHeight/3*2), width, headHeight);
                                fullbodyRect = new Int32Rect((int)(shoulderLeft.X/1.1), (int)(head.Y / 1.2), (int)(width * 1.7), (int)((ankleRight.Y - head.Y)*1.1));
                                if (shoulderLeft.X + width * 1.3 < 640 && (int)((ankleRight.Y-head.Y)*1.1) < 480)
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
            }
            if (fullbodyBool)
            {
                if (frameCount % 30 == 0)
                {
                    try
                    {
                        CroppedBitmap cropBody = new CroppedBitmap(_bitmap, fullbodyRect);
                        cropBody.Save("C:\\KinectFullBodyImage\\" + timeNow + ".jpg", ImageFormat.Jpeg);
                    }
                    catch
                    {
                    }
                    
                }
                fullbodyBool = false;
            }
            if (rectBool)
            {         
                // save a cropped RGB image every 30 frames
                if (frameCount % 30 == 0)
                {
                    CroppedBitmap crop = new CroppedBitmap(_bitmap, necktieColorRect);
                    CroppedBitmap cropCollar = new CroppedBitmap(_bitmap, collarColorRect);
                    CroppedBitmap cropGlass = new CroppedBitmap(_bitmap, glassColorRect);
                    cropCollar.Save("C:\\KinectColorImage\\" + timeNow + "_collar.jpg",ImageFormat.Jpeg);
                    cropGlass.Save("C:\\KinectColorImage\\" + timeNow + "_glass.jpg", ImageFormat.Jpeg);
                    JpegBitmapEncoder encoder = new JpegBitmapEncoder();
                    encoder.Frames.Add(BitmapFrame.Create(crop));
                    FileStream fstream = new FileStream("C:\\KinectColorImage\\" + timeNow + "_necktie.jpg", FileMode.Create, FileAccess.Write);
                    encoder.Save(fstream);
                    fstream.Close();
                }
                rectBool = false;
            }

            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    Debug.Assert(depthFrame.Width == 640 && depthFrame.Height == 480, "This app only uses 640x480.");

                    if (_depthPixels.Length != depthFrame.PixelDataLength)
                    {
                        //_depthMap = new WriteableBitmap(640, 480, 96.0, 96.0, PixelFormats.Bgr32, null);
                        //   _depthPixels = new short[depthFrame.PixelDaataLength];
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
                            DepthImagePoint shoulderRight = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ShoulderRight].Position, DepthImageFormat.Resolution640x480Fps30);
                            DepthImagePoint shoulderLeft = this._sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.ShoulderLeft].Position, DepthImageFormat.Resolution640x480Fps30);

                            int height = Math.Abs(head.Y - hipCenter.Y);
                            int width = Math.Abs(shoulderLeft.X - shoulderRight.X);
                            int headHeight = Math.Abs(head.Y - shoulderCenter.Y);

                            // for necktie
                            necktieDepthRect = new Int32Rect(shoulderLeft.X, (head.Y + shoulderCenter.Y) / 2, width, height);
                            collarDepthRect = new Int32Rect(shoulderLeft.X, (head.Y + shoulderCenter.Y) / 2, width, height *3 /7 );
                            glassDepthRect = new Int32Rect(shoulderLeft.X, (head.Y - headHeight/3 * 2), width, headHeight);

                            if (width > 50 && height > 50 && headHeight > 10 && height < 400 && width < 600 && shoulderLeft.X > 0 && shoulderCenter.Y > 0 && head.Y - headHeight / 3 * 2 > 0 && (head.Y + shoulderCenter.Y) > 0)
                                rectBool = true;

                        }
                        else if (skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                        {

                        }

                    }
                    if (rectBool)
                    {
                        if (frameCount % 30 == 0)
                        {
                           
                            // save cropped image
                            /*
                            CroppedBitmap crop = new CroppedBitmap(_depthMap, necktieDepthRect);                            
                            crop.Save("C:\\KinectDepthImage\\" + timeNow + "_necktie_depth.jpg", ImageFormat.Jpeg);
                            */                            
                            
                            Thread neckThread = new Thread( ()=>writeDepthData(_depthPixels, "C:\\KinectDepthData\\" + timeNow + "_necktie.txt", necktieDepthRect.X, necktieDepthRect.Y, necktieDepthRect.Width, necktieDepthRect.Height));
                            neckThread.Start();

                            Thread collarThread = new Thread(() => writeDepthData(_depthPixels, "C:\\KinectDepthData\\" + timeNow + "_collar.txt", collarDepthRect.X, collarDepthRect.Y, collarDepthRect.Width, collarDepthRect.Height));
                            collarThread.Start();

                            Thread glassThread = new Thread(() => writeDepthData(_depthPixels, "C:\\KinectDepthData\\" + timeNow + "_glass.txt", glassDepthRect.X, glassDepthRect.Y, glassDepthRect.Width, glassDepthRect.Height));
                            glassThread.Start();                                                       
                            
                        }
                        rectBool = false;
                    }                    
                }
            }
            frameCount++;
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
