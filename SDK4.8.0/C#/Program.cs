using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Drawing;
using System.Drawing.Imaging;
using System;

namespace CsharpDemo
{
    class Program
    {
        [StructLayout(LayoutKind.Sequential)]
        public struct DirVector
        {
            public double vx;
            public double vy;
            public double vz;
        }
        public struct MarkerPosition
        {
            public double x;
            public double y;
            public double z;
            public int type;
            public bool RF;
        }
        public struct TrackingData
        {
            public int matchStatus;
            public int matchedPlane;
            public int markerNum;
            public int pointNum;

            public double tx;
            public double ty;
            public double tz;

            public double ox;
            public double oy;
            public double oz;

            public double qw;
            public double qx;
            public double qy;
            public double qz;

            public double rxx;
            public double ryx;
            public double rzx;
            public double rxy;
            public double ryy;
            public double rzy;
            public double rxz;
            public double ryz;
            public double rzz;

            public double dx;
            public double dy;
            public double dz;

            public double error;
            public int index;
        }
        public struct PointInf
        {
            public float x;
            public float y;
            public float z;
            public float r;
            public float g;
            public float b;
        };
        public struct ToolInf
        {
            public string name;
            public bool markerType;
            public int minNumMarker;
            public int planeNum;
            public int pointNum;
            public double dirx;
            public double diry;
            public double dirz;
            public double pinx;
            public double piny;
            public double pinz;
            public double calbError;
            public double maxFRE;
            public double maxAngle;
            public int structureType;
            public int algorithmType;
        };
        public struct NetAdaptorInfo
        {
            public string name;
            public string connectionType;
            public int speed;
            public bool linked;
        };

        public struct RTDeviceInfo
        {
            public string hostname;
            public string IP;
            public string MAC;
            public int deviceType;
            public int firmwareVersionL;
            public int firmwareVersionM;
            public int firmwareVersionH;
        };

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void autoDeviceScan(ref int num);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void getScannedDevice(int i, StringBuilder hostname, StringBuilder ip);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int connectDevice(string hostname);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void disconnectDevice();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern RTDeviceInfo getDeviceInfo();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern ushort generateAROM(string directory, ToolInf tool, int[] markerPlaneNum, DirVector[] markerPlaneDir, MarkerPosition[] markers, MarkerPosition[] points, double radius = 5);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern bool saveAROM(string directory, string toolName);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void loadPassiveToolAROM(string path);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getTrackToolsNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void getTrackToolsName(int index, StringBuilder str, int strLen);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setTrackingDataTransmissionType(int type);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void startTracking();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void stopTracking();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getTrackingToolCurrentNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern TrackingData getSelectedTrackingData(int index);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern MarkerPosition getSelectedMarkerData(int indexT, int indexM);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getAllMarkersNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getPassiveMarkersNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getActiveMarkersNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern DirVector getGravityVector();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern MarkerPosition getMarker(int index);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getUnMatchedMarkersNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern MarkerPosition getUnMatchedMarker(int index);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void trackingUpdate();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void toolUpdate();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void startImaging();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void stopImaging();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern IntPtr getLeftImagingData();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern IntPtr getRightImagingData();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void getIFImageSize(ref int size);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setAreaDisplay(short width, short height, short leftX, short leftY, short rightX, short rightY);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setAreaHidden();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void startMonitoring(bool r);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void stopMonitoring();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void getMonitorImageSize(ref int size);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setMonitorImageResolution(bool resolution);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void monitoringUpdate();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern IntPtr getMonitorImagingData();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern ushort getConnectionStatus();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern ushort pivotTipCalibration(string toolname, TrackingData[] trackingdataVect, int dataNum, ref double tip, ref double offset, ref double error, bool update = false);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern ushort fixedTipCalibration(string toolname, TrackingData trackingdataVect, MarkerPosition point, ref double tip, ref double offset, bool update = false);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern ushort fixedDirCalibration(string toolname, TrackingData trackingdataVect, ref double vec, ref double dir, bool update = false);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void beep(int t);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void laser(bool s);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void laserOn(int t);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern NetAdaptorInfo getNetAdaptor();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern ushort getSystemAlert();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setMaterial(int material);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void reconstruction(int width, int height, int leftX, int leftY, int rightX, int rightY);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern PointInf getPointCloud(int index);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int getPointNum();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern IntPtr getTextureImageUChar();

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void getDepthImage(float[] depthImage);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void getDepthImageSize(ref int size);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern int project2DImagePosition(int x, int y);
        
        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setIFLuminance(double l, int side);
        
        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern void setIFContrast(double c, int side);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern bool savePointCloud(string path, bool texture);

        [DllImport("ARMDCSDLLWarpperPro")]
        public static extern bool saveLog(string path);

        public static bool connectflag = false;

        public static Thread updateThread1 = null;// for tracking
        public static Thread updateThread2 = null;// for monitoring if the device is RT-PRO or RT-MAXV

        public static TrackingData trackingTooldata1;
        public static TrackingData trackingTooldata2;
        public static List<TrackingData> trackingdataVect = new List<TrackingData>();
        public static DirVector gravity;
        public static byte[] leftImg;
        public static byte[] rightImg;
        public static byte[] centerImg;
        public static int[] VMsize = { 0, 0 };
        public static int[] IFsize = { 0, 0 };

        static void Main(string[] args)
        {
            //generate a tool calibration file (*.arom) for tracking
            DirVector dir;
            dir.vx = 0;
            dir.vy = 0;
            dir.vz = 0;
            MarkerPosition a = new MarkerPosition();
            a.x = 0.00;
            a.y = 0.00;
            a.z = 0.00;
            MarkerPosition b = new MarkerPosition();
            b.x = 0.00;
            b.y = -50.00;
            b.z = 0.00;
            MarkerPosition c = new MarkerPosition();
            c.x = -25.00;
            c.y = -100.00;
            c.z = 0.00;
            MarkerPosition d = new MarkerPosition();
            d.x = 25.00;
            d.y = -135.00;
            d.z = 0.00;

            ToolInf tool = new ToolInf();
            tool.name = "ToolStandard";
            tool.markerType = false;
            tool.minNumMarker = 3;
            tool.planeNum = 1;
            tool.pointNum = 0;
            tool.dirx = 0;
            tool.diry = 0;
            tool.dirz = 0;
            tool.pinx = 0;
            tool.piny = 0;
            tool.pinz = 0;
            tool.calbError = -1;
            tool.maxFRE = 1;
            tool.maxAngle = 60;
            tool.structureType = 0;
            tool.algorithmType = 1;

            DirVector[] markerPlaneDir = { dir };
            int[] markerPlaneNum = { 4 };
            MarkerPosition[] markers = new MarkerPosition[] {a,b,c,d };
            MarkerPosition[] points = new MarkerPosition[] {};

            generateAROM("./Tool/", tool, markerPlaneNum, markerPlaneDir, markers, points);

            string deviceHostname;
            bool auto = false;
            if (auto)
            {
                //link the first scanned device
                int scannedDeviceNum = 0 ;
                while (scannedDeviceNum == 0)
                {
                    autoDeviceScan(ref scannedDeviceNum);
                }
                StringBuilder hostname = new StringBuilder(2 ^ 31);
                StringBuilder ip = new StringBuilder(2 ^ 31);
                getScannedDevice(0, hostname, ip);
                deviceHostname = hostname.ToString();
            }
            else
            {
                deviceHostname = "RT-PRO300113.local";
            }
            
            //connect device
            if (connectDevice(deviceHostname) == 0)
            {
                RTDeviceInfo deviceInfo = getDeviceInfo();
                Console.WriteLine("Connection successed!");
                Console.WriteLine("Device type is " + deviceInfo.deviceType);
                Console.WriteLine("Device IP is " + deviceInfo.IP);
                Console.WriteLine("Device MAC is " + deviceInfo.MAC);

                NetAdaptorInfo netInfo = getNetAdaptor();
                Console.WriteLine("Net adaptor name is " + netInfo.name);

                loadPassiveToolAROM("./Tool");
                int toolnum = getTrackToolsNum();
                Console.WriteLine("Tracking tool number is: " + toolnum);

                //start tracking
                setTrackingDataTransmissionType(1);
                startTracking();
                startImaging();
                startMonitoring(false);

                getIFImageSize(ref IFsize[0]);
                int IFwidth = IFsize[0];
                int IFheight = IFsize[1];
                //Console.WriteLine("Infrared image size is (" + IFwidth + "," + IFheight + ")");

                getMonitorImageSize(ref VMsize[0]);
                int VMwidth = VMsize[0];
                int VMheight = VMsize[1];
                Console.WriteLine("Monitor image size is (" + VMwidth + "," + VMheight + ")");

                if (deviceInfo.deviceType > 4)
                {
                    leftImg = new byte[IFwidth * IFheight];
                    rightImg = new byte[IFwidth * IFheight];
                    centerImg = new byte[VMwidth * VMheight * 3];

                    //establish monitoring thread
                    connectflag = true;
                    updateThread2 = new Thread(() => { RTUpdateMonitoring(); });
                    updateThread2.Start();
                }
                else if (deviceInfo.deviceType == 4)
                {
                    leftImg = new byte[IFwidth * IFheight];
                    rightImg = new byte[IFwidth * IFheight];
                    centerImg = new byte[VMwidth * VMheight * 3];

                    //establish monitoring thread
                    connectflag = true;
                    updateThread2 = new Thread(() => { RTUpdateMonitoring(); });
                    updateThread2.Start();
                }
                else
                {
                    connectflag = true;
                }

                //establish tracking thread
                updateThread1 = new Thread(() => { RTUpdateTracking(); });
                updateThread1.Start();

                while (connectflag)
                {
                    ToolUpdate();
                }

                Disconnect();
            }
            else
            {
                Console.WriteLine(getConnectionStatus());
                Console.WriteLine("Connection failed!");
                Console.WriteLine("Press Enter to continue...");
            }
        }

        public static void RTUpdateTracking()
        {
            while (connectflag)
            {
                //show system alert information
                if (getSystemAlert() != 0)
                    Console.WriteLine("SystemAlert: " + getSystemAlert());

                //interuption check
                if (getConnectionStatus() != 0)
                {
                    Console.WriteLine("ConnectionStatus: " + getConnectionStatus());
                    connectflag = false;
                    return;
                }

                //update tracking data 
                trackingUpdate();

                Marshal.Copy(getLeftImagingData(), leftImg, 0, IFsize[0] * IFsize[1]);
                Marshal.Copy(getRightImagingData(), rightImg, 0, IFsize[0] * IFsize[1]);
            }
        }

        public static void RTUpdateMonitoring()
        {
            while (connectflag)
            {
                //show system alert information
                if (getSystemAlert() != 0)
                    Console.WriteLine("SystemAlert: " + getSystemAlert());

                //interuption check
                if (getConnectionStatus() != 0)
                {
                    Console.WriteLine("ConnectionStatus: " + getConnectionStatus());
                    connectflag = false;
                    return;
                }

                //update monitoring data 
                monitoringUpdate();

                //save the image
                //Marshal.Copy(getMonitorImagingData(), centerImg, 0, VMsize[0] * VMsize[1] * 3);
                //Bitmap bitmap = new Bitmap(VMsize[0], VMsize[1], PixelFormat.Format24bppRgb);
                //BitmapData bitmapData = bitmap.LockBits(new Rectangle(0, 0, VMsize[0], VMsize[1]), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
                //System.Runtime.InteropServices.Marshal.Copy(centerImg, 0, bitmapData.Scan0, VMsize[0] * VMsize[1] * 3);
                //bitmap.UnlockBits(bitmapData);
                //bitmap.Save("monitorImage.jpg", ImageFormat.Jpeg);
                //bitmap.Dispose();
            }
        }

        public static void ToolUpdate()
        {
            if (connectflag)
            {
                //update tool data 
                toolUpdate();

                gravity = getGravityVector();
                //Debug.Log("Gravity vector is " + gravity.vx + ", " + gravity.vy + ", " + gravity.vz);

                int allMarkerNum = getAllMarkersNum();
                int passiveMarkerNum = getPassiveMarkersNum();
                int activeMarkerNum = getActiveMarkersNum();
                Console.WriteLine("All markers number is " + allMarkerNum);
                Console.WriteLine("All passive markers number is " + passiveMarkerNum);
                Console.WriteLine("All active markers number is " + activeMarkerNum);
                for (int i = 0; i < allMarkerNum; i++)
                {
                    MarkerPosition marker = getMarker(i);
                    Console.WriteLine("Marker " + (i+1) + "/" + allMarkerNum + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
                }

                //apply the tracking information
                for (int i = 0; i < getTrackingToolCurrentNum(); i++)
                {
                    StringBuilder toolname = new StringBuilder(2 ^ 31);

                    getTrackToolsName(i, toolname, toolname.Capacity);
                    string name = toolname.ToString();

                    if (name == "ToolScribing")
                    {
                        trackingTooldata1 = getSelectedTrackingData(i);
                        trackingTooldata1.index = i;
                    }
                    else if (name == "ToolStandard")
                    {
                        trackingTooldata2 = getSelectedTrackingData(i);
                        trackingTooldata2.index = i;
                    }
                }

                int straymarkerNum = getUnMatchedMarkersNum();
                Console.WriteLine("Stray marker number is " + straymarkerNum);
                for (int i = 0; i < straymarkerNum; i++)
                {
                    MarkerPosition marker = getUnMatchedMarker(i);
                    Console.WriteLine("Marker " + (i+1) + "/" + straymarkerNum + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
                }

                //update tool in scene according to its tracking information
                if (trackingTooldata1.matchStatus == 1)
                {
                    Console.WriteLine("Tool(" + trackingTooldata1.index + ")/tip: " + trackingTooldata1.tx + ", " + trackingTooldata1.ty + ", " + trackingTooldata1.tz);
                    Console.WriteLine("Tool(" + trackingTooldata1.index + ")/offset: " + trackingTooldata1.ox + ", " + trackingTooldata1.oy + ", " + trackingTooldata1.oz);
                    Console.WriteLine("Tool(" + trackingTooldata1.index + ")/quaternion: " + trackingTooldata1.qx + ", " + trackingTooldata1.qy + ", " + trackingTooldata1.qz + ", " + trackingTooldata1.qw);
                    Console.WriteLine("Tool(" + trackingTooldata1.index + ")/rotation: " + trackingTooldata1.rxx + ", " + trackingTooldata1.rxy + ", " + trackingTooldata1.rxz + ", "
                                                                                         + trackingTooldata1.ryx + ", " + trackingTooldata1.ryy + ", " + trackingTooldata1.ryz + ", "
                                                                                         + trackingTooldata1.rzx + ", " + trackingTooldata1.rzy + ", " + trackingTooldata1.rzz + ", ");

                    for (int j = 0; j < trackingTooldata1.markerNum; j++)
                    {
                        MarkerPosition marker = getSelectedMarkerData(trackingTooldata1.index, j);
                        //Console.WriteLine("Tool(" + trackingTooldata1.index + ")/Marker(" + j + ")-" + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
                    }
                }
                else
                {
                    Console.WriteLine("Tool(" + trackingTooldata1.index + ") is missing");
                }

                if (trackingTooldata2.matchStatus == 1)
                {
                    Console.WriteLine("Tool(" + trackingTooldata2.index + ")/tip: " + trackingTooldata2.tx + ", " + trackingTooldata2.ty + ", " + trackingTooldata2.tz);
                    Console.WriteLine("Tool(" + trackingTooldata2.index + ")/offset: " + trackingTooldata2.ox + ", " + trackingTooldata2.oy + ", " + trackingTooldata2.oz);
                    Console.WriteLine("Tool(" + trackingTooldata2.index + ")/quaternion: " + trackingTooldata2.qx + ", " + trackingTooldata2.qy + ", " + trackingTooldata2.qz + ", " + trackingTooldata2.qw);
                    Console.WriteLine("Tool(" + trackingTooldata2.index + ")/rotation: " + trackingTooldata2.rxx + ", " + trackingTooldata2.rxy + ", " + trackingTooldata2.rxz + ", "
                                                                                         + trackingTooldata2.ryx + ", " + trackingTooldata2.ryy + ", " + trackingTooldata2.ryz + ", "
                                                                                         + trackingTooldata2.rzx + ", " + trackingTooldata2.rzy + ", " + trackingTooldata2.rzz + ", ");

                    for (int j = 0; j < trackingTooldata2.markerNum; j++)
                    {
                        MarkerPosition marker = getSelectedMarkerData(trackingTooldata2.index, j);
                        //Console.WriteLine("Tool(" + trackingTooldata2.index + ")/Marker(" + j + ")-" + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
                    }

                    //tool calibration
                    TrackingData calibData = getSelectedTrackingData(trackingTooldata2.index);
                    if (trackingdataVect.Count < 100)
                    {
                        if (trackingdataVect.Count > 0)
                        {
                            if (calibData.qw != trackingdataVect[trackingdataVect.Count - 1].qw ||
                                calibData.qx != trackingdataVect[trackingdataVect.Count - 1].qx ||
                                calibData.qy != trackingdataVect[trackingdataVect.Count - 1].qy ||
                                calibData.qz != trackingdataVect[trackingdataVect.Count - 1].qz)
                            {
                                trackingdataVect.Add(calibData);
                            }
                        }
                        else
                        {
                            trackingdataVect.Add(calibData);
                        }
                    }
                    else if (trackingdataVect.Count == 100)
                    {
                        int dataNum = trackingdataVect.Count;
                        double[] tip = { 0, 0, 0 };
                        double[] offset = { 0, 0, 0 };
                        double error = -1;
                        TrackingData[] trackingdataArray = new TrackingData[trackingdataVect.Count];
                        trackingdataArray = trackingdataVect.ToArray();

                        ushort res = pivotTipCalibration("ToolStandard", trackingdataArray, dataNum, ref tip[0], ref offset[0], ref error, true);
                        if (res == 0)
                        {
                            Console.WriteLine("######################################################################################################");
                            Console.WriteLine("Tip-x:" + tip[0] + " y:" + tip[1] + " z:" + tip[2]);
                            Console.WriteLine("Offset-x:" + offset[0] + " y:" + offset[1] + " z:" + offset[2]);
                            Console.WriteLine("Error: " + error);
                        }
                        trackingdataVect.Add(calibData);
                    }
                }
                else
                {
                    Console.WriteLine("Tool(" + trackingTooldata2.index + ") is missing");
                }
            }
        }

        public static void Disconnect()
        {
            connectflag = false;

            if (updateThread2 == null)
            {
                while (updateThread1.IsAlive)
                    Thread.Sleep(1);
            }
            else
            {
                while (updateThread1.IsAlive || updateThread2.IsAlive)
                    Thread.Sleep(1);
            }

            Console.WriteLine("Demo finish");

            stopImaging();
            stopTracking();
            stopMonitoring();
            disconnectDevice();

            leftImg = null;
            rightImg = null;
            centerImg = null;
            VMsize = null;
            IFsize = null;

            saveLog("./record.log");
        }

        public static void Reconstruction()
        {
            //reconstruction in the area 
            reconstruction(360, 363, 776, 418, 776, 418);

            //show on the scene
            int num = getPointNum();
            for (int i = 0; i < num; ++i)
            {
                float px = getPointCloud(i).x;
                float py = getPointCloud(i).y;
                float pz = getPointCloud(i).z;

                float pr = getPointCloud(i).r;
                float pg = getPointCloud(i).g;
                float pb = getPointCloud(i).b;
            }

            byte[] textureImg = new byte[1920 * 1200 * 3];
            Marshal.Copy(getTextureImageUChar(), textureImg, 0, 1920 * 1200 * 3);
            Bitmap bitmap = new Bitmap(1920, 1200, PixelFormat.Format24bppRgb);
            BitmapData bitmapData = bitmap.LockBits(new Rectangle(0, 0, 1920, 1200), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
            System.Runtime.InteropServices.Marshal.Copy(centerImg, 0, bitmapData.Scan0, 1920 * 1200 * 3);
            bitmap.UnlockBits(bitmapData);
            bitmap.Save("textureImage.jpg", ImageFormat.Jpeg);
            bitmap.Dispose();

            //save the point cloud
            savePointCloud("./PointCloud.ply", true);
            beep(40);
        }
    }
}