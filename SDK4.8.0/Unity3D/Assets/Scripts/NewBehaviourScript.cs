using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System;

public class NewBehaviourScript : MonoBehaviour
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
    public static extern void autoDeviceScan(ref int size);

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

    bool connectflag = false;

    Thread updateThread1 = null;
    Thread updateThread2 = null;

    TrackingData trackingTooldata1;
    TrackingData trackingTooldata2;
    DirVector gravity;
    byte[] leftImg;
    byte[] rightImg;
    byte[] centerImg;
    public Texture2D leftTexture;
    public Texture2D rightTexture;
    public Texture2D centerTexture;
    int[] VMsize = { 0, 0 };
    int[] IFsize = { 0, 0 };

    // Start is called before the first frame update
    void Start()
    {
        string deviceHostname;
        bool auto = false;
        if (auto)
        {
            //link the first scanned device
            int scannedDeviceNum = 0;
            while (scannedDeviceNum == 0)
            {
                autoDeviceScan(ref scannedDeviceNum);
            }
            StringBuilder hostname = new StringBuilder(2 ^ 31);
            StringBuilder ip = new StringBuilder(2 ^ 31);
            getScannedDevice(0, hostname, ip);
            deviceHostname = hostname.ToString();
            Debug.Log("The scanned device hostname is " + deviceHostname);
        }
        else
        {
            deviceHostname = "RT-PRO300113.local";
        }

        if (connectDevice(deviceHostname) == 0)
        {
            RTDeviceInfo deviceInfo = getDeviceInfo();
            Debug.Log("Connection successed! Device type is " + deviceInfo.deviceType);
            loadPassiveToolAROM(UnityEngine.Application.dataPath + "/Tool");
            int toolnum = getTrackToolsNum();
            Debug.Log("Tracking tool number is: " + toolnum);

            //start tracking
            setTrackingDataTransmissionType(4);
            startTracking();
            startImaging();
            startMonitoring(false);

            getIFImageSize(ref IFsize[0]);
            int IFwidth = IFsize[0];
            int IFheight = IFsize[1];
            //Debug.Log("Infrared image size is (" + IFwidth + "," + IFheight + ")");

            getMonitorImageSize(ref VMsize[0]);
            int VMwidth = VMsize[0];
            int VMheight = VMsize[1];
            //Debug.Log("Monitor image size is (" + VMwidth + "," + VMheight + ")");

            if (deviceInfo.deviceType > 4)
            {
                leftTexture = new Texture2D(IFwidth, IFheight, TextureFormat.R8, false);
                rightTexture = new Texture2D(IFwidth, IFheight, TextureFormat.R8, false);
                centerTexture = new Texture2D(VMwidth, VMheight, TextureFormat.RGB24, false);
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
                leftTexture = new Texture2D(IFwidth, IFheight, TextureFormat.R8, false);
                rightTexture = new Texture2D(IFwidth, IFheight, TextureFormat.R8, false);
                centerTexture = new Texture2D(VMwidth, VMheight, TextureFormat.RGB24, false);
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
                leftTexture = new Texture2D(IFwidth, IFheight, TextureFormat.R8, false);
                rightTexture = new Texture2D(IFwidth, IFheight, TextureFormat.R8, false);

                connectflag = true;
            }

            //establish tracking thread
            updateThread1 = new Thread(() => { RTUpdateTracking(); });
            updateThread1.Start();

            setAreaDisplay(360, 363, 776, 418, 776, 418);
        }
        else
        {
            Debug.Log(getConnectionStatus());
            Debug.Log("Connection failed!");
            Debug.Log("Press Enter to continue...");
        }
    }

    public void RTUpdateTracking()
    {
        while (connectflag)
        {
            //show system alert information
            if (getSystemAlert() != 0)
                Debug.Log("SystemAlert: " + getSystemAlert());

            //interuption check
            if (getConnectionStatus() != 0)
            {
                Debug.Log("ConnectionStatus: " + getConnectionStatus());
                connectflag = false;
                return;
            }

            //update tracking data 
            trackingUpdate();

            Marshal.Copy(getLeftImagingData(), leftImg, 0, IFsize[0] * IFsize[1]);
            Marshal.Copy(getRightImagingData(), rightImg, 0, IFsize[0] * IFsize[1]);
        }
    }

    public void RTUpdateMonitoring()
    {
        while (connectflag)
        {
            //show system alert information
            if (getSystemAlert() != 0)
                Debug.Log("SystemAlert: " + getSystemAlert());

            //interuption check
            if (getConnectionStatus() != 0)
            {
                Debug.Log("ConnectionStatus: " + getConnectionStatus());
                connectflag = false;
                return;
            }           

            //update monitoring data 
            monitoringUpdate();

            Marshal.Copy(getMonitorImagingData(), centerImg, 0, VMsize[0] * VMsize[1] * 3) ;
        }
    }

    // Update is called once per frame
    void Update()
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
            //Debug.Log("All markers number is " + allMarkerNum);
            //Debug.Log("All passive markers number is " + passiveMarkerNum);
            Debug.Log("All active markers number is " + activeMarkerNum);
            for (int i = 0; i < allMarkerNum; i++)
            {
                MarkerPosition marker = getMarker(i);
                //Debug.Log("Marker " + i + "/" + size + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
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
            //Debug.Log("Stray marker number is " + straymarkerNum);
            for (int i = 0; i < straymarkerNum; i++)
            {
                MarkerPosition marker = getUnMatchedMarker(i);
                //Debug.Log("Marker " + i + "/" + straymarkerNum + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
            }

            //update tool in scene according to its tracking information
            GameObject toolscribing = GameObject.Find("ToolScribing/default");
            if (trackingTooldata1.matchStatus == 1)
            {
                toolscribing.transform.position = new Vector3(-(float)(trackingTooldata1.ox / 1000.0), -(float)(trackingTooldata1.oy / 1000.0), (float)(trackingTooldata1.oz / 1000.0));
                toolscribing.transform.rotation = new Quaternion((float)trackingTooldata1.qx, -(float)trackingTooldata1.qy, (float)trackingTooldata1.qz, (float)trackingTooldata1.qw);
                toolscribing.GetComponent<MeshRenderer>().material.color = new Color(0, 1, 0, 1);

                for (int j = 0; j < trackingTooldata1.markerNum; j++)
                {
                    MarkerPosition marker = getSelectedMarkerData(trackingTooldata1.index, j);
                    //Debug.Log("Tool(" + trackingTooldata1.index + ")/Marker(" + j + ")-" + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
                }
            }
            else
                toolscribing.GetComponent<MeshRenderer>().material.color = new Color(1, 0, 0, 1);

            GameObject toolstandard = GameObject.Find("ToolStandard/default");
            if (trackingTooldata2.matchStatus == 1)
            {
                toolstandard.transform.position = new Vector3(-(float)(trackingTooldata2.ox / 1000.0), -(float)(trackingTooldata2.oy / 1000.0), (float)(trackingTooldata2.oz / 1000.0));
                toolstandard.transform.rotation = new Quaternion((float)trackingTooldata2.qx, -(float)trackingTooldata2.qy, (float)trackingTooldata2.qz, (float)trackingTooldata2.qw);
                toolstandard.GetComponent<MeshRenderer>().material.color = new Color(0, 1, 0, 1);

                for (int j = 0; j < trackingTooldata2.markerNum; j++)
                {
                    MarkerPosition marker = getSelectedMarkerData(trackingTooldata2.index, j);
                    //Debug.Log("Tool(" + trackingTooldata2.index + ")/Marker(" + j + ")-" + " position: " + marker.x + ", " + marker.y + ", " + marker.z);
                }
            }
            else
                toolstandard.GetComponent<MeshRenderer>().material.color = new Color(1, 0, 0, 1);

            //show left and right images
            leftTexture.LoadRawTextureData(leftImg);
            leftTexture.Apply();
            GameObject leftRawImageContainer = GameObject.Find("LeftImage/Canvas/RawImage");
            RawImage leftRawImage = leftRawImageContainer.GetComponent<RawImage>();
            leftRawImage.texture = leftTexture;

            rightTexture.LoadRawTextureData(rightImg);
            rightTexture.Apply();
            GameObject rightRawImageContainer = GameObject.Find("RightImage/Canvas/RawImage");
            RawImage rightRawImage = rightRawImageContainer.GetComponent<RawImage>();
            rightRawImage.texture = rightTexture;

            //show center image
            centerTexture.LoadRawTextureData(centerImg);
            centerTexture.Apply();
            GameObject centerRawImageContainer = GameObject.Find("CenterImage/Canvas/RawImage");
            RawImage centerRawImage = centerRawImageContainer.GetComponent<RawImage>();
            centerRawImage.texture = centerTexture;
        }
    }

    void OnDestroy()
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

        Debug.Log("Demo finish");

        stopImaging();
        stopTracking();
        stopMonitoring();
        disconnectDevice();

        saveLog("./record.log");

        leftImg = null;
        rightImg = null;
        centerImg = null;
        VMsize = null;
        IFsize = null;
        Resources.UnloadUnusedAssets();
    }

    //link to the button
    public void Click()
    {
        //reconstruction in the area 
        reconstruction(360, 363, 776, 418, 776, 418);

        //show on the scene
        CreateMesh();

        //save the point cloud
        savePointCloud("./PointCloud.ply", true);
        beep(40);
    }

    void CreateMesh()
    {

        GameObject pointobj = new GameObject();
        pointobj.name = "reconstructed point cloud";
        pointobj.transform.localScale = new Vector3(100, 100, 20);
        Material mat = new Material(Shader.Find("Custom/VertexColor"));
        pointobj.AddComponent<MeshFilter>();
        pointobj.AddComponent<MeshRenderer>();

        Mesh meshNeed = new Mesh();
        pointobj.GetComponent<MeshFilter>().mesh = meshNeed;
        pointobj.GetComponent<MeshRenderer>().material = mat;

        int num = getPointNum();
        Debug.Log("point number is " + num);
        Vector3[] points = new Vector3[num];
        Color[] colors = new Color[num];
        int[] indecies = new int[num];
        for (int i = 0; i < num; ++i)
        {
            points[i] = new Vector3(-getPointCloud(i).x / 1000.0f, -getPointCloud(i).y / 1000.0f, getPointCloud(i).z / 1000.0f);
            indecies[i] = i;
            colors[i] = new Color(getPointCloud(i).r, getPointCloud(i).g, getPointCloud(i).b);
        }

        meshNeed.vertices = points;
        meshNeed.colors = colors;
        meshNeed.SetIndices(indecies, MeshTopology.Points, 0);
    }
}