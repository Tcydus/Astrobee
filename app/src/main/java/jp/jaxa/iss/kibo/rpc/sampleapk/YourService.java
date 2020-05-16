package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.yanzhenjie.zbar.Config;
import com.yanzhenjie.zbar.Image;
import com.yanzhenjie.zbar.ImageScanner;
import com.yanzhenjie.zbar.Symbol;
import com.yanzhenjie.zbar.SymbolSet;


import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;
import java.util.List;


import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static java.lang.Math.sqrt;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        api.judgeSendStart();


        String pos_x = GotoQR(11.45, -5.7f, 4.588f, 0.0f, 0.0f, 0.0f,1.0f);
        api.judgeSendDiscoveredQR(0,pos_x);
        String pos_z = GotoQR(11.00f, -5.50f, 4.40f, 0.707f, 0.0f, -0.707f,0.0f);
        api.judgeSendDiscoveredQR(2,pos_z);
//        String pos_y = GotoQR(10.95f, -5.958f, 5.42f, -0.707f, 0.0f, -0.707f,0.0f); //Old (right side near airlock
        String pos_y = GotoQR(10.95f, -5.958f, 5.42f, 0.5f, 0.5f, 0.5f,-0.5f); //New (above side near airlock)
        api.judgeSendDiscoveredQR(1,pos_y);
//
        viaMove(10.50f, -6.45f, 5.44f, 0.0f, 0.0f, 0.0f, 0.0f);
//        viaMove(11.00f, -7.15f, 5.44f, 0.0f, 0.0f, 0.707f, -0.707f);

        String pos_qy = GotoQR(11.47, -7.958, 5.083, 0.0, 0.0, 0.0,1.0);
        api.judgeSendDiscoveredQR(4,pos_qy);
        String pos_qz = GotoQR(10.917, -7.658, 5.42, 0.707f, 0.0f, 0.707f,0.0f);
        api.judgeSendDiscoveredQR(5,pos_qz);
        String pos_qx = GotoQR(10.38, -7.542, 4.783, 0.0, 0.0, 1.0,0.0);
        api.judgeSendDiscoveredQR(3,pos_qx);

        String[] temp_p3_x = pos_x.split(" ");
        String[] temp_p3_y = pos_y.split(" ");
        String[] temp_p3_z = pos_z.split(" ");
        String[] temp_p3_qx = pos_qx.split(" ");
        String[] temp_p3_qy = pos_qy.split(" ");
        String[] temp_p3_qz = pos_qz.split(" ");

        double p3_x = Double.parseDouble(temp_p3_x[1]);
        double p3_y = Double.parseDouble(temp_p3_y[1]);
        double p3_z = Double.parseDouble(temp_p3_z[1]);
        double p3_qx = Double.parseDouble(temp_p3_qx[1]);
        double p3_qy = Double.parseDouble(temp_p3_qy[1]);
        double p3_qz = Double.parseDouble(temp_p3_qz[1]);
        double p3_qw = sqrt(1.00f - (p3_qx*p3_qx) - (p3_qy*p3_qy) - (p3_qz*p3_qz)); //t


        Log.d("QR","x = " + p3_x + " y = " + p3_y + " z = " + p3_z + " qx = " + p3_qx + " qy = " + p3_qy + " qz = " + p3_qz + " qw = " + p3_qw);


//        viaMove(10.65f,-7.54f,4.4f,0,0,0.707f,-0.707f);
//        viaMove(10.65f,-9.48f,4.4f,0,0,0.707f,-0.707f);

        viaMove(10.95f,-9.2f,5.35f,0,0,0,0);

        int id = GotoAR(p3_x,p3_y,p3_z,p3_qx,p3_qy,p3_qz,p3_qw);
        api.judgeSendDiscoveredAR(Integer.toString(id));

        api.laserControl(true);

        api.judgeSendFinishSimulation();

    }
    @Override
    protected void runPlan2(){
        // write here your plan 2
    }
    @Override
    protected void runPlan3(){
        // write here your plan 3
    }
    // You can add your method

    public String scanQRImage(){
        String contents = null;

        Log.d("QRDiscover","B4 getMat");
        Mat capture_mat = api.getMatNavCam();


        byte[] pixel = new byte[1280*960];
        Log.d("QRDiscover","B4 getpixel");
        capture_mat.get(0,0,pixel);
        Image barcode = new Image(1280,960,"Y800");
        barcode.setData(pixel);


        ImageScanner reader = new ImageScanner();
        reader.setConfig(Symbol.NONE, Config.ENABLE,0);
        reader.setConfig(Symbol.QRCODE,Config.ENABLE,1);
        Log.d("QRDiscover","B4 scan");
        int result = reader.scanImage(barcode);
        Log.d("QRDiscover","B4 for");

        if(result != 0){
            SymbolSet symbolSet = reader.getResults();
            for(Symbol symbol : symbolSet){
                contents = symbol.getData();
            }
        }

        return contents;
    }


    public String GotoQR(double pos_x, double pos_y, double pos_z,
                         double qua_x, double qua_y, double qua_z,
                         double qua_w)
    {
        String decoded = null;
        final Point p = new Point(pos_x,pos_y,pos_z);
        final Quaternion q = new Quaternion((float)qua_x,(float)qua_y,(float)qua_z,(float)qua_w);

        Log.d("QRDiscover","Before moveto");
        moveToWrapper(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
        int count = 0;
        while(decoded == null && count++ < 5)
        {
            if(count > 1)
                api.moveTo(p,q,false);
            decoded = scanQRImage();
        }
        Log.d("QRDiscover","count : " + count);
        Log.d("QRDiscover","content : " + decoded);
        return  decoded;
    }

    public void moveToWrapper(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, false);
        int loopCounter = 0;
        while(!result.hasSucceeded()|| loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
    }

    public void viaMove(double pos_x, double pos_y, double pos_z,
                        double qua_x, double qua_y, double qua_z,
                        double qua_w){
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        final Point point = new Point(pos_x, pos_y, pos_z);

        api.moveTo(point,quaternion,false);
    }


    public int GotoAR(double pos_x, double pos_y, double pos_z,
                      double qua_x, double qua_y, double qua_z,
                      double qua_w)
    {

        Mat ids = new Mat();
        int id = -1;
        boolean state = false;
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detectorParameters = DetectorParameters.create();

//        detectorParameters.set_adaptiveThreshWinSizeMax(3);
//        detectorParameters.set_adaptiveThreshWinSizeMin(13);

        detectorParameters.set_maxMarkerPerimeterRate(0.8);
        detectorParameters.set_minMarkerPerimeterRate(0.02);

        while(id < 0) {

            if(!state)
                moveToWrapper(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            else
                viaMove(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            state = !state;
            Log.d("getIDs", "B4 getMat");
            Mat source = api.getMatNavCam();
            Log.d("getIDs", "B4 Blur");
//            Mat blur = new Mat();
//            Imgproc.GaussianBlur(source,blur,new Size(5,5),0);
            List<Mat> corners = new ArrayList<>();

            try {
                Log.d("getIDs", "B4 Detect");
//                Aruco.detectMarkers(blur, dictionary, corners, ids,detectorParameters);
                Aruco.detectMarkers(source, dictionary, corners, ids,detectorParameters);
                Log.d("getIDs", "After Detect");
                id = (int) ids.get(0,0)[0];
//                getArPos(corners);
            } catch (Exception e) {
                Log.d("getIDs", "Error");
            }
        }
        Log.d("getIDs", "Id : " + id);
        return id;
    }
    public void getArPos(List<Mat> corner){

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        Aruco.estimatePoseSingleMarkers(corner,5,getCamMat(),getDistMat(),rvec,tvec);

        Log.d("Aruco","tvec row : " + tvec.rows() + "tvec col" + tvec.cols());
        Log.d("Aruco","rvec row : " + rvec.rows() + "rvec col" + rvec.cols());

        for(int i = 0;i<tvec.rows();i++)
        {
            for(int j = 0;j<tvec.cols();j++)
                for(int c = 0;c<tvec.channels();c++)
                    Log.d("Aruco","tvec rol" + i +" col " + j + " : " + tvec.get(i,j)[c]);
        }

        for(int i = 0;i<rvec.rows();i++)
        {
            for(int j = 0;j<rvec.cols();j++)
                for(int c = 0;c<tvec.channels();c++)
                    Log.d("Aruco","rvec rol" + i +" col " + j + " : " + rvec.get(i,j)[c]);
        }

        Mat rot_mat = new Mat();
        Calib3d.Rodrigues(rvec,rot_mat);

        for(int i = 0;i<rot_mat.rows();i++)
        {
            for(int j = 0;j<rot_mat.cols();j++)
                for(int c = 0;c<rot_mat.channels();c++)
                    Log.d("Aruco","rot_mat rol" + i +" col " + j + " : " + rot_mat.get(i,j)[c]);
        }

        Point AR_p = new Point(tvec.get(0,0)[0] * 0.01f,0,tvec.get(0,0)[1] * 0.01f);
        Quaternion AR_q = new Quaternion(0,0,0,0);
        api.relativeMoveTo(AR_p,AR_q,false);

    }

    public Mat getCamMat(){
        Mat cam_mat = new Mat(3,3, CvType.CV_64F);
        double[] cam_value = {344.173397, 0.000000, 630.793795, 0.000000, 344.277922, 487.033834, 0.000000,
                0.000000, 1.000000};
        cam_mat.put(0,0,cam_value);
        return  cam_mat;
    }

    public Mat getDistMat(){
        Mat dist_mat = new Mat(1,5, CvType.CV_64F);
        double[] dist_value = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
        dist_mat.put(0,0,dist_value);
        return dist_mat;
    }

}