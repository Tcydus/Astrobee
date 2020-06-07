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
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee4
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        api.judgeSendStart();

        String pos_x = GotoQR(11.35, -5.66f, 4.53f, 0.0f, 0.0f, 1.0f,0.0f,1);
        api.judgeSendDiscoveredQR(0,pos_x);

        String pos_z = GotoQR(10.92f, -5.54f, 4.53f, -0.707f, 0.0f, -0.707f,0.0f,1);
        api.judgeSendDiscoveredQR(2,pos_z);

        String pos_y = GotoQR(10.98f, -5.91f, 5.42f, 0.0f, 0.0f, 0.0f,0.0f,0); //New (above side near airlock)
        api.judgeSendDiscoveredQR(1,pos_y);

        moveToWrapper(10.50f, -6.45f, 5.44f, 0.0f, 0.0f, 0.0f, 0.0f);

//        String pos_qy = GotoQR(11.45f, -7.96f, 5.08f, 0.0f, 0.0f, 0.0f,1.0f,0);
        String pos_qy = GotoQR(11.45f,  -8.05f, 5.08f, 0.0f, 0.0f, 1.0f,0.0f,1);
        api.judgeSendDiscoveredQR(4,pos_qy);
        String pos_qz = GotoQR(11.2f, -7.82f, 5.43f, 0.0f, 0.0f, 0.0f,0.0f,0);
        api.judgeSendDiscoveredQR(5,pos_qz);
//        String pos_qx = GotoQR(10.45f, -7.66f, 4.88f, 0.0f, 0.0f, 0.0f,0.0f,0);
        String pos_qx = GotoQR(10.45f, -7.5f, 4.78f, 0.0f, 0.0f, 1.0f,0.0f,0);
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


        p3_x = constrain(p3_x,10.45,11.45);
        p3_y = constrain(p3_y,-9.55,-3.20);
        p3_z = constrain(p3_z,4.40,5.40);


        Log.d("QR","x = " + p3_x + " y = " + p3_y + " z = " + p3_z + " qx = " + p3_qx + " qy = " + p3_qy + " qz = " + p3_qz + " qw = " + p3_qw);


//        viaMove(10.95f,-9.2f,5.35f,0,0,0,0);
        moveToWrapper(10.95f,-9.55f,5.35f,0,0,0.707,-0.707);
        Mat spare_ar = api.getMatNavCam();
        int id = GotoAR(p3_x,p3_y,p3_z,p3_qx,p3_qy,p3_qz,p3_qw,spare_ar);
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

    public String GotoQR(double pos_x, double pos_y, double pos_z,
                         double qua_x, double qua_y, double qua_z,
                         double qua_w,int cam)
    {
        String decoded = null;
        final Point p = new Point(pos_x,pos_y,pos_z);
        final Quaternion q = new Quaternion((float)qua_x,(float)qua_y,(float)qua_z,(float)qua_w);

        moveToWrapper(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
        int count = 0;

        while(decoded == null)
        {
            count++;
            if(count > 1)
                api.moveTo(p,q,false);
            decoded = scanQRImage(cam);

        }
        Log.d("QRDiscover","count : " + count + " content : " + decoded);
        return  decoded;
    }

    public String scanQRImage(int cam){
        String contents = null;
        Mat capture_mat = new Mat();
        if(cam == 0)
            capture_mat = api.getMatNavCam();
        else if(cam == 1)
            capture_mat = api.getMatDockCam();


        byte[] pixel = new byte[1280*960];
        capture_mat.get(0,0,pixel);
        Image barcode = new Image(1280,960,"Y800");
        barcode.setData(pixel);


        ImageScanner reader = new ImageScanner();
        reader.setConfig(Symbol.NONE, Config.ENABLE,0);
        reader.setConfig(Symbol.QRCODE,Config.ENABLE,1);
        int result = reader.scanImage(barcode);

        if(result != 0){
            SymbolSet symbolSet = reader.getResults();
            for(Symbol symbol : symbolSet){
                contents = symbol.getData();
            }
        }

        return contents;
    }

    public double constrain(double value,double min,double max){
        if(value > max)
            value = max;
        else if(value < min)
            value = min;
        return value;
    }

    public void moveToWrapper(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){
        final int LOOP_MAX = 2;
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
                      double qua_w,Mat spare_ar)
    {

        Mat ids = new Mat();
        Mat thresh = new Mat();
        int id = -1,count = 0,flag = 0;
        boolean state = false;
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters detectorParameters = DetectorParameters.create();

//        detectorParameters.set_adaptiveThreshWinSizeMax(13);
//        detectorParameters.set_adaptiveThreshWinSizeMin(3);

        detectorParameters.set_maxMarkerPerimeterRate(1.0);
        detectorParameters.set_minMarkerPerimeterRate(0.02);
        detectorParameters.set_adaptiveThreshConstant(25);
        detectorParameters.set_minDistanceToBorder(1);
        detectorParameters.set_polygonalApproxAccuracyRate(0.15);
        detectorParameters.set_cornerRefinementMethod(Aruco.CORNER_REFINE_NONE);

        List<Mat> corners = new ArrayList<>();
        List<Mat> reject = new ArrayList<>();

        Log.d("getIDs", "B4 Detect center_spare");
        Aruco.detectMarkers(spare_ar, dictionary, corners, ids, detectorParameters);
        Log.d("getIDs", "After center_spare");
        flag = 2;
        try {
            id = (int) ids.get(0,0)[0];
        }
        catch (Exception e) {
            // Nothing ///
        }

        while(id < 0) {

            if(count == 2)
                moveToWrapper(10.95f,-9.55f,5.1f,0,0,0.707,-0.707);
            else if(!state)
                moveToWrapper(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            else
                viaMove(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);

            state = !state;
            Mat source = api.getMatNavCam();


            try {
                if(count == 2)
                {
                    Log.d("getIDs", "B4 Detect black_spare");
                    Aruco.detectMarkers(source, dictionary, corners, ids,detectorParameters);
                    Log.d("getIDs", "After Detect black_spare");
                    flag = 4;
                    if(ids.get(0,0) == null){
                        Imgproc.threshold(source, thresh, 25, 255, Imgproc.THRESH_BINARY);
                        Log.d("getIDs", "B4 Detect black_spare_thresh");
                        Aruco.detectMarkers(thresh, dictionary, corners, ids,detectorParameters);
                        Log.d("getIDs", "After black_spare_thresh");
                        flag = 5;
                    }
                }
                else {
                    Log.d("getIDs", "B4 Detect P3");
                    Aruco.detectMarkers(source, dictionary, corners, ids, detectorParameters);
                    Log.d("getIDs", "After Detect P3");
                    flag = 1;


                    if (ids.get(0, 0) == null) {
                        Imgproc.threshold(source, thresh, 25, 255, Imgproc.THRESH_BINARY);
                        Log.d("getIDs", "B4 Detect P3_thresh");
//                        Aruco.detectMarkers(thresh, dictionary, corners, ids, detectorParameters);
                        Aruco.detectMarkers(thresh, dictionary, corners, ids, detectorParameters,reject,getCamMat(),getDistMat());
                        Log.d("getIDs", "After P3_thresh");
                        flag = 3;
                    }
                }

                count++;
                id = (int) ids.get(0,0)[0];
            } catch (Exception e) {
                Log.d("getIDs", "Error");
            }
        }
        Log.d("ARFinish", "Id : " + id + " Flag = " + flag + " count = " + count);
        return id;
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