package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.yanzhenjie.zbar.Config;
import com.yanzhenjie.zbar.Image;
import com.yanzhenjie.zbar.ImageScanner;
import com.yanzhenjie.zbar.Symbol;
import com.yanzhenjie.zbar.SymbolSet;


import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;


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
        String pos_y = GotoQR(10.917f, -5.958f, 5.42f, -0.707f, 0.0f, -0.707f,0.0f);
        api.judgeSendDiscoveredQR(1,pos_y);
//
        viaMove(10.50f, -6.45f, 5.44f, 0.0f, 0.0f, 0.0f, 0.0f);
        viaMove(11.00f, -7.15f, 5.44f, 0.0f, 0.0f, 0.707f, -0.707f);

        String pos_qz = GotoQR(10.917, -7.658, 5.42, 0.0f, -0.707f, 0.0f,0.707f);
        api.judgeSendDiscoveredQR(5,pos_qz);
        String pos_qy = GotoQR(11.47, -7.958, 5.083, 0.0, 0.0, 0.0,1.0);
        api.judgeSendDiscoveredQR(4,pos_qy);
        String pos_qx = GotoQR(10.38, -7.542, 4.783, 0.0, 0.0, 1.0,0.0);
        api.judgeSendDiscoveredQR(3,pos_qx);

        String[] temp_p3_x = pos_x.split("pos_x, ");
        String[] temp_p3_y = pos_y.split("pos_y, ");
        String[] temp_p3_z = pos_z.split("pos_z, ");
        String[] temp_p3_qx = pos_qx.split("qua_x, ");
        String[] temp_p3_qy = pos_qy.split("qua_y, ");
        String[] temp_p3_qz = pos_qz.split("qua_z, ");

        float p3_x = Float.parseFloat(temp_p3_x[1]);
        float p3_y = Float.parseFloat(temp_p3_y[1]);
        float p3_z = Float.parseFloat(temp_p3_z[1]);
        float p3_qx = Float.parseFloat(temp_p3_qx[1]);
        float p3_qy = Float.parseFloat(temp_p3_qy[1]);
        float p3_qz = Float.parseFloat(temp_p3_qz[1]);
        float p3_qw = (float) sqrt(1.00f - (p3_qx*p3_qx) - (p3_qy*p3_qy) - (p3_qz*p3_qz));


        Log.d("QR","x = " + p3_x + " y = " + p3_y + " z = " + p3_z + " qx = " + p3_qx + " qy = " + p3_qy + " qz = " + p3_qz + " qw = " + p3_qw);


        viaMove(10.7f,-7.54f,5.1f,0,0,0,0);
        viaMove(10.7f,-9.48f,5.1f,0,0,0,0);



//        int id = GotoAR(10.95,-9.59,5.40,0,0,0.707,-0.707);


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
//        Bitmap snapshot = api.getBitmapNavCam();
//        Bitmap bMap = Bitmap.createBitmap(snapshot,256,160,768,640);
//        int width = bMap.getWidth();
//        int height = bMap.getHeight();
//        int[] pixel = new int[width*height];
//        bMap.getPixels(pixel,0,width,0,0,width,height);
//        Image barcode = new Image(width,height,"RGB4");

        Mat capture_mat = api.getMatNavCam();
        byte[] pixel = new byte[1280*960];
        capture_mat.get(0,0,pixel);
        Image barcode = new Image(1280,960,"Y800");
        barcode.setData(pixel);


        ImageScanner reader = new ImageScanner();
        reader.setConfig(Symbol.NONE, Config.ENABLE,0);
        reader.setConfig(Symbol.QRCODE,Config.ENABLE,1);


//        Image barcode2 = barcode.convert("Y800");
//        int result = reader.scanImage(barcode2);
            int result = reader.scanImage(barcode);

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
            Log.d("QRDiscover","Before getMatNav");
//            decoded = ScanQRFromMat(getRectMat(api.getMatNavCam(),320,192,640,576));
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
        byte counter = 0;
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

        while(id < 0 && counter++ < 5) {

            moveToWrapper(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            qua_w *= -1;
            Mat source = api.getMatNavCam();
            List<Mat> corners = new ArrayList<>();

            try {
                Aruco.detectMarkers(source, dictionary, corners, ids);
                id = (int) ids.get(0,0)[0];
            } catch (Exception e) {
                Log.d("getIDs", "Error");
            }
        }
        Log.d("getIDs", "Id : " + id);
        return id;
    }
}