package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.Binarizer;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.DetectorResult;
import com.google.zxing.common.GlobalHistogramBinarizer;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.detector.Detector;
import com.google.zxing.qrcode.decoder.Decoder;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Rect;


import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;


import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        api.judgeSendStart();


        String pos_x = GotoQR(11.44, -5.7f, 4.588f, 0.0f, 0.0f, 0.0f,1.0f);
        api.judgeSendDiscoveredQR(0,pos_x);
        String pos_z = GotoQR(11.00f, -5.50f, 4.40f, 0.707f, 0.0f, -0.707f,0.0f);
        api.judgeSendDiscoveredQR(2,pos_z);
        String pos_y = GotoQR(10.917f, -5.958f, 5.42f, 0.5f, -0.5f, 0.5f,0.5f);
        api.judgeSendDiscoveredQR(1,pos_y);
//
        viaMove(10.50f, -6.45f, 5.44f, 0.0f, 0.0f, 0.0f, 0.0f);
        viaMove(11.00f, -7.15f, 5.44f, 0.0f, 0.0f, 0.707f, -0.707f);

        String pos_qz = GotoQR(10.917, -7.658, 5.42, 0.5f, -0.5f, 0.5f,0.5f);
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
        float p3_qw = 1.00f - (p3_qx*p3_qx) - (p3_qy*p3_qy) - (p3_qz*p3_qz);

        Log.d("QR","x = " + p3_x + " y = " + p3_y + " z = " + p3_z + " qx = " + p3_qx + " qy = " + p3_qy + " qz = " + p3_qz + " qw = " + p3_qw);


        viaMove(10.7f,-7.54f,5.1f,0,0,0,0);
        viaMove(10.7f,-9.48f,5.1f,0,0,0,0);
        boolean done = false;
        int ar_try = 0;
        while (!done && ar_try++ <= 5) {
            try {
//                moveToWrapper(p3_x,p3_y,p3_z,p3_qx,p3_qy,p3_qz,p3_qw);
//                p3_qw *= -1;
                moveToWrapper(10.95,-9.59,5.40,0,0,0.707,-0.707);
                Mat Ar = api.getMatNavCam();
                Mat Ar_id = getIDs(Ar);
                int id = (int) Ar_id.get(0, 0)[0];
                Log.d("getID", "ID = " + Integer.toString(id));
                api.judgeSendDiscoveredAR(Integer.toString(id));
                done = true;
            }
            catch (Exception e) {
                Log.d("getID", "getID Error :");
            }
        }

//        api.laserControl(true);

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

    //----------------------------------------------Zxing method for Decode QR----------------------------------------------
//    int[] intArray = new int[1280*960];
//    LuminanceSource source = new RGBLuminanceSource(1280,960,intArray);
//    BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
//    QRCodeReader reader = new QRCodeReader();

//    public String scanQRImage(Bitmap bMap){
//        String contents = null;
//        //copy pixel data from the Bitmap into the 'intArray' array
//        Log.d("QRDiscover","Before getPixel");
//        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
//        Log.d("QRDiscover","Before Luminance");
//        source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
//        Log.d("QRDiscover","Before BinaryBitmap");
//        bitmap = new BinaryBitmap(new HybridBinarizer(source));
//
////        Log.d("QRDiscover","Before Hint");
////        HashMap<DecodeHintType, Object> hint = new HashMap<>();
////        hint.put(DecodeHintType.PURE_BARCODE,Boolean.TRUE);
////        Vector<BarcodeFormat> decodeFormats = new Vector<>();
////        decodeFormats.addElement(BarcodeFormat.QR_CODE);
////        hint.put(DecodeHintType.POSSIBLE_FORMATS, decodeFormats);
////        reader.setHints(hint);
//        try {
//            Log.d("QRDiscover","Before decode");
//            com.google.zxing.Result result = reader.decode(bitmap);
//            contents = result.getText();
//            reader.reset();
//            Log.d("QRDiscover","content : " + contents);
//        }
//        catch (Exception e) {
//            Log.d("QRDiscover","Can't Decode");
//        }
//        return contents;
//    }

    public String scanQRImage2(Bitmap bMap)
    {
        String contents = null;
        //copy pixel data from the Bitmap into the 'intArray' array
        int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];
        Log.d("QRDiscover","Before getPixel");
        bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
        Log.d("QRDiscover","Before Luminance");
        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
        Log.d("QRDiscover","Before BinaryBitmap");
        BinaryBitmap bitmap = new BinaryBitmap(new GlobalHistogramBinarizer(source));

//        Log.d("QRDiscover","Before Hint");
//        HashMap<DecodeHintType, Object> hint = new HashMap<>();
//        hint.put(DecodeHintType.PURE_BARCODE,Boolean.TRUE);
//        Vector<BarcodeFormat> decodeFormats = new Vector<>();
//        decodeFormats.addElement(BarcodeFormat.QR_CODE);
//        hint.put(DecodeHintType.POSSIBLE_FORMATS, decodeFormats);

        try
        {
//            Log.d("QRDiscover","Before getBlackMatrix");
            Log.d("QRDiscover","Before Detector");
            DetectorResult detectorResult = new Detector(bitmap.getBlackMatrix()).detect();
            Log.d("QRDiscover","Before Decode");
            DecoderResult decoderResult = new Decoder().decode(detectorResult.getBits());
            contents = decoderResult.getText();

            Log.d("QRDiscover","content : " + contents);
        }
        catch (Exception e){
            Log.d("QRDiscover","Can't Decode");
        }

        return contents;
    }

    public static Mat getRectMat(Mat source,int x,int y,int width,int height){
        Rect rectCrop = new Rect(x, y, width, height);
        return  source.submat(rectCrop);
    }

    BitMatrix gbitMatrix = new BitMatrix(640,576);

    void MatToBit(Mat source)
    {
//        BitMatrix bitMatrix = new BitMatrix(768,640);
        gbitMatrix.clear();
        byte[] byteArray  = new byte[640*576];

        Log.d("QRDiscover","Before loop");
        int count = 0;
        byte first_time = 0;
        while (count < 50000 && first_time < 2) {
            count = 0;
            source.get(0,0,byteArray);
            for (int height = 0, i = 0; height < 576; height++) {
                for (int width = 0; width < 640; width++, i++) {
                    if ((byteArray[i] >> 7) == 0) {
                        gbitMatrix.set(width, height);
                        count++;
                    }
                }
            }
            source = getRectMat(api.getMatNavCam(),320,192,640,576);
            Log.d("QRDiscover","Count = " + count);
            ++first_time;
        }
        Log.d("QRDiscover","After loop");
    }

    public String ScanQRFromMat(Mat source)
    {
        String contents = null;
        Log.d("QRDiscover","Before MatToBit");
//        BitMatrix source_bit = MatToBit(source);
        MatToBit(source);
        try {
            Log.d("QRDiscover","Before Detector");
            DetectorResult detectorResult = new Detector(gbitMatrix).detect();
            Log.d("QRDiscover","Before Decode");
            DecoderResult decoderResult = new Decoder().decode(detectorResult.getBits());
            contents = decoderResult.getText();
            Log.d("QRDiscover","content : " + contents);
        }
        catch (Exception e)
        {
            Log.d("QRDiscover","Can't Decode");
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
            decoded = ScanQRFromMat(getRectMat(api.getMatNavCam(),320,192,640,576));
        }
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
    public Mat getIDs(Mat source)
    {
        Mat ids = new Mat();
        try {
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            Aruco.detectMarkers(source, dictionary, corners, ids);
        }
        catch (Exception e) {
            Log.d("getIDs","Error");
        }
        return ids;
    }
}