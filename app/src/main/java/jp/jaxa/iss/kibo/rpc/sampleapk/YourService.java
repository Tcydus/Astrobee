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

//        moveToWrapper(11.49, -5.7f, 4.588f, 0.0, 0.0, 0.0,1.0f);
//        Mat temp_mat = api.getMatNavCam();
//
////            Log.d("Mat info", "width = " + temp_mat.width() + " height = " + temp_mat. + " chanel = " + temp_mat.channels());
//        Log.d("Image info", "row = 0 col = 0 chanel = 0 = " + temp_mat.get(0, 0)[0]);
//        Log.d("Image info", "row = 10 col = 10 chanel = 0 = " + temp_mat.get(10, 10)[0]);
//        int width = 0,height = 0;
//            for(height = 0;height < 960;height++)
//            {
//                for(width=0;width < 1280;width++)
//                {
//                    try {
//                        double aa = temp_mat.get(height,width)[0];
////                        if (temp_mat.get(height, width)[0] > 127)
////                            Log.d("Image info", "row = " + height + " col = " + width + "Data = " + temp_mat.get(height, width)[0]);
//                    }
//                    catch (Exception e){
//                        Log.d("Mat info","Error at row : " + height + " col : " + width);
//                    }
//                }
//            }




        String pos_x = GotoQR(11.49, -5.7f, 4.588f, 0.0, 0.0, 0.0,1.0f);
        api.judgeSendDiscoveredQR(0,pos_x);
//        String pos_z = GotoQR(11.00, -5.50, 4.40, 0.0, 0.707, 0.0,0.707);
//        api.judgeSendDiscoveredQR(2,pos_z);
//        String pos_y = GotoQR(10.917, -5.958, 5.42, 0.0, -0.707, 0.0,0.707);
//        api.judgeSendDiscoveredQR(1,pos_y);
//
//        moveToWrapper(10.50, -6.45, 5.44, 0.0, 0.0, 0.0, 0.0);
//        moveToWrapper(11.00, -7.15, 5.44, 0.0, 0.0, 0.707, -0.707);
//
//        String pos_qz = GotoQR(10.917, -7.658, 5.42, 0.0, -0.707, 0.0,0.707);
//        api.judgeSendDiscoveredQR(5,pos_qz);
//        String pos_qy = GotoQR(11.47, -7.958, 5.083, 0.0, 0.0, 0.0,1.0);
//        api.judgeSendDiscoveredQR(4,pos_qy);
//        String pos_qx = GotoQR(10.38, -7.542, 4.783, 0.0, 0.0, 1.0,0.0);
//        api.judgeSendDiscoveredQR(3,pos_qx);

//        String[] temp_p3_x = pos_x.split("pos_x, ");
//        String[] temp_p3_y = pos_y.split("pos_y, ");
//        String[] temp_p3_z = pos_z.split("pos_z, ");
//        String[] temp_p3_qx = pos_qx.split("qua_x, ");
//        String[] temp_p3_qy = pos_qy.split("qua_y, ");
//        String[] temp_p3_qz = pos_qz.split("qua_z, ");
//
//        float p3_x = Float.parseFloat(temp_p3_x[1]);
//        float p3_y = Float.parseFloat(temp_p3_y[1]);
//        float p3_z = Float.parseFloat(temp_p3_z[1]);
//        float p3_qx = Float.parseFloat(temp_p3_qx[1]);
//        float p3_qy = Float.parseFloat(temp_p3_qy[1]);
//        float p3_qz = Float.parseFloat(temp_p3_qz[1]);
//        float p3_qw = 1.00f - (p3_qx*p3_qx) - (p3_qy*p3_qy) - (p3_qz*p3_qz);
//
//        Log.d("QR","x = " + p3_x + " y = " + p3_y + " z = " + p3_z + " qx = " + p3_qx + " qy = " + p3_qy + " qz = " + p3_qz + " qw = " + p3_qw);
//
//
//        moveToWrapper(10.7f,-7.54f,5.1f,0,0,0,0);
//        moveToWrapper(10.7f,-9.48f,5.1f,0,0,0,0);
//        boolean done = false;
//        int ar_try = 0;
//        while (!done && ar_try++ <= 5) {
//            try {
//                moveToWrapper(p3_x,p3_y,p3_z,p3_qx,p3_qy,p3_qz,p3_qw);
//                p3_qw *= -1;
//                Mat Ar = api.getMatNavCam();
//                Mat Ar_id = getIDs(Ar);
//                int id = (int) Ar_id.get(0, 0)[0];
//                Log.d("getID", "ID = " + Integer.toString(id));
//                api.judgeSendDiscoveredAR(Integer.toString(id));
//                done = true;
//            }
//            catch (Exception e) {
//                Log.d("getID", "getID Error :");
//            }
//        }

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

    public static BitMatrix MatToBit(Mat source)
    {
        BitMatrix bitMatrix = new BitMatrix(768,640);
//        bitMatrix.clear();
//        Log.d("QRDiscover","Before set region");
//        bitMatrix.setRegion(0,0,768,640);
        byte[] byteArray  = new byte[768*640];
        Log.d("QRDiscover","Before get");
        source.get(0,0,byteArray);
        Log.d("QRDiscover","Before loop");
        int count = 0;
        for(int height = 0,i=0;height < 640; height++)
        {
            for(int width = 0;width < 768; width++,i++)
            {
                if(byteArray[i] < 0) {
                    bitMatrix.unset(width, height);
                    count++;
                }
                else {
                    bitMatrix.set(width,height);
                }
            }
        }
        Log.d("MatToBit","count = " + count);
        return bitMatrix;
    }


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

    public static String ScanQRFromMat(Mat source)
    {
        String contents = null;
        Log.d("QRDiscover","Before MatToBit");
        BitMatrix source_bit = MatToBit(source);
        try {
            Log.d("QRDiscover","Before Detector");
            DetectorResult detectorResult = new Detector(source_bit).detect();
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

//    public String scanQRImage3(Mat matQR){
//        Bitmap bMap = null;
//        Mat dstMat = null;
//
//        Log.d("QRDiscover","Before Detect");
//        QRCodeDetector qrCodeDetector = new QRCodeDetector();
//        qrCodeDetector.detect(matQR,dstMat);
//
//        Log.d("QRDiscover","Before matToBitmap");
//        Utils.matToBitmap(dstMat,bMap);
//
//        String contents = null;
//        //copy pixel data from the Bitmap into the 'intArray' array
//        int[] intArray2 = new int[bMap.getHeight()*bMap.getWidth()];
//        Log.d("QRDiscover","Before getPixel");
//        bMap.getPixels(intArray2, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
//        Log.d("QRDiscover","Before Luminance");
//        LuminanceSource source2 = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray2);
//        Log.d("QRDiscover","Before BinaryBitmap");
//        BinaryBitmap bitmap2 = new BinaryBitmap(new HybridBinarizer(source2));
//
//        Log.d("QRDiscover","Before Hint");
//        HashMap<DecodeHintType, Object> hint = new HashMap<>();
//        Vector<BarcodeFormat> decodeFormats = new Vector<>();
//        decodeFormats.addElement(BarcodeFormat.QR_CODE);
//        hint.put(DecodeHintType.PURE_BARCODE,Boolean.TRUE);
//        hint.put(DecodeHintType.POSSIBLE_FORMATS, decodeFormats);
//
//        try {
//            Log.d("QRDiscover","Before decode");
//            com.google.zxing.Result result = reader.decode(bitmap2,hint);
//            contents = result.getText();
//            reader.reset();
//
//            Log.d("QRDiscover","content : " + contents);
//        }
//        catch (Exception e) {
//            Log.d("QRDiscover","Can't Decode");
//        }
//        return contents;
//    }

    //----------------------------------------------End--------------------------------------------------------------

    //----------------------------------------------Bitmap To Monochrome----------------------------------------------
//    public BitMatrix getMonochrome(Bitmap bmap)
//    {
//        BitMatrix bitMatrix = new BitMatrix(bmap.getWidth(),bmap.getHeight());
//        Bitmap bmpMonochrome = Bitmap.createBitmap(bmap.getWidth(), bmap.getHeight(), Bitmap.Config.ARGB_8888);
//        Canvas canvas = new Canvas(bmpMonochrome);
//        ColorMatrix ma = new ColorMatrix();
//        ma.setSaturation(0);
//        Paint paint = new Paint();
//        paint.setColorFilter(new ColorMatrixColorFilter(ma));
//        canvas.drawBitmap(bmap, 0, 0, paint);
//
//       int[] pixel = new int[bmpMonochrome.getWidth()*bmpMonochrome.getHeight()];
//       bmpMonochrome.getPixels(pixel, 0, bmpMonochrome.getWidth(), 0, 0, bmpMonochrome.getWidth(), bmpMonochrome.getHeight());
//       int h,w,index=0;
//       for(h = 0;h < bmpMonochrome.getHeight();h++)
//       {
//           for(w = 0;w < bmpMonochrome.getWidth();w++)
//           {
//               int temp_pixel = pixel[index] & 0xFF;
//               if(temp_pixel < 128)
//                   bitMatrix.unset(w,h);
//               else
//                   bitMatrix.set(w,h);
//               index++;
//           }
//       }
//       return  bitMatrix;
//    }
    //----------------------------------------------End Bitmap To Monochrome----------------------------------------------

    public String GotoQR(double pos_x, double pos_y, double pos_z,
                         double qua_x, double qua_y, double qua_z,
                         double qua_w)
    {
        String decoded = null;

        int count = 0;
        while(decoded == null && count++ < 5)
        {
            Log.d("QRDiscover","Before moveto");
            moveToWrapper(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
//            api.moveTo(point,quaternion,false);

//            Bitmap snapshot = api.getBitmapNavCam();
//            Log.d("QRDiscover","After getBitmap");
//            Bitmap snapshot2 = Bitmap.createBitmap(snapshot,256,160,768,640);
//            Bitmap snapshot3 = Bitmap.createScaledBitmap(snapshot2,400,400,false);
//            Log.d("QRDiscover","After crop");

            Log.d("QRDiscover","Before getMatNav");
            Mat snapshot = api.getMatNavCam();
            Log.d("QRDiscover","Before Rect");
            Rect rectCrop = new Rect(256, 160, 768, 640);
            Log.d("QRDiscover","Before submat");
            Mat snapshot2 = snapshot.submat(rectCrop);
            Log.d("QRDiscover","Before scan");
            decoded = ScanQRFromMat(snapshot2);
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