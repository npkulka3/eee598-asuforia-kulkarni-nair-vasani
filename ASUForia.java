package com.example.neera.once_more;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.Image;
import android.view.Surface;

import static com.example.neera.once_more.MainActivity.bmpIn;

/**
 * Created by neera on 12/4/2017.
 */

public  class ASUForia {
    static {
        System.loadLibrary("native-lib");
    }
//created constructor
    ASUForia(PoseListener lister_arg, Bitmap ref_img, Surface cam_Surface)
    {
        myListener = lister_arg;
    }

    private PoseListener myListener;

    interface PoseListener {
         void onPose(Image cam_Frame,float[] rvec,float[] tvec );
    }

   private void endEstimation(){

   }

    public Bitmap onImageAvailable(byte[] imageBytes){
        Bitmap bmp = BitmapFactory.decodeByteArray(imageBytes,0,imageBytes.length);
        Bitmap out=bmp.copy(bmp.getConfig(),true);
        int width=bmp.getWidth();
        int height=bmp.getHeight();
        //Log.d("Is it Crashing", "NOOO ");
        int[] pixels = new int[width*height];
        //Log.d("Is it Crashing Now","MAYBE");
        int [] pixels_new = new int[width*height];
        int offset=0;
        int stride = width;int w_1 = bmpIn.getWidth();
        int h_1 = bmpIn.getHeight();
        int str_1 = w_1;
        int [] pixels_ref = new int [w_1*h_1];
        bmpIn.getPixels(pixels_ref,offset,str_1,0,0,w_1,h_1);
        //bmpIn.recycle();
        bmp.getPixels(pixels,offset,stride,0,0,width,height);
        bmp.recycle();

        // byte [] image_out = new byte[imageBytes.length];
        pixels_new=nativePoseEstimation(pixels,pixels_ref,width,height,w_1,h_1);
        //Bitmap bmp1 =BitmapFactory.decodeByteArray(image_out,0,image_out.length);
        out.setPixels(pixels_new,offset,stride,0,0,width,height);
        //myListener.onPose();
        return out;

    }
    public static native int[] nativePoseEstimation(int[] pixels, int[] pixels_ref , int imgW, int imgH, int w_1, int h_1);
}
