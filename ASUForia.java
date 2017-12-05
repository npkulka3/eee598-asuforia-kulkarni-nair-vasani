package com.example.neera.once_more;
/*
ASUForia class is declared here. The interface PoseListener() is defined alongwith onPose().
onImageAvailable() is used to call nativePoseEstimation() in native-lib.cpp
TODO: in endEstimation() : 1. release Camera for other use
                           2. stop all Background thread and process
 */
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.Image;
import android.view.Surface;

import static com.example.neera.once_more.MainActivity.bmpIn;


//declare the class
public  class ASUForia {
    static {
        System.loadLibrary("native-lib");
    }
//created constructor
    ASUForia(PoseListener lister_arg, Bitmap ref_img, Surface cam_Surface)
    {
        new_Listener = lister_arg;
    }

    private PoseListener new_Listener;

    interface PoseListener {
        //setup onPose() function
         void onPose(Image cam_Frame,float[] rvec,float[] tvec );
    }

   private void endEstimation(){

   }
/*
Receive Camera Frame(as a byte array) from Camera Preview.
Converting byte array to int array for camera Image and reference Image and calling nativePoseEstimation() in C++
nativePoseEstimation() returns an int array of output image pixels
 */
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
        //converting reference image to int array
        bmpIn.getPixels(pixels_ref,offset,str_1,0,0,w_1,h_1);
        //bmpIn.recycle();
        //convertig Camera Image to int array
        bmp.getPixels(pixels,offset,stride,0,0,width,height);
        bmp.recycle();

        // calling nativePoseEstimation in C++
        pixels_new=nativePoseEstimation(pixels,pixels_ref,width,height,w_1,h_1);

        //converting output aray of pixels to Bitmap and return the bitmap
        out.setPixels(pixels_new,offset,stride,0,0,width,height);

        return out;

    }
    public static native int[] nativePoseEstimation(int[] pixels, int[] pixels_ref , int imgW, int imgH, int w_1, int h_1);
}
