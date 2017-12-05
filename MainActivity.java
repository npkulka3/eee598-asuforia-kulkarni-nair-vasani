package com.example.neera.once_more;

/*
The ASUForia object is instantiated here. The CameraFrame is sent from camera Preview to onimageAvailable()
in the asuForia class.
The function returns a bitmap image which contains a line drawn on the input camera frame .
PoseListenner is setup with onPose().
12/4/17: Unable to utilize the onPose() function.
 */
import android.content.Context;
import android.content.Intent;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.media.Image;
import android.provider.Settings;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.Surface;
import android.widget.TextView;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
//import android.support.design.widget.FloatingActionButton;
//import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import java.util.Vector;
import org.opencv.imgcodecs.Imgcodecs;


public class MainActivity extends AppCompatActivity {

    //initialize the ASUForia object
    //It is static
    static ASUForia asuForia;
    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }
    public static Bitmap bmpIn;
    //public static Context GlobalContext = null;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        //  MainActivity.GlobalContext=this;
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main_check);
        if (null == savedInstanceState) {
            getSupportFragmentManager().beginTransaction()
                    .replace(R.id.main_open, Camera2BasicFragment.newInstance())
                    .commit();

        }

        //get reference image from resoource folder and store it globally as a bitmap
        BitmapFactory.Options opts = new BitmapFactory.Options();
        opts.inPreferredConfig = Bitmap.Config.ARGB_8888; // Each pixel is 4 bytes: Alpha, Red, Green, Blue
        bmpIn = BitmapFactory.decodeResource(getResources(), R.drawable.vuforia, opts);
        //    int h_t = bmpIn.getWidth();
        //string = getPackageCodePath();

        //Setup Pose Listener
        final ASUForia.PoseListener new_PoseListen = new ASUForia.PoseListener() {
            @Override
            //TODO:: Utilize the onPose function
            public void onPose(Image cam_Frame, float[] rvec, float[] tvec) {

            }
        };
        //Create new ASUForia object
        //Unable to pass Surface , passing it as null.
        Surface cam  = null;
        asuForia = new ASUForia(new_PoseListen,bmpIn,cam);

    }


//Receiving the input camera frame from Camera Preview
//Call onImageAvailable() from ASUForia
    public static   Bitmap getImage_from_C(byte[] imageBytes)
    {
        byte[] check = imageBytes;
        Bitmap out_1 = asuForia.onImageAvailable(check);
        return out_1;
    };

//TODO: Call startEstimation()
//Not sure how to implement
    @Override
    protected void onResume()
    {

        super.onResume();
        //    BitmapFactory.Options opts = new BitmapFactory.Options();
        //    opts.inPreferredConfig = Bitmap.Config.ARGB_8888; // Each pixel is 4 bytes: Alpha, Red, Green, Blue
        //    bmpIn = BitmapFactory.decodeResource(getResources(), R.drawable.vuforia, opts);
        //asuForia.ImageAvailable();
    }
// Setup onPause, this releases the camera for usage
    @Override
    protected void onPause()
    {
        //TODO: Release the camera and stop background thread
        super.onPause();
        int x=3;
    }

    //public static   native int[] getImage(int[] pixels, int[] pixels_ref , int imgW, int imgH, int w_1, int h_1);
}
