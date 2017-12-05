package com.example.neera.once_more;

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


        BitmapFactory.Options opts = new BitmapFactory.Options();
        opts.inPreferredConfig = Bitmap.Config.ARGB_8888; // Each pixel is 4 bytes: Alpha, Red, Green, Blue
        bmpIn = BitmapFactory.decodeResource(getResources(), R.drawable.vuforia, opts);
        //    int h_t = bmpIn.getWidth();
        //string = getPackageCodePath();


        final ASUForia.PoseListener new_PoseListen = new ASUForia.PoseListener() {
            @Override
            public void onPose(Image cam_Frame, float[] rvec, float[] tvec) {

            }
        };
        Surface cam  = null;
        asuForia = new ASUForia(new_PoseListen,bmpIn,cam);

    }


    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public static   Bitmap getImage_from_C(byte[] imageBytes)
    {
        byte[] check = imageBytes;
        Bitmap out_1 = asuForia.onImageAvailable(check);
        return out_1;


    };

    @Override
    protected void onResume()
    {

        super.onResume();
        //    BitmapFactory.Options opts = new BitmapFactory.Options();
        //    opts.inPreferredConfig = Bitmap.Config.ARGB_8888; // Each pixel is 4 bytes: Alpha, Red, Green, Blue
        //    bmpIn = BitmapFactory.decodeResource(getResources(), R.drawable.vuforia, opts);
        //asuForia.ImageAvailable();
    }

    @Override
    protected void onPause()
    {
        super.onPause();
        int x=3;
    }

    //public static   native int[] getImage(int[] pixels, int[] pixels_ref , int imgW, int imgH, int w_1, int h_1);
}
