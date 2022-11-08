package edu.cmu.cs.dronebrain;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationManager;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.StrictMode;
import android.util.Log;
import android.view.WindowManager;
import android.webkit.URLUtil;

import com.google.protobuf.Any;
import com.google.protobuf.ByteString;
import com.google.protobuf.InvalidProtocolBufferException;
import com.parrot.drone.groundsdk.ManagedGroundSdk;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_java;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.function.Consumer;

import dalvik.system.DexClassLoader;
import edu.cmu.cs.dronebrain.impl.ElijahCloudlet;
import edu.cmu.cs.dronebrain.impl.parrotanafi.ParrotAnafi;
import edu.cmu.cs.dronebrain.interfaces.CloudletItf;
import edu.cmu.cs.dronebrain.interfaces.DroneItf;
import edu.cmu.cs.dronebrain.interfaces.FlightScript;
import edu.cmu.cs.dronebrain.interfaces.Platform;
import edu.cmu.cs.gabriel.client.comm.ServerComm;
import edu.cmu.cs.gabriel.client.results.ErrorType;
import edu.cmu.cs.gabriel.protocol.Protos.InputFrame;
import edu.cmu.cs.gabriel.protocol.Protos.ResultWrapper;
import edu.cmu.cs.gabriel.protocol.Protos.PayloadType;
import edu.cmu.cs.steeleagle.Protos;
import edu.cmu.cs.steeleagle.Protos.Extras;


public class MainActivity extends Activity implements Consumer<ResultWrapper> {
    private ManagedGroundSdk sdk = null;
    /** FOR TESTING ONLY -- Eventually, this will be passed to us by the main activity **/
    private Platform platform = Platform.ANAFI;
    /** Connectivity Manager to retrieve network infos **/
    private ConnectivityManager cm = null;
    /** LTE network object **/
    private Network LTEnetwork = null;
    /** Currently executing FlightScript **/
    private FlightScript MS = null;
    /** Runnable holding the executing FlightScript **/
    private Thread scriptThread = null;
    /** Current drone object **/
    private DroneItf drone = null;
    /** Current cloudlet object **/
    private CloudletItf cloudlet = null;
    /** For Gabriel client connection **/
    private ServerComm serverComm = null;
    /** Script URL send from commander **/
    private String scriptUrl = null;
    private Handler loopHandler = null;
    private int heartbeatsSent = 0;
    /** Log tag **/
    String TAG = "DroneBrain";
    String SOURCE = "command";
    UUID uuid = UUID.randomUUID();


    // Based on
    // https://github.com/protocolbuffers/protobuf/blob/master/src/google/protobuf/compiler/java/java_message.cc#L1387
    private static Any pack(Extras engineFields) {
        return Any.newBuilder()
                .setTypeUrl("type.googleapis.com/cnc.Extras")
                .setValue(engineFields.toByteString())
                .build();
    }

    public String getWifiSSID() {
        WifiManager wm = (WifiManager) this.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        if (wm != null) {
            return wm.getConnectionInfo().getSSID();
        } else {
            return "";
        }
    }

    @Override
    public void accept(ResultWrapper resultWrapper) {
        // Forward OpenScout results to the cloudlet so that it can process them
        if (cloudlet != null && (resultWrapper.getResultProducerName().getValue().equals("openscout-object")
                || resultWrapper.getResultProducerName().getValue().equals("openscout-face"))) {
            cloudlet.processResults(resultWrapper);
            return;
        }

        if (resultWrapper.getResultsCount() != 1) {
            Log.e(TAG, "Got " + resultWrapper.getResultsCount() + " results in output from COMMAND.");
            return;
        }

        ResultWrapper.Result result = resultWrapper.getResults(0);
        try {
            Extras extras = Extras.parseFrom(resultWrapper.getExtras().getValue());
            ByteString r = result.getPayload();
            if (extras.hasCmd()) {
                Protos.Command cmd = extras.getCmd();
                if (cmd.getHalt()) {
                    Log.i(TAG, "Killswitch signaled from commander.");
                    if (MS != null) {
                        MS.kill();
                        //scriptThread.interrupt();
                        Log.i(TAG, "Interrupting script thread.");
                    }
                    finish();
                }

                if (URLUtil.isValidUrl(cmd.getScriptUrl())) {
                    Log.i(TAG, "Flight script sent by commander.");
                    scriptUrl = cmd.getScriptUrl();
                    Log.i(TAG, scriptUrl);
                    if (retrieveFlightScript())
                        executeFlightScript();
                }
            }
        } catch (InvalidProtocolBufferException e) {
            Log.e(TAG, "Protobuf Error", e);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (result.getPayloadType() != PayloadType.TEXT) {
            Log.e(TAG, "Got result of type " + result.getPayloadType().name());
            return;
        }

    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        sdk = ManagedGroundSdk.obtainSession(this);

        CountDownLatch lteLatch = new CountDownLatch(1);
        /** Create LTE network binding **/
        cm = (ConnectivityManager) getApplicationContext().getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkRequest.Builder req = new NetworkRequest.Builder();
        req.addTransportType(NetworkCapabilities.TRANSPORT_CELLULAR);
        cm.requestNetwork(req.build(), new ConnectivityManager.NetworkCallback() {
            @Override
            public void onAvailable(Network network) {
                /** Assign LTE network object **/
                LTEnetwork = network;
                lteLatch.countDown();
                Log.d(TAG, "LTE network successfully acquired");
            }
        }
        );
        try {
            lteLatch.await();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        Log.d(TAG, "Got LTE network object");

        Consumer<ErrorType> onDisconnect = errorType -> {
            Log.e(TAG, "Disconnect Error: " + errorType.name());
            finish();
        };

        serverComm = ServerComm.createServerComm(
                this, BuildConfig.GABRIEL_HOST, BuildConfig.PORT, getApplication(), onDisconnect, LTEnetwork);

        loopHandler = new Handler();
        loopHandler.postDelayed(gabrielLoop, 1);
        Log.d(TAG, "Finished OnCreate");
    }

    // Used to get watch GPS location before drone GPS is active.
    private double[] getWatchGPS() {
        LocationManager lm = (LocationManager)getSystemService(LOCATION_SERVICE);
        double[] gps = new double[2];
        gps[0] = 0.0;
        gps[1] = 0.0;
        try {
            Location loc =  lm.getLastKnownLocation(LocationManager.GPS_PROVIDER);
            if (loc != null) {
                gps[0] = loc.getLatitude();
                gps[1] = loc.getLongitude();
            } else {
                loc = lm.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
                if (loc != null) {
                    gps[0] = loc.getLatitude();
                    gps[1] = loc.getLongitude();
                }
            }
        } catch(SecurityException e)  {
            Log.e(TAG, e.getMessage());
        }
        return gps;
    }

    public Network getNetworkObjectForCurrentWifiConnection() {
        List<Network> networks = Arrays.asList(cm.getAllNetworks());
        for (Network network : networks) {
            NetworkCapabilities capabilities = cm.getNetworkCapabilities(network);
            if (capabilities.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) {
                return network;
            }
        }
        return null;
    }

    private void bindProcessToWifi() {
        Network network = getNetworkObjectForCurrentWifiConnection();
        if (network != null) {
            Boolean success = cm.bindProcessToNetwork(network);
            Log.d("WifiConnect", success.toString());
        }
    }

    private Runnable gabrielLoop = new Runnable() {

        @Override
        public void run() {
            heartbeatsSent++;
            serverComm.sendSupplier(() -> {
                String p = "heartbeat";

                Extras extras;
                Extras.Builder extrasBuilder = Extras.newBuilder();
                Protos.Location.Builder lb = Protos.Location.newBuilder();
                try {
                    if (drone != null) {
                        lb.setLatitude(drone.getLat());
                        lb.setLongitude(drone.getLon());
                        lb.setAltitude(drone.getAlt());
                        extrasBuilder.setLocation(lb);
                    } else {
                        double[] gps = getWatchGPS();
                        lb.setLatitude(gps[0]);
                        lb.setLongitude(gps[1]);
                        extrasBuilder.setLocation(lb);
                    }

                    extrasBuilder.setDroneId(uuid.toString());

                    if(heartbeatsSent < 2) {
                        extrasBuilder.setRegistering(true);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
                extras = extrasBuilder.build();

                return InputFrame.newBuilder()
                        .setPayloadType(PayloadType.TEXT)
                        .addPayloads(ByteString.copyFromUtf8(p))
                        .setExtras(pack(extras))
                        .build();
            }, SOURCE, false);
            loopHandler.postDelayed(this, 1000);
        }
    };

    protected boolean retrieveFlightScript() {
        boolean success = false;
        /** Download flight plan **/
        File f = new File(getDir("dex", MODE_PRIVATE), "classes.dex");
        try {
            Log.d(TAG, "Starting flight plan download");
            download(scriptUrl, f);
            /** Read data from flight plan **/
            DexClassLoader classLoader = new DexClassLoader(
                    f.getAbsolutePath(), getDir("outdex", MODE_PRIVATE).getAbsolutePath(),
                    null, getClassLoader());
            Class clazz = classLoader.loadClass("edu.cmu.cs.dronebrain.MS");
            Log.i(TAG, clazz.toString());
            MS = (FlightScript)clazz.newInstance();
            Log.d(TAG, "Getting drone and cloudlet...");
            drone = getDrone(platform); // Get the corresponding drone
            cloudlet = getCloudlet(); // Get the corresponding cloudlet
            success = true;
        } catch (Exception e) {
            Log.e(TAG, "Download failed! " + e.toString());
            e.printStackTrace();
        }
        return success;
    }

    protected void executeFlightScript() {
        try {
            /** Execute flight plan **/
            Log.d(TAG, "Executing flight plan!");
            /** Bind all future sockets to Wifi so that streaming works.
             * Our LTE connections will persist. **/
            bindProcessToWifi();
            MS.init(drone, cloudlet);
            scriptThread = new Thread(MS);
            scriptThread.start();
        } catch (Exception e) {
            Log.e(TAG, e.toString());
        }
    }

    @Override
    protected void onStart(){
        super.onStart();
        Loader.load(opencv_java.class);
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (drone != null) {
            try {
                drone.kill();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();
        heartbeatsSent = 0;
    }

    private DroneItf getDrone(Platform platform) throws Exception {
        if (platform == Platform.ANAFI) {
            return new ParrotAnafi(sdk);
        } else {
            throw new Exception("Unsupported platform: " + platform.name());
        }
    }

    private CloudletItf getCloudlet() {
        return new ElijahCloudlet(serverComm, uuid);
    }

    /** This needs to change in future to pull from CNC module **/
    private void download(String url, File dex) throws Exception {
        BufferedInputStream bis;
        BufferedOutputStream bos;
        try {
            URL urlObject = new URL(url);
            HttpURLConnection httpConn = (HttpURLConnection) LTEnetwork.openConnection(urlObject);
            int responseCode = httpConn.getResponseCode();
            if (responseCode != HttpURLConnection.HTTP_OK) {
                throw new RuntimeException("Download didn't work, http status: " + responseCode);
            }
            bis = new BufferedInputStream(httpConn.getInputStream());
            bos = new BufferedOutputStream(new FileOutputStream(dex));
            copy(bis, bos);
            bis.close();
            bos.close();
        } catch (IOException e) {
            throw new IOException(e);
        }
    }

    /** Helper function used by download **/
    private void copy(BufferedInputStream src, OutputStream dst) throws IOException {
        int len;
        byte[] buf = new byte[8 * 1024];
        while ((len = src.read(buf, 0, 8 * 1024)) > 0) {
            dst.write(buf, 0, len);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }
}
