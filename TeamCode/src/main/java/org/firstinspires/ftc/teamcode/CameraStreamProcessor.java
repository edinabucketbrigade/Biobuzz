package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.ByteArrayOutputStream;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Grabs every frame the camera produces and keeps the latest one as a Bitmap.
 * FtcDashboard pulls that Bitmap through the CameraStreamSource interface.
 */
public class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final AtomicLong frameCount = new AtomicLong(0);
    private byte[] cachedJpeg = null;
    private long cachedJpegFrame = -1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        frameCount.incrementAndGet();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Nothing to draw on the Driver Hub preview.
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    /** Number of frames received so far (0 until the camera delivers its first frame). */
    public long getFrameCount() {
        return frameCount.get();
    }

    /** Latest frame as a JPEG. Encoded only when asked, and cached until the next frame arrives. */
    public synchronized byte[] getJpeg(int quality) {
        long id = frameCount.get();
        if (cachedJpeg != null && cachedJpegFrame == id) return cachedJpeg;
        ByteArrayOutputStream out = new ByteArrayOutputStream(32 * 1024);
        lastFrame.get().compress(Bitmap.CompressFormat.JPEG, quality, out);
        cachedJpeg = out.toByteArray();
        cachedJpegFrame = id;
        return cachedJpeg;
    }
}