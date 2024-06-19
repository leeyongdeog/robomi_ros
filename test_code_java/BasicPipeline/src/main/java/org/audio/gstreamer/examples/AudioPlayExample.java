package org.audio.gstreamer.examples;

import java.util.concurrent.TimeUnit;

import org.freedesktop.gstreamer.Gst;
import org.freedesktop.gstreamer.Pipeline;

public class AudioPlayExample {

    private static Pipeline pipeline;

    public static void main(String[] args) throws Exception {
        // GStreamer 초기화
        Utils.configurePaths();
        Gst.init("AudioPlayExample", args);

        // 파이프라인 생성 및 재생
        pipeline = (Pipeline) Gst.parseLaunch("autoaudiosrc ! autoaudiosink");
        pipeline.play();

        Gst.getExecutor().schedule(Gst::quit, 10, TimeUnit.SECONDS);
        // 메인 루프 실행 (GStreamer 종료 방지)
         Gst.main();
    }
}
