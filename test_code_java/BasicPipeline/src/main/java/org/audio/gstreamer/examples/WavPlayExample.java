package org.audio.gstreamer.examples;

import org.freedesktop.gstreamer.Gst;
import org.freedesktop.gstreamer.Pipeline;

import java.util.concurrent.TimeUnit;

public class WavPlayExample {

    private static Pipeline pipeline;

    public static void main(String[] args) throws Exception {
        // GStreamer 초기화
        Utils.configurePaths();
        Gst.init("WavePlayExample", args);

        String wavFile1 = WavPlayExample.class.getResource("/bbommi.wav").toString();
//        System.out.println("wavFile1 OK: " + wavFile1); // 'file:' 접두어 문자열 제거 필요

        // 파이프라인 생성 및 재생
//        pipeline = (Pipeline) Gst.parseLaunch("filesrc location=/home/yh6/Documents/BasicPipeline/src/main/res/bbommi.wav ! decodebin ! autoaudiosink");
        String pipeDesc = "filesrc location="+wavFile1.substring(5)+" ! decodebin ! autoaudiosink";
//        System.out.println("pipeDesc OK: " + pipeDesc);
        pipeline = (Pipeline) Gst.parseLaunch(pipeDesc);
        pipeline.play();
        Gst.getExecutor().schedule(Gst::quit, 10, TimeUnit.SECONDS);
        // 메인 루프 실행 (GStreamer 종료 방지)
         Gst.main();
    }
}
