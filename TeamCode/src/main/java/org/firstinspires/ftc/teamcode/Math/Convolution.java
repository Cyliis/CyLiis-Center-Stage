package org.firstinspires.ftc.teamcode.Math;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

public class Convolution {
    public static ArrayList<Complex> convolve(ArrayList<Complex> a, ArrayList<Complex> b){

        int asize = a.size();
        int bsize = b.size();

        for(int i = 0;i<bsize-1;i++) a.add(new Complex(0,0));
        for(int i = 0;i<asize-1;i++) b.add(new Complex(0,0));

        int log2n = (int)(Math.log(a.size())/Math.log(2)) + 1;

        while(a.size()<(1<<log2n)){
            a.add(new Complex(0,0));
            b.add(new Complex(0,0));
        }

        a = FFT.fft(a,log2n);
        b = FFT.fft(b,log2n);
//        System.out.println(a);
//        System.out.println(b);
        for(int i = 0;i<(1<<log2n);i++){
            a.set(i,a.get(i).mult(b.get(i)));
        }
//        System.out.println(a);
        a = FFT.ift(a, log2n);

        ArrayList<Complex> ans = new ArrayList<>();

        for(int i = 0;i<asize+bsize-1;i++) ans.add(a.get(i));

        return ans;
    }

    public static ArrayList<Complex> offsetConvolution(ArrayList<Complex> a, ArrayList<Complex> b, int offset){

        int asize = a.size();
        int bsize = b.size();

        for(int i = 0;i<bsize-1;i++) a.add(new Complex(0,0));
        for(int i = 0;i<asize-1;i++) b.add(new Complex(0,0));

        int log2n = (int)(Math.log(a.size())/Math.log(2)) + 1;

        while(a.size()<(1<<log2n)){
            a.add(new Complex(0,0));
            b.add(new Complex(0,0));
        }

        a = FFT.fft(a,log2n);
        b = FFT.fft(b,log2n);
//        System.out.println(a);
//        System.out.println(b);
        for(int i = 0;i<(1<<log2n);i++){
            a.set(i,a.get(i).mult(b.get(i)));
        }
//        System.out.println(a);
        a = FFT.ift(a, log2n);

        ArrayList<Complex> ans = new ArrayList<>();

        for(int i = 0;i<asize;i++) ans.add(a.get(offset+i));

        return ans;
    }
}
