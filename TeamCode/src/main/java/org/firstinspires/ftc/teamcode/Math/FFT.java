package org.firstinspires.ftc.teamcode.Math;

import java.util.ArrayList;
import java.util.Collections;

import static java.lang.Math.PI;
import static java.lang.Math.log;

public class FFT {
    private static int bitReverse(int x, int log2n){
        int n = 0;
        int x2 = x;
        for(int i = 0;i<log2n;i++){
            n*=2;
            if(x2%2==1) n+=1;
            x2/=2;
        }
        return n;
    }
    public static ArrayList<Complex> fft(ArrayList<Complex> f, int log2n){
        int n = 1 << log2n;
        ArrayList<Complex> ans=new ArrayList<>(Collections.nCopies(n, new Complex(0,0)));
        Complex J = new Complex(0, 1);
        for (int i = 0; i < n; ++i) {
            ans.set(bitReverse(i, log2n), f.get(i));
        }
        for (int s = 1; s <= log2n; ++s) {
            int m = 1 << s;
            int m2 = m >> 1;
            Complex w = new Complex(1, 0);
            Complex wm = Complex.exp(J.mult(new Complex(-1,0)).mult(new Complex(PI / m2,0)));
            for (int j = 0; j < m2; ++j) {
                for (int k = j; k < n; k += m) {
                    Complex t = w.mult(ans.get(k+m2));
                    Complex u = ans.get(k);
                    ans.set(k,u.add(t));
                    ans.set(k + m2, u.add(t.mult(new Complex(-1,0))));
                }
                w = w.mult(wm);
            }
        }
        return ans;
    }
    public static ArrayList<Complex> ift(ArrayList<Complex> s, int log2n){
        int n = (1<<log2n);
        ArrayList<Complex> ans = new ArrayList<>(s);
        for(int i = 0;i<n;i++){
            ans.set(i, s.get(i).conj());
        }
        ans = fft(ans, log2n);
        for(int i = 0;i<n;i++){
            ans.set(i, ans.get(i).conj().mult(new Complex(1.0/(double)n,0)));
        }
        return ans;
    }
}
