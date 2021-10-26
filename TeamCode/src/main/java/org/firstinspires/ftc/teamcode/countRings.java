package org.firstinspires.ftc.teamcode;

public class countRings {
    public static int countRing(int size){
        int mode = 0;
        if (size == 0){
            mode = 1;
        }else if(size >= Constant.min && size <= Constant.max){
            mode = 2;
        }else{
            mode = 3;
        }
        return mode;
    }
}
