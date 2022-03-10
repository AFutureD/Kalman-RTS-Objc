# Kalman-RTS-Objc

Usage


```

int main(int argc, char *argv[]) {
    @autoreleasepool {
        KalmanFilter * kf = [KalmanFilter new];
        NSArray * arr = @[
            @-53, @-54, @-55, @-59, @-59, @-60, @-52, @-44, @-44, @-49, @-45, @-46, @-46, @-49, @-47, @-50, @-50, @-50, @-46, @-43, @-43, @-44, @-47, @-46, @-46, @-46, @-46, @-46, @-44, @-42, @-46, @-46, @-46, @-43, @-46
        ];
        
        [kf setupRTSCount:arr.count];
        
        NSArray * kamlam = [kf estimateArray:arr];
        
        NSArray * rts = [kf smoothArrayWithRTS];
        
        NSLog(@"kalman: %@ rts: %@", kamlam, rts);
    }
}


```